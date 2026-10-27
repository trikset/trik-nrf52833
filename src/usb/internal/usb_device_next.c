/*
 * Copyright (c) 2018 Intel Corporation
 * Copyright (c) 2023 Nordic Semiconductor ASA
 * Copyright (c) 2026 CyberTech Labs Ltd
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * This file is based on the Zephyr Bluetooth HCI USB, Zephyr Bluetooth CDC ACM transport
 * implementation. Modified and adapted by CyberTech Labs Ltd.
 */

#include <errno.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/usb/usbd.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/usb/udc.h>
#include <zephyr/usb/usb_ch9.h>
#include "usb_device_api.h"
#include "protocol_schema.h"

LOG_MODULE_REGISTER(usb_module, LOG_LEVEL_INF);

extern struct net_buf *take_periphery_snapshot(uint64_t *in_ts_us);
extern int telemetry_control(struct request_ctx *ctx, uint32_t period_us, uint8_t enable);

#define DT_DRV_COMPAT           zephyr_usb_raw
#define RAW_USB_CLASS_ENABLED   0
#define RAW_USB_CLASS_SUSPENDED 1
#define RAW_USB_RX_FIFO_BUSY    2
#define PACKAGE_TYPE_TELEMETRY  1
#define PACKAGE_TYPE_STREAM     2

/// Using a ~256-byte buffer allows grouping multiple USB bulk packets (MPS=64)
/// into a single higher-level transfer handled by the UDC driver. The hardware
/// still transmits data in 64-byte chunks, but the driver processes them as one
/// logical transfer.
///
/// This reduces the number of completed transfer events (EP_FINISHED),
/// minimizes net_buf allocation/free operations, and decreases DMA setup
/// and synchronization overhead. It also lowers the number of context
/// switches and callback invocations in the Zephyr USB stack.
///
/// The optimization is platform-specific and based on the behavior of the
/// nRF UDC driver (see zephyr/drivers/usb/udc/udc_nrf.c).
#define USB_POOL_BUF_SIZE 265
#define UDC_CREDITS       2

struct usb_raw_buf_info {
	struct udc_buf_info udc;
	uintptr_t pkg_type;
} __packed;

#if CONFIG_USBD_RAW_BUF_POOL
UDC_BUF_POOL_DEFINE(usb_raw_ep_pool, DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) * 4, USB_POOL_BUF_SIZE,
		    sizeof(struct usb_raw_buf_info), NULL);
static struct net_buf *usb_raw_buf_alloc(struct usbd_class_data *const c_data, const uint8_t ep,
					 k_timeout_t timeout)
{
	ARG_UNUSED(c_data);
	struct net_buf *buf;
	struct usb_raw_buf_info *rbi;

	buf = net_buf_alloc(&usb_raw_ep_pool, timeout);
	if (buf) {
		rbi = (struct usb_raw_buf_info *)udc_get_buf_info(buf);
		rbi->udc.ep = ep;
	}
	return buf;
}
#else
static struct net_buf *usb_raw_buf_alloc(struct usbd_class_data *const c_data, const uint8_t ep,
					 k_timeout_t timeout)
{
	ARG_UNUSED(timeout);
	return usbd_ep_buf_alloc(c_data, ep, USB_POOL_BUF_SIZE);
}
#endif

struct usb_raw_desc {
	struct usb_if_descriptor if0;
	struct usb_ep_descriptor ep_out;
	struct usb_ep_descriptor ep_in;
	struct usb_ep_descriptor ep_intr_out;
	struct usb_ep_descriptor ep_iso_in;
	struct usb_desc_header nil_desc;
};

struct usb_raw_data_config {
	struct usbd_class_data *c_data;
	struct usb_raw_desc *const desc;
	const struct usb_desc_header **const fs_desc;
};

struct usb_raw_data {
	const struct device *dev;
	struct usbd_class_data *c_data;
	struct request_ctx *ctx;
	struct k_sem telemetry_tick;
	struct k_sem telemetry_slot;
	struct k_sem stream_slot;
	struct k_sem telemetry_iso_slot;
	atomic_t state;
};

static uint8_t usb_raw_get_bulk_out(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct usb_raw_data_config *cfg = dev->config;
	struct usb_raw_desc *desc = cfg->desc;
	return desc->ep_out.bEndpointAddress;
}

static uint8_t usb_raw_get_bulk_in(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct usb_raw_data_config *cfg = dev->config;
	struct usb_raw_desc *desc = cfg->desc;
	return desc->ep_in.bEndpointAddress;
}

static uint8_t usb_raw_get_intr_out(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct usb_raw_data_config *cfg = dev->config;
	struct usb_raw_desc *desc = cfg->desc;
	return desc->ep_intr_out.bEndpointAddress;
}

static int usb_raw_rx_fifo_handler(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	struct net_buf *buf;
	int ret;

	if (!atomic_test_bit(&usb_data->state, RAW_USB_CLASS_ENABLED) ||
	    atomic_test_bit(&usb_data->state, RAW_USB_CLASS_SUSPENDED)) {
		LOG_WRN("USB configuration is not enabled or suspended");
		return -EPERM;
	}

	if (atomic_test_and_set_bit(&usb_data->state, RAW_USB_RX_FIFO_BUSY)) {
		LOG_WRN("RX FIFO busy, skipping handler");
		return -EBUSY;
	}

	buf = usb_raw_buf_alloc(c_data, usb_raw_get_bulk_out(c_data), K_NO_WAIT);
	if (buf == NULL) {
		LOG_WRN("UDC buffer pool exhausted! Throttling USB host.");
		return -ENOMEM;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for 0x%02x", usb_raw_get_bulk_out(c_data));
		net_buf_unref(buf);
	}

	return ret;
}

static struct net_buf *get_udc_buf(struct usbd_class_data *const c_data, k_timeout_t timeout)
{
	struct net_buf *buf;
	buf = usb_raw_buf_alloc(c_data, usb_raw_get_bulk_in(c_data), timeout);
	if (buf == NULL) {
		LOG_WRN("UDC buffer tx pool exhausted! Throttling USB host.");
		return NULL;
	}
	return buf;
}

/* Enqueue a prepared payload buffer onto the raw IN endpoint.
 *
 * Returns 0 on success (the UDC driver now owns `udc_buf` until the transfer
 * completes) or a negative errno otherwise. On failure the caller must unref
 * `udc_buf` and release the corresponding TX slot itself.
 */
static int usb_raw_tx_enqueue(struct usbd_class_data *const c_data, struct net_buf *const raw_buf,
			      struct net_buf *const udc_buf)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	int ret;

	if (!atomic_test_bit(&usb_data->state, RAW_USB_CLASS_ENABLED) ||
	    atomic_test_bit(&usb_data->state, RAW_USB_CLASS_SUSPENDED)) {
		/* The host can be disconnected while telemetry is still running;
		 * the TX thread then retries once per telemetry period, so only
		 * log the transition, not every retry.
		 */
		static int64_t last_log_ms;

		int64_t now_ms = k_uptime_get();
		if (now_ms - last_log_ms > 1000) {
			LOG_WRN("USB transport not ready, dropping packet");
			last_log_ms = now_ms;
		}
		return -EPERM;
	}

	net_buf_add_mem(udc_buf, raw_buf->data, raw_buf->len);

	ret = usbd_ep_enqueue(c_data, udc_buf);
	if (ret) {
		LOG_ERR("Failed to usbd_ep_enqueue: %d", ret);
	}

	return ret;
}

/// Most incoming USB (Host->device) requests are currently control requests related to the
/// periphery. Processing such packets is relatively simple and takes tens of microseconds, as it
/// involves parsing the header and invoking a callback (for example, retrieving analog sensor
/// values, resetting the encoder, setting PWM period/duty cycle, or handling rare Bluetooth control
/// commands).
///
/// In future implementations, perform_usb_request for specific task types can be made asynchronous
/// by introducing an additional pool for incoming packets. This would allow immediate freeing of
/// the input buffer and queuing for deferred processing, increasing throughput at the cost of
/// higher latency (RTT).
///
/// The current implementation processes the buffer immediately in perform_usb_request, assuming
/// that:
///  - logical message size is limited to <= 256 bytes + header (protocol-level constraint),
///  - the host sends one logical message per libusb submission (URB), serialized under a mutex.
///
/// USB guarantees packet (transaction) boundaries (e.g. 64-byte FS bulk max packet size),
/// but it does not define message boundaries. Message framing is always a protocol-level concern.
///
/// In Linux, each libusb transfer submission maps to a separate URB. URBs are queued independently
/// into the USB core and are not interleaved in a way that would mix payload data between them.
/// However, the host controller may still split a single URB into multiple USB transactions
/// (e.g. multiple 64-byte packets for full-speed bulk), or complete it using a short packet
/// condition.
///
/// A short packet (< 64 bytes for full-speed bulk) signals end-of-transfer, not end-of-URB.
/// It is a transfer-level completion condition used by the USB protocol to indicate that
/// no further data follows for that endpoint in the current transfer.
///
/// When a short packet is received or the internal buffer (265 bytes) is filled,
/// accumulated data is treated as one logical message at the device (UDC driver) level.
/// If transfers are aligned to max packet size (64 bytes), an explicit framing mechanism
/// (length field, delimiter, or ZLP-based protocol) is required to preserve message boundaries.
///
/// To support logical packets larger than 256 bytes, it is necessary to extend usb_raw_data with
/// fields such as accumulated length and buffer state, and assemble payloads across multiple USB
/// transfers sequentially.
static int usb_raw_intr_rx_handler(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	struct net_buf *buf;
	int ret;

	if (!atomic_test_bit(&usb_data->state, RAW_USB_CLASS_ENABLED) ||
	    atomic_test_bit(&usb_data->state, RAW_USB_CLASS_SUSPENDED)) {
		return -EPERM;
	}

	buf = usb_raw_buf_alloc(c_data, usb_raw_get_intr_out(c_data), K_NO_WAIT);
	if (buf == NULL) {
		LOG_WRN("UDC buffer pool exhausted for intr OUT!");
		return -ENOMEM;
	}

	ret = usbd_ep_enqueue(c_data, buf);
	if (ret) {
		LOG_ERR("Failed to enqueue net_buf for intr 0x%02x", usb_raw_get_intr_out(c_data));
		net_buf_unref(buf);
	}

	return ret;
}

static int usb_raw_intr_out_cb(struct usbd_class_data *const c_data, struct net_buf *const buf,
			       const int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;

	if (err) {
		goto restart_intr_out;
	}

	int perform_err = perform_usb_request(usb_data->ctx, buf);

	if (perform_err != 0) {
		LOG_ERR("Error processing intr cmd: %d", perform_err);
	}

restart_intr_out:
	net_buf_unref(buf);
	return usb_raw_intr_rx_handler(c_data);
}

static int usb_raw_out_cb(struct usbd_class_data *const c_data, struct net_buf *const buf,
			  const int err)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;

	if (err) {
		goto restart_out_transfer;
	}

	int perform_err = perform_usb_request(usb_data->ctx, buf);

	if (perform_err != 0) {
		LOG_ERR("Error sending to driver %d", err);
	}

restart_out_transfer:
	net_buf_unref(buf);
	atomic_clear_bit(&usb_data->state, RAW_USB_RX_FIFO_BUSY);

	return usb_raw_rx_fifo_handler(c_data);
}

static void usb_raw_enable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;

	atomic_set_bit(&usb_data->state, RAW_USB_CLASS_ENABLED);

	if (usb_raw_rx_fifo_handler(c_data)) {
		LOG_ERR("Failed to start OUT transfer");
	}

	if (usb_raw_intr_rx_handler(c_data)) {
		LOG_ERR("Failed to start intr OUT transfer");
	}
}

static void usb_raw_disable(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	atomic_clear_bit(&usb_data->state, RAW_USB_CLASS_ENABLED);
	atomic_clear_bit(&usb_data->state, RAW_USB_CLASS_SUSPENDED);
	LOG_INF("Configuration disabled");
}

static void *usb_raw_get_desc(struct usbd_class_data *const c_data, const enum usbd_speed speed)
{
	const struct device *dev = usbd_class_get_private(c_data);
	const struct usb_raw_data_config *cfg = dev->config;
	return cfg->fs_desc;
}

static void usb_raw_suspended(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	atomic_set_bit(&usb_data->state, RAW_USB_CLASS_SUSPENDED);
}

static void usb_raw_resumed(struct usbd_class_data *const c_data)
{
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	atomic_clear_bit(&usb_data->state, RAW_USB_CLASS_SUSPENDED);
}

static int usb_raw_request(struct usbd_class_data *const c_data, struct net_buf *buf, int err)
{
	struct usbd_context *uds_ctx = usbd_class_get_ctx(c_data);
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;
	struct usb_raw_buf_info *rbi = (struct usb_raw_buf_info *)udc_get_buf_info(buf);

	if (err) {
		LOG_ERR("USB ep 0x%02x request error: %d", rbi->udc.ep, err);
	}

	if (rbi->udc.ep == usb_raw_get_bulk_in(c_data)) {
		err = usbd_ep_buf_free(uds_ctx, buf);
		uintptr_t type = rbi->pkg_type;
		if (type == PACKAGE_TYPE_TELEMETRY) {
			k_sem_give(&usb_data->telemetry_slot);
		} else {
			k_sem_give(&usb_data->stream_slot);
		}
		return err;
	}

#if IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO)
	if (rbi->udc.ep == CONFIG_RAW_USB_TELEMETRY_ISO_EP) {
		/* One isochronous frame has been transmitted. Release the frame
		 * slot so the ISO telemetry thread can arm the next frame.
		 */
		err = usbd_ep_buf_free(uds_ctx, buf);
		k_sem_give(&usb_data->telemetry_iso_slot);
		return err;
	}
#endif

	if (rbi->udc.ep == usb_raw_get_bulk_out(c_data)) {
		return usb_raw_out_cb(c_data, buf, err);
	}

	if (rbi->udc.ep == usb_raw_get_intr_out(c_data)) {
		return usb_raw_intr_out_cb(c_data, buf, err);
	}

	return usbd_ep_buf_free(uds_ctx, buf);
}

static int usb_raw_init(struct usbd_class_data *const c_data)
{
	ARG_UNUSED(c_data);
	return 0;
}

static void usb_raw_update(struct usbd_class_data *const c_data, uint8_t iface, uint8_t alternate)
{
	LOG_INF("Interface update: iface %u, alt %u", iface, alternate);
}

static int usbd_raw_ctd(struct usbd_class_data *const c_data,
			const struct usb_setup_packet *const setup, const struct net_buf *const buf)
{
	if (USB_REQTYPE_GET_TYPE(setup->bmRequestType) != USB_REQTYPE_TYPE_VENDOR) {
		LOG_WRN("Non-vendor request type 0x%02x", setup->bmRequestType);
		return -ENOTSUP;
	}

	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *usb_data = dev->data;

	if (setup->bRequest == USB_RAW_REQ_SET_TELEMETRY) {
		if (buf->len < 5) {
			LOG_WRN("Telemetry request too short: %u < 5", buf->len);
			return -EMSGSIZE;
		}
		uint32_t period_us = sys_get_le32(buf->data);
		uint8_t enable = buf->data[4];
		return telemetry_control(usb_data->ctx, period_us, enable);
	}

	int err = perform_cmd_request(usb_data->ctx, buf);

	if (err) {
		LOG_ERR("perform_cmd_request failed: %d", err);
	}

	return 0;
}

static int usb_raw_cth(struct usbd_class_data *const c_data,
		       const struct usb_setup_packet *const setup, struct net_buf *const buf)
{
	ARG_UNUSED(c_data);

	if (setup->bRequest != USB_RAW_REQ_GET_VERSION) {
		errno = -ENOTSUP;
		return 0;
	}

	const char *version = CONFIG_RAW_USB_VERSION;
	size_t len = MIN(setup->wLength, net_buf_tailroom(buf));
	net_buf_add_mem(buf, version, MIN(len, strlen(version)));

	return 0;
}

struct usbd_class_api usbd_usb_raw_api = {
	.request = usb_raw_request,
	.update = usb_raw_update,
	.enable = usb_raw_enable,
	.control_to_dev = usbd_raw_ctd,
	.control_to_host = usb_raw_cth,
	.disable = usb_raw_disable,
	.suspended = usb_raw_suspended,
	.resumed = usb_raw_resumed,
	.get_desc = usb_raw_get_desc,
	.init = usb_raw_init,
};

/// BULK TX thread: serves the request/response stream FIFO on the BULK IN
/// endpoint.
///
/// With the isochronous telemetry transport compiled in, periodic periphery
/// data is sent on the ISO endpoint, so this thread only drains the stream.
/// Otherwise it also serves telemetry: when a UDC credit or buffer is busy the
/// tick is skipped and the freshest snapshot is sent on the next poll
/// (latest-wins, a new sample arrives shortly).
static void usb_raw_tx_thread(void *p1, void *p2, void *p3)
{
	struct usbd_class_data *const c_data = p1;
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *data = dev->data;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

#if IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO)
	struct k_poll_event events[1];
#else
	struct k_poll_event events[2];
	k_poll_event_init(&events[1], K_POLL_TYPE_SEM_AVAILABLE, 0, &data->telemetry_tick);
#endif
	k_poll_event_init(&events[0], K_POLL_TYPE_FIFO_DATA_AVAILABLE, K_POLL_MODE_NOTIFY_ONLY,
			  data->ctx->tx_queue);

	struct net_buf *buf;
	struct net_buf *udc_buf;

	while (1) {
		k_poll(events, ARRAY_SIZE(events), K_USEC(250));

#if !IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO)
		if (events[1].state == K_POLL_STATE_SEM_AVAILABLE) {
			events[1].state = K_POLL_STATE_NOT_READY;

			if (k_sem_take(&data->telemetry_slot, K_NO_WAIT) != 0) {
				continue;
			}

			udc_buf = get_udc_buf(c_data, K_NO_WAIT);
			if (!udc_buf) {
				k_sem_give(&data->telemetry_slot);
				continue;
			}

			k_sem_take(&data->telemetry_tick, K_NO_WAIT);

			buf = take_periphery_snapshot(NULL);
			if (buf) {
				((struct usb_raw_buf_info *)udc_get_buf_info(udc_buf))->pkg_type =
					PACKAGE_TYPE_TELEMETRY;
				if (usb_raw_tx_enqueue(c_data, buf, udc_buf)) {
					k_sem_give(&data->telemetry_slot);
					net_buf_unref(udc_buf);
				}
				net_buf_unref(buf);
			} else {
				k_sem_give(&data->telemetry_slot);
				net_buf_unref(udc_buf);
			}
		}
#endif

		if (events[0].state == K_POLL_STATE_FIFO_DATA_AVAILABLE) {
			events[0].state = K_POLL_STATE_NOT_READY;

			if (k_sem_take(&data->stream_slot, K_NO_WAIT) != 0) {
				continue;
			}

			udc_buf = get_udc_buf(c_data, K_NO_WAIT);

			if (!udc_buf) {
				k_sem_give(&data->stream_slot);
				continue;
			}

			buf = k_fifo_get(data->ctx->tx_queue, K_NO_WAIT);
			if (!buf) {
				k_sem_give(&data->stream_slot);
				net_buf_unref(udc_buf);
				continue;
			}

			((struct usb_raw_buf_info *)udc_get_buf_info(udc_buf))->pkg_type =
				PACKAGE_TYPE_STREAM;
			if (usb_raw_tx_enqueue(c_data, buf, udc_buf)) {
				k_sem_give(&data->stream_slot);
				net_buf_unref(udc_buf);
			}
			net_buf_unref(buf);
		}
	}
}

#if IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO)
/// ISO telemetry thread.
///
/// Two independent pacing gates, implemented as blocking semaphore waits so
/// the thread sleeps when there is nothing to do (no busy polling):
///  1. telemetry_tick - a fresh snapshot is waiting. It is given by the TIMER2
///     sampling ISR once per host-requested period; this gate limits the send
///     rate to the sample rate and prevents re-sending a stale frame.
///  2. telemetry_iso_slot - the previous ISO frame finished on the bus. The
///     UDC request callback releases it on every completed transfer.
///
/// The on-the-wire cadence (~1 kHz, one packet per full-speed frame) comes
/// from the USB frames themselves: the next frame is armed right after the
/// previous one completes, so at most one transfer is ever in flight.
///
/// Buffer ownership: the snapshot (`buf`, protocol RX pool) is copied into a
/// UDC endpoint buffer (`udc_buf`) by usb_raw_tx_enqueue() and unref'd here
/// right after the copy; `udc_buf` is owned by the controller once enqueued
/// and is freed by the request callback together with the slot release.
/// On any failure this thread returns the slot and frees what it still owns.
///
/// BULK and ISO telemetry are mutually exclusive compile-time options: the
/// thread, its stack and the SYS_INIT hook are only allocated when
/// CONFIG_RAW_USB_TELEMETRY_ISO is set, so a BULK build carries no ISO thread.
static void usb_raw_iso_telemetry_thread(void *p1, void *p2, void *p3)
{
	struct usbd_class_data *const c_data = p1;
	const struct device *dev = usbd_class_get_private(c_data);
	struct usb_raw_data *data = dev->data;
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct net_buf *buf;
	struct net_buf *udc_buf;
	uint64_t ts = 0;

	while (1) {
		k_sem_take(&data->telemetry_tick, K_FOREVER);
		k_sem_take(&data->telemetry_iso_slot, K_FOREVER);

		buf = take_periphery_snapshot(&ts);
		if (buf == NULL) {
			k_sem_give(&data->telemetry_iso_slot);
			continue;
		}

		udc_buf = usb_raw_buf_alloc(c_data, CONFIG_RAW_USB_TELEMETRY_ISO_EP, K_NO_WAIT);
		if (udc_buf == NULL) {
			k_sem_give(&data->telemetry_iso_slot);
			net_buf_unref(buf);
			continue;
		}

		((struct usb_raw_buf_info *)udc_get_buf_info(udc_buf))->pkg_type =
			PACKAGE_TYPE_TELEMETRY;
		if (usb_raw_tx_enqueue(c_data, buf, udc_buf)) {
			k_sem_give(&data->telemetry_iso_slot);
			net_buf_unref(udc_buf);
		}
		net_buf_unref(buf);
	}
}
#endif /* CONFIG_RAW_USB_TELEMETRY_ISO */

#define USB_RAW_DEFINE_DESCRIPTOR(n)                                                               \
	static struct usb_raw_desc usb_raw_desc_##n = {                                            \
		.if0 =                                                                             \
			{                                                                          \
				.bLength = sizeof(struct usb_if_descriptor),                       \
				.bDescriptorType = USB_DESC_INTERFACE,                             \
				.bInterfaceNumber = 0,                                             \
				.bNumEndpoints =                                                   \
					(IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO) ? 3 : 2) +       \
					DT_INST_NODE_HAS_PROP(n, intr_out_ep),                     \
				.bInterfaceClass = USB_BCC_VENDOR,                                 \
				.bInterfaceSubClass = 0,                                           \
				.bInterfaceProtocol = 0,                                           \
			},                                                                         \
		.ep_out =                                                                          \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = DT_INST_PROP(n, bulk_out_ep),                  \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64),                             \
				.bInterval = 0,                                                    \
			},                                                                         \
		.ep_in =                                                                           \
			{                                                                          \
				.bLength = sizeof(struct usb_ep_descriptor),                       \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = DT_INST_PROP(n, bulk_in_ep),                   \
				.bmAttributes = USB_EP_TYPE_BULK,                                  \
				.wMaxPacketSize = sys_cpu_to_le16(64),                             \
				.bInterval = 0,                                                    \
			},                                                                         \
		.ep_intr_out =                                                                     \
			{                                                                          \
				.bLength = DT_INST_NODE_HAS_PROP(n, intr_out_ep)                   \
						   ? sizeof(struct usb_ep_descriptor)              \
						   : 0,                                            \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = DT_INST_PROP_OR(n, intr_out_ep, 0),            \
				.bmAttributes = USB_EP_TYPE_INTERRUPT,                             \
				.wMaxPacketSize = sys_cpu_to_le16(64),                             \
				.bInterval = 1,                                                    \
			},                                                                         \
		.ep_iso_in =                                                                       \
			{                                                                          \
				.bLength = IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO)                \
						   ? sizeof(struct usb_ep_descriptor)              \
						   : 0,                                            \
				.bDescriptorType = USB_DESC_ENDPOINT,                              \
				.bEndpointAddress = CONFIG_RAW_USB_TELEMETRY_ISO_EP,               \
				.bmAttributes = USB_EP_TYPE_ISO,                                   \
				.wMaxPacketSize = sys_cpu_to_le16(128),                            \
				.bInterval = 1,                                                    \
			},                                                                         \
		.nil_desc = {.bLength = 0, .bDescriptorType = 0},                                  \
	};                                                                                         \
	static const struct usb_desc_header *usb_raw_fs_desc_##n[] = {                             \
		(struct usb_desc_header *)&usb_raw_desc_##n.if0,                                   \
		(struct usb_desc_header *)&usb_raw_desc_##n.ep_out,                                \
		(struct usb_desc_header *)&usb_raw_desc_##n.ep_in,                                 \
		(struct usb_desc_header *)&usb_raw_desc_##n.ep_intr_out,                           \
		(struct usb_desc_header *)&usb_raw_desc_##n.ep_iso_in,                             \
		(struct usb_desc_header *)&usb_raw_desc_##n.nil_desc,                              \
	};

#define USBD_RAW_DT_DEVICE_DEFINE(n)                                                               \
                                                                                                   \
	USB_RAW_DEFINE_DESCRIPTOR(n);                                                              \
                                                                                                   \
	static K_FIFO_DEFINE(usb_raw_tx_fifo_##n);                                                 \
	static K_FIFO_DEFINE(usb_raw_poll_fifo_##n);                                               \
	static K_KERNEL_STACK_DEFINE(usb_tx_stack_##n, 1024);                                      \
	static struct k_thread usb_tx_thread_##n;                                                  \
                                                                                                   \
	USBD_DEFINE_CLASS(usb_raw_##n, &usbd_usb_raw_api, (void *)DEVICE_DT_GET(DT_DRV_INST(n)),   \
			  NULL);                                                                   \
                                                                                                   \
	static const struct usb_raw_data_config usb_raw_config_##n = {                             \
		.c_data = &usb_raw_##n,                                                            \
		.desc = &usb_raw_desc_##n,                                                         \
		.fs_desc = usb_raw_fs_desc_##n,                                                    \
	};                                                                                         \
                                                                                                   \
	static struct request_ctx request_ctx_##n = {                                              \
		.tx_queue = &usb_raw_tx_fifo_##n,                                                  \
	};                                                                                         \
                                                                                                   \
	static struct usb_raw_data usb_raw_data_##n = {                                            \
		.dev = DEVICE_DT_GET(DT_DRV_INST(n)),                                              \
		.c_data = &usb_raw_##n,                                                            \
		.ctx = &request_ctx_##n,                                                           \
		.telemetry_tick = Z_SEM_INITIALIZER(usb_raw_data_##n.telemetry_tick, 0, 1),        \
		.telemetry_slot = Z_SEM_INITIALIZER(usb_raw_data_##n.telemetry_slot, UDC_CREDITS,  \
						    UDC_CREDITS),                                  \
		.stream_slot =                                                                     \
			Z_SEM_INITIALIZER(usb_raw_data_##n.stream_slot, UDC_CREDITS, UDC_CREDITS), \
		.telemetry_iso_slot =                                                              \
			Z_SEM_INITIALIZER(usb_raw_data_##n.telemetry_iso_slot, 1, 1),              \
	};                                                                                         \
                                                                                                   \
	static int usb_raw_instance_init_##n(const struct device *dev)                             \
	{                                                                                          \
		struct usb_raw_data *data = dev->data;                                             \
		request_ctx_##n.telemetry_tick = &data->telemetry_tick;                            \
		k_thread_create(&usb_tx_thread_##n, usb_tx_stack_##n,                              \
				K_KERNEL_STACK_SIZEOF(usb_tx_stack_##n), usb_raw_tx_thread,        \
				(void *)data->c_data, NULL, NULL, 4, 0, K_NO_WAIT);                \
		k_thread_name_set(&usb_tx_thread_##n, "usb_raw_tx_" #n);                           \
                                                                                                   \
		return 0;                                                                          \
	}                                                                                          \
                                                                                                   \
	DEVICE_DT_INST_DEFINE(n, usb_raw_instance_init_##n, NULL, &usb_raw_data_##n,               \
			      &usb_raw_config_##n, POST_KERNEL,                                    \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, NULL);

DT_INST_FOREACH_STATUS_OKAY(USBD_RAW_DT_DEVICE_DEFINE)

#if IS_ENABLED(CONFIG_RAW_USB_TELEMETRY_ISO)
/*
 * The ISO telemetry transport is a compile-time alternative to the BULK path
 * (they are mutually exclusive), so the dedicated thread and its stack are
 * only allocated when this option is enabled.
 *
 * This firmware uses a single "zephyr,usb-raw" instance, which the ISO thread
 * binds to. Guarded below to fail loudly if that assumption ever changes.
 */
BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 1,
	     "ISO telemetry supports exactly one zephyr,usb-raw instance");

static K_KERNEL_STACK_DEFINE(usb_iso_telemetry_stack, 1024);
static struct k_thread usb_iso_telemetry_thread;

static int usb_iso_telemetry_thread_start(void)
{
	struct usb_raw_data *data = &usb_raw_data_0;

	k_thread_create(&usb_iso_telemetry_thread, usb_iso_telemetry_stack,
			K_KERNEL_STACK_SIZEOF(usb_iso_telemetry_stack),
			usb_raw_iso_telemetry_thread, (void *)data->c_data, NULL, NULL, 4, 0,
			K_NO_WAIT);
	k_thread_name_set(&usb_iso_telemetry_thread, "usb_iso_telem");
	return 0;
}

SYS_INIT(usb_iso_telemetry_thread_start, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
#endif /* CONFIG_RAW_USB_TELEMETRY_ISO */
