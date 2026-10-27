/* Copyright 2026 CyberTech Labs Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License. */

#include <zephyr/net_buf.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include "protocol_schema.h"
#include "usb_device_api.h"
#include "periphery/periphery_addresses.h"

LOG_MODULE_REGISTER(protocol_schema, LOG_LEVEL_INF);

NET_BUF_POOL_FIXED_DEFINE(usb_rx_pool, CONFIG_USB_PROTO_RX_COUNT, CONFIG_USB_PROTO_RX_MAX_SIZE, 0,
			  NULL);

/// Receive a USB buffer from the device-side RX Pool.
/// Returns NULL on timeout or exhaustion of available buffers.
static struct net_buf *usb_buf_get_rx(k_timeout_t timeout)
{
	return net_buf_alloc(&usb_rx_pool, timeout);
}

/// Enqueue a ready-to-send packet into the TX FIFO.
/// The TX thread wakes up via k_poll when data is available.
static void push_task(struct request_ctx *ctx, struct net_buf *buf)
{
	k_fifo_put(ctx->tx_queue, buf);
}

int create_packet(const struct comm_header *in_hdr, const void *data, size_t len,
		  struct net_buf **out)
{
	struct net_buf *buf = usb_buf_get_rx(K_NO_WAIT);
	*out = buf;

	if (!buf) {
		return -ENOMEM;
	}

	if (len > CONFIG_USB_PROTO_RX_MAX_SIZE - HEADER_SIZE) {
		LOG_ERR("Payload too large: %zu", len);
		return -EMSGSIZE;
	}

	struct comm_header hdr = *in_hdr;
	hdr.len = len;

	net_buf_add_mem(buf, &hdr, HEADER_SIZE);
	net_buf_add_mem(buf, data, len);
	return 0;
}

void push_packet(struct request_ctx *ctx, struct net_buf *buf)
{
	push_task(ctx, buf);
}

static int perform_stream_request(struct request_ctx *ctx, struct net_buf *buf)
{
	if (buf->len < HEADER_SIZE) {
		LOG_ERR("Buffer too small for header");
		return -EMSGSIZE;
	}
	struct comm_header hdr;

	memcpy(&hdr, buf->data, HEADER_SIZE);

	if (hdr.len > CONFIG_USB_PROTO_RX_MAX_SIZE - HEADER_SIZE) {
		LOG_ERR("Invalid payload length: %u", hdr.len);
		return -EMSGSIZE;
	}

	const size_t payload_len = buf->len - HEADER_SIZE;

	if (payload_len < hdr.len) {
		LOG_ERR("Payload mismatch: %zu < %u", payload_len, hdr.len);
		return -EBADMSG;
	}

	net_buf_pull(buf, HEADER_SIZE);
	int err;
	switch (hdr.type) {
	case PERIPHERY:
		err = perform_periphery_request(ctx, &hdr, buf);
		break;

	default:
		LOG_ERR("Unknown stream type %u", hdr.type);
		return -EINVAL;
	}

	return err;
}

int perform_usb_request(struct request_ctx *ctx, struct net_buf *buf)
{
	return perform_stream_request(ctx, buf);
}

int perform_cmd_request(struct request_ctx *ctx, const struct net_buf *buf)
{
	ARG_UNUSED(buf);
	return 0;
}
