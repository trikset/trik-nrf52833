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

#include <stdint.h>
#include <zephyr/device.h>
#include <zephyr/usb/bos.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/usb/usbd.h>
#include "usb_device_api.h"

LOG_MODULE_REGISTER(usb_raw_init);

USBD_DEVICE_DEFINE(usb_raw, DEVICE_DT_GET(DT_NODELABEL(zephyr_udc0)), CONFIG_RAW_USB_VID,
		   CONFIG_RAW_USB_PID);

USBD_DESC_LANG_DEFINE(usb_raw_lang);
USBD_DESC_MANUFACTURER_DEFINE(usb_raw_mfr, CONFIG_RAW_USB_MANUFACTURER);
USBD_DESC_PRODUCT_DEFINE(usb_raw_product, CONFIG_RAW_USB_PRODUCT);
IF_ENABLED(CONFIG_HWINFO, (USBD_DESC_SERIAL_NUMBER_DEFINE(usb_raw_sn)));

USBD_DESC_CONFIG_DEFINE(fs_cfg_desc, "FS Configuration");

static const uint8_t attributes =
	IS_ENABLED(CONFIG_RAW_USB_SELF_POWERED) ? USB_SCD_SELF_POWERED : 0;

USBD_CONFIGURATION_DEFINE(usb_raw_fs_config, attributes, CONFIG_CUSTOM_USBD_MAX_POWER,
			  &fs_cfg_desc);

const struct device *const usb_cmd_dev = DEVICE_DT_GET(DT_NODELABEL(usb_raw_cmd));

static struct usbd_context *sample_usbd;

static int register_usb_raw_class_x(struct usbd_context *const uds_ctx, const enum usbd_speed speed,
				    const char *device_string)
{
	int err;
	err = usbd_register_class(&usb_raw, device_string, speed, 1);
	if (err) {
		LOG_ERR("Failed to register classes");
		return err;
	}

	return err;
}

static int register_usb_raw_x(struct usbd_context *const uds_ctx, const enum usbd_speed speed)
{
	struct usbd_config_node *cfg_nd;
	int err;
	cfg_nd = &usb_raw_fs_config;

	err = usbd_add_configuration(uds_ctx, speed, cfg_nd);
	if (err) {
		LOG_ERR("Failed to add configuration");
		return err;
	}

	err = register_usb_raw_class_x(uds_ctx, speed, "usb_raw_0");
	if (err) {
		LOG_ERR("Failed to register usb_raw_0 class");
		return err;
	}

	err = register_usb_raw_class_x(uds_ctx, speed, "bt_hci_0");
	if (err) {
		LOG_ERR("Failed to register bt_hci_0 class");
		return err;
	}

	return usbd_device_set_code_triple(uds_ctx, speed, 0, 0, 0);
}

static struct usbd_context *usb_raw_init_device(usbd_msg_cb_t msg_cb)
{
	int err;
	err = usbd_add_descriptor(&usb_raw, &usb_raw_lang);
	if (err) {
		LOG_ERR("Failed to initialize %s (%d)", "language descriptor", err);
		return NULL;
	}

	err = usbd_add_descriptor(&usb_raw, &usb_raw_mfr);
	if (err) {
		LOG_ERR("Failed to initialize %s (%d)", "manufacturer descriptor", err);
		return NULL;
	}

	err = usbd_add_descriptor(&usb_raw, &usb_raw_product);
	if (err) {
		LOG_ERR("Failed to initialize %s (%d)", "product descriptor", err);
		return NULL;
	}

	IF_ENABLED(CONFIG_HWINFO, (err = usbd_add_descriptor(&usb_raw, &usb_raw_sn);))
	if (err) {
		LOG_ERR("Failed to initialize %s (%d)", "SN descriptor", err);
		return NULL;
	}

	err = register_usb_raw_x(&usb_raw, USBD_SPEED_FS);
	if (err) {
		return NULL;
	}

	if (msg_cb != NULL) {
		err = usbd_msg_register_cb(&usb_raw, msg_cb);
		if (err) {
			LOG_ERR("Failed to register message callback");
			return NULL;
		}
	}

	err = usbd_init(&usb_raw);
	if (err) {
		LOG_ERR("Failed to initialize %s (%d)", "device support", err);
		return NULL;
	}

	return &usb_raw;
}

static void msg_cb(struct usbd_context *const ctx, const struct usbd_msg *msg)
{
	if (usbd_can_detect_vbus(ctx)) {
		if (msg->type == USBD_MSG_VBUS_READY) {
			if (usbd_enable(ctx)) {
				LOG_ERR("Failed to enable device support");
			}
		}

		if (msg->type == USBD_MSG_VBUS_REMOVED) {
			if (usbd_disable(ctx)) {
				LOG_ERR("Failed to disable device support");
			}
		}
	}
}

int enable_usb_device_next(void)
{
	int err;

	if (!device_is_ready(usb_cmd_dev)) {
		LOG_ERR("Usb cmd device not ready");
		return 0;
	}

	sample_usbd = usb_raw_init_device(msg_cb);
	if (sample_usbd == NULL) {
		LOG_ERR("Failed to initialize USB device");
		return -ENODEV;
	}

	if (!usbd_can_detect_vbus(sample_usbd)) {
		err = usbd_enable(sample_usbd);
		if (err) {
			LOG_ERR("Failed to enable device support");
			return err;
		}
	}

	LOG_INF("USB device support enabled");
	return 0;
}
