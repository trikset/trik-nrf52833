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

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "usb/usb_device_api.h"
#include "periphery_addresses.h"
#include "i2c_handler.h"

LOG_MODULE_REGISTER(usb_raw_echo, LOG_LEVEL_INF);

int main(void)
{
	int ret = 0;
	ret = enable_usb_device_next();
	if (ret != 0) {
		LOG_ERR("Failed to enable USB device support");
		return 0;
	}

	ret = init_periphery();

	if (ret != 0) {
		LOG_ERR("Failed to enable periphery");
		return 0;
	}

	ret = init_i2c_transport();
	if (ret != 0) {
		LOG_ERR("Failed to enable I2C transport");
		return 0;
	}

	while (1) {
		k_sleep(K_FOREVER);
	}

	return 0;
}
