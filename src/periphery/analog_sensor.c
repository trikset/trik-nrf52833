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

#include "analog_sensor.h"
#include "port_table.h"
#include "saadc.h"

#include <errno.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>

struct analog_sensor {
	bool present;
	struct gpio_dt_spec enable;
	struct gpio_dt_spec out;
	uint8_t ain;
	uint8_t port_id;
};

#define ANALOG_SENSOR_PORT(node) DT_PHANDLE(node, port)

#define ANALOG_SENSOR_CHECK(node)                                                                  \
	BUILD_ASSERT(DT_NODE_HAS_COMPAT(ANALOG_SENSOR_PORT(node), trik_analog_port),               \
		     "analog-sensor port must be trik,analog-port");

DT_FOREACH_STATUS_OKAY(trik_analog_sensor, ANALOG_SENSOR_CHECK)

#define ANALOG_SENSOR_ENTRY(node)                                                                  \
	[DT_REG_ADDR(ANALOG_SENSOR_PORT(node))] = {                                                \
		.present = true,                                                                   \
		.enable = GPIO_DT_SPEC_GET_OR(ANALOG_SENSOR_PORT(node), enable, {0}),              \
		.out = GPIO_DT_SPEC_GET_OR(ANALOG_SENSOR_PORT(node), out, {0}),                    \
		.ain = DT_IO_CHANNELS_INPUT_BY_IDX(ANALOG_SENSOR_PORT(node), 0),                   \
		.port_id = DT_REG_ADDR(ANALOG_SENSOR_PORT(node)),                                  \
	},

static const struct analog_sensor analog_sensors[PORT_COUNT] = {
	DT_FOREACH_STATUS_OKAY(trik_analog_sensor, ANALOG_SENSOR_ENTRY)};

int analog_sensor_init(void)
{
	for (uint8_t i = 0; i < PORT_COUNT; i++) {
		const struct analog_sensor *sensor = &analog_sensors[i];

		if (!sensor->present) {
			continue;
		}

		if (sensor->out.port != NULL) {
			if (!gpio_is_ready_dt(&sensor->out)) {
				return -ENODEV;
			}

			int err = gpio_pin_configure_dt(&sensor->out, GPIO_DISCONNECTED);

			if (err != 0) {
				return err;
			}
		}

		if (sensor->enable.port != NULL) {
			if (!gpio_is_ready_dt(&sensor->enable)) {
				return -ENODEV;
			}

			int err = gpio_pin_configure_dt(&sensor->enable, GPIO_OUTPUT_INACTIVE);

			if (err != 0) {
				return err;
			}

			err = gpio_pin_set_dt(&sensor->enable, 1);
			if (err != 0) {
				return err;
			}
		}
	}

	return 0;
}

int32_t analog_sensor_get_value(uint8_t port_id)
{
	if (port_id >= PORT_COUNT) {
		return 0;
	}

	const struct analog_sensor *sensor = &analog_sensors[port_id];

	if (!sensor->present) {
		return 0;
	}

	return saadc_get_value_by_ain(sensor->ain);
}
