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

#include "port_table.h"
#include "analog_sensor.h"
#include "encoder.h"
#include "pwm.h"

#include <zephyr/devicetree.h>

/* ---- encoder ports (E1..E4) ---- */

static int32_t port_e1_get(void)
{
	return encoder_get_value(0);
}
static int32_t port_e2_get(void)
{
	return encoder_get_value(1);
}
static int32_t port_e3_get(void)
{
	return encoder_get_value(2);
}
static int32_t port_e4_get(void)
{
	return encoder_get_value(3);
}

static int port_e1_reset(void)
{
	encoder_reset(0);
	return 0;
}
static int port_e2_reset(void)
{
	encoder_reset(1);
	return 0;
}
static int port_e3_reset(void)
{
	encoder_reset(2);
	return 0;
}
static int port_e4_reset(void)
{
	encoder_reset(3);
	return 0;
}

/* ---- analog sensor ports (A1..A6) — read-only ---- */

static int32_t port_a1_get(void)
{
	return analog_sensor_get_value(DT_REG_ADDR(DT_NODELABEL(port_a1)));
}
static int32_t port_a2_get(void)
{
	return analog_sensor_get_value(DT_REG_ADDR(DT_NODELABEL(port_a2)));
}
static int32_t port_a3_get(void)
{
	return analog_sensor_get_value(DT_REG_ADDR(DT_NODELABEL(port_a3)));
}
static int32_t port_a4_get(void)
{
	return analog_sensor_get_value(DT_REG_ADDR(DT_NODELABEL(port_a4)));
}
static int32_t port_a5_get(void)
{
	return analog_sensor_get_value(DT_REG_ADDR(DT_NODELABEL(port_a5)));
}
static int32_t port_a6_get(void)
{
	return analog_sensor_get_value(DT_REG_ADDR(DT_NODELABEL(port_a6)));
}

/* ---- servo ports (A7..A10) — read + write pulse width ---- */

static int32_t port_a7_get(void)
{
	return get_pwm_channel_value(0);
}
static int32_t port_a8_get(void)
{
	return get_pwm_channel_value(1);
}
static int32_t port_a9_get(void)
{
	return get_pwm_channel_value(2);
}
static int32_t port_a10_get(void)
{
	return get_pwm_channel_value(3);
}

static int servo_set_value(uint8_t ch_idx, struct net_buf *buf)
{
	if (buf->len >= 8) {
		uint32_t period_ns = net_buf_pull_le32(buf);
		uint32_t pulse_ns = net_buf_pull_le32(buf);
		return set_pwm_channel_value(ch_idx, period_ns, pulse_ns);
	}
	uint32_t pulse_ns = net_buf_pull_le32(buf);
	return set_pwm_channel_value(ch_idx, get_module_period(ch_idx), pulse_ns);
}

static int port_a7_set(struct net_buf *buf)
{
	return servo_set_value(0, buf);
}
static int port_a8_set(struct net_buf *buf)
{
	return servo_set_value(1, buf);
}
static int port_a9_set(struct net_buf *buf)
{
	return servo_set_value(2, buf);
}
static int port_a10_set(struct net_buf *buf)
{
	return servo_set_value(3, buf);
}

/*
 * Static dispatch table.
 *
 * Each descriptor maps a logical port name (E1 .. A10) to concrete
 * hardware operations.  Order is fixed at compile time and defines the wire
 * format of telemetry snapshots.
 *
 * Intentionally NOT a flexible / per-port runtime configuration table —
 * see port_table.h for protocol extensibility notes.
 */
const struct port_descriptor port_table[PORT_COUNT] = {
	[0] = {.number = 0, .get_value = port_e1_get, .set_value = NULL, .reset = port_e1_reset},
	[1] = {.number = 1, .get_value = port_e2_get, .set_value = NULL, .reset = port_e2_reset},
	[2] = {.number = 2, .get_value = port_e3_get, .set_value = NULL, .reset = port_e3_reset},
	[3] = {.number = 3, .get_value = port_e4_get, .set_value = NULL, .reset = port_e4_reset},
	[4] = {.number = 4, .get_value = port_a1_get, .set_value = NULL, .reset = NULL},
	[5] = {.number = 5, .get_value = port_a2_get, .set_value = NULL, .reset = NULL},
	[6] = {.number = 6, .get_value = port_a3_get, .set_value = NULL, .reset = NULL},
	[7] = {.number = 7, .get_value = port_a4_get, .set_value = NULL, .reset = NULL},
	[8] = {.number = 8, .get_value = port_a5_get, .set_value = NULL, .reset = NULL},
	[9] = {.number = 9, .get_value = port_a6_get, .set_value = NULL, .reset = NULL},
	[10] = {.number = 10, .get_value = port_a7_get, .set_value = port_a7_set, .reset = NULL},
	[11] = {.number = 11, .get_value = port_a8_get, .set_value = port_a8_set, .reset = NULL},
	[12] = {.number = 12, .get_value = port_a9_get, .set_value = port_a9_set, .reset = NULL},
	[13] = {.number = 13, .get_value = port_a10_get, .set_value = port_a10_set, .reset = NULL},
};
