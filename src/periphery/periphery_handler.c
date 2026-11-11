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

#include "periphery_addresses.h"
#include "port_table.h"
#include "telemetry.h"
#include "analog_sensor.h"
#include "encoder.h"
#include "saadc.h"
#include "pwm.h"
#include <zephyr/net_buf.h>
#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <nrfx_timer.h>

LOG_MODULE_REGISTER(perf_module, LOG_LEVEL_INF);

/*
 * Telemetry sampling clock.
 *
 * Why not a Zephyr k_timer on the LFCLK RTC system clock:
 *  - 1 ms is not an integer number of 32768 Hz ticks, so a k_timer period
 *    cannot be represented exactly and drifts systematically;
 *  - the LFCLK internal RC (~1.5 %) shifts both the cadence and the per-sample
 *    timestamps embedded in every packet;
 *  - a soft "tick -> semaphore -> thread" chain adds scheduling jitter between
 *    the tick and the actual snapshot.
 *
 * Instead TIMER2 counts from HFCLK and auto-reloads in hardware
 * (SHORT: COMPARE0 -> CLEAR): the period is exact in microseconds, drift-free,
 * and needs no software re-arming. The snapshot is built inside the timer ISR,
 * so the sample time is exact and honestly timestamped at the moment of read.
 *
 * TIMER0 is owned by MPSL (BLE controller), TIMER1 is used by the nRF
 * 802.15.4 radio driver, TIMER2 is the telemetry clock, and TIMER3 is the
 * SAADC sample clock.
 */
#define TELEMETRY_TIMER_INST_IDX 2
#define TELEMETRY_TIMER_IRQ_PRIO IRQ_PRIO_LOWEST

static struct request_ctx *global_ctx;
static uint32_t control_period_us;
static atomic_t control_enabled;

static nrfx_timer_t telemetry_timer =
	NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(TELEMETRY_TIMER_INST_IDX));

/*
 * Latest produced telemetry sample, waiting to be picked up by the USB TX
 * thread. Only one slot is kept (latest-wins): if the transport is busy, old
 * samples are dropped instead of queueing up.
 *
 * The port value loop is wrapped in irq_lock() to make it consistent with the
 * SAADC ISR (which copies the analog sample buffer).  No packet buffer is
 * allocated or built here: serialization is done by the transport thread.
 */
static struct telemetry_snapshot pending_snapshot;
static bool snapshot_ready;

/*
 * Periodic entry point: samples the ports at the exact timer instant and
 * publishes the raw values into the latest-wins slot. Transport-agnostic - it
 * is driven by the TIMER2 ISR for both the BULK and the ISO path; the
 * transport thread later serializes the snapshot into a USB packet.
 * Context: interrupt (TIMER2), hence ISR-safe only.
 */
static void telemetry_produce(void)
{
	unsigned int key;

	if (!atomic_get(&control_enabled) || global_ctx == NULL) {
		return;
	}

	key = irq_lock();
	pending_snapshot.ts_ticks = k_uptime_ticks();

	for (int i = 0; i < PORT_COUNT; i++) {
		pending_snapshot.values[i] = port_table[i].get_value();
	}

	snapshot_ready = true;
	irq_unlock(key);

	k_sem_give(global_ctx->telemetry_tick);
}

static void telemetry_timer_handler(nrf_timer_event_t event, void *context)
{
	ARG_UNUSED(event);
	ARG_UNUSED(context);
	telemetry_produce();
}

static void telemetry_timer_isr(const void *arg)
{
	ARG_UNUSED(arg);
	nrfx_timer_irq_handler(&telemetry_timer);
}

static int telemetry_timer_start(uint32_t period_us)
{
	uint32_t ticks;

	if (period_us == 0) {
		return -EINVAL;
	}

	atomic_set(&control_enabled, 1);
	nrfx_timer_disable(&telemetry_timer);
	nrfx_timer_clear(&telemetry_timer);
	ticks = nrfx_timer_us_to_ticks(&telemetry_timer, period_us);
	LOG_INF("telemetry timer set: period=%u us, ticks=%u", period_us, ticks);
	nrfx_timer_extended_compare(&telemetry_timer, NRF_TIMER_CC_CHANNEL0, ticks,
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	control_period_us = period_us;
	nrfx_timer_enable(&telemetry_timer);
	return 0;
}

static void telemetry_timer_stop(void)
{
	unsigned int key;

	nrfx_timer_disable(&telemetry_timer);
	atomic_set(&control_enabled, 0);

	key = irq_lock();
	snapshot_ready = false;
	irq_unlock(key);
}

static int telemetry_timer_setup(void)
{
	nrfx_timer_config_t config =
		NRFX_TIMER_DEFAULT_CONFIG(NRF_TIMER_BASE_FREQUENCY_GET(telemetry_timer.p_reg));
	config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	config.p_context = &telemetry_timer;

	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER2), TELEMETRY_TIMER_IRQ_PRIO, telemetry_timer_isr,
		    NULL, 0);
	int err = nrfx_timer_init(&telemetry_timer, &config, telemetry_timer_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to init telemetry timer: %d", err);
		return -EIO;
	}
	irq_enable(NRFX_IRQ_NUMBER_GET(NRF_TIMER2));

	return 0;
}

bool take_periphery_snapshot(struct telemetry_snapshot *out)
{
	unsigned int key;
	bool ready;

	key = irq_lock();
	ready = snapshot_ready;
	if (ready) {
		*out = pending_snapshot;
		snapshot_ready = false;
	}
	irq_unlock(key);

	return ready;
}

int init_periphery(void)
{
	int err = analog_sensor_init();
	if (err) {
		return err;
	}

	err = init_encoders();
	if (err) {
		return err;
	}

	err = init_analog_sensors();
	if (err) {
		return err;
	}

	err = init_pwm_channels();
	if (err) {
		return err;
	}

	atomic_clear(&control_enabled);
	global_ctx = NULL;

	err = telemetry_timer_setup();
	if (err) {
		return err;
	}

	return 0;
}

int telemetry_control(struct request_ctx *ctx, uint32_t period_us, uint8_t enable)
{
	global_ctx = ctx;

	LOG_INF("telemetry control: enable=%u, period=%u us", enable, period_us);

	if (enable) {
		if (atomic_get(&control_enabled)) {
			if (period_us != control_period_us) {
				return telemetry_timer_start(period_us);
			}
		} else {
			return telemetry_timer_start(period_us);
		}
	} else {
		telemetry_timer_stop();
	}

	return 0;
}

static int perform_control_request(struct request_ctx *ctx, struct net_buf *buf)
{
	if (buf->len < 5) {
		return -EMSGSIZE;
	}

	uint32_t period_us = net_buf_pull_le32(buf);
	uint8_t enable = net_buf_pull_u8(buf);

	return telemetry_control(ctx, period_us, enable);
}

static int perform_port_request(struct request_ctx *ctx, struct comm_header *hdr,
				struct net_buf *buf)
{
	if (buf->len < 2) {
		return -EMSGSIZE;
	}

	uint8_t port_id = net_buf_pull_u8(buf);
	uint8_t command = net_buf_pull_u8(buf);

	if (port_id >= PORT_COUNT) {
		return -EINVAL;
	}

	const struct port_descriptor *port = &port_table[port_id];

	switch (command) {
	case CMD_PORT_GET_VALUE: {
		int32_t val = port->get_value();
		uint8_t payload[1 + sizeof(val)];
		payload[0] = port_id;
		memcpy(payload + 1, &val, sizeof(val));

		struct net_buf *packet;
		int err = create_packet(hdr, payload, sizeof(payload), &packet);
		if (err) {
			return err;
		}
		push_packet(ctx, packet);
		return 0;
	}
	case CMD_PORT_SET_VALUE:
		if (!port->set_value) {
			return -ENOTSUP;
		}
		if (buf->len < 4) {
			return -EMSGSIZE;
		}
		return port->set_value(buf);

	case CMD_PORT_RESET:
		if (!port->reset) {
			return -ENOTSUP;
		}
		return port->reset();

	default:
		return -ENOTSUP;
	}
}

static int perform_loopback_request(struct request_ctx *ctx, struct comm_header *hdr,
				    struct net_buf *buf)
{
	struct net_buf *packet;
	int err = create_packet(hdr, buf->data, buf->len, &packet);
	if (err) {
		return err;
	}
	push_packet(ctx, packet);
	return 0;
}

int perform_periphery_request(struct request_ctx *ctx, struct comm_header *hdr, struct net_buf *buf)
{
	if (buf->len < 1) {
		return -EMSGSIZE;
	}
	uint8_t device = net_buf_pull_u8(buf);

	switch (device) {
	case TYPE_PORT:
		if (hdr->req_id == REQ_ID_BATCH) {
			int err;
			while (buf->len > 0) {
				err = perform_port_request(ctx, hdr, buf);
				if (err) {
					return err;
				}
			}
			return 0;
		}
		return perform_port_request(ctx, hdr, buf);
	case TYPE_LOOPBACK:
		return perform_loopback_request(ctx, hdr, buf);
	case TYPE_CONTROL:
		return perform_control_request(ctx, buf);
	default:
		return -ENODEV;
	}
}
