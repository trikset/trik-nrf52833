/*
 * Copyright (c) 2022 - 2025, Nordic Semiconductor ASA
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Modifications:
 * Copyright (c) CyberTech Labs Ltd.
 */

#include <nrfx_saadc.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_timer.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(saadc, LOG_LEVEL_INF);

#define SAADC_CHANNEL_CONFIG(_pin, _gain, _acq, _index)                                            \
	{                                                                                          \
		.channel_config =                                                                  \
			{                                                                          \
				.resistor_p = NRF_SAADC_RESISTOR_PULLUP,                           \
				.resistor_n = NRF_SAADC_RESISTOR_DISABLED,                         \
				.gain = _gain,                                                     \
				.reference = NRF_SAADC_REFERENCE_INTERNAL,                         \
				.acq_time = _acq,                                                  \
				.mode = NRF_SAADC_MODE_SINGLE_ENDED,                               \
				.burst = NRF_SAADC_BURST_DISABLED,                                 \
			},                                                                         \
		.pin_p = _pin, .pin_n = NRFX_ANALOG_INPUT_DISABLED, .channel_index = _index,       \
	}

#define DT_DRV_COMPAT trik_analog_port

#define SAADC_TIMER_INST_IDX 3
#define SAADC_FREQUENCY      1000UL
#define TIME_TO_WAIT_US      (uint32_t)(1000000UL / SAADC_FREQUENCY)
#define BUFFER_COUNT         2UL
#define SAADC_CH_COUNT       DT_NUM_INST_STATUS_OKAY(trik_analog_port)
#define BUFFER_SIZE          SAADC_CH_COUNT
#define RESOLUTION           NRF_SAADC_RESOLUTION_12BIT

BUILD_ASSERT(SAADC_CH_COUNT > 0, "No analog ports configured");
BUILD_ASSERT(DT_IO_CHANNELS_INPUT_BY_IDX(DT_NODELABEL(port_a1), 0) == 3, "A1 must use AIN3");

#define SAADC_CHANNEL_DEFINE(inst)                                                                 \
	SAADC_CHANNEL_CONFIG((nrfx_analog_input_t)DT_INST_IO_CHANNELS_INPUT(inst),                 \
			     NRF_SAADC_GAIN1_6, NRF_SAADC_ACQTIME_20US, inst),

static const nrfx_saadc_channel_t m_channels[] = {
	DT_INST_FOREACH_STATUS_OKAY(SAADC_CHANNEL_DEFINE)};

BUILD_ASSERT(ARRAY_SIZE(m_channels) == SAADC_CH_COUNT, "SAADC channel count mismatch");

static nrf_saadc_value_t m_stable_sensor_data[SAADC_CH_COUNT];

static bool m_data_initialized = false;

/** @brief Samples buffer to store values from a SAADC channel. */
static nrf_saadc_value_t m_sample_buffers[BUFFER_COUNT][BUFFER_SIZE];

/** @brief Array of the GPPI channels. */
static nrfx_gppi_handle_t m_gppi_handles[2];

typedef enum {
	SAADC_SAMPLING,     ///< Triggers SAADC sampling task on external timer event.
	SAADC_START_ON_END, ///< Triggers SAADC start task on SAADC end event.
} gppi_handle_purpose_t;

static void saadc_handler(nrfx_saadc_evt_t const *p_event)
{
	int status;
	(void)status;

	static uint16_t buffer_index = 1;
	uint16_t samples_number;

	switch (p_event->type) {
	case NRFX_SAADC_EVT_CALIBRATEDONE:
		status = nrfx_saadc_mode_trigger();
		NRFX_ASSERT(status == 0);
		break;

	case NRFX_SAADC_EVT_READY:
		nrfx_gppi_conn_enable(m_gppi_handles[SAADC_SAMPLING]);
		break;

	case NRFX_SAADC_EVT_BUF_REQ:
		status = nrfx_saadc_buffer_set(m_sample_buffers[buffer_index++], BUFFER_SIZE);
		NRFX_ASSERT(status == 0);
		buffer_index = buffer_index % BUFFER_COUNT;
		break;

	case NRFX_SAADC_EVT_DONE:
		samples_number = p_event->data.done.size;
		memcpy(m_stable_sensor_data, p_event->data.done.p_buffer,
		       sizeof(m_stable_sensor_data));
		m_data_initialized = true;
		break;

	case NRFX_SAADC_EVT_FINISHED:
		nrfx_gppi_conn_disable(m_gppi_handles[SAADC_SAMPLING]);
		break;

	default:
		break;
	}
}

int init_analog_sensors(void)
{
	int status;
	(void)status;
#if defined(__ZEPHYR__)
	IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SAADC), IRQ_PRIO_LOWEST, nrfx_saadc_irq_handler, 0, 0);
#endif
	status = nrfx_saadc_init(NRFX_SAADC_DEFAULT_CONFIG_IRQ_PRIORITY);
	NRFX_ASSERT(status == 0);

	nrfx_timer_t timer_inst = NRFX_TIMER_INSTANCE(NRF_TIMER_INST_GET(SAADC_TIMER_INST_IDX));
	uint32_t base_frequency = NRF_TIMER_BASE_FREQUENCY_GET(timer_inst.p_reg);
	nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(base_frequency);
	timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;
	timer_config.p_context = &timer_inst;

	status = nrfx_timer_init(&timer_inst, &timer_config, NULL);
	NRFX_ASSERT(status == 0);

	nrfx_timer_clear(&timer_inst);

	/* Creating variable desired_ticks to store the output of nrfx_timer_us_to_ticks function.
	 */
	uint32_t desired_ticks = nrfx_timer_us_to_ticks(&timer_inst, TIME_TO_WAIT_US);

	/*
	 * Setting the timer channel NRF_TIMER_CC_CHANNEL0 in the extended compare mode to clear
	 * the timer and to not trigger an interrupt if the internal counter register is equal to
	 * desired_ticks.
	 */
	nrfx_timer_extended_compare(&timer_inst, NRF_TIMER_CC_CHANNEL0, desired_ticks,
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);

	status = nrfx_saadc_channels_config(m_channels, SAADC_CH_COUNT);
	NRFX_ASSERT(status == 0);

	/*
	 * Setting the advanced configuration with triggering sampling by the internal timer
	 * disabled (internal_timer_cc = 0) and without software start task on end event
	 * (start_on_end = false).
	 */
	nrfx_saadc_adv_config_t adv_config = NRFX_SAADC_DEFAULT_ADV_CONFIG;
	adv_config.internal_timer_cc = 0;
	adv_config.start_on_end = false;

	uint32_t channel_mask = nrfx_saadc_channels_configured_get();
	status = nrfx_saadc_advanced_mode_set(channel_mask, RESOLUTION, &adv_config, saadc_handler);
	NRFX_ASSERT(status == 0);

	status = nrfx_saadc_buffer_set(m_sample_buffers[0], BUFFER_SIZE);
	NRFX_ASSERT(status == 0);

	/*
	 * Allocate a dedicated channel and configure endpoints of that channel so that the timer
	 * compare event is connected with the SAADC sample task. This means that each time the
	 * timer interrupt occurs, the SAADC sampling will be triggered.
	 */
	status = nrfx_gppi_conn_alloc(
		nrfx_timer_compare_event_address_get(&timer_inst, NRF_TIMER_CC_CHANNEL0),
		nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_SAMPLE),
		&m_gppi_handles[SAADC_SAMPLING]);
	NRFX_ASSERT(status == 0);

	/*
	 * Allocate a dedicated channel and configure endpoints of that so that the SAADC event end
	 * is connected with the SAADC task start. This means that each time the SAADC fills up the
	 * result buffer, the SAADC will be restarted and the result buffer will be prepared in RAM.
	 */
	status = nrfx_gppi_conn_alloc(nrf_saadc_event_address_get(NRF_SAADC, NRF_SAADC_EVENT_END),
				      nrf_saadc_task_address_get(NRF_SAADC, NRF_SAADC_TASK_START),
				      &m_gppi_handles[SAADC_START_ON_END]);
	NRFX_ASSERT(status == 0);

	nrfx_timer_enable(&timer_inst);

	nrfx_gppi_conn_enable(m_gppi_handles[SAADC_START_ON_END]);

	status = nrfx_saadc_offset_calibrate(saadc_handler);
	NRFX_ASSERT(status == 0);
	return status;
}

int32_t saadc_get_value_by_ain(uint8_t ain)
{
	if (!m_data_initialized || ain > NRFX_ANALOG_EXTERNAL_AIN7) {
		return 0;
	}

	nrfx_analog_input_t input = (nrfx_analog_input_t)ain;

	for (uint8_t i = 0; i < SAADC_CH_COUNT; i++) {
		if (m_channels[i].pin_p == input) {
			return (int32_t)m_stable_sensor_data[i];
		}
	}

	return 0;
}

nrf_saadc_value_t get_last_sensor_value(uint8_t channel_idx)
{
	if (m_data_initialized && channel_idx < SAADC_CH_COUNT) {
		return m_stable_sensor_data[channel_idx];
	}
	return 0;
}
