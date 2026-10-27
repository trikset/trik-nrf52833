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

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(pwm, LOG_LEVEL_INF);

struct pwm_chan_config {
	struct pwm_dt_spec spec;
	struct gpio_dt_spec enable;
	struct gpio_dt_spec in;
	uint8_t module_idx;
};

static const struct pwm_chan_config pwm_channels[] = {
	{.spec = PWM_DT_SPEC_GET(DT_NODELABEL(port_a7)),
	 .enable = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a7), enable, {0}),
	 .in = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a7), in, {0}),
	 .module_idx = 0},
	{.spec = PWM_DT_SPEC_GET(DT_NODELABEL(port_a8)),
	 .enable = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a8), enable, {0}),
	 .in = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a8), in, {0}),
	 .module_idx = 1},
	{.spec = PWM_DT_SPEC_GET(DT_NODELABEL(port_a9)),
	 .enable = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a9), enable, {0}),
	 .in = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a9), in, {0}),
	 .module_idx = 2},
	{.spec = PWM_DT_SPEC_GET(DT_NODELABEL(port_a10)),
	 .enable = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a10), enable, {0}),
	 .in = GPIO_DT_SPEC_GET_OR(DT_NODELABEL(port_a10), in, {0}),
	 .module_idx = 3},
};

static uint32_t module_periods[4] = {PWM_MSEC(20), PWM_MSEC(20), PWM_MSEC(20), PWM_MSEC(20)};
static uint32_t last_pulses[4];

int set_pwm_channel_value(uint8_t ch_idx, uint32_t period_ns, uint32_t pulse_ns)
{
	if (ch_idx >= ARRAY_SIZE(pwm_channels)) {
		return -EINVAL;
	}

	const struct pwm_chan_config *chan = &pwm_channels[ch_idx];

	if (period_ns == 0) {
		last_pulses[ch_idx] = 0;
		return pwm_set_dt(&chan->spec, module_periods[chan->module_idx], 0);
	}

	if (pulse_ns > period_ns) {
		pulse_ns = period_ns;
	}

	module_periods[chan->module_idx] = period_ns;
	last_pulses[ch_idx] = pulse_ns;

	return pwm_set_dt(&chan->spec, period_ns, pulse_ns);
}

uint32_t get_module_period(uint8_t ch_idx)
{
	if (ch_idx >= ARRAY_SIZE(pwm_channels)) {
		return 0;
	}
	return module_periods[pwm_channels[ch_idx].module_idx];
}

int32_t get_pwm_channel_value(uint8_t ch_idx)
{
	if (ch_idx >= ARRAY_SIZE(pwm_channels)) {
		return 0;
	}
	return (int32_t)last_pulses[ch_idx];
}

int init_pwm_channels(void)
{
	for (int i = 0; i < ARRAY_SIZE(pwm_channels); i++) {
		const struct pwm_chan_config *chan = &pwm_channels[i];

		if (chan->in.port != NULL) {
			if (!gpio_is_ready_dt(&chan->in)) {
				return -ENODEV;
			}

			int err = gpio_pin_configure_dt(&chan->in, GPIO_DISCONNECTED);

			if (err != 0) {
				return err;
			}
		}

		if (chan->enable.port != NULL) {
			if (!gpio_is_ready_dt(&chan->enable)) {
				return -ENODEV;
			}

			int err = gpio_pin_configure_dt(&chan->enable, GPIO_OUTPUT_INACTIVE);

			if (err != 0) {
				return err;
			}

			err = gpio_pin_set_dt(&chan->enable, 1);
			if (err != 0) {
				return err;
			}
		}

		if (!device_is_ready(chan->spec.dev)) {
			return -ENODEV;
		}

		module_periods[chan->module_idx] = chan->spec.period;

		pwm_set_dt(&chan->spec, chan->spec.period, 0);
	}
	return 0;
}
