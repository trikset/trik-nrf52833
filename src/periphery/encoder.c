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

#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>

LOG_MODULE_REGISTER(encoder, LOG_LEVEL_INF);

struct encoder_ctx {
	struct gpio_dt_spec phase_a;
	struct gpio_dt_spec phase_b;
	struct gpio_callback cb_data;
	atomic_t count;
	uint8_t last_a;
};

#define ENCODER_PORT_ENTRY(node)                                                                   \
	[DT_REG_ADDR(node)] = {                                                                    \
		.phase_a = GPIO_DT_SPEC_GET(node, a_gpios),                                        \
		.phase_b = GPIO_DT_SPEC_GET(node, b_gpios),                                        \
	},

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(trik_encoder_port) > 0, "No encoder ports configured");

static struct encoder_ctx m_encoders[] = {
	DT_FOREACH_STATUS_OKAY(trik_encoder_port, ENCODER_PORT_ENTRY)};

static void encoder_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
	struct encoder_ctx *enc = CONTAINER_OF(cb, struct encoder_ctx, cb_data);

	uint8_t a = gpio_pin_get_dt(&enc->phase_a);

	if (a == enc->last_a) {
		return;
	}

	enc->last_a = a;
	uint8_t b = gpio_pin_get_dt(&enc->phase_b);

	if (a == b) {
		atomic_dec(&enc->count);
	} else {
		atomic_inc(&enc->count);
	}
}

static int setup_encoder(struct encoder_ctx *enc)
{
	int ret;

	if (!gpio_is_ready_dt(&enc->phase_a) || !gpio_is_ready_dt(&enc->phase_b)) {
		LOG_ERR("GPIO not ready");
		return -ENODEV;
	}

	ret = gpio_pin_configure_dt(&enc->phase_a, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	ret = gpio_pin_configure_dt(&enc->phase_b, GPIO_INPUT);
	if (ret) {
		return ret;
	}

	enc->last_a = gpio_pin_get_dt(&enc->phase_a);

	ret = gpio_pin_interrupt_configure_dt(&enc->phase_a, GPIO_INT_EDGE_BOTH);
	if (ret) {
		LOG_ERR("IRQ A failed (%d)", ret);
		return ret;
	}

	gpio_init_callback(&enc->cb_data, encoder_handler, BIT(enc->phase_a.pin));

	return gpio_add_callback_dt(&enc->phase_a, &enc->cb_data);
}

int init_encoders(void)
{
	for (size_t i = 0; i < ARRAY_SIZE(m_encoders); i++) {
		int err = setup_encoder(&m_encoders[i]);
		if (err) {
			LOG_ERR("Encoder %d init failed: %d", (int)i, err);
			return err;
		}
	}

	LOG_INF("Encoders initialized: %d", (int)ARRAY_SIZE(m_encoders));
	return 0;
}

int32_t encoder_get_value(uint32_t index)
{
	if (index >= ARRAY_SIZE(m_encoders)) {
		return 0;
	}

	return (int32_t)atomic_get(&m_encoders[index].count);
}

void encoder_reset(uint32_t index)
{
	if (index < ARRAY_SIZE(m_encoders)) {
		atomic_set(&m_encoders[index].count, 0);
	}
}
