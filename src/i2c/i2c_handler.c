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

#include <nrfx_twis.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include "i2c_handler.h"
#include "periphery/periphery_addresses.h"
#include "periphery/port_table.h"

LOG_MODULE_REGISTER(i2c_module, LOG_LEVEL_INF);

#define RX_BUF_SIZE 9
#define TX_BUF_SIZE 4

NET_BUF_POOL_FIXED_DEFINE(i2c_pool, 1, 8, 0, NULL);

static uint8_t rx_buf[RX_BUF_SIZE];
static uint8_t tx_buf[TX_BUF_SIZE];

static nrfx_twis_t twis_inst = NRFX_TWIS_INSTANCE(NRF_TWIS_INST_GET(1));

static int32_t pending_get_value;
static bool pending_get;

struct i2c_request {
	uint8_t port_id;
	uint8_t command;
	uint8_t len;
	uint8_t data[8];
};

K_MSGQ_DEFINE(i2c_req_queue, sizeof(struct i2c_request), 4, 1);

static void i2c_request_worker(void *p1, void *p2, void *p3)
{
	struct i2c_request req;

	(void)p1;
	(void)p2;
	(void)p3;

	while (1) {
		if (k_msgq_get(&i2c_req_queue, &req, K_FOREVER) != 0) {
			continue;
		}

		if (req.port_id >= PORT_COUNT) {
			continue;
		}

		const struct port_descriptor *port = &port_table[req.port_id];

		if (req.command == CMD_PORT_SET_VALUE && port->set_value) {
			struct net_buf *buf = net_buf_alloc(&i2c_pool, K_NO_WAIT);

			if (buf) {
				net_buf_add_mem(buf, req.data, req.len);
				(void)port->set_value(buf);
				net_buf_unref(buf);
			}
		} else if (req.command == CMD_PORT_RESET && port->reset) {
			port->reset();
		}
	}
}

K_THREAD_DEFINE(i2c_request_worker_tid, 1024, i2c_request_worker, NULL, NULL, NULL,
		K_PRIO_PREEMPT(10), 0, 0);

static void i2c_event_handler(nrfx_twis_event_t const *p_evt)
{
	switch (p_evt->type) {
	case NRFX_TWIS_EVT_WRITE_REQ:
		pending_get = false;
		if (nrfx_twis_rx_prepare(&twis_inst, rx_buf, RX_BUF_SIZE) != 0) {
			LOG_ERR("I2C RX prepare failed");
		}
		break;

	case NRFX_TWIS_EVT_WRITE_DONE: {
		size_t rx_amt = nrfx_twis_rx_amount(&twis_inst);
		if (rx_amt < 1) {
			break;
		}

		// Temporary encoding: (port_id << 4) | command
		// Allows GET in one i2c_smbus_read_block_data call.
		uint8_t port_id = rx_buf[0] >> 4;
		uint8_t command = rx_buf[0] & 0x0F;

		if (port_id >= PORT_COUNT) {
			break;
		}

		const struct port_descriptor *port = &port_table[port_id];

		switch (command) {
		case CMD_PORT_GET_VALUE:
			pending_get_value = port->get_value();
			pending_get = true;
			break;

		case CMD_PORT_SET_VALUE:
		case CMD_PORT_RESET: {
			bool is_set = (command == CMD_PORT_SET_VALUE);

			if (is_set && (!port->set_value || rx_amt < 5)) {
				break;
			}
			if (!is_set && !port->reset) {
				break;
			}

			struct i2c_request req = {
				.port_id = port_id,
				.command = command,
				.len = is_set ? (uint8_t)(rx_amt - 1) : 0,
			};

			if (is_set) {
				memcpy(req.data, rx_buf + 1, req.len);
			}

			if (k_msgq_put(&i2c_req_queue, &req, K_NO_WAIT) != 0) {
				LOG_WRN("I2C request queue full");
			}
			break;
		}
		}
		break;
	}

	case NRFX_TWIS_EVT_READ_REQ:
		if (pending_get) {
			tx_buf[0] = (uint8_t)(pending_get_value);
			tx_buf[1] = (uint8_t)(pending_get_value >> 8);
			tx_buf[2] = (uint8_t)(pending_get_value >> 16);
			tx_buf[3] = (uint8_t)(pending_get_value >> 24);
			if (nrfx_twis_tx_prepare(&twis_inst, tx_buf, 4) != 0) {
				LOG_ERR("I2C TX prepare failed");
			}
			pending_get = false;
		} else {
			if (nrfx_twis_tx_prepare(&twis_inst, NULL, 0) != 0) {
				LOG_ERR("I2C TX prepare failed");
			}
		}
		break;

	case NRFX_TWIS_EVT_READ_DONE:
	case NRFX_TWIS_EVT_READ_ERROR:
	case NRFX_TWIS_EVT_WRITE_ERROR:
		break;

	case NRFX_TWIS_EVT_GENERAL_ERROR:
		LOG_ERR("I2C general error: 0x%08x", p_evt->data.error);
		break;
	}
}

int init_i2c_transport(void)
{
	nrfx_twis_config_t cfg = NRFX_TWIS_DEFAULT_CONFIG(I2C_SCL_PIN, I2C_SDA_PIN, I2C_SLAVE_ADDR);

	int err = nrfx_twis_init(&twis_inst, &cfg, i2c_event_handler);
	if (err) {
		LOG_ERR("I2C init failed: %d", err);
		return err;
	}

	nrfx_twis_enable(&twis_inst);
	LOG_INF("I2C slave ready on addr 0x%02x", I2C_SLAVE_ADDR);

	return 0;
}