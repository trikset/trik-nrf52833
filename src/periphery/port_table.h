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

#ifndef PORT_TABLE_H
#define PORT_TABLE_H

#include <stdint.h>
#include <zephyr/net_buf.h>

/*
 * Static dispatch port table.
 *
 * Each port wraps a hardware sensor or actuator behind a uniform interface:
 *   get_value  — read the current value (required)
 *   set_value  — write a new value (NULL if read-only)
 *   reset      — reset to initial state (NULL if not supported)
 *
 * Dispatch is compile-time: no virtual tables, no runtime type checks.
 *
 * Order in the table defines serialization order in telemetry snapshots:
 *   E1..E4  — quadrature encoders (int32_t)
 *   A1..A6  — analog sensors, via SAADC (int32_t)
 *   A7..A10 — servos, via PWM channel last-set pulse width (int32_t)
 *
 * Protocol note:
 *   The wire protocol supports per-port runtime configuration (enable/disable,
 *   reordering, subscription filtering), but the current implementation uses
 *   a compile-time fixed table for simplicity.
 */

typedef int32_t (*port_get_value_fn)(void);
typedef int (*port_set_value_fn)(struct net_buf *buf);
typedef int (*port_reset_fn)(void);

struct port_descriptor {
	uint8_t number;
	port_get_value_fn get_value;
	port_set_value_fn set_value;
	port_reset_fn reset;
};

#define PORT_COUNT 14

extern const struct port_descriptor port_table[];

#endif /* PORT_TABLE_H */
