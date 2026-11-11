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

#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#include "port_table.h"

/*
 * Raw sample captured by the telemetry timer ISR. Serialization into the USB
 * packet happens later, in the transport thread, so the ISR only performs the
 * time-critical part: reading the ports at the exact sampling instant.
 */
struct telemetry_snapshot {
	uint64_t ts_ticks;
	int32_t values[PORT_COUNT];
};

/* Copy the latest snapshot out of the single latest-wins slot.
 * Returns false when no new sample is pending. */
bool take_periphery_snapshot(struct telemetry_snapshot *out);

#endif /* TELEMETRY_H */
