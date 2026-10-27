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

#ifndef PWM_CONTROL_H
#define PWM_CONTROL_H

#include <zephyr/types.h>

int init_pwm_channels(void);
int set_pwm_channel_value(uint8_t ch_idx, uint32_t period_ns, uint32_t pulse_ns);
int32_t get_pwm_channel_value(uint8_t ch_idx);
uint32_t get_module_period(uint8_t ch_idx);

#endif
