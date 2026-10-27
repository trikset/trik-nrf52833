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

#ifndef PERIPHERY_ADDRESSES_H
#define PERIPHERY_ADDRESSES_H

#include "protocol_schema.h"

#define TYPE_LOOPBACK 0x00
#define TYPE_CONTROL  0x06
#define TYPE_PORT     0x07

#define CMD_PORT_GET_VALUE 0x01
#define CMD_PORT_SET_VALUE 0x02
#define CMD_PORT_RESET     0x03

int perform_periphery_request(struct request_ctx *ctx, struct comm_header *hdr,
			      struct net_buf *buf);
int init_periphery(void);

#endif
