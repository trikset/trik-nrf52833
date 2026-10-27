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

#ifndef PROTOCOL_SCHEMA_H
#define PROTOCOL_SCHEMA_H

#include <stdint.h>
#include <zephyr/kernel.h>
#include <zephyr/net_buf.h>

/// Vendor control request used by the host to read the running firmware version.
#define USB_RAW_REQ_GET_VERSION 0x01

/// Vendor control request used to configure telemetry (period_us + enable).
#define USB_RAW_REQ_SET_TELEMETRY 0x02

/// The USB device may be composite or logically partitioned into multiple execution pipelines.
///
/// Each pipeline is typically mapped to a separate endpoint set (or logical queue abstraction),
/// allowing independent processing of requests and responses.
///
/// When issuing a request, the host or upper layer must specify the target execution context
/// (tx_queue), so that the response can be routed back correctly. In asynchronous scenarios,
/// the device may also generate responses or follow-up requests targeting the same queue.
///
/// telemetry_tick is signaled by the timer ISR and consumed by the TX thread via k_poll.
struct request_ctx {
	struct k_fifo *tx_queue;
	struct k_sem *telemetry_tick;
};

#define HEADER_SIZE     sizeof(struct comm_header)
#define FLAG_LAST_CHUNK 1
#define REQ_ID_BATCH    0xFD

/// Protocol header used for routing and reconstructing logical messages over USB transfers.
///
/// Fields:
/// - type    : request routing type (e.g. peripheral, bluetooth, etc.)
/// - flags   : processing metadata (e.g. segmentation, end-of-stream markers)
/// - req_id  : request identifier used to match async responses
/// - seq_id  : sequence number for multi-packet logical messages
/// - len     : length of the logical payload (not USB transaction size)
struct comm_header {
	uint8_t type;
	uint8_t flags;
	uint8_t req_id;
	uint8_t seq_id;
	uint16_t len;
} __packed;

typedef enum {
	PERIPHERY = 0x01,
	BLUETOOTH = 0x02,
} stream_request_type_t;

/// Process a received USB logical message.
int perform_usb_request(struct request_ctx *ctx, struct net_buf *buf);

/// Process a command-level request (typically control or vendor-specific messages).
int perform_cmd_request(struct request_ctx *ctx, const struct net_buf *buf);

/// Create a package for further USB processing
int create_packet(const struct comm_header *in_hdr, const void *data, size_t len,
		  struct net_buf **out);

/// Enqueue a task for asynchronous processing.
/// Ownership of `buf` is transferred to the task system.
void push_packet(struct request_ctx *ctx, struct net_buf *buf);

#endif
