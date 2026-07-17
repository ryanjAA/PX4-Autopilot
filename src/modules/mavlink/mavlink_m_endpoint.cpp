/****************************************************************************
 *
 * Copyright (c) 2026 Ryan Johnston. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "mavlink_main.h"
#include "mavlink_m_endpoint.h"
#include "mavlink_m_profile.h"

#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <parameters/param.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/time.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <pthread.h>

#if defined(MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY) && defined(MAVLINK_MSG_ID_TARGET_CUE)

#include <dataman/dataman.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/aags_mavlink_m_status.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_global_position.h>

namespace
{

using namespace time_literals;

constexpr uint32_t kRecordMagic = 0x4d564d32U; // "MVM2"
constexpr uint32_t kFrameMagic = 0x4d564652U; // "MVFR"
constexpr uint16_t kStorageVersion = 2;
constexpr unsigned kRecordCount = 4;
constexpr unsigned kFragmentsPerRecord = 4;
constexpr unsigned kFrozenFrameBytes = 160;
constexpr uint64_t kCapabilityLifetimeUsec = 15ULL * 1000ULL * 1000ULL;
constexpr uint64_t kCapabilityPeriodUsec = 5ULL * 1000ULL * 1000ULL;
constexpr uint64_t kSourceFutureToleranceUsec = 30ULL * 1000ULL * 1000ULL;
constexpr uint64_t kSourceMaximumAgeUsec = 5ULL * 60ULL * 1000ULL * 1000ULL;
constexpr uint64_t kLinkFreshUsec = 2ULL * 1000ULL * 1000ULL;
constexpr uint8_t kTotalReplyAttempts = 5;
constexpr uint64_t kInitialRetryUsec = 750ULL * 1000ULL;
constexpr uint64_t kMinimumUtcUsec = 1577836800ULL * 1000ULL * 1000ULL; // 2020-01-01
constexpr unsigned kApprovedSourceCount = 8;

enum class RecordState : uint8_t {
	Empty = 0,
	Pending = 1,
	Active = 2,
	EnRoute = 3,
	Ready = 4,
	Complete = 5,
	Rejected = 6,
	Aborted = 7,
	Failed = 8,
	Expired = 9,
};

enum class FrameKind : uint8_t {
	Receipt = 0,
	Decision = 1,
	Status = 2,
};

struct PersistedRecord {
	uint32_t magic{kRecordMagic};
	uint32_t checksum{0};
	uint64_t received_utc_usec{0};
	mavlink_target_cue_t cue{};
	uint32_t status_sequence{0};
	uint16_t version{kStorageVersion};
	uint8_t state{static_cast<uint8_t>(RecordState::Empty)};
	uint8_t source_system{0};
	uint8_t source_component{0};
	uint8_t ingress_channel{0};
	uint8_t decision_retry_remaining{0};
	uint8_t status_retry_remaining{0};
	uint8_t reserved[2]{};
};

struct PersistedFrame {
	uint32_t magic{kFrameMagic};
	uint32_t checksum{0};
	uint32_t cue_id{0};
	uint16_t version{kStorageVersion};
	uint8_t kind{0};
	uint8_t length{0};
	uint8_t bytes[kFrozenFrameBytes]{};
};

struct RuntimeRecord {
	PersistedRecord persisted{};
	PersistedFrame receipt{};
	PersistedFrame decision{};
	PersistedFrame status{};
	hrt_abstime next_decision_retry{0};
	hrt_abstime next_status_retry{0};
};

struct ApprovedSource {
	uint64_t expires_utc_usec{0};
	uint8_t system{0};
	uint8_t component{0};
	uint8_t endpoint_type{0};
};

static_assert(sizeof(PersistedRecord) <= DM_KEY_MAVLINK_M_INBOX_SIZE,
	      "MAVLink-M record exceeds one dataman item");
static_assert(sizeof(PersistedFrame) <= DM_KEY_MAVLINK_M_INBOX_SIZE,
	      "MAVLink-M frozen frame exceeds one dataman item");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
	      + MAVLINK_SIGNATURE_BLOCK_LEN <= kFrozenFrameBytes,
	      "ACK frame exceeds frozen storage");
static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES
	      + MAVLINK_SIGNATURE_BLOCK_LEN <= kFrozenFrameBytes,
	      "task-status frame exceeds frozen storage");

pthread_mutex_t g_mutex = PTHREAD_MUTEX_INITIALIZER;
RuntimeRecord g_records[kRecordCount]{};
ApprovedSource g_approved_sources[kApprovedSourceCount]{};
hrt_abstime g_last_capability[MAVLINK_COMM_NUM_BUFFERS]{};
hrt_abstime g_link_seen[MAVLINK_COMM_NUM_BUFFERS]{};
bool g_initialized{false};
bool g_storage_ready{false};
bool g_center_latched{false};
bool g_signing_warning_printed{false};
hrt_abstime g_last_initialization_attempt{0};
hrt_abstime g_last_status_publish{0};

uORB::Subscription g_rc_channels_sub{ORB_ID(rc_channels)};
uORB::Subscription g_global_position_sub{ORB_ID(vehicle_global_position)};
uORB::Publication<aags_mavlink_m_status_s> g_status_pub{ORB_ID(aags_mavlink_m_status)};

param_t g_enable_param{PARAM_INVALID};
param_t g_signing_required_param{PARAM_INVALID};
param_t g_rc_channel_param{PARAM_INVALID};
param_t g_rc_reject_param{PARAM_INVALID};
param_t g_rc_accept_param{PARAM_INVALID};
param_t g_rc_center_param{PARAM_INVALID};

class MutexGuard
{
public:
	MutexGuard() { pthread_mutex_lock(&g_mutex); }
	~MutexGuard() { pthread_mutex_unlock(&g_mutex); }
};

uint32_t fnv1a(const void *data, size_t length)
{
	const auto *bytes = static_cast<const uint8_t *>(data);
	uint32_t hash = 2166136261U;

	for (size_t i = 0; i < length; ++i) {
		hash ^= bytes[i];
		hash *= 16777619U;
	}

	return hash;
}

uint32_t record_checksum(PersistedRecord record)
{
	record.checksum = 0;
	return fnv1a(&record, sizeof(record));
}

uint32_t frame_checksum(PersistedFrame frame)
{
	frame.checksum = 0;
	return fnv1a(&frame, sizeof(frame));
}

bool valid_record(const PersistedRecord &record)
{
	return record.magic == kRecordMagic
	       && record.version == kStorageVersion
	       && record.checksum == record_checksum(record)
	       && record.state <= static_cast<uint8_t>(RecordState::Expired);
}

bool valid_frame(const PersistedFrame &frame, uint32_t cue_id, FrameKind kind)
{
	return frame.magic == kFrameMagic
	       && frame.version == kStorageVersion
	       && frame.cue_id == cue_id
	       && frame.kind == static_cast<uint8_t>(kind)
	       && frame.length > 0
	       && frame.length <= kFrozenFrameBytes
	       && frame.checksum == frame_checksum(frame);
}

unsigned dataman_index(unsigned slot, unsigned fragment)
{
	return slot * kFragmentsPerRecord + fragment;
}

bool write_record(unsigned slot, PersistedRecord *record)
{
	record->checksum = record_checksum(*record);
	return dm_write(DM_KEY_MAVLINK_M_INBOX, dataman_index(slot, 0),
			record, sizeof(*record)) == static_cast<ssize_t>(sizeof(*record));
}

bool write_frame(unsigned slot, unsigned fragment, PersistedFrame *frame)
{
	frame->checksum = frame_checksum(*frame);
	return dm_write(DM_KEY_MAVLINK_M_INBOX, dataman_index(slot, fragment),
			frame, sizeof(*frame)) == static_cast<ssize_t>(sizeof(*frame));
}

bool parameter_bool(param_t &handle, const char *name, bool default_value)
{
	if (handle == PARAM_INVALID) {
		handle = param_find(name);
	}

	int32_t value = default_value ? 1 : 0;

	if (handle != PARAM_INVALID) {
		param_get(handle, &value);
	}

	return value != 0;
}

int32_t parameter_int(param_t &handle, const char *name, int32_t default_value)
{
	if (handle == PARAM_INVALID) {
		handle = param_find(name);
	}

	int32_t value = default_value;

	if (handle != PARAM_INVALID) {
		param_get(handle, &value);
	}

	return value;
}

float parameter_float(param_t &handle, const char *name, float default_value)
{
	if (handle == PARAM_INVALID) {
		handle = param_find(name);
	}

	float value = default_value;

	if (handle != PARAM_INVALID) {
		param_get(handle, &value);
	}

	return value;
}

bool endpoint_enabled()
{
	return parameter_bool(g_enable_param, "MVM_ENABLE", true);
}

bool signing_required()
{
	return parameter_bool(g_signing_required_param, "MVM_SIGN_REQ", false);
}

uint64_t utc_now_usec()
{
	struct timespec now {};
	px4_clock_gettime(CLOCK_REALTIME, &now);

	if (now.tv_sec <= 0) {
		return 0;
	}

	const uint64_t value = static_cast<uint64_t>(now.tv_sec) * 1000000ULL
			       + static_cast<uint64_t>(now.tv_nsec / 1000);
	return value >= kMinimumUtcUsec ? value : 0;
}

RecordState state_of(const RuntimeRecord &record)
{
	return static_cast<RecordState>(record.persisted.state);
}

bool terminal_state(RecordState state)
{
	return state == RecordState::Complete || state == RecordState::Rejected
	       || state == RecordState::Aborted || state == RecordState::Failed
	       || state == RecordState::Expired;
}

bool active_state(RecordState state)
{
	return state == RecordState::Active || state == RecordState::EnRoute
	       || state == RecordState::Ready;
}

uint8_t wire_state(RecordState state)
{
	switch (state) {
	case RecordState::Active: return MAVLINK_M_TASK_STATE_ACTIVE;
	case RecordState::EnRoute: return MAVLINK_M_TASK_STATE_EN_ROUTE;
	case RecordState::Ready: return MAVLINK_M_TASK_STATE_READY;
	case RecordState::Complete: return MAVLINK_M_TASK_STATE_COMPLETE;
	case RecordState::Aborted: return MAVLINK_M_TASK_STATE_ABORTED;
	case RecordState::Failed: return MAVLINK_M_TASK_STATE_FAILED;
	case RecordState::Expired: return MAVLINK_M_TASK_STATE_EXPIRED;
	default: return MAVLINK_M_TASK_STATE_UNKNOWN;
	}
}

const char *state_name(RecordState state)
{
	switch (state) {
	case RecordState::Empty: return "empty";
	case RecordState::Pending: return "pending";
	case RecordState::Active: return "active";
	case RecordState::EnRoute: return "en-route";
	case RecordState::Ready: return "ready";
	case RecordState::Complete: return "complete";
	case RecordState::Rejected: return "rejected";
	case RecordState::Aborted: return "aborted";
	case RecordState::Failed: return "failed";
	case RecordState::Expired: return "expired";
	}

	return "invalid";
}

void initialize_locked()
{
	const hrt_abstime now = hrt_absolute_time();

	if (g_initialized || (g_last_initialization_attempt != 0
			      && now - g_last_initialization_attempt < 1_s)) {
		return;
	}

	g_last_initialization_attempt = now;
	bool backend_responded = false;

	for (unsigned slot = 0; slot < kRecordCount; ++slot) {
		RuntimeRecord restored{};
		const ssize_t record_length = dm_read(DM_KEY_MAVLINK_M_INBOX,
						     dataman_index(slot, 0),
						     &restored.persisted,
						     sizeof(restored.persisted));

		if (record_length >= 0) {
			backend_responded = true;
		}

		if (record_length != static_cast<ssize_t>(sizeof(restored.persisted))
		    || !valid_record(restored.persisted)
		    || state_of(restored) == RecordState::Empty) {
			continue;
		}

		const uint32_t cue_id = restored.persisted.cue.cue_id;

		if (dm_read(DM_KEY_MAVLINK_M_INBOX, dataman_index(slot, 1),
			    &restored.receipt, sizeof(restored.receipt))
		    != static_cast<ssize_t>(sizeof(restored.receipt))
		    || !valid_frame(restored.receipt, cue_id, FrameKind::Receipt)) {
			continue;
		}

		if (restored.persisted.decision_retry_remaining > 0) {
			if (dm_read(DM_KEY_MAVLINK_M_INBOX, dataman_index(slot, 2),
				    &restored.decision, sizeof(restored.decision))
			    != static_cast<ssize_t>(sizeof(restored.decision))
			    || !valid_frame(restored.decision, cue_id, FrameKind::Decision)) {
				restored.persisted.decision_retry_remaining = 0;
			}
		}

		if (restored.persisted.status_retry_remaining > 0) {
			if (dm_read(DM_KEY_MAVLINK_M_INBOX, dataman_index(slot, 3),
				    &restored.status, sizeof(restored.status))
			    != static_cast<ssize_t>(sizeof(restored.status))
			    || !valid_frame(restored.status, cue_id, FrameKind::Status)) {
				restored.persisted.status_retry_remaining = 0;
			}
		}

		g_records[slot] = restored;
	}

	if (backend_responded) {
		g_storage_ready = true;
		g_initialized = true;
		PX4_INFO("MAVLink-M durable inbox ready");
	}
}

bool same_profile_string(const char *field, size_t field_size, const char *expected)
{
	const size_t expected_length = strlen(expected);
	return expected_length < field_size
	       && memcmp(field, expected, expected_length) == 0
	       && field[expected_length] == '\0';
}

void approve_source_locked(const mavlink_message_t *message,
			   const mavlink_mavlink_m_capability_t &capability)
{
	const uint64_t now = utc_now_usec();

	if (now == 0 || message->sysid == 0 || message->compid == 0
	    || capability.endpoint_system != message->sysid
	    || capability.endpoint_component != message->compid
	    || capability.protocol_major != AAGS_MAVLINK_M_PROTOCOL_MAJOR
	    || capability.protocol_minor != AAGS_MAVLINK_M_PROTOCOL_MINOR
	    || memcmp(capability.profile_sha256, AAGS_MAVLINK_M_PROFILE_SHA256_BYTES,
		      sizeof(AAGS_MAVLINK_M_PROFILE_SHA256_BYTES)) != 0
	    || !same_profile_string(capability.profile_id, sizeof(capability.profile_id),
				    AAGS_MAVLINK_M_PROFILE_ID)
	    || !same_profile_string(capability.profile_version, sizeof(capability.profile_version),
				    AAGS_MAVLINK_M_PROFILE_VERSION)
	    || capability.valid_until_usec <= now
	    || capability.valid_until_usec > now + 30ULL * 1000ULL * 1000ULL
	    || capability.time_usec > now + kSourceFutureToleranceUsec
	    || now > capability.time_usec + kSourceMaximumAgeUsec
	    || (capability.associated_system != 0
		&& capability.associated_system != mavlink_system.sysid)
	    || (capability.capability_flags & MAVLINK_M_CAPABILITY_SIGNING_REQUIRED) != 0) {
		return;
	}

	unsigned selected = kApprovedSourceCount;
	unsigned oldest = 0;

	for (unsigned i = 0; i < kApprovedSourceCount; ++i) {
		if (g_approved_sources[i].system == message->sysid
		    && g_approved_sources[i].component == message->compid) {
			selected = i;
			break;
		}

		if (g_approved_sources[i].expires_utc_usec <= now) {
			selected = i;
		}

		if (g_approved_sources[i].expires_utc_usec
		    < g_approved_sources[oldest].expires_utc_usec) {
			oldest = i;
		}
	}

	if (selected == kApprovedSourceCount) {
		selected = oldest;
	}

	g_approved_sources[selected].system = message->sysid;
	g_approved_sources[selected].component = message->compid;
	g_approved_sources[selected].endpoint_type = capability.endpoint_type;
	g_approved_sources[selected].expires_utc_usec = capability.valid_until_usec;
}

bool source_approved_locked(uint8_t system, uint8_t component)
{
	const uint64_t now = utc_now_usec();

	if (now == 0) {
		return false;
	}

	for (const ApprovedSource &source : g_approved_sources) {
		if (source.system == system && source.component == component
		    && source.expires_utc_usec > now) {
			return true;
		}
	}

	return false;
}

bool valid_cue(const mavlink_message_t *message, const mavlink_target_cue_t &cue,
	       uint64_t now)
{
	const bool confidence_valid = std::isnan(cue.confidence_score)
				      || (cue.confidence_score >= 0.f
					  && cue.confidence_score <= 1.f);
	return message->len >= MAVLINK_MSG_ID_TARGET_CUE_LEN
	       && message->sysid != 0 && message->compid != 0
	       && cue.target_system == mavlink_system.sysid
	       && cue.target_component == mavlink_system.compid
	       && cue.assigned_system == mavlink_system.sysid
	       && cue.cue_id != 0 && cue.origin_sysid != 0
	       && cue.time_usec != 0 && cue.valid_until_usec > cue.time_usec
	       && cue.valid_until_usec > now
	       && cue.time_usec <= now + kSourceFutureToleranceUsec
	       && now <= cue.time_usec + kSourceMaximumAgeUsec
	       && cue.lat >= -900000000 && cue.lat <= 900000000
	       && cue.lon >= -1800000000 && cue.lon <= 1800000000
	       && cue.cue_type >= MAVLINK_M_CUE_TYPE_INVESTIGATE
	       && cue.cue_type <= MAVLINK_M_CUE_TYPE_MARK
	       && confidence_valid;
}

PersistedFrame freeze_message(const mavlink_message_t &message, uint32_t cue_id,
			      FrameKind kind)
{
	PersistedFrame frame{};
	frame.cue_id = cue_id;
	frame.kind = static_cast<uint8_t>(kind);
	const uint16_t length = mavlink_msg_to_send_buffer(frame.bytes, &message);

	if (length <= kFrozenFrameBytes) {
		frame.length = static_cast<uint8_t>(length);
		frame.checksum = frame_checksum(frame);
	}

	return frame;
}

PersistedFrame build_ack_frame(uint8_t channel, const PersistedRecord &record,
			       uint8_t result, const char *reason, uint64_t now)
{
	mavlink_message_t response{};
	mavlink_msg_mavlink_m_ack_pack_chan(
		mavlink_system.sysid, mavlink_system.compid, channel, &response,
		now, MAVLINK_MSG_ID_TARGET_CUE, record.cue.cue_id,
		record.source_system, mavlink_system.sysid, result, reason,
		record.source_system, record.source_component);
	return freeze_message(response, record.cue.cue_id, FrameKind::Decision);
}

PersistedFrame build_receipt_frame(uint8_t channel, const PersistedRecord &record,
				   uint64_t now)
{
	PersistedFrame frame = build_ack_frame(channel, record,
					      MAVLINK_M_ACK_RECEIVED,
					      "durably stored", now);
	frame.kind = static_cast<uint8_t>(FrameKind::Receipt);
	frame.checksum = frame_checksum(frame);
	return frame;
}

PersistedFrame build_status_frame(uint8_t channel, const PersistedRecord &record,
				  RecordState state, const char *reason,
				  uint16_t reason_code, uint64_t now)
{
	mavlink_message_t response{};
	mavlink_msg_mavlink_m_task_status_pack_chan(
		mavlink_system.sysid, mavlink_system.compid, channel, &response,
		now, MAVLINK_MSG_ID_TARGET_CUE, record.cue.cue_id,
		record.status_sequence, reason_code,
		record.source_system, record.source_component,
		wire_state(state), reason);
	return freeze_message(response, record.cue.cue_id, FrameKind::Status);
}

bool link_is_live(uint8_t channel)
{
	return channel < MAVLINK_COMM_NUM_BUFFERS
	       && g_link_seen[channel] != 0
	       && hrt_elapsed_time(&g_link_seen[channel]) <= kLinkFreshUsec;
}

bool send_frozen(const PersistedFrame &frame, uint8_t channel)
{
	if (frame.length == 0 || frame.length > kFrozenFrameBytes
	    || !link_is_live(channel)) {
		return false;
	}

	mavlink_start_uart_send(static_cast<mavlink_channel_t>(channel), frame.length);
	mavlink_send_uart_bytes(static_cast<mavlink_channel_t>(channel),
				frame.bytes, frame.length);
	mavlink_end_uart_send(static_cast<mavlink_channel_t>(channel), frame.length);
	return true;
}

int find_existing(uint8_t system, uint8_t component, uint32_t cue_id)
{
	for (unsigned i = 0; i < kRecordCount; ++i) {
		const PersistedRecord &record = g_records[i].persisted;

		if (state_of(g_records[i]) != RecordState::Empty
		    && record.source_system == system
		    && record.source_component == component
		    && record.cue.cue_id == cue_id) {
			return static_cast<int>(i);
		}
	}

	return -1;
}

int find_new_slot(uint64_t now)
{
	int oldest_terminal = -1;

	for (unsigned i = 0; i < kRecordCount; ++i) {
		const RuntimeRecord &record = g_records[i];

		if (state_of(record) == RecordState::Empty) {
			return static_cast<int>(i);
		}

		if (terminal_state(state_of(record))
		    && record.persisted.decision_retry_remaining == 0
		    && record.persisted.status_retry_remaining == 0
		    && (record.persisted.cue.valid_until_usec + 60ULL * 1000ULL * 1000ULL < now)
		    && (oldest_terminal < 0
			|| record.persisted.received_utc_usec
			< g_records[oldest_terminal].persisted.received_utc_usec)) {
			oldest_terminal = static_cast<int>(i);
		}
	}

	return oldest_terminal;
}

int oldest_pending()
{
	int selected = -1;

	for (unsigned i = 0; i < kRecordCount; ++i) {
		if (state_of(g_records[i]) == RecordState::Pending
		    && (selected < 0 || g_records[i].persisted.received_utc_usec
			< g_records[selected].persisted.received_utc_usec)) {
			selected = static_cast<int>(i);
		}
	}

	return selected;
}

int current_active()
{
	for (unsigned i = 0; i < kRecordCount; ++i) {
		if (active_state(state_of(g_records[i]))) {
			return static_cast<int>(i);
		}
	}

	return -1;
}

void send_current_response(RuntimeRecord &record)
{
	const uint8_t channel = record.persisted.ingress_channel;

	if (state_of(record) == RecordState::Pending) {
		send_frozen(record.receipt, channel);
		return;
	}

	if (valid_frame(record.decision, record.persisted.cue.cue_id,
			FrameKind::Decision)) {
		send_frozen(record.decision, channel);
	}

	if (valid_frame(record.status, record.persisted.cue.cue_id,
			FrameKind::Status)) {
		send_frozen(record.status, channel);
	}
}

bool commit_decision(unsigned slot, RecordState next_state, uint8_t result,
		     const char *reason)
{
	RuntimeRecord next = g_records[slot];
	const uint64_t now = utc_now_usec();

	if (now == 0) {
		PX4_WARN("MAVLink-M decision blocked: UTC unavailable");
		return false;
	}

	next.persisted.state = static_cast<uint8_t>(next_state);
	next.persisted.status_sequence = next_state == RecordState::Active ? 1 : 0;
	next.persisted.decision_retry_remaining = kTotalReplyAttempts;
	next.persisted.status_retry_remaining =
		next_state == RecordState::Active ? kTotalReplyAttempts : 0;
	next.decision = build_ack_frame(next.persisted.ingress_channel,
					next.persisted, result, reason, now);

	if (!valid_frame(next.decision, next.persisted.cue.cue_id,
			 FrameKind::Decision)
	    || !write_frame(slot, 2, &next.decision)) {
		PX4_ERR("MAVLink-M decision ACK persistence failed");
		return false;
	}

	if (next_state == RecordState::Active) {
		next.status = build_status_frame(next.persisted.ingress_channel,
						 next.persisted, next_state,
						 "pilot accepted", 0, now);

		if (!valid_frame(next.status, next.persisted.cue.cue_id,
				 FrameKind::Status)
		    || !write_frame(slot, 3, &next.status)) {
			PX4_ERR("MAVLink-M task status persistence failed");
			return false;
		}
	}

	if (!write_record(slot, &next.persisted)) {
		PX4_ERR("MAVLink-M decision commit failed");
		return false;
	}

	next.next_decision_retry = 0;
	next.next_status_retry = 0;
	g_records[slot] = next;
	g_center_latched = false;
	return true;
}

bool commit_lifecycle(unsigned slot, RecordState next_state, const char *reason,
		      uint16_t reason_code)
{
	RuntimeRecord next = g_records[slot];
	const RecordState previous = state_of(next);

	if (!active_state(previous)) {
		return false;
	}

	if ((previous == RecordState::Ready && next_state == RecordState::EnRoute)
	    || next_state == RecordState::Active
	    || next_state == RecordState::Pending
	    || next_state == RecordState::Rejected
	    || next_state == RecordState::Empty) {
		return false;
	}

	const uint64_t now = utc_now_usec();

	if (now == 0) {
		return false;
	}

	next.persisted.state = static_cast<uint8_t>(next_state);
	++next.persisted.status_sequence;

	if (next.persisted.status_sequence == 0) {
		next.persisted.status_sequence = 1;
	}

	next.persisted.status_retry_remaining = kTotalReplyAttempts;
	next.status = build_status_frame(next.persisted.ingress_channel,
					 next.persisted, next_state,
					 reason, reason_code, now);

	if (!valid_frame(next.status, next.persisted.cue.cue_id,
			 FrameKind::Status)
	    || !write_frame(slot, 3, &next.status)
	    || !write_record(slot, &next.persisted)) {
		PX4_ERR("MAVLink-M lifecycle commit failed");
		return false;
	}

	next.next_status_retry = 0;
	g_records[slot] = next;
	return true;
}

void publish_status_locked(bool force)
{
	const hrt_abstime now = hrt_absolute_time();

	if (!force && g_last_status_publish != 0
	    && now - g_last_status_publish < 500_ms) {
		return;
	}

	g_last_status_publish = now;
	int selected = current_active();

	if (selected < 0) {
		selected = oldest_pending();
	}

	aags_mavlink_m_status_s status{};
	status.timestamp = now;
	status.bearing_rad = NAN;
	status.distance_m = NAN;
	status.alt = NAN;
	status.confidence_score = NAN;

	if (selected >= 0) {
		const PersistedRecord &record = g_records[selected].persisted;
		status.cue_id = record.cue.cue_id;
		status.status_sequence = record.status_sequence;
		status.valid_until_usec = record.cue.valid_until_usec;
		status.lat = record.cue.lat;
		status.lon = record.cue.lon;
		status.alt = record.cue.alt;
		status.confidence_score = record.cue.confidence_score;
		status.state = record.state;
		status.source_system = record.source_system;
		status.source_component = record.source_component;
		status.cue_type = record.cue.cue_type;
		status.target_class = record.cue.target_class;
		status.target_force = record.cue.target_force;
		status.local_decision_available = state_of(g_records[selected]) == RecordState::Pending;
		memcpy(status.name, record.cue.name, sizeof(status.name));

		vehicle_global_position_s position{};

		if (g_global_position_sub.copy(&position)
		    && PX4_ISFINITE(position.lat) && PX4_ISFINITE(position.lon)
		    && position.lat >= -90.0 && position.lat <= 90.0
		    && position.lon >= -180.0 && position.lon <= 180.0) {
			const double cue_lat = static_cast<double>(record.cue.lat) * 1e-7;
			const double cue_lon = static_cast<double>(record.cue.lon) * 1e-7;
			status.distance_m = get_distance_to_next_waypoint(
						    position.lat, position.lon, cue_lat, cue_lon);
			status.bearing_rad = get_bearing_to_next_waypoint(
						     position.lat, position.lon, cue_lat, cue_lon);
			status.position_valid = PX4_ISFINITE(status.distance_m)
						&& PX4_ISFINITE(status.bearing_rad);
		}
	}

	g_status_pub.publish(status);
}

void send_retries_locked(hrt_abstime now)
{
	for (unsigned slot = 0; slot < kRecordCount; ++slot) {
		RuntimeRecord &record = g_records[slot];
		const uint8_t channel = record.persisted.ingress_channel;

		if (record.persisted.decision_retry_remaining > 0
		    && (record.next_decision_retry == 0
			|| now >= record.next_decision_retry)
		    && link_is_live(channel)
		    && valid_frame(record.decision, record.persisted.cue.cue_id,
				   FrameKind::Decision)) {
			const uint8_t attempt =
				kTotalReplyAttempts - record.persisted.decision_retry_remaining;
			--record.persisted.decision_retry_remaining;
			write_record(slot, &record.persisted);
			send_frozen(record.decision, channel);
			const uint8_t exponent = attempt > 3 ? 3 : attempt;
			record.next_decision_retry = now
				+ kInitialRetryUsec * (1ULL << exponent);
		}

		if (record.persisted.status_retry_remaining > 0
		    && (record.next_status_retry == 0
			|| now >= record.next_status_retry)
		    && link_is_live(channel)
		    && valid_frame(record.status, record.persisted.cue.cue_id,
				   FrameKind::Status)) {
			const uint8_t attempt =
				kTotalReplyAttempts - record.persisted.status_retry_remaining;
			--record.persisted.status_retry_remaining;
			write_record(slot, &record.persisted);
			send_frozen(record.status, channel);
			const uint8_t exponent = attempt > 3 ? 3 : attempt;
			record.next_status_retry = now
				+ kInitialRetryUsec * (1ULL << exponent);
		}
	}
}

void expire_records_locked(uint64_t utc_now)
{
	if (utc_now == 0) {
		return;
	}

	for (unsigned slot = 0; slot < kRecordCount; ++slot) {
		const RecordState state = state_of(g_records[slot]);

		if ((state == RecordState::Pending || active_state(state))
		    && g_records[slot].persisted.cue.valid_until_usec <= utc_now) {
			if (state == RecordState::Pending) {
				commit_decision(slot, RecordState::Expired,
						MAVLINK_M_ACK_EXPIRED, "cue expired");

				// Rejected/expired offers have no accepted-task lifecycle.
				g_records[slot].persisted.status_retry_remaining = 0;
				write_record(slot, &g_records[slot].persisted);

			} else {
				commit_lifecycle(slot, RecordState::Expired,
						 "task expired", 0);
			}
		}
	}
}

void process_rc_locked()
{
	const int32_t configured_channel = parameter_int(
			g_rc_channel_param, "MVM_RC_CH", 0);

	if (configured_channel <= 0 || configured_channel > 18) {
		g_center_latched = false;
		return;
	}

	rc_channels_s rc{};

	if (!g_rc_channels_sub.update(&rc)) {
		return;
	}

	if (rc.signal_lost || rc.channel_count < configured_channel
	    || hrt_elapsed_time(&rc.timestamp_last_valid) > 500_ms) {
		g_center_latched = false;
		return;
	}

	const float reject_threshold = parameter_float(
			g_rc_reject_param, "MVM_RC_REJ", -0.65f);
	const float accept_threshold = parameter_float(
			g_rc_accept_param, "MVM_RC_ACC", 0.65f);
	const float center_threshold = parameter_float(
			g_rc_center_param, "MVM_RC_CTR", 0.25f);
	const float value = rc.channels[configured_channel - 1];

	if (!PX4_ISFINITE(value)
	    || !(reject_threshold < -center_threshold
		 && accept_threshold > center_threshold)) {
		g_center_latched = false;
		return;
	}

	if (fabsf(value) <= center_threshold) {
		g_center_latched = true;
		return;
	}

	if (!g_center_latched) {
		return;
	}

	const int pending = oldest_pending();

	if (pending < 0) {
		return;
	}

	if (value >= accept_threshold) {
		if (current_active() >= 0) {
			PX4_WARN("MAVLink-M cue queued: active task must finish first");
			g_center_latched = false;
			return;
		}

		commit_decision(static_cast<unsigned>(pending), RecordState::Active,
				MAVLINK_M_ACK_ACCEPTED, "pilot accepted");

	} else if (value <= reject_threshold) {
		commit_decision(static_cast<unsigned>(pending), RecordState::Rejected,
				MAVLINK_M_ACK_REJECTED, "pilot rejected");
	}
}

void send_capability_locked(Mavlink *link, uint64_t now_utc)
{
	const uint8_t channel = static_cast<uint8_t>(link->get_channel());

	if (channel >= MAVLINK_COMM_NUM_BUFFERS || now_utc == 0
	    || !link->should_transmit()
	    || (g_last_capability[channel] != 0
		&& hrt_elapsed_time(&g_last_capability[channel]) < kCapabilityPeriodUsec)) {
		return;
	}

	mavlink_message_t capability{};
	const uint32_t flags = MAVLINK_M_CAPABILITY_CUE_RECEIVE
			       | MAVLINK_M_CAPABILITY_APPLICATION_RECEIPT
			       | MAVLINK_M_CAPABILITY_LOCAL_DECISION
			       | MAVLINK_M_CAPABILITY_TASK_STATUS
			       | MAVLINK_M_CAPABILITY_INERT_ONLY;
	char profile_id[32]{};
	char profile_version[32]{};
	strncpy(profile_id, AAGS_MAVLINK_M_PROFILE_ID, sizeof(profile_id) - 1);
	strncpy(profile_version, AAGS_MAVLINK_M_PROFILE_VERSION,
		sizeof(profile_version) - 1);
	mavlink_msg_mavlink_m_capability_pack_chan(
		mavlink_system.sysid, mavlink_system.compid, channel, &capability,
		now_utc, now_utc + kCapabilityLifetimeUsec, flags,
		MAVLINK_M_ENDPOINT_PX4_PILOT,
		mavlink_system.sysid, mavlink_system.compid,
		AAGS_MAVLINK_M_PROTOCOL_MAJOR, AAGS_MAVLINK_M_PROTOCOL_MINOR,
		AAGS_MAVLINK_M_PROFILE_SHA256_BYTES,
		profile_id, profile_version,
		0);
	uint8_t bytes[MAVLINK_MAX_PACKET_LEN]{};
	const uint16_t length = mavlink_msg_to_send_buffer(bytes, &capability);
	mavlink_start_uart_send(static_cast<mavlink_channel_t>(channel), length);
	mavlink_send_uart_bytes(static_cast<mavlink_channel_t>(channel), bytes, length);
	mavlink_end_uart_send(static_cast<mavlink_channel_t>(channel), length);
	g_last_capability[channel] = hrt_absolute_time();
}

bool decide_locked(bool accept)
{
	const int pending = oldest_pending();

	if (pending < 0) {
		PX4_WARN("no pending MAVLink-M cue");
		return false;
	}

	if (accept && current_active() >= 0) {
		PX4_WARN("active MAVLink-M cue must finish before accepting queued cue");
		return false;
	}

	return commit_decision(static_cast<unsigned>(pending),
			       accept ? RecordState::Active : RecordState::Rejected,
			       accept ? MAVLINK_M_ACK_ACCEPTED : MAVLINK_M_ACK_REJECTED,
			       accept ? "pilot accepted" : "pilot rejected");
}

bool lifecycle_locked(RecordState next_state, const char *reason)
{
	const int active = current_active();

	if (active < 0) {
		PX4_WARN("no active MAVLink-M cue");
		return false;
	}

	return commit_lifecycle(static_cast<unsigned>(active), next_state, reason, 0);
}

} // namespace

void MavlinkMEndpoint::handle_message(Mavlink *link,
				      const mavlink_message_t *message)
{
	if (!link || !message || !endpoint_enabled()) {
		return;
	}

	MutexGuard guard;
	initialize_locked();

	if (!g_storage_ready || signing_required()) {
		return;
	}

	const uint8_t channel = static_cast<uint8_t>(link->get_channel());

	if (channel < MAVLINK_COMM_NUM_BUFFERS) {
		g_link_seen[channel] = hrt_absolute_time();
	}

	if (message->msgid == MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY) {
		if (message->len >= MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY_LEN) {
			mavlink_mavlink_m_capability_t capability{};
			mavlink_msg_mavlink_m_capability_decode(message, &capability);
			approve_source_locked(message, capability);
		}

		return;
	}

	if (message->msgid != MAVLINK_MSG_ID_TARGET_CUE
	    || !source_approved_locked(message->sysid, message->compid)) {
		return;
	}

	mavlink_target_cue_t cue{};
	mavlink_msg_target_cue_decode(message, &cue);
	const uint64_t now = utc_now_usec();

	if (now == 0 || !valid_cue(message, cue, now)) {
		// A malformed/expired frame is never an application receipt.
		if (cue.target_system == mavlink_system.sysid
		    && cue.target_component == mavlink_system.compid
		    && cue.cue_id != 0 && message->sysid != 0 && message->compid != 0) {
			PersistedRecord rejected{};
			rejected.cue = cue;
			rejected.source_system = message->sysid;
			rejected.source_component = message->compid;
			rejected.ingress_channel = channel;
			const uint8_t result =
				(now != 0 && cue.valid_until_usec <= now)
				? MAVLINK_M_ACK_EXPIRED : MAVLINK_M_ACK_FAILED;
			const PersistedFrame response = build_ack_frame(
				channel, rejected, result,
				result == MAVLINK_M_ACK_EXPIRED ? "cue expired" : "invalid cue",
				now);
			send_frozen(response, channel);
		}

		return;
	}

	const int existing = find_existing(message->sysid, message->compid,
					   cue.cue_id);

	if (existing >= 0) {
		RuntimeRecord &record = g_records[existing];

		if (memcmp(&record.persisted.cue, &cue, sizeof(cue)) == 0) {
			record.persisted.ingress_channel = channel;
			write_record(static_cast<unsigned>(existing), &record.persisted);
			send_current_response(record);

		} else {
			PersistedRecord collision = record.persisted;
			collision.ingress_channel = channel;
			const PersistedFrame response = build_ack_frame(
				channel, collision, MAVLINK_M_ACK_FAILED,
				"immutable cue collision", now);
			send_frozen(response, channel);
		}

		return;
	}

	const int slot = find_new_slot(now);

	if (slot < 0) {
		PersistedRecord busy{};
		busy.cue = cue;
		busy.source_system = message->sysid;
		busy.source_component = message->compid;
		busy.ingress_channel = channel;
		const PersistedFrame response = build_ack_frame(
			channel, busy, MAVLINK_M_ACK_FAILED,
			"durable inbox full", now);
		send_frozen(response, channel);
		return;
	}

	RuntimeRecord pending{};
	pending.persisted.state = static_cast<uint8_t>(RecordState::Pending);
	pending.persisted.received_utc_usec = now;
	pending.persisted.cue = cue;
	pending.persisted.source_system = message->sysid;
	pending.persisted.source_component = message->compid;
	pending.persisted.ingress_channel = channel;
	pending.receipt = build_receipt_frame(channel, pending.persisted, now);

	// Frame fragments are written first; the record is the commit marker.
	if (!valid_frame(pending.receipt, cue.cue_id, FrameKind::Receipt)
	    || !write_frame(static_cast<unsigned>(slot), 1, &pending.receipt)
	    || !write_record(static_cast<unsigned>(slot), &pending.persisted)) {
		PX4_ERR("MAVLink-M cue durable commit failed");
		return;
	}

	g_records[slot] = pending;
	send_frozen(g_records[slot].receipt, channel);
	publish_status_locked(true);
	PX4_INFO("MAVLink-M cue %u pending from %u:%u",
		 (unsigned)cue.cue_id, (unsigned)message->sysid,
		 (unsigned)message->compid);
}

void MavlinkMEndpoint::update(Mavlink *link)
{
	if (!link || !endpoint_enabled()) {
		return;
	}

	MutexGuard guard;
	initialize_locked();

	const uint8_t channel = static_cast<uint8_t>(link->get_channel());

	if (channel < MAVLINK_COMM_NUM_BUFFERS) {
		g_link_seen[channel] = hrt_absolute_time();
	}

	if (!g_storage_ready) {
		return;
	}

	if (signing_required()) {
		if (!g_signing_warning_printed) {
			PX4_WARN("MVM_SIGN_REQ=1: PX4 1.14 key validation is unavailable; endpoint disabled");
			g_signing_warning_printed = true;
		}

		return;
	}

	g_signing_warning_printed = false;
	const uint64_t utc_now = utc_now_usec();
	send_capability_locked(link, utc_now);
	expire_records_locked(utc_now);
	process_rc_locked();
	send_retries_locked(hrt_absolute_time());
	publish_status_locked(false);
}

int MavlinkMEndpoint::command(int argc, char *argv[])
{
	if (argc < 1) {
		PX4_INFO("usage: mavlink mavlink_m <status|accept|reject|enroute|ready|complete|abort|fail>");
		return PX4_ERROR;
	}

	MutexGuard guard;
	initialize_locked();

	if (!g_storage_ready) {
		PX4_ERR("MAVLink-M durable storage unavailable");
		return PX4_ERROR;
	}

	if (!strcmp(argv[0], "status")) {
		for (unsigned i = 0; i < kRecordCount; ++i) {
			const PersistedRecord &record = g_records[i].persisted;

			if (state_of(g_records[i]) != RecordState::Empty) {
				PX4_INFO("slot %u cue=%u from=%u:%u state=%s decision_retry=%u status_retry=%u",
					 i, (unsigned)record.cue.cue_id,
					 (unsigned)record.source_system,
					 (unsigned)record.source_component,
					 state_name(state_of(g_records[i])),
					 (unsigned)record.decision_retry_remaining,
					 (unsigned)record.status_retry_remaining);
			}
		}

		PX4_INFO("storage=%s rc_center_latched=%s signing_required=%s",
			 g_storage_ready ? "ready" : "unavailable",
			 g_center_latched ? "yes" : "no",
			 signing_required() ? "yes (endpoint disabled)" : "no");
		return PX4_OK;
	}

	bool success = false;

	if (!strcmp(argv[0], "accept")) {
		success = decide_locked(true);

	} else if (!strcmp(argv[0], "reject")) {
		success = decide_locked(false);

	} else if (!strcmp(argv[0], "enroute")) {
		success = lifecycle_locked(RecordState::EnRoute, "pilot reports en route");

	} else if (!strcmp(argv[0], "ready")) {
		success = lifecycle_locked(RecordState::Ready, "pilot reports ready");

	} else if (!strcmp(argv[0], "complete")) {
		success = lifecycle_locked(RecordState::Complete, "pilot reports complete");

	} else if (!strcmp(argv[0], "abort")) {
		success = lifecycle_locked(RecordState::Aborted, "pilot aborted task");

	} else if (!strcmp(argv[0], "fail")) {
		success = lifecycle_locked(RecordState::Failed, "pilot reports failure");

	} else {
		PX4_WARN("unknown MAVLink-M command: %s", argv[0]);
		return PX4_ERROR;
	}

	publish_status_locked(true);
	return success ? PX4_OK : PX4_ERROR;
}

void MavlinkMEndpoint::print_status()
{
	char *arguments[] {const_cast<char *>("status")};
	command(1, arguments);
}

#else

void MavlinkMEndpoint::handle_message(Mavlink *, const mavlink_message_t *)
{
}

void MavlinkMEndpoint::update(Mavlink *)
{
}

int MavlinkMEndpoint::command(int, char *[])
{
	PX4_WARN("firmware was not built with CONFIG_MAVLINK_DIALECT=mavlink_m");
	return PX4_ERROR;
}

void MavlinkMEndpoint::print_status()
{
	PX4_INFO("MAVLink-M endpoint not compiled");
}

#endif
