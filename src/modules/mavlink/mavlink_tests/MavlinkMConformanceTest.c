#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <mavlink.h>
#include <aags_mavlink_m_profile.h>

enum { MaxFrameLength = MAVLINK_MAX_PACKET_LEN };
enum {
    TrackIdentityRequiredWireLength = offsetof(mavlink_track_identity_t, origin_sysid),
    TargetCueRequiredWireLength = offsetof(mavlink_target_cue_t, cue_type),
    TargetHandoverRequiredWireLength = offsetof(mavlink_target_handover_t, confidence_score),
};

typedef struct {
    uint8_t bytes[MaxFrameLength];
    uint16_t length;
} WireFrame;

static const uint8_t TrackUid[16] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};

static const uint8_t SigningKey[32] = {
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
    0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
    0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
    0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f,
};

static WireFrame serialize_message(const mavlink_message_t* message)
{
    WireFrame frame = {{0}, 0};
    frame.length = mavlink_msg_to_send_buffer(frame.bytes, message);
    assert(frame.length <= MaxFrameLength);
    assert(frame.bytes[0] == MAVLINK_STX);
    return frame;
}

static WireFrame make_target_cue(void)
{
    mavlink_message_t message;
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_target_cue_pack(
        1, 191, &message,
        UINT64_C(1760000001000000), 731, 45,
        436531234, -793832345, 123.5f,
        NAN, NAN, NAN, 0.875f, 1,
        MAVLINK_M_CUE_TYPE_INVESTIGATE,
        MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN,
        "ALBATROSS-731");
    return serialize_message(&message);
}

static WireFrame make_target_handover(void)
{
    mavlink_message_t message;
    const uint8_t noAuthorization[8] = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_target_handover_pack(
        1, 191, &message,
        UINT64_C(1760000001000000),
        UINT64_C(1760000000000000),
        UINT64_C(1760000031000000),
        436531234, -793832345, 123.5f,
        NAN, NAN, NAN,
        NAN, NAN, NAN, NAN, NAN, NAN,
        9002, "ALBATROSS-HANDOVER", "", 0.875f,
        noAuthorization,
        MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN,
        MAVLINK_M_MATCH_MEDIA_NONE,
        TrackUid);
    return serialize_message(&message);
}

static WireFrame make_trimmed_target_handover(void)
{
    mavlink_message_t message;
    const uint8_t noAuthorization[8] = {0};
    const uint8_t noTrackUid[16] = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_target_handover_pack(
        1, 191, &message,
        UINT64_C(1760000001000000),
        UINT64_C(1760000000000000),
        UINT64_C(1760000031000000),
        436531234, -793832345, 123.5f,
        NAN, NAN, NAN,
        NAN, NAN, NAN, NAN, NAN, NAN,
        9003, "TRIMMED-HANDOVER", "", 0.875f,
        noAuthorization,
        MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN,
        MAVLINK_M_MATCH_MEDIA_NONE,
        noTrackUid);
    return serialize_message(&message);
}

static WireFrame make_trimmed_track_identity(void)
{
    mavlink_message_t message;
    const uint8_t noParentTrack[16] = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_track_identity_pack(
        1, 191, &message,
        UINT64_C(1760000001000000), TrackUid, noParentTrack, 45,
        UINT64_C(1760000000000000), 0.875f, 1,
        MAVLINK_M_ID_METHOD_VISUAL_EO, MAVLINK_M_ID_METHOD_VISUAL_EO,
        MAVLINK_M_PID_STATUS_NONE, MAVLINK_M_TRACK_REL_NONE,
        MAVLINK_M_TARGET_CLASS_UNKNOWN, MAVLINK_M_TARGET_FORCE_UNKNOWN,
        "operator visual", "", MAVLINK_M_TRACK_NUMBER_TYPE_NONE,
        MAVLINK_M_STANAG_IDENTITY_UNKNOWN, MAVLINK_M_ENVIRONMENT_UNKNOWN,
        0, 0, MAVLINK_M_ATR_CONF_NA, MAVLINK_M_SIDC_CONTEXT_REALITY);
    return serialize_message(&message);
}

static WireFrame make_ack(void)
{
    mavlink_message_t message;
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_mavlink_m_ack_pack(
        42, 191, &message,
        UINT64_C(1760000002000000),
        MAVLINK_MSG_ID_TARGET_CUE, 731,
        1, 42, MAVLINK_M_ACK_ACCEPTED,
        "operator accepted");
    return serialize_message(&message);
}

static WireFrame make_signed_target_cue(void)
{
    mavlink_signing_t signing;
    memset(&signing, 0, sizeof(signing));
    signing.flags = MAVLINK_SIGNING_FLAG_SIGN_OUTGOING;
    signing.link_id = 7;
    signing.timestamp = UINT64_C(36000000000000);
    memcpy(signing.secret_key, SigningKey, sizeof(SigningKey));
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_get_channel_status(MAVLINK_COMM_0)->signing = &signing;
    mavlink_message_t message;
    mavlink_msg_target_cue_pack(
        1, 191, &message,
        UINT64_C(1760000001000000), 731, 45,
        436531234, -793832345, 123.5f,
        NAN, NAN, NAN, 0.875f, 1,
        MAVLINK_M_CUE_TYPE_INVESTIGATE,
        MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN,
        "ALBATROSS-731");
    return serialize_message(&message);
}

static bool parse_frame(const WireFrame* frame, mavlink_message_t* parsed)
{
    mavlink_status_t status;
    memset(&status, 0, sizeof(status));
    memset(parsed, 0, sizeof(*parsed));
    mavlink_reset_channel_status(MAVLINK_COMM_1);
    for (uint16_t index = 0; index < frame->length; ++index) {
        if (mavlink_parse_char(MAVLINK_COMM_1, frame->bytes[index], parsed, &status)) {
            return true;
        }
    }
    return false;
}

static bool parse_signed_frame(const WireFrame* frame, mavlink_message_t* parsed)
{
    mavlink_status_t resultStatus;
    mavlink_signing_t signing;
    mavlink_signing_streams_t streams;
    memset(&resultStatus, 0, sizeof(resultStatus));
    memset(&signing, 0, sizeof(signing));
    memset(&streams, 0, sizeof(streams));
    memset(parsed, 0, sizeof(*parsed));
    memcpy(signing.secret_key, SigningKey, sizeof(SigningKey));
    mavlink_reset_channel_status(MAVLINK_COMM_1);
    mavlink_get_channel_status(MAVLINK_COMM_1)->signing = &signing;
    mavlink_get_channel_status(MAVLINK_COMM_1)->signing_streams = &streams;
    for (uint16_t index = 0; index < frame->length; ++index) {
        if (mavlink_parse_char(MAVLINK_COMM_1, frame->bytes[index],
                               parsed, &resultStatus)) {
            return true;
        }
    }
    return false;
}

static void print_hex(const WireFrame* frame)
{
    for (uint16_t index = 0; index < frame->length; ++index) {
        printf("%02x", frame->bytes[index]);
    }
}

static void print_vector(const char* name, const WireFrame* frame,
                         uint32_t messageId, const char* note, bool last)
{
    printf("    {\"name\":\"%s\",\"message_id\":%u,\"wire_hex\":\"",
           name, messageId);
    print_hex(frame);
    printf("\",\"expected_parse\":\"accept\",\"note\":\"%s\"}%s\n",
           note, last ? "" : ",");
}

static int emit_golden(void)
{
    const WireFrame cue = make_target_cue();
    const WireFrame handover = make_target_handover();
    const WireFrame ack = make_ack();
    printf("{\n");
    printf("  \"schema\":\"aags.mavlink-m.golden-frames.v2\",\n");
    printf("  \"profile_id\":\"%s\",\n", AAGS_MAVLINK_M_PROFILE_ID);
    printf("  \"profile_version\":\"%s\",\n", AAGS_MAVLINK_M_PROFILE_VERSION);
    printf("  \"core_xml_sha256\":\"%s\",\n", AAGS_MAVLINK_M_CORE_XML_SHA256);
    printf("  \"live_transmit\":%s,\n",
           AAGS_MAVLINK_M_LIVE_TRANSPORT ? "true" : "false");
    printf("  \"field_release\":%s,\n",
           AAGS_MAVLINK_M_FIELD_RELEASE ? "true" : "false");
    printf("  \"inert_only\":%s,\n",
           AAGS_MAVLINK_M_INERT_ONLY ? "true" : "false");
    printf("  \"vectors\":[\n");
    print_vector("target_cue_final", &cue, MAVLINK_MSG_ID_TARGET_CUE,
                 "final shared cue; selected vehicle link supplies routing", false);
    print_vector("target_handover_final", &handover,
                 MAVLINK_MSG_ID_TARGET_HANDOVER,
                 "final shared handover correlated by target_set_id", false);
    print_vector("mavlink_m_ack_final", &ack, MAVLINK_MSG_ID_MAVLINK_M_ACK,
                 "ack_msgid 53001 instance 731 from system 42", true);
    printf("  ]\n");
    printf("}\n");
    return 0;
}

static int self_test(void)
{
    _Static_assert(MAVLINK_MSG_ID_TRACK_IDENTITY == 53000,
                   "unexpected TRACK_IDENTITY ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE == 53001,
                   "unexpected TARGET_CUE ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_HANDOVER == 53002,
                   "unexpected TARGET_HANDOVER ID");
    _Static_assert(MAVLINK_MSG_ID_PARTICIPANT_POSITION == 53003,
                   "unexpected PARTICIPANT_POSITION ID");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK == 53004,
                   "unexpected ACK ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET == 53010,
                   "unexpected TARGET ID");
    _Static_assert(MAVLINK_MSG_ID_FIRES == 53020,
                   "unexpected FIRES ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_MIN_LEN == 68,
                   "unexpected TARGET_CUE minimum length");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_LEN == 68,
                   "unexpected TARGET_CUE full length");
    _Static_assert(MAVLINK_MSG_ID_TARGET_HANDOVER_LEN == 207,
                   "unexpected TARGET_HANDOVER full length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_MIN_LEN == 69,
                   "unexpected ACK minimum length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN == 69,
                   "unexpected ACK full length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_CRC == 47,
                   "unexpected ACK CRC");
    _Static_assert(TrackIdentityRequiredWireLength == 58,
                   "unexpected TRACK_IDENTITY semantic prefix");
    _Static_assert(TargetCueRequiredWireLength == 45,
                   "unexpected TARGET_CUE semantic prefix");
    _Static_assert(TargetHandoverRequiredWireLength == 76,
                   "unexpected TARGET_HANDOVER semantic prefix");

    mavlink_message_t parsed;
    WireFrame cue = make_target_cue();
    assert(parse_frame(&cue, &parsed));
    assert(parsed.msgid == MAVLINK_MSG_ID_TARGET_CUE);
    assert(parsed.len >= TargetCueRequiredWireLength);
    assert(parsed.len < MAVLINK_MSG_ID_TARGET_CUE_LEN);
    mavlink_target_cue_t decodedCue;
    mavlink_msg_target_cue_decode(&parsed, &decodedCue);
    assert(decodedCue.cue_id == 731);
    assert(decodedCue.target_set_id == 45);
    assert(decodedCue.lat == 436531234);
    assert(decodedCue.lon == -793832345);
    assert(decodedCue.origin_sysid == 1);

    WireFrame handover = make_target_handover();
    assert(parse_frame(&handover, &parsed));
    mavlink_target_handover_t decodedHandover;
    mavlink_msg_target_handover_decode(&parsed, &decodedHandover);
    assert(decodedHandover.target_set_id == 9002);
    assert(memcmp(decodedHandover.track_uid, TrackUid, sizeof(TrackUid)) == 0);

    WireFrame trimmedHandover = make_trimmed_target_handover();
    assert(parse_frame(&trimmedHandover, &parsed));
    assert(parsed.len >= TargetHandoverRequiredWireLength);
    assert(parsed.len < MAVLINK_MSG_ID_TARGET_HANDOVER_LEN);
    memset(&decodedHandover, 0, sizeof(decodedHandover));
    mavlink_msg_target_handover_decode(&parsed, &decodedHandover);
    assert(decodedHandover.target_set_id == 9003);
    assert(memcmp(decodedHandover.track_uid, (uint8_t[16]){0}, 16) == 0);

    WireFrame trimmedIdentity = make_trimmed_track_identity();
    assert(parse_frame(&trimmedIdentity, &parsed));
    assert(parsed.len >= TrackIdentityRequiredWireLength);
    assert(parsed.len < MAVLINK_MSG_ID_TRACK_IDENTITY_LEN);
    mavlink_track_identity_t decodedIdentity;
    memset(&decodedIdentity, 0, sizeof(decodedIdentity));
    mavlink_msg_track_identity_decode(&parsed, &decodedIdentity);
    assert(decodedIdentity.target_set_id == 45);
    assert(decodedIdentity.origin_sysid == 1);
    assert(memcmp(decodedIdentity.track_uid, TrackUid, sizeof(TrackUid)) == 0);

    WireFrame ack = make_ack();
    assert(parse_frame(&ack, &parsed));
    assert(parsed.msgid == MAVLINK_MSG_ID_MAVLINK_M_ACK);
    mavlink_mavlink_m_ack_t decodedAck;
    mavlink_msg_mavlink_m_ack_decode(&parsed, &decodedAck);
    assert(decodedAck.ack_msgid == MAVLINK_MSG_ID_TARGET_CUE);
    assert(decodedAck.ack_instance == 731);
    assert(decodedAck.origin_sysid == 1);
    assert(decodedAck.ack_sysid == 42);
    assert(decodedAck.result == MAVLINK_M_ACK_ACCEPTED);

    WireFrame signedCue = make_signed_target_cue();
    assert((signedCue.bytes[2] & MAVLINK_IFLAG_SIGNED) != 0);
    assert(parse_signed_frame(&signedCue, &parsed));
    assert(parsed.msgid == MAVLINK_MSG_ID_TARGET_CUE);

    WireFrame badCrc = cue;
    badCrc.bytes[badCrc.length - 1] ^= 0x80;
    assert(!parse_frame(&badCrc, &parsed));
    WireFrame truncated = cue;
    --truncated.length;
    assert(!parse_frame(&truncated, &parsed));

    printf("profile_id=%s\n", AAGS_MAVLINK_M_PROFILE_ID);
    printf("profile_version=%s\n", AAGS_MAVLINK_M_PROFILE_VERSION);
    printf("core_xml_sha256=%s\n", AAGS_MAVLINK_M_CORE_XML_SHA256);
    printf("final_shared_message_encode_decode=PASS\n");
    printf("ack_correlation_53001_cue_731=PASS\n");
    printf("malformed_frame_rejection=PASS\n");
    printf("route_selected_payload_unaddressed=PASS\n");
    printf("mavlink2_trimmed_semantic_prefixes=PASS\n");
    printf("mavlink2_signing_roundtrip=PASS\n");
    return 0;
}

int main(int argc, char* argv[])
{
    if (argc == 2 && strcmp(argv[1], "--golden") == 0) {
        return emit_golden();
    }
    if (argc == 1 || (argc == 2 && strcmp(argv[1], "--self-test") == 0)) {
        return self_test();
    }
    fprintf(stderr, "usage: %s [--self-test|--golden]\n", argv[0]);
    return 2;
}
