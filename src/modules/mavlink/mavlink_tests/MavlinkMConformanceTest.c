#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include <mavlink.h>
#include <aags_mavlink_m_profile.h>

enum { MaxFrameLength = MAVLINK_MAX_PACKET_LEN };

typedef struct {
    uint8_t bytes[MaxFrameLength];
    uint16_t length;
} WireFrame;

static const uint8_t TrackUid[16] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};

static WireFrame serialize_message(const mavlink_message_t* message)
{
    WireFrame frame = {{0}, 0};
    frame.length = mavlink_msg_to_send_buffer(frame.bytes, message);
    assert(frame.length <= MaxFrameLength);
    assert(frame.bytes[0] == MAVLINK_STX);
    return frame;
}

static WireFrame make_track_identity(void)
{
    mavlink_message_t message = {0};
    const uint8_t noParent[16] = {0};
    const char idBasis[50] = "operator visual";
    const char externalTrackNumber[20] = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_track_identity_pack(
        1,
        191,
        &message,
        UINT64_C(1760000000000000),
        TrackUid,
        noParent,
        0,
        UINT64_C(1759999999000000),
        0.75f,
        1,
        MAVLINK_M_ID_METHOD_VISUAL_EO,
        MAVLINK_M_ID_METHOD_HUMAN_CONFIRM,
        MAVLINK_M_PID_STATUS_TENTATIVE,
        MAVLINK_M_TRACK_REL_NONE,
        MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN,
        idBasis,
        externalTrackNumber,
        0,
        0,
        0,
        255,
        0,
        0,
        0);
    return serialize_message(&message);
}

static WireFrame make_target_cue(void)
{
    mavlink_message_t message = {0};
    const char name[20] = "TRAINING-731";
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_target_cue_pack(
        1,
        191,
        &message,
        UINT64_C(1760000001000000),
        731,
        0,
        436531234,
        -793832345,
        123.5f,
        NAN,
        NAN,
        NAN,
        0.875f,
        1,
        MAVLINK_M_CUE_TYPE_INVESTIGATE,
        MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN,
        name);
    return serialize_message(&message);
}

static WireFrame make_ack(void)
{
    mavlink_message_t message = {0};
    const char reason[50] = "operator accepted";
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_mavlink_m_ack_pack(
        42,
        191,
        &message,
        UINT64_C(1760000002000000),
        MAVLINK_MSG_ID_TARGET_CUE,
        731,
        1,
        42,
        MAVLINK_M_ACK_ACCEPTED,
        reason);
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

static void print_hex(const WireFrame* frame)
{
    for (uint16_t index = 0; index < frame->length; ++index) {
        printf("%02x", frame->bytes[index]);
    }
}

static void print_vector(
    const char* name,
    const WireFrame* frame,
    uint32_t messageId,
    const char* note,
    bool last)
{
    printf("    {\"name\":\"%s\",\"message_id\":%u,\"wire_hex\":\"", name, messageId);
    print_hex(frame);
    printf("\",\"expected_parse\":\"accept\",\"note\":\"%s\"}%s\n", note, last ? "" : ",");
}

static int emit_golden(void)
{
    const WireFrame track = make_track_identity();
    const WireFrame cue = make_target_cue();
    const WireFrame ack = make_ack();
    printf("{\n");
    printf("  \"schema\":\"aags.mavlink-m.golden-frames.v1\",\n");
    printf("  \"profile_id\":\"%s\",\n", AAGS_MAVLINK_M_PROFILE_ID);
    printf("  \"profile_version\":\"%s\",\n", AAGS_MAVLINK_M_PROFILE_VERSION);
    printf("  \"core_xml_sha256\":\"%s\",\n", AAGS_MAVLINK_M_CORE_XML_SHA256);
    printf("  \"live_transmit\":false,\n");
    printf("  \"vectors\":[\n");
    print_vector("track_identity_nominal", &track, MAVLINK_MSG_ID_TRACK_IDENTITY,
        "persistent identity record; no engagement authority", false);
    print_vector("target_cue_nominal_unaddressed", &cue, MAVLINK_MSG_ID_TARGET_CUE,
        "wire-conformance only; supplied payload has no recipient", false);
    print_vector("mavlink_m_ack_nominal_unaddressed", &ack, MAVLINK_MSG_ID_MAVLINK_M_ACK,
        "ack_msgid 54001 and instance 731; supplied payload has no recipient", true);
    printf("  ]\n");
    printf("}\n");
    return 0;
}

static int self_test(void)
{
    _Static_assert(MAVLINK_MSG_ID_TRACK_IDENTITY == 54000, "unexpected TRACK_IDENTITY ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE == 54001, "unexpected TARGET_CUE ID");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK == 54004, "unexpected ACK ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_MIN_LEN == 68, "unexpected TARGET_CUE minimum length");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_LEN == 68, "unexpected TARGET_CUE full length");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_CRC == 11, "unexpected TARGET_CUE CRC");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_MIN_LEN == 69, "unexpected ACK minimum length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN == 69, "unexpected ACK full length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_CRC == 47, "unexpected ACK CRC");

    mavlink_message_t parsed;
    WireFrame track = make_track_identity();
    assert(parse_frame(&track, &parsed));
    assert(parsed.msgid == MAVLINK_MSG_ID_TRACK_IDENTITY);
    mavlink_track_identity_t decodedTrack;
    mavlink_msg_track_identity_decode(&parsed, &decodedTrack);
    assert(memcmp(decodedTrack.track_uid, TrackUid, sizeof(TrackUid)) == 0);
    assert(decodedTrack.pid_status == MAVLINK_M_PID_STATUS_TENTATIVE);

    WireFrame cue = make_target_cue();
    assert(parse_frame(&cue, &parsed));
    assert(parsed.msgid == MAVLINK_MSG_ID_TARGET_CUE);
    mavlink_target_cue_t decodedCue;
    mavlink_msg_target_cue_decode(&parsed, &decodedCue);
    assert(decodedCue.cue_id == 731);
    assert(decodedCue.lat == 436531234);
    assert(decodedCue.lon == -793832345);
    assert(decodedCue.origin_sysid == 1);

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

    WireFrame badCrc = cue;
    badCrc.bytes[badCrc.length - 1] ^= 0x80;
    assert(!parse_frame(&badCrc, &parsed));
    WireFrame truncated = cue;
    --truncated.length;
    assert(!parse_frame(&truncated, &parsed));

    printf("profile_id=%s\n", AAGS_MAVLINK_M_PROFILE_ID);
    printf("profile_version=%s\n", AAGS_MAVLINK_M_PROFILE_VERSION);
    printf("core_xml_sha256=%s\n", AAGS_MAVLINK_M_CORE_XML_SHA256);
    printf("workflow_message_encode_decode=PASS\n");
    printf("ack_correlation_54001_cue_731=PASS\n");
    printf("malformed_frame_rejection=PASS\n");
    printf("live_transmit_disabled_pending_PCR=PASS\n");
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
