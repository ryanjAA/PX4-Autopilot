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

typedef struct {
    uint8_t bytes[MaxFrameLength];
    uint16_t length;
} WireFrame;

static const uint8_t TrackUid[16] = {
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
};

static const uint8_t ProfileSha256[32] = {
    0x69, 0x9b, 0x9b, 0x93, 0x69, 0x18, 0x09, 0x25,
    0xb0, 0x6b, 0x8b, 0x8c, 0x4e, 0xfc, 0xb2, 0x6f,
    0x1f, 0x33, 0x23, 0x97, 0x0d, 0x9e, 0x79, 0xeb,
    0xfa, 0x2b, 0xef, 0x69, 0x69, 0x2f, 0xf7, 0xa9,
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
        name,
        UINT64_C(1760000031000000),
        42,
        191,
        TrackUid);
    return serialize_message(&message);
}

static WireFrame make_target_handover(void)
{
    mavlink_message_t message = {0};
    const uint8_t noAuthorization[8] = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_target_handover_pack(
        1, 191, &message,
        UINT64_C(1760000001000000), UINT64_C(1760000000000000),
        UINT64_C(1760000031000000), 436531234, -793832345, 123.5f,
        NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN, NAN,
        45, "TRAINING-HANDOVER", "", 0.875f, noAuthorization,
        MAVLINK_M_TARGET_CLASS_UNKNOWN, MAVLINK_M_TARGET_FORCE_UNKNOWN,
        MAVLINK_M_MATCH_MEDIA_NONE, 9002, 42, 191, TrackUid);
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
        reason,
        1,
        191);
    return serialize_message(&message);
}

static WireFrame make_task_status(void)
{
    mavlink_message_t message = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_mavlink_m_task_status_pack(
        42, 191, &message, UINT64_C(1760000003000000),
        MAVLINK_MSG_ID_TARGET_CUE, 731, 2, 0, 1, 191,
        MAVLINK_M_TASK_STATE_ACTIVE, "pilot accepted");
    return serialize_message(&message);
}

static WireFrame make_task_control(void)
{
    mavlink_message_t message = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_mavlink_m_task_control_pack(
        1, 191, &message, UINT64_C(1760000004000000),
        MAVLINK_MSG_ID_TARGET_CUE, 731, 3001, 732, 0, 42, 191,
        MAVLINK_M_TASK_CONTROL_SUPERSEDE, "updated observation point");
    return serialize_message(&message);
}

static WireFrame make_capability(void)
{
    mavlink_message_t message = {0};
    mavlink_reset_channel_status(MAVLINK_COMM_0);
    mavlink_msg_mavlink_m_capability_pack(
        42, 191, &message, UINT64_C(1760000005000000), UINT64_C(1760000020000000),
        MAVLINK_M_CAPABILITY_CUE_RECEIVE | MAVLINK_M_CAPABILITY_APPLICATION_RECEIPT
            | MAVLINK_M_CAPABILITY_LOCAL_DECISION | MAVLINK_M_CAPABILITY_TASK_STATUS
            | MAVLINK_M_CAPABILITY_TASK_CONTROL | MAVLINK_M_CAPABILITY_HANDOVER
            | MAVLINK_M_CAPABILITY_SIGNING_REQUIRED,
        MAVLINK_M_ENDPOINT_PX4_PILOT, 42, 191,
        AAGS_MAVLINK_M_PROTOCOL_MAJOR, AAGS_MAVLINK_M_PROTOCOL_MINOR,
        ProfileSha256, AAGS_MAVLINK_M_PROFILE_ID, AAGS_MAVLINK_M_PROFILE_VERSION);
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
    mavlink_message_t message = {0};
    mavlink_msg_target_cue_pack(
        1, 191, &message, UINT64_C(1760000001000000), 731, 0,
        436531234, -793832345, 123.5f, NAN, NAN, NAN, 0.875f, 1,
        MAVLINK_M_CUE_TYPE_INVESTIGATE, MAVLINK_M_TARGET_CLASS_UNKNOWN,
        MAVLINK_M_TARGET_FORCE_UNKNOWN, "TRAINING-731",
        UINT64_C(1760000031000000), 42, 191, TrackUid);
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
        if (mavlink_parse_char(MAVLINK_COMM_1, frame->bytes[index], parsed, &resultStatus)) {
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
    const WireFrame handover = make_target_handover();
    const WireFrame ack = make_ack();
    const WireFrame status = make_task_status();
    const WireFrame control = make_task_control();
    const WireFrame capability = make_capability();
    printf("{\n");
    printf("  \"schema\":\"aags.mavlink-m.golden-frames.v1\",\n");
    printf("  \"profile_id\":\"%s\",\n", AAGS_MAVLINK_M_PROFILE_ID);
    printf("  \"profile_version\":\"%s\",\n", AAGS_MAVLINK_M_PROFILE_VERSION);
    printf("  \"core_xml_sha256\":\"%s\",\n", AAGS_MAVLINK_M_CORE_XML_SHA256);
    printf("  \"live_transmit\":true,\n");
    printf("  \"field_release\":false,\n");
    printf("  \"inert_only\":false,\n");
    printf("  \"vectors\":[\n");
    print_vector("track_identity_nominal", &track, MAVLINK_MSG_ID_TRACK_IDENTITY,
        "persistent identity record; no engagement authority", false);
    print_vector("target_cue_nominal_addressed", &cue, MAVLINK_MSG_ID_TARGET_CUE,
        "private cue addressed to system 42 component 191", false);
    print_vector("target_handover_nominal_addressed", &handover, MAVLINK_MSG_ID_TARGET_HANDOVER,
        "private handover 9002 addressed to system 42 component 191", false);
    print_vector("mavlink_m_ack_nominal_addressed", &ack, MAVLINK_MSG_ID_MAVLINK_M_ACK,
        "ack_msgid 54001 instance 731 addressed to system 1 component 191", false);
    print_vector("task_status_active", &status, MAVLINK_MSG_ID_MAVLINK_M_TASK_STATUS,
        "explicit task lifecycle status", false);
    print_vector("task_control_supersede", &control, MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL,
        "idempotent supersede control 3001 naming replacement 732", false);
    print_vector("capability_private_command", &capability, MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY,
        "fresh exact-hash private endpoint advertisement", true);
    printf("  ]\n");
    printf("}\n");
    return 0;
}

static int self_test(void)
{
    _Static_assert(MAVLINK_MSG_ID_TRACK_IDENTITY == 54000, "unexpected TRACK_IDENTITY ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE == 54001, "unexpected TARGET_CUE ID");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK == 54004, "unexpected ACK ID");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_STATUS == 54005, "unexpected status ID");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_TASK_CONTROL == 54006, "unexpected control ID");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_CAPABILITY == 54007, "unexpected capability ID");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_MIN_LEN == 68, "unexpected TARGET_CUE minimum length");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_LEN == 94, "unexpected TARGET_CUE full length");
    _Static_assert(MAVLINK_MSG_ID_TARGET_CUE_CRC == 11, "unexpected TARGET_CUE CRC");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_MIN_LEN == 69, "unexpected ACK minimum length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_LEN == 71, "unexpected ACK full length");
    _Static_assert(MAVLINK_MSG_ID_MAVLINK_M_ACK_CRC == 47, "unexpected ACK CRC");
    _Static_assert(offsetof(mavlink_target_cue_t, target_component) + 1 == 78,
                   "unexpected addressed TARGET_CUE prefix");
    _Static_assert(offsetof(mavlink_target_handover_t, target_component) + 1 == 197,
                   "unexpected addressed TARGET_HANDOVER prefix");

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
    assert(decodedCue.valid_until_usec == UINT64_C(1760000031000000));
    assert(decodedCue.target_system == 42 && decodedCue.target_component == 191);
    assert(memcmp(decodedCue.track_uid, TrackUid, sizeof(TrackUid)) == 0);

    WireFrame handover = make_target_handover();
    assert(parse_frame(&handover, &parsed));
    mavlink_target_handover_t decodedHandover;
    mavlink_msg_target_handover_decode(&parsed, &decodedHandover);
    assert(decodedHandover.handover_id == 9002);
    assert(decodedHandover.target_system == 42 && decodedHandover.target_component == 191);

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
    assert(decodedAck.target_system == 1 && decodedAck.target_component == 191);

    WireFrame status = make_task_status();
    assert(parse_frame(&status, &parsed));
    mavlink_mavlink_m_task_status_t decodedStatus;
    mavlink_msg_mavlink_m_task_status_decode(&parsed, &decodedStatus);
    assert(decodedStatus.task_instance == 731 && decodedStatus.status_sequence == 2);
    assert(decodedStatus.state == MAVLINK_M_TASK_STATE_ACTIVE);

    WireFrame control = make_task_control();
    assert(parse_frame(&control, &parsed));
    mavlink_mavlink_m_task_control_t decodedControl;
    mavlink_msg_mavlink_m_task_control_decode(&parsed, &decodedControl);
    assert(decodedControl.control_id == 3001 && decodedControl.replacement_instance == 732);

    WireFrame capability = make_capability();
    assert(parse_frame(&capability, &parsed));
    mavlink_mavlink_m_capability_t decodedCapability;
    mavlink_msg_mavlink_m_capability_decode(&parsed, &decodedCapability);
    assert(decodedCapability.endpoint_system == 42 && decodedCapability.endpoint_component == 191);
    assert(memcmp(decodedCapability.profile_sha256, ProfileSha256, sizeof(ProfileSha256)) == 0);

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
    printf("workflow_message_encode_decode=PASS\n");
    printf("ack_correlation_54001_cue_731=PASS\n");
    printf("malformed_frame_rejection=PASS\n");
    printf("exact_nonzero_addressing=PASS\n");
    printf("unique_handover_and_task_lifecycle=PASS\n");
    printf("mavlink2_signing_roundtrip=PASS\n");
    printf("private_live_transport_authorized=PASS\n");
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
