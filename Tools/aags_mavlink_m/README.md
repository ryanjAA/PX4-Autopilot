# AAGS MAVLink-M PX4 endpoint

This branch carries the PX4 receiving endpoint for the exact provisional AAGS
profile supplied in `references.zip`:

- profile: `aags-handoff-54xxx-core`
- version: `handoff-54xxx-2026-07-13`
- canonical core XML SHA-256:
  `8ab02215d036f454bf76fee9d73985fa639f2b8ca9509bf24cc51b0cb35d3b4b`
- wire protocol: MAVLink 2

The dialect file is
`src/modules/mavlink/mavlink/message_definitions/v1.0/mavlink_m.xml`.
PX4's default MAVLink dialect is changed to `mavlink_m`, including SITL.
The previously incomplete vendored generator is pinned to pymavlink 2.4.49,
includes its fixed C templates, and runs with `PYTHONHASHSEED=0`. All 24
generated MAVLink-M message headers are byte-identical to the AAGS artifacts.
PX4 retains its own `common.xml` dependency so v1.14 platform messages and
extensions remain available; consequently the whole aggregate header tree is
not claimed to have the AAGS/QGC fielded-common tree hash. The custom protocol
headers and workflow frames are the cross-repository conformance surface.

## Contract boundary

The supplied partner manifest is provisional, has `field_release=false`, and
has live AAGS `TARGET_CUE` transmission disabled. Its workflow allowlist is
only `TRACK_IDENTITY`, `TARGET_CUE`, and `MAVLINK_M_ACK`.

The current `TARGET_CUE` and `MAVLINK_M_ACK` payloads have no recipient
component fields. The profile has no task-lifecycle message, cue expiry,
capability advertisement, cancellation, or transport-authentication contract.
`TARGET_HANDOVER` has neither a unique offer ID nor approved workflow status.
PX4 therefore does not claim addressed-mesh or field conformance:

- `MAV_M_MODE=0` is the default and ignores all cue traffic.
- Mode 1 is a point-to-point development compatibility mode bound to one
  MAVLink instance and one exact sender system/component.
- `TARGET_HANDOVER` returns `UNSUPPORTED` with ACK instance zero. PX4 does not
  misuse `target_set_id` as a handover ID.
- No custom message outside the current allowlist is transmitted. Normal PX4
  heartbeat/position telemetry remains the vehicle-discovery source.
- A real shared mesh must wait for the addressed-cue/ACK PCR and matching AAGS
  encoder. Link and sender filtering cannot replace payload addressing.

The included endpoint tool can exercise the PX4 receiver over a dedicated
bench link before the AAGS live-send gate is approved. It is not an alternate
field release.

## Behavior implemented on PX4

A valid cue from the configured link and sender is checked for:

- MAVLink 2 message ID, generated layout, length, and CRC;
- exact configured packet sender and a nonzero sensing-platform `origin_sysid`;
- nonzero cue ID and immutable payload for that ID;
- UTC timestamp age/future bounds;
- WGS84 bounds, finite/NaN rules, confidence range, cue intent, and enum range;
- a bounded two-item inbox and an eight-item durable replay/terminal cache.

Packet sender identity and sensing-platform identity are kept separate. The
configured `MAV_M_SRC_SYS`/`MAV_M_SRC_CMP` identify the AAGS component that
encoded the packet. `TARGET_CUE.origin_sysid` identifies the sensing platform
described by the canonical field and may differ when AAGS relays a contact.

The approved `TRACK_IDENTITY` workflow message is accepted from that same
configured packet source. A fresh, nonzero `target_set_id` can bind its stable
track UUID to a cue carrying the same target-set and sensing-origin values. The
identity enriches display/audit state only; it cannot authorize navigation or
engagement. A target-set collision with a different UUID is ignored.

`RECEIVED` is sent only after an atomic, CRC-protected state commit to
`PX4_STORAGEDIR/mavlink_m_state.bin`. The persisted record is bound to the
profile hash, MAVLink instance, and sender identity. Pending/active state is
restored after reboot. A changed payload reusing an ID is rejected; identical
replays repeat the current ACK.

Because `TARGET_CUE` has no expiry field, `MAV_M_MAX_AGE` is also used as its
local fail-closed lifetime measured from the cue UTC timestamp. Default is 30
seconds. Expiry is persisted before the `EXPIRED` ACK.

The local pilot decision input is deliberately restricted to a physical
`input_rc` source. MAVLink `RC_CHANNELS_OVERRIDE`, unknown input, stale input,
loss, and failsafe cannot accept or reject a cue. The switch must cross center
after boot and after every decision:

- low (`<= MAV_M_RC_REJ`): reject the oldest pending cue; if none is pending,
  abort the active cue;
- center: arm exactly one subsequent decision edge;
- high (`>= MAV_M_RC_ACC`): accept the oldest pending cue;
- accepting while another cue is active queues the new cue instead of
  overwriting the active one.

Acceptance only changes this endpoint's task-display state. The handler never
publishes a position setpoint, mission item, vehicle command, flight-mode or
arming command, payload command, or actuator command.

The `mavlink_m_target_status` uORB topic exposes pending/active/queued/terminal
state plus target bearing, relative bearing, and range. A common MAVLink
`DEBUG_VECT` named `AAGS_TGT` exports absolute bearing in `x`, relative bearing
in `y`, and range metres in `z` at 5 Hz for an OSD/GCS adapter. This telemetry
does not command the aircraft.

## Vehicle setup

Build the exact flight-controller target that will be installed. For example:

```sh
make px4_fmu-v5_default
```

Keep the endpoint disabled while configuring it. On the PX4 shell:

```text
mavlink status
param set MAV_M_MODE 0
param set MAV_M_INST 0
param set MAV_M_SRC_SYS 255
param set MAV_M_SRC_CMP 190
param set MAV_M_RC_CH 5
param set MAV_M_RC_REJ 1300
param set MAV_M_RC_ACC 1700
param set MAV_M_MAX_AGE 30
param save
```

`MAV_M_INST` is the zero-based instance printed by `mavlink status`. Set the
source IDs to the AAGS MAVLink sender identity; AAGS currently obtains these
from its MAVLink protocol settings. Select a dedicated serial/radio instance,
not a shared vehicle mesh. Verify MAVLink 2 end to end and test the configured
physical RC switch, receiver loss, and failsafe on the bench.

Only after those checks, enable compatibility mode and reboot:

```text
param set MAV_M_MODE 1
param save
reboot
```

For a network link, the normal `MAV_n_*` settings still control UDP/serial
transport. The compatibility mode adds application filtering; it does not add
encryption, authentication, signing, or payload recipient fields.

## SITL and bench probe

Build and run the in-process simulator:

```sh
make px4_sitl_default
make px4_sitl sihsim_quadx
```

Configure `MAV_M_MODE=1` at the PX4 prompt. The default SITL instance 0 listens
on UDP 18570 and sends to UDP 14550. In another terminal:

```sh
Tools/aags_mavlink_m/endpoint_tool.py track \
  --target-set 45 --origin-system 1 \
  --track-uid 00112233-4455-6677-8899-aabbccddeeff

Tools/aags_mavlink_m/endpoint_tool.py cue \
  --instance 731 --target-set 45 --origin-system 1 \
  --lat 45.4671 --lon -73.7578 --alt 50 \
  --name "training cue"
```

Expected response:

```text
ACK msgid=54001 instance=731 ... result=RECEIVED reason='stored pending pilot decision'
```

Inspect PX4's normalized state with:

```text
listener mavlink_m_target_status 1
```

For a dedicated serial radio instead of SITL UDP:

```sh
Tools/aags_mavlink_m/endpoint_tool.py \
  --serial /dev/ttyUSB0 --baud 57600 cue \
  --lat 45.4671 --lon -73.7578 --alt 50
```

The tool generates its Python binding from the checked-in XML and refuses to
run if the hash differs. Install the repository Python requirements first.

## Verification

The C conformance binary is built with SITL testing enabled:

```sh
build/px4_sitl_default/mavlink_m_conformance_test --self-test
build/px4_sitl_default/mavlink_m_conformance_test --golden
```

The golden output must be byte-identical to the AAGS
`mavlink-m/golden/workflow-frames.json` file. The live endpoint probe should
also cover wrong-source silence, immutable-ID collision rejection, queue-full
rejection, restart restore, local expiry, and rejection of remote RC override.

Before any shared-network or real-flight release, the partner must provide and
both repositories must adopt the same approved addressed profile, live AAGS
encoder, ACK recipient/correlation rules, capability advertisement, task
lifecycle/cancel semantics, and transport-authentication policy. Re-run the
golden and end-to-end tests after that protocol change; do not enable the
current compatibility mode as a substitute.
