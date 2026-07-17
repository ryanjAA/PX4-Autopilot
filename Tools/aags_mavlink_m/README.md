# Private AAGS to PX4 MAVLink-M command profile

This branch implements the owner-authorized private command profile in
both AAGS and PX4. It supports live cue and handover transport. PX4 can also
publish navigation, flight-mode, arming, and payload ROI vehicle commands after
the task has been durably received, locally accepted, and made active. Command
authority is disabled by default and controlled by `MAV_M_ACTION`.

Profile identity:

- ID: `aags-private-command-54xxx`
- version: `private-command-2026-07-17-v2`
- canonical XML SHA-256:
  `699b9b9369180925b06b8b8c4efcb26f1f3323970d9e79ebfa2bef69692ff7a9`
- provisional private IDs: `54000` through `54007`
- wire protocol: MAVLink 2

The implemented workflow uses exact nonzero system/component addresses,
addressed ACKs, UUID track identity carried on every cue/handover, explicit
task status, idempotent cancel/supersede controls, unique handover IDs, fresh
exact-hash capability advertisements, durable application storage, and bounded
retries.

## Supported build targets

The private endpoint is an explicit board opt-in. This branch pins the same
`mavlink_m` dialect and endpoint implementation for both supported targets:

```sh
make px4_sitl_default
make px4_sitl_test
cmake --build build/px4_sitl_test --target mavlink_m_conformance_test
ctest --test-dir build/px4_sitl_test -R '^mavlink_m_conformance$' --output-on-failure
make ark_fmu-v6x_default
```

The focused test commands use the `px4_sitl_test` configuration and run the
profile conformance case through CTest.

The ARK v6X flashable artifact is
`build/ark_fmu-v6x_default/ark_fmu-v6x_default.px4`. Other boards retain the
ordinary `common` dialect unless their defconfig explicitly selects both
`CONFIG_MAVLINK_DIALECT="mavlink_m"` and
`CONFIG_MAVLINK_M_PRIVATE_PROFILE=y`.

Selecting the endpoint at build time does not enable it at runtime:
`MAV_M_MODE=0` remains the fail-closed default on SITL and ARK v6X. Command
authority also remains disabled until `MAV_M_ACTION` is set.

## PX4 configuration

`MAV_M_MODE=0` is the default and ignores all private task traffic.

- `MAV_M_MODE=1`: private lab mode, unsigned.
- `MAV_M_MODE=2`: private physical-link mode. MAVLink 2 signing is
  mandatory for every frame on the selected link.

Configure the exact MAVLink instance and AAGS packet source before enabling:

```text
param set MAV_M_MODE 0
param set MAV_M_INST 0
param set MAV_M_SRC_SYS 255
param set MAV_M_SRC_CMP 190
param set MAV_M_RC_CH 5
param set MAV_M_RC_REJ 1300
param set MAV_M_RC_ACC 1700
param set MAV_M_MAX_AGE 30
param set MAV_M_ACTION 0
param save
```

`MAV_M_INST` is the zero-based instance shown by `mavlink status`. The local
PX4 recipient is its current `MAV_SYS_ID` and autopilot component ID (normally
component `1`). A cue for any other payload address is silently isolated.

The physical RC control is local authority only: center arms one decision,
high accepts the oldest pending task, and low rejects it (or aborts the active
task when no pending task exists). MAVLink RC override, stale/lost input, and
failsafe input cannot decide a task.

For headless SITL or a local vehicle console, the same operator decision can be
made without an RC switch. This publishes only an internal, one-second-lived
uORB event on that vehicle; it is not a MAVLink command or RC override:

```text
mavlink task accept
mavlink task reject
mavlink task accept 731 54001
```

The optional instance and message ID pin the decision to one exact pending
task. A mismatched, stale, or differently addressed decision is ignored.

To let an accepted active task command the vehicle, set `MAV_M_ACTION` to the
sum of the required bits:

- `1`: publish `VEHICLE_CMD_DO_REPOSITION` to the accepted target.
- `2`: publish `VEHICLE_CMD_DO_SET_MODE` to `AUTO_LOITER`.
- `4`: publish `VEHICLE_CMD_COMPONENT_ARM_DISARM` with arm and normal checks.
- `8`: publish payload ROI and mount GPS-point commands to the target.

For the full next-phase behavior requested here:

```text
param set MAV_M_ACTION 15
param save
reboot
```

Cancel, reject, supersede, and expiry clear payload ROI when payload authority
was used. If navigation authority was used and a fresh global position is
available, PX4 also sends a reposition command to the current position so the
guided target does not remain pointed at the cancelled task. Cancel does not
auto-disarm.

For unsigned lab use:

```text
param set MAV_M_MODE 1
param save
reboot
```

## Signing-key provisioning

Generate one random 32-byte key outside both repositories and provision the
same bytes at both endpoints. Never commit it.

```sh
openssl rand 32 > mavlink_m_signing.key
chmod 600 mavlink_m_signing.key
```

On flight hardware, copy it to:

```text
/fs/microsd/mavlink_m_signing.key
```

On PX4 SITL it belongs at `<rootfs>/mavlink_m_signing.key`. PX4 also accepts
exactly 64 hexadecimal characters. Then configure a unique signing link ID and
enable physical mode:

```text
param set MAV_M_LNK_ID 7
param set MAV_M_MODE 2
param save
reboot
```

Use a dedicated physical task link in mode 2: PX4 rejects every unsigned frame
on the selected link and signs every outbound frame on it.

Start AAGS with the same key and explicit private runtime enablement:

```sh
AAGS_MAVLINK_M_PRIVATE_ENABLE=1 \
AAGS_MAVLINK_M_SIGNING_MODE=required \
AAGS_MAVLINK_M_SIGNING_KEY_FILE=/secure/path/mavlink_m_signing.key \
./AAGS
```

For an isolated unsigned lab link, omit the signing variables but retain
`AAGS_MAVLINK_M_PRIVATE_ENABLE=1`.

## Using AAGS

Build AAGS with:

```sh
qmake qgroundcontrol.pro CONFIG+=AAGS_MAVLINK_M CONFIG+=AAGS_TACTICAL_CONTACTS
make -j4
```

In the cue composer, select an exact destination and link, choose either an
observation action or `Handover / transfer`, and complete the live-send
confirmation. The task roster shows receipts and explicit lifecycle status.
`CANCEL` sends an idempotent addressed control. The network inbox can
accept/reject incoming cues or handovers and report READY, COMPLETE, or ABORT.

Capability advertisements are emitted every five seconds. A task is not sent
or accepted without a fresh advertisement matching the exact profile hash,
version, signing policy, and required capabilities.

## Bench command-line probe

The tool generates its Python dialect from the checked-in XML and refuses a
hash mismatch. It sends a capability advertisement before each task and packs
the task once for at most two additional byte-identical lab retries.

```sh
Tools/aags_mavlink_m/endpoint_tool.py \
  --source-system 255 --source-component 190 \
  --target-system 1 --target-component 1 \
  cue --instance 731 \
  --track-uid 00112233-4455-6677-8899-aabbccddeeff \
  --lat 45.4671 --lon -73.7578 --alt 50 --valid-for 30

Tools/aags_mavlink_m/endpoint_tool.py \
  --target-system 1 --target-component 1 \
  handover --instance 9002 --target-set 45 \
  --track-uid 00112233-4455-6677-8899-aabbccddeeff \
  --lat 45.4671 --lon -73.7578 --alt 50

Tools/aags_mavlink_m/endpoint_tool.py \
  --target-system 1 --target-component 1 \
  control --task-msgid 54001 --task-instance 731 \
  --action 0 --reason "operator cancel"
```

Add `--signing-key /secure/path/mavlink_m_signing.key --signing-link-id 7`
before the subcommand for a signed physical-link probe.

MAVLink 2 signing deliberately rejects a previously authenticated signed
envelope as a replay. The immutable application payload/control ID remains
identical, but a post-delivery retry needs a fresh signing timestamp. Unsigned
lab tests additionally prove whole-frame byte identity.

## Verification

```sh
build/px4_sitl_default/mavlink_m_conformance_test --self-test

Tools/aags_mavlink_m/run_sitl_acceptance.py \
  --json-output /tmp/px4-aags-private-sitl.json

Tools/aags_mavlink_m/run_sitl_acceptance.py --signed \
  --json-output /tmp/px4-aags-private-sitl-signed.json
```

The suite uses an isolated PX4 instance and UDP link, exercises cue, handover,
cancel, supersede, status, wrong-source isolation, persistence/restart,
bounded inbox behavior, RC-override rejection, expiry, and signing, and emits a
machine-readable report. Physical RC/failsafe behavior, command authority, and
the exact installed hardware target still require bench acceptance before use.
