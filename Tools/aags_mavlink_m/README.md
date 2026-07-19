# Finalized MAVLink-M cue endpoint

This PX4 branch implements a receiving endpoint for the finalized
[`Dronecode/mavlink-military`](https://github.com/Dronecode/mavlink-military)
dialect used by AAGS.

Branch scope:

- `v1.14-CAM-MAV-M` contains the production-oriented receiver, authority
  state machine, owner decisions, ACK behavior, action policy, safety gates,
  conformance sources, endpoint tool, and ARK v6X board enablement.
- `v1.14-CAM-MAV-M-SITL` is based on this branch and adds the workstation
  macOS, current-Clang, Gazebo Classic, dual-runner, and SITL board support.
- Production hardware does not require the workstation compatibility changes.

Profile identity:

- profile: `dronecode-mavlink-military`
- version: `main@23cec367`
- canonical XML SHA-256:
  `02a88f79b9b4e58b271d9b0012a06087ac4ec975c75a8b1d7ba704c43bb55a8f`
- `TARGET_CUE`: message `53001`, 68-byte payload
- `TARGET_HANDOVER`: message `53002`, 207-byte payload
- `MAVLINK_M_ACK`: message `53004`, 69-byte payload

The selected MAVLink route supplies the destination. Finalized identity, cue,
handover, and ACK payloads do not contain private target-system/component
fields. PX4 therefore consumes message IDs `53000`, `53001`, `53002`, and
`53004` locally and excludes them from generic MAVLink forwarding, even when
other MAVLink instances have forwarding enabled. Use a dedicated
non-forwarding route for AAGS. The old 54xxx capability, task-control, and
task-status messages are not part of this endpoint.

## Build

The ARK v6X board configuration in this core branch opts into the `mavlink_m`
dialect and receiver:

```sh
PATH=/tmp/px4-aags-venv/bin:$PATH make ark_fmu-v6x_default
```

Enable the same Kconfig option and dialect on each production board that needs
direct AAGS vehicle cue support.

The endpoint is still disabled at runtime by default.

## Configure PX4

First identify the zero-based instance carrying the AAGS connection:

```text
mavlink status
```

The optional `v1.14-CAM-MAV-M-SITL` branch supplies a dual-Gazebo runner. It
uses each vehicle's stock instance `0` link for local AAGS cue and
owner-control traffic (`MAV_M_SAME_EP=1`). It creates instance `4` later as
telemetry-only visibility for the other AAGS; that observer route is
intentionally not a cue endpoint. On any vehicle or launch topology, use the
instance reported by `mavlink status`; do not infer `MAV_M_INST` from a UDP
port or MAVLink channel number.

Then choose one of two exact endpoint layouts:

- With `MAV_M_SAME_EP=0` (default), `MAV_M_INST` receives the cue from the
  sending/peer AAGS and returns the authoritative `MAVLINK_M_ACK`, while a
  distinct owner AAGS identity on `MAV_M_CTL_INST` sees the pending review
  snapshot and requests Accept/Reject.
- With `MAV_M_SAME_EP=1`, one exact local-owner AAGS endpoint offers the cue,
  sees the pending review snapshot, and requests Accept/Reject. The source,
  control identity, MAVLink instance, and (in signed mode) signing link ID
  must match exactly.

For example, a vehicle receiving cues from AAGS `253/190` but owned by AAGS
`254/190` can use:

```text
param set MAV_M_MODE 1
param set MAV_M_SAME_EP 0
param set MAV_M_INST 4
param set MAV_M_SRC_SYS 253
param set MAV_M_SRC_CMP 190
param set MAV_M_CTL_INST 0
param set MAV_M_CTL_SYS 254
param set MAV_M_CTL_CMP 190
param set MAV_M_MAX_AGE 60
param set MAV_M_ACTION 0
param save
reboot
```

For a vehicle locally owned by AAGS `253/190` on MAVLink instance `0`, use:

```text
param set MAV_M_MODE 1
param set MAV_M_SAME_EP 1
param set MAV_M_INST 0
param set MAV_M_SRC_SYS 253
param set MAV_M_SRC_CMP 190
param set MAV_M_CTL_INST 0
param set MAV_M_CTL_SYS 253
param set MAV_M_CTL_CMP 190
param set MAV_M_MAX_AGE 60
param set MAV_M_ACTION 0
param save
reboot
```

This local-owner mode is the intended second half of an AAGS-to-AAGS
handover: the receiving AAGS first accepts and adds the handed-over contact,
then sends a new vehicle cue to its own PX4. PX4 still exposes that cue as
Pending and does not navigate until the operator explicitly selects
**ACCEPT VEHICLE CUE** (or makes the equivalent RC/local-console decision).

`MAV_M_MODE=1` is unsigned lab mode. `MAV_M_MODE=2` requires MAVLink 2 signing
on the selected physical link. Frames from another source or another MAVLink
instance are ignored.

Network decisions are disabled by default (`MAV_M_CTL_INST=-1`,
`MAV_M_CTL_SYS=0`). With `MAV_M_SAME_EP=0`, both the control instance and
owner identity must differ from the cue endpoint. With `MAV_M_SAME_EP=1`,
both must match exactly; partial overlap is invalid and disables network
decisions. Same-endpoint mode changes who may present the cue, not the
acceptance safeguard: sending never auto-accepts or moves the aircraft.
Wrong-instance, broadcast-target, stale-ID, wrong-source, and malformed
decision requests are denied and never reach PX4's generic command path.

Route and source parameters fail closed even if written outside their metadata
bounds: instances must be `0..5`, source/owner system and component IDs must
each be `1..255`, and mode-2 signing link IDs must be `0..255`. The two signing
IDs must differ for separate endpoints and match for the same endpoint.
Invalid values disable the affected endpoint instead of wrapping through an
8-bit cast. A corrupt `MAV_M_MAX_AGE` outside `0..600` is corrected to the
safe 30-second default.

`MAV_M_MAX_AGE` is the replay window and pending lifetime for `TARGET_CUE`.
`TARGET_HANDOVER` also carries its own explicit expiry. Setting the parameter
to zero disables the cue age window, but unset/non-UTC timestamps are still
rejected.

## What the operator sees

After PX4 durably stores a valid cue it immediately returns:

```text
MAVLINK_M_ACK result=RECEIVED
```

AAGS should show the outbound item as **Received**. On the owner AAGS, open
**Contacts → TASKS → VEHICLE CUES**. PX4 publishes a receiver-local,
cue-ID-correlated review snapshot only on `MAV_M_CTL_INST`. The card shows the
source, exact latitude/longitude, stored MSL altitude, cue type, and current
`MAV_M_ACTION` effect before Accept becomes available. A missing, stale, or
mismatched coordinate/policy snapshot disables Accept.

For diagnostics, PX4 also exposes the pending item through:

```text
listener mavlink_m_target_status
```

The status includes cue ID, source, label, coordinates, range/bearing, queue
depth, and whether a local decision is pending. PX4 also emits `DEBUG_VECT`
named `AAGS_TGT` for an OSD integration.

Select **ACCEPT VEHICLE CUE** or **REJECT** in that owner card. The UI request
uses standard targeted `COMMAND_LONG/MAV_CMD_USER_1` as a receiver-local
control channel; it does not add a task-control message to the finalized
MAVLink-M dialect. `COMMAND_ACK` confirms only that PX4 queued the authorized
local decision. The peer cue sender receives the authoritative
`MAVLINK_M_ACK ACCEPTED/REJECTED` after durable state transition, while the
owner card changes to ACTIVE. Both AAGS displays then show the aircraft
red/flashing.

The local PX4 console remains an alternative:

```text
mavlink task accept
mavlink task reject
```

An exact cue can be selected when more than one is pending:

```text
mavlink task accept 731 53001
mavlink task reject 731 53001
```

Acceptance returns `MAVLINK_M_ACK result=ACCEPTED`; rejection returns
`result=REJECTED`. AAGS uses that ACK to show the vehicle accepted the cue.

## RC acceptance switch

Set `MAV_M_RC_CH` to a physical RC channel:

```text
param set MAV_M_RC_CH 5
param set MAV_M_RC_REJ 1300
param set MAV_M_RC_ACC 1700
```

The switch is low=Reject, center=neutral, high=Accept. It must pass through
center after boot and after every decision. Lost/stale RC, failsafe input, and
MAVLink RC override cannot accept or reject a cue.

## Optional safe navigation

The default `MAV_M_ACTION=0` is receipt/ACK/display only.

To allow an already-operating vehicle to navigate laterally to a locally
accepted `INVESTIGATE` cue without changing altitude:

```text
param set MAV_M_ACTION 1
param save
```

Value `1` samples the aircraft's current AMSL altitude at the acceptance
instant and publishes a `VEHICLE_CMD_DO_REPOSITION` target at the cue
latitude/longitude and that sampled altitude. In other words, PX4 creates an
invisible destination waypoint level with the aircraft. It deliberately
ignores `TARGET_CUE.alt`, so a ground target cannot make a normal transit
descend toward terrain or climb.

MAVLink-M has no `INTERCEPT` cue-type enum. The finalized values are
`INVESTIGATE`, `OBSERVE`, and `MARK`; this implementation does not invent a
fourth wire value. If a trusted AAGS source must explicitly allow vertical
intercept navigation, configure the receiving aircraft locally:

```text
param set MAV_M_ACTION 2
param save
```

Value `2` is a local trusted-source AAGS profile. An accepted `INVESTIGATE`
cue with a finite `TARGET_CUE.alt` repositions to that AMSL altitude. A cue
whose altitude is NaN still repositions at the aircraft's acceptance-time
AMSL altitude. This is a receiver-wide policy for accepted `INVESTIGATE`
cues, not a per-cue wire instruction: the finalized message has no field that
distinguishes Level Reposition from Intercept. Select mode `1` on the receiving
aircraft for level movement and mode `2` only when that aircraft is intentionally
configured to use target altitude. Mode `1` never uses the cue altitude even if
a sender includes one.

The sender must resolve altitude provenance before transmission because
`TARGET_CUE` carries only one MSL altitude float, not an altitude-source enum.
The AAGS cascade is: operator-entered AMSL altitude, otherwise terrain AMSL,
otherwise the explicit `0 m AMSL` fallback, and finally NaN only when the
altitude remains unknown. PX4 cannot distinguish those sources after receipt;
it sees only the resulting finite value or NaN.

Both navigation modes publish only after durable receipt and local acceptance,
while the vehicle is already airborne, armed, and in Hold (`AUTO_LOITER`),
with fresh aircraft global position and land-detector state showing no landed,
maybe-landed, ground-contact, or freefall condition, and with no active
failsafe or failure-detector indication. If those conditions are not already
true, acceptance and its ACK still succeed but no motion command is published.
The check and command attempt happen once, at that explicit local acceptance.
An accepted cue does not begin moving later after arming, takeoff, entering
Hold, or restart/restore. If another cue is already active, an acceptance
attempt leaves the new cue Pending/Received and returns no false `ACCEPTED`
ACK. The active cue must first be explicitly aborted or expire, followed by a
fresh local acceptance of the pending cue. Legacy accepted-queue state is
restored as Pending for the same reason.

PX4 1.14 Commander requires the
`MAV_DO_REPOSITION_FLAGS_CHANGE_MODE` bit (`param2=1`) to acknowledge
`DO_REPOSITION`. The MAVLink-M flight-state gate already requires fresh
`AUTO_LOITER`, so this reasserts the current Hold intention rather than
selecting a different flight mode. Neither navigation mode arms, controls a
payload, authorizes engagement, or converts a handover/OBSERVE/MARK cue into
movement. `MAVLINK_M_ACK` remains the authoritative cue receipt/acceptance
decision; `COMMAND_ACK` reports the separate PX4 reposition command result.

Values from the obsolete bitmask implementation, including `15`, are
sanitized to receipt-only mode so an old parameter cannot retain arm/mode or
payload authority.

## One-computer two-AAGS SITL test

This procedure requires `v1.14-CAM-MAV-M-SITL`. Two computers are not
required. Run two AAGS processes with different MAVLink system IDs. The
runner assigns AAGS `254/190` as the local owner of SYS44 and AAGS `253/190`
as the local owner of SYS45. Both stations can display both vehicles, but
instance `4` on each PX4 is telemetry-only; cueing the other station's vehicle
directly over that observer route is deliberately rejected.

1. In AAGS 254, tag/select a contact and send an AAGS **Handover** to AAGS
   253.
2. In AAGS 253, open the Network Inbox, review the handover, and select
   **ACCEPT + ADD**. This creates a local contact; it does not yet move SYS45.
3. In AAGS 253, select that contact, choose local Vehicle 45, and send a new
   vehicle cue.
4. AAGS 253 shows the cue as **Received** after PX4 durably stores it.
5. Still in AAGS 253, open **Contacts → TASKS → VEHICLE CUES**. Review the
   coordinate, target MSL altitude, and the explicit motion policy.
6. Select **ACCEPT VEHICLE CUE** or **REJECT**.
7. On Accept, status changes to ACTIVE, Vehicle 45 flashes red, and
   `MAV_M_ACTION=1` or `2` performs the safeguarded reposition.

The reverse path is symmetrical: AAGS 253 hands a contact to AAGS 254, then
AAGS 254 creates and accepts the local cue for SYS44. To test only the local
vehicle leg, start at step 3 with any contact already stored in the owning
AAGS.

If `MAV_M_ACTION=1` or `2`, the aircraft must already be airborne, armed, safe,
and in Hold at the instant Accept is selected. Acceptance is durable even when
that motion gate fails, but PX4 deliberately does not start moving later.

The repository bench sender can isolate AAGS from the test:

```sh
Tools/aags_mavlink_m/endpoint_tool.py \
  --source-system 255 --source-component 190 \
  --udp-bind 127.0.0.1:14551 \
  --udp-destination 127.0.0.1:18671 \
  cue --instance 731 --cue-type 1 \
  --lat 45.4671 --lon -73.7578 --alt 50 \
  --name ALBATROSS-731
```

For a handover, `target_set_id` is the finalized ACK correlation instance:

```sh
Tools/aags_mavlink_m/endpoint_tool.py \
  --udp-bind 127.0.0.1:14551 \
  --udp-destination 127.0.0.1:18671 \
  handover --target-set 9002 \
  --track-uid 00112233-4455-6677-8899-aabbccddeeff \
  --lat 45.4671 --lon -73.7578 --alt 50
```

## Signed physical link

Provision the same nonzero 32-byte key at both endpoints. Do not commit it:

```sh
openssl rand 32 > mavlink_m_signing.key
chmod 600 mavlink_m_signing.key
```

PX4 reads `PX4_STORAGEDIR/mavlink_m_signing.key`. In separate-endpoint mode,
set a unique signing link ID for each physical route and enable mode 2:

```text
param set MAV_M_LNK_ID 7
param set MAV_M_CTL_LNK 8
param set MAV_M_MODE 2
param save
reboot
```

Both selected physical links then reject unsigned frames and sign their
outbound ACK/status traffic. The two signing link IDs must differ.

In same-endpoint mode, both parameters identify the one physical route and
must match:

```text
param set MAV_M_SAME_EP 1
param set MAV_M_LNK_ID 7
param set MAV_M_CTL_LNK 7
param set MAV_M_MODE 2
param save
reboot
```

## Verification

The conformance source is portable. The command below uses the optional SITL
branch or another configured host test build:

```sh
PATH=/tmp/px4-aags-venv/bin:$PATH \
  cmake --build build/px4_sitl_default \
  --target mavlink_m_conformance_test

build/px4_sitl_default/mavlink_m_conformance_test --self-test
build/px4_sitl_default/mavlink_m_conformance_test --golden
```

The generated cue, handover, and ACK frames must be byte-identical to AAGS
`mavlink-m/golden/workflow-frames.json`.
