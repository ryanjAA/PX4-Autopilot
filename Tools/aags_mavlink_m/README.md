# Finalized MAVLink-M cue endpoint

This PX4 branch implements a receiving endpoint for the finalized
[`Dronecode/mavlink-military`](https://github.com/Dronecode/mavlink-military)
dialect used by AAGS.

Branch scope:

- `v1.14-CAM-MAV-M` contains the production-oriented receiver, authority
  state machine, owner decisions, ACK behavior, action policy, safety gates,
  conformance sources, endpoint tool, and ARK v6X board enablement.
- `v1.14-CAM-MAV-M-SITL` is based on `v1.14-CAM-MAV-M` and adds the
  workstation-specific macOS and current-Clang compatibility changes, Gazebo
  Classic support, SITL board enablement, and the dual-instance validation
  runner.
- Production hardware should use `v1.14-CAM-MAV-M` unless these SITL and host
  compatibility changes are specifically needed.

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

The branch board configurations opt into the `mavlink_m` dialect and
receiver. Build the target provided by the selected branch:

```sh
PATH=/tmp/px4-aags-venv/bin:$PATH make px4_sitl_default
PATH=/tmp/px4-aags-venv/bin:$PATH make ark_fmu-v6x_default
```

The SITL target is supplied by `v1.14-CAM-MAV-M-SITL`. Enable the same Kconfig
option and dialect on each additional production board that needs direct AAGS
vehicle cue support.

The endpoint is still disabled at runtime by default.

## Configure PX4

First identify the zero-based instance carrying the AAGS connection:

```text
mavlink status
```

The `v1.14-CAM-MAV-M-SITL` branch supplies a dual-Gazebo runner. It uses each
vehicle's stock instance `0` link for local AAGS cue and owner-control traffic
(`MAV_M_SAME_EP=1`). It creates instance `4` later as telemetry-only
visibility for the other AAGS; that observer route is intentionally not a cue
endpoint. On any vehicle or launch topology, use the instance reported by
`mavlink status`; do not infer `MAV_M_INST` from a UDP port or MAVLink channel
number.

Then choose one of two route layouts:

- With `MAV_M_SAME_EP=0` (default), `MAV_M_INST` receives the cue from the
  sending/peer AAGS and returns the authoritative `MAVLINK_M_ACK`, while a
  distinct owner AAGS identity on `MAV_M_CTL_INST` sees the pending review
  snapshot and requests Accept/Reject.
- With `MAV_M_SAME_EP=1`, one local owner-control route offers the cue,
  sees the pending review snapshot, and requests Accept/Reject. The source,
  control system selector, exact component, MAVLink instance, and (in signed
  mode) signing link ID must match.

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

On a mesh where any authorized AAGS station may offer or decide a cue, use
`-1` for the relevant system selector:

```text
param set MAV_M_SRC_SYS -1
param set MAV_M_SRC_CMP 190
param set MAV_M_CTL_SYS -1
param set MAV_M_CTL_CMP 190
```

The wildcard matches any nonzero system ID only on `MAV_M_INST` for cues and
only on `MAV_M_CTL_INST` for decisions. The component check remains exact.
Signed mode still requires the configured signing link and key. Observer,
forwarding, and other MAVLink instances do not gain authority. With
`MAV_M_SAME_EP=1`, use the same system selector on both sides, either the same
exact system ID or `-1` for both. With `MAV_M_SAME_EP=0`, the two instances
must remain distinct. Different wildcard-authorized sources must use unique cue
IDs while their tasks remain stored because an owner decision carries no cue
source-system field.

`MAV_M_MODE=1` is unsigned lab mode. `MAV_M_MODE=2` requires MAVLink 2 signing
on the selected physical link. Frames from another MAVLink instance, a wrong
component, or a system outside an exact selector are ignored.

Network decisions are disabled by default (`MAV_M_CTL_INST=-1`,
`MAV_M_CTL_SYS=0`). With `MAV_M_SAME_EP=0`, the control instance must differ
from the cue instance. Identical exact system/component identities are rejected,
while wildcard identities may overlap because the configured routes preserve
the two roles. With `MAV_M_SAME_EP=1`, the routes and source selectors must
match; partial overlap is invalid and disables network decisions.
Same-endpoint mode changes who may present the cue, not the
acceptance safeguard: sending never auto-accepts or moves the aircraft.
Wrong-instance, broadcast-target, stale-ID, wrong-source, and malformed
decision requests are denied and never reach PX4's generic command path.

Route and source parameters fail closed even if written outside their metadata
bounds: instances must be `0..5`; cue system selectors must be `-1` or
`1..255`; owner system selectors must be `-1`, `1..255`, or `0` only when
network decisions are disabled; component IDs must be `1..255`; and mode-2
signing link IDs must be `0..255`. The two signing IDs must differ for separate
endpoints and match for the same endpoint.
Invalid values disable the affected endpoint instead of wrapping through an
8-bit cast. A corrupt `MAV_M_MAX_AGE` outside `0..600` is corrected to the
safe 30-second default.

Changing `MAV_M_INST`, `MAV_M_SRC_SYS`, or `MAV_M_SRC_CMP` invalidates the old
receiver identity and its stored tasks. If an active cue issued navigation,
PX4 first publishes a current-position stop. The selector change is rejected
and the prior values are restored if that stop or persisted-state invalidation
cannot be confirmed. A state file whose route, source selector, component, or
profile does not match the selected endpoint is deleted, so switching a
selector back later cannot resurrect an old task.

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
`MAV_M_ACTION` permission before Accept becomes available. A missing, stale, or
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
MAVLink-M dialect. For Accept, command parameter 5 selects the effect for that
exact cue: `0` uses the legacy default, `1` selects level travel, and `2`
selects intercept. PX4 rejects an explicit selection above the locally
configured `MAV_M_ACTION` permission ceiling or on a cue other than
`INVESTIGATE`. Reject commands require parameter 5 to be `0`. `COMMAND_ACK`
confirms only that PX4 queued the authorized local decision. The peer cue
sender receives the authoritative
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

Value `1` permits level travel. It samples the aircraft's current AMSL altitude
at the acceptance instant and publishes a `VEHICLE_CMD_DO_REPOSITION` target
at the cue latitude/longitude and that sampled altitude. In other words, PX4
creates an invisible destination waypoint level with the aircraft. It
deliberately ignores `TARGET_CUE.alt`, so a ground target cannot make a normal
transit descend toward terrain or climb.

MAVLink-M has no `INTERCEPT` cue-type enum. The finalized values are
`INVESTIGATE`, `OBSERVE`, and `MARK`; this implementation does not invent a
fourth wire value. If a trusted AAGS source must explicitly allow vertical
intercept navigation, configure the receiving aircraft locally:

```text
param set MAV_M_ACTION 2
param set MAV_M_INT_RAD 25
param set MAV_M_INT_DWL 3
param set MAV_M_INT_DZ 100
param save
```

Value `2` permits either level travel or intercept for each accepted cue. A
finite-altitude `INVESTIGATE` cue accepted with effect `2` uses one guarded
approach and exact crossing:

1. PX4 sends a PX4-internal fly-through request to Navigator at the cue
   latitude/longitude and `TARGET_CUE.alt`.
2. For fixed wing, Navigator calculates the required horizontal slope distance
   from the shallower limit imposed by `FW_P_LIM_MIN` or `FW_P_LIM_MAX` and
   `FW_T_SINK_MAX` or `FW_T_CLMB_MAX`. The true-airspeed bound is twice
   `FW_AIRSPD_MAX`. Navigator adds the horizontal wind magnitude and three
   standard deviations from the north/east wind variances to obtain a
   conservative ground-speed bound. It converts both pitch and vertical-rate
   limits to ground-relative gradients, then applies a 0.8 margin. The wind
   sample must be no more than 2 seconds old, both components and variances
   must be finite, and both variances must be nonnegative. Missing or invalid
   wind data rejects fixed-wing Intercept. A calculated approach longer than
   100 km is also rejected. The aircraft flies level to the calculated
   approach entry, then follows the bounded slope to the cue. If the cue is
   too close for that slope, Navigator places the entry behind the aircraft on
   the same inbound line. The aircraft first flies away level, turns, and then
   approaches the cue. At the entry, fixed wing remains on the level leg until
   fresh valid horizontal ground velocity is at least 3 m/s and aligned within
   60 degrees of the entry-to-target vector. An outbound pass cannot promote
   the slope. A cue at the current coordinate uses current heading to construct
   the entry. Multicopters fly directly to the cue coordinate and altitude.
3. Fixed-wing altitude interpolation is anchored to the exact target plane for
   this private setpoint. It does not finish at the ordinary waypoint
   acceptance radius. Navigator accepts the first crossing only when both the
   horizontal and vertical misses are no more than 5 m. A PX4-local,
   token-matched Navigator ACK proves which exact request completed.
4. After crossing, the aircraft enters a target-centered loiter at cue
   altitude. After it remains continuously inside the effective arrival radius
   for `MAV_M_INT_DWL`, PX4 reports the intercept as complete. It does not send
   a second altitude reposition.

`MAV_M_INT_RAD` is only the configured post-crossing dwell radius. It never
changes the fixed 5 m horizontal and vertical fly-through hit tests. For fixed wing, the
effective post-crossing dwell radius is
`max(MAV_M_INT_RAD, abs(NAV_LOITER_RAD) + 10 m)` so an aircraft established on
its commanded loiter circle can satisfy the dwell. Multicopters use
`MAV_M_INT_RAD`. Leaving the radius resets the complete dwell. The mandatory
target-plane crossing and matching internal completion ACK are separate from
this radius and always occur first. The request token remains inside PX4 and
the private command and ACK are not emitted on MAVLink.

If the first bounded crossing misses, Navigator still establishes the planned
target-centered loiter so the aircraft remains near the cue, but it holds the
interpolated crossing altitude or current aircraft AMSL if interpolation is
unavailable. It clears the exact-altitude flag and never continues descending
toward cue altitude after a failed crossing. PX4 reports the intercept as
`ABORTED`, records the miss, and permanently blocks completion for that
acceptance. The operator must abort it and accept a fresh cue before another
attempt. A safety, policy, source, or setpoint failure instead cancels the
private navigation request and commands a hold at the aircraft's current
position.
`MAV_M_INT_DZ` is the maximum
permitted absolute difference between acceptance altitude and cue altitude.
A larger difference prevents that intercept.

Explicit effect `2` requires a finite cue altitude and is denied when the
altitude is NaN. The effect selection belongs to the receiver-local acceptance
command, not `TARGET_CUE`: the finalized message has no field that
distinguishes level travel from intercept. Set `MAV_M_ACTION=1` to permit only
level movement. Set it to `2` only when that aircraft is authorized to let the
local operator choose either level travel or intercept. Effect `1` never uses
the cue altitude even if a sender includes one. Effect `0` preserves
compatibility by using the current `MAV_M_ACTION` value as the default for that
acceptance. Under that legacy default, a NaN altitude with
`MAV_M_ACTION=2` remains a one-phase level reposition.

The sender must resolve altitude provenance before transmission because
`TARGET_CUE` carries only one MSL altitude float, not an altitude-source enum.
The AAGS cascade is: operator-entered AMSL altitude, otherwise terrain AMSL,
otherwise the explicit `0 m AMSL` fallback, and finally NaN only when the
altitude remains unknown. PX4 cannot distinguish those sources after receipt;
it sees only the resulting finite value or NaN.

Both navigation modes publish movement only after durable receipt and local
acceptance, while the vehicle is already airborne, armed, and in Hold
(`AUTO_LOITER`), with fresh aircraft global position and land-detector state
showing no landed, maybe-landed, ground-contact, or freefall condition, and
with no active failsafe or failure-detector indication. If those conditions
are not already true, PX4 leaves the cue Pending, returns
`MAVLINK_M_ACK_RECEIVED` with a `movement blocked` reason, and publishes no
motion command. The operator must make a fresh acceptance decision after the
vehicle is ready.

For a finite-altitude action-2 cue, PX4 continuously rechecks the exact active
cue, expiry, source freshness, action and intercept parameters, armed
airborne-Hold state, failsafe/failure state, global position, vertical limit,
and ownership of the level-entry and exact-target setpoints. While the level
entry is active, Loiter revalidates both that endpoint and the target on every
cycle. PX4 1.14 cannot prove that the whole fixed-wing approach corridor stays
inside a restrictive geofence, so exact Intercept is rejected whenever an
actual polygon, circle, `GF_MAX_HOR_DIST`, or `GF_MAX_VER_DIST` restriction is
configured and `GF_ACTION` is `LOITER`, `RTL`, `TERMINATE`, or `LAND`.
PX4's default `GF_ACTION=LOITER` does not create a fence by itself and does not
block Intercept when all four restrictions are absent. `NONE` and `WARN` allow
Intercept with a configured fence. After crossing PX4 requires ownership of
the promoted target-centered loiter. Task abort or expiry, mode exit, disarm, failsafe,
source staleness, setpoint override, action or intercept-policy change, or
restart permanently changes that acceptance to intercept phase `ABORTED`.
Restoring a gate does not resume the dwell. Restarted active assignments remain
visible for audit but never resume the intercept.

`listener mavlink_m_target_status` reports `intercept_phase` as `NONE`,
`TRANSIT`, `DWELL`, `COMPLETE`, or `ABORTED`. The owner link sends
the same numeric value as `NAMED_VALUE_INT AAGS_IPHS`.

A movement cue is accepted only when PX4 can issue its requested navigation
command at that exact instant. A failed armed, airborne, Hold, failsafe,
position-freshness, source-freshness, or intercept-altitude gate returns
`MAVLINK_M_ACK_RECEIVED` with a `movement blocked` reason and leaves the cue
Pending. It does not become Active
and cannot begin moving later after arming, takeoff, entering Hold, or
restart/restore. The operator must make a fresh acceptance decision after the
gate is restored. OBSERVE, MARK, handover, and receipt-only INVESTIGATE remain
valid non-movement acceptances and do not require the flight-state gate.
Invalid or over-permission local effects also leave the durable cue Pending and
repeat `MAVLINK_M_ACK_RECEIVED` with the blocking reason. `FAILED`, `EXPIRED`,
`REJECTED`, and `ACCEPTED` are emitted only when PX4's durable task state agrees
with that result.

Receiving a new cue while another cue is Active does not change either task.
The new cue remains Pending until an explicit local acceptance. On acceptance,
PX4 first validates the new cue and every movement gate. If it cannot execute
the requested effect, the old cue remains Active and the new cue remains
Pending/Received. If preflight succeeds, one durable transition marks the old
cue Aborted and the new cue Active. A moving replacement publishes its new
navigation command directly, without an intervening hold. A nonmoving
replacement stops navigation issued by the old cue. PX4 then sends the old
cue's `REJECTED` ACK with a superseded reason before sending the new cue's
`ACCEPTED` ACK. A persistence or command-publication failure restores the old
Active cue and leaves the new cue Pending when the replacement command was not
published and rollback can be persisted. If command publication succeeds but
its command-state write fails, PX4 requests a navigation stop, retains the
durable new `Active/Received` stage, and reports that an explicit Abort is
required. It does not falsely restore the old task after its navigation was
replaced. If the stop publication is unconfirmed, the new command flags remain
visible so Abort can retry it.

A restart during the narrow `Active/Received` stage cannot resume movement.
PX4 recognizes a movement task with no committed command flags, restores it as
Pending, and requires a new explicit acceptance. If only the MAVLink module
restarts while Navigator still holds the superseded command, PX4 recovers the
command-bearing predecessor from the newest terminal record and retries a
current-position stop.

For movement, `ACCEPTED` proves that PX4 passed its receiver-side safety gates,
published the navigation request, and durably recorded that publication. It
does not promise that Navigator will remain able to complete the route after
conditions change. A later intercept rejection, cancellation, safety-gate
loss, or target miss remains visible as failed or aborted status. Legacy
accepted-queue state is restored as Pending and never resumes movement without
a fresh local decision.

PX4 1.14 Commander requires the
`MAV_DO_REPOSITION_FLAGS_CHANGE_MODE` bit (`param2=1`) to acknowledge
the level Travel command.
The private fly-through command is internal to PX4 and cannot be serialized as
MAVLink `COMMAND_LONG`. The MAVLink-M flight-state gate already requires fresh
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
7. On a movement Accept, status changes to ACTIVE and Vehicle 45 flashes red
   only after PX4 issues the safeguarded reposition command. A blocked command
   returns RECEIVED with the blocking reason and the cue remains Pending for a
   fresh decision.

The reverse path is symmetrical: AAGS 253 hands a contact to AAGS 254, then
AAGS 254 creates and accepts the local cue for SYS44. To test only the local
vehicle leg, start at step 3 with any contact already stored in the owning
AAGS.

If `MAV_M_ACTION=1` or `2`, the aircraft must already be airborne, armed, safe,
and in Hold at the instant Accept is selected. A failed motion gate declines
the acceptance with a repeated RECEIVED result and leaves the cue Pending. PX4 never reports the movement cue as
Accepted and never starts it later without another operator decision.

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

The conformance source is portable. The command below uses the SITL branch or
another configured host test build:

```sh
PATH=/tmp/px4-aags-venv/bin:$PATH \
  cmake --build build/px4_sitl_default \
  --target mavlink_m_conformance_test

build/px4_sitl_default/mavlink_m_conformance_test --self-test
build/px4_sitl_default/mavlink_m_conformance_test --golden
```

The generated cue, handover, and ACK frames must be byte-identical to AAGS
`mavlink-m/golden/workflow-frames.json`.
