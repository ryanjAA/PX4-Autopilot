# Finalized MAVLink-M cue endpoint

This PX4 branch implements a receiving endpoint for the finalized
[`Dronecode/mavlink-military`](https://github.com/Dronecode/mavlink-military)
dialect used by AAGS.

Branch scope:

- `v1.16-CAM-MAV-M` is based on PX4 `v1.16.1` at
  `94cb2012792b2ae89f0b147cfee53ee31ae550be`.
- The branch contains the production receiver, authority state machine, owner
  decisions, ACK behavior, action policy, safety gates, conformance sources,
  endpoint tool, production board enablement, and SITL validation tools.
- The dual-instance runner uses the normal PX4 1.16 Gazebo Harmonic support.
  It does not include the PX4 1.14 workstation compatibility patch set.

Profile identity:

- profile: `dronecode-mavlink-military`
- version: `main@23cec367`
- canonical XML SHA-256:
  `02a88f79b9b4e58b271d9b0012a06087ac4ec975c75a8b1d7ba704c43bb55a8f`
- `TARGET_CUE`: message `53001`, 68-byte payload
- `TARGET_HANDOVER`: message `53002`, 207-byte payload
- `MAVLINK_M_ACK`: message `53004`, 69-byte payload

The canonical XML is stored at
`src/modules/mavlink/mavlink_m/message_definitions/v1.0/mavlink_m.xml`.
Header and endpoint-tool binding generation use the pinned pymavlink `2.4.49`
gitlink with Python 3.9, `PYTHONHASHSEED=0`, and the tracked package shim. The
build fails if the generator version, Python version, or XML hash changes.

The selected MAVLink route supplies the destination. Finalized identity, cue,
handover, and ACK payloads do not contain private target-system/component
fields. PX4 therefore consumes message IDs `53000`, `53001`, `53002`, and
`53004` locally and excludes them from generic MAVLink forwarding, even when
the selected instance or other MAVLink instances have forwarding enabled. The
selected instance may be a normal GCS route or a dedicated AAGS route, but its
instance, peer mode, source authority, and signing policy must be configured
explicitly. The old 54xxx capability, task-control, and task-status messages
are not part of this endpoint.

## Build

The branch board configurations opt into the `mavlink_m` dialect and
receiver. From a fresh clone, initialize all pinned submodules and create a
Python 3.9 environment with the normal PX4 build requirements:

```sh
git submodule update --init --recursive
python3.9 -m venv .venv-px4
. .venv-px4/bin/activate
python -m pip install --upgrade pip
python -m pip install -r Tools/setup/requirements.txt

make px4_sitl_default
make ark_fmu-v6x_default
make px4_fmu-v6c_default
```

`python3.9` must be installed and discoverable on `PATH`. The build
deliberately selects that interpreter for MAVLink-M generation. The
`mavlink_m_generator` submodule is pinned to pymavlink `2.4.49`, generation
uses `PYTHONHASHSEED=0`, and the canonical XML hash is checked during
configuration. A different interpreter, generator revision, or XML fails
closed instead of creating different wire headers.

The AAGS-enabled FMUv6X and FMUv6C configurations deliberately omit
`CONFIG_MODULES_UXRCE_DDS_CLIENT`. PX4 1.16 has insufficient flash for both
the complete MAVLink-M receiver and the ROS 2 uXRCE-DDS client while retaining
the selected flight, navigation, camera, logging, and MAVLink features. The
configure output states this tradeoff explicitly. These firmware images do
not provide the PX4 ROS 2/DDS client. A deployment that requires ROS 2 must
use a separate board configuration, re-enable the client, and recover the
required flash from features that deployment does not need.

Enable the same Kconfig option and dialect on each additional production board
that needs direct AAGS vehicle cue support.

The endpoint is still disabled at runtime by default.

## Configure PX4

First identify the zero-based instance carrying the AAGS connection:

```text
mavlink status
```

The branch supplies `run_dual_gz.sh` for a two-aircraft Gazebo Harmonic lab.
Each vehicle uses its stock instance `0` as one Fleet route for cue,
owner-control, and observer traffic (`MAV_M_SAME_EP=1`). A bounded peer table
lets both AAGS stations see the same receiver-confirmed state without adding
an observer MAVLink instance. On any vehicle or launch topology, use the
instance reported by `mavlink status`; do not infer `MAV_M_INST` from a UDP
port or MAVLink channel number.

Then choose one of two exact endpoint layouts:

- With `MAV_M_SAME_EP=0` (default), `MAV_M_INST` receives the cue from the
  sending/peer AAGS and returns the authoritative `MAVLINK_M_ACK`, while a
  distinct owner AAGS identity on `MAV_M_CTL_INST` sees the pending review
  snapshot and requests Accept/Reject.
- With `MAV_M_SAME_EP=1`, one physical MAVLink instance carries cue-source and
  owner-control traffic. `MAV_M_SRC_SYS/CMP` and `MAV_M_CTL_SYS/CMP` remain
  independent identity checks. In signed mode the two signing link IDs must
  match because there is only one physical route.

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

For a vehicle that accepts cues from any AAGS but reserves owner decisions for
AAGS `253/190` on MAVLink instance `0`, use:

```text
param set MAV_M_MODE 1
param set MAV_M_SAME_EP 1
param set MAV_M_INST 0
param set MAV_M_SRC_SYS -1
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
`MAV_M_CTL_SYS=0`). With `MAV_M_SAME_EP=0`, the control instance must differ
from the cue instance. With `MAV_M_SAME_EP=1`, both roles use the same
instance, but their source and control identity selectors may differ. For
example, `MAV_M_SRC_SYS=-1` may receive cues from every authorized station
while `MAV_M_CTL_SYS=253` reserves Accept, Reject, and Abort for station 253.
Set `MAV_M_CTL_SYS=-1` only when every station on that secured route is
intended to have control authority. Same-endpoint mode changes who may present
or decide a cue, not the acceptance safeguard: sending never auto-accepts or
moves the aircraft.
Wrong-instance, broadcast-target, stale-ID, wrong-source, and malformed
decision requests are denied and never reach PX4's generic command path.

Route and source parameters fail closed even if written outside their metadata
bounds: instances must fit the MAVLink instance count compiled into the
selected PX4 target, source and owner system IDs must be `1..255` or the
documented `-1` wildcard, component IDs must be an exact `1..255`, and mode-2
signing link IDs must be `0..255`. The two signing IDs must differ for
separate endpoints and match for the same endpoint. Invalid values disable the
affected endpoint instead of wrapping through an 8-bit cast. A corrupt
`MAV_M_MAX_AGE` outside `0..600` is corrected to the safe 30-second default.

## Multi-AAGS UDP fleet route

Each PX4 vehicle can now serve several AAGS stations from one selected UDP
MAVLink instance. No router or additional vehicle computer is required for
this bounded direct mode.

PX4 learns a station only after receiving a complete, CRC-valid
`MAV_TYPE_GCS` heartbeat on a selected MAVLink-M route. The component must
match `MAV_M_SRC_CMP` on the cue route or `MAV_M_CTL_CMP` on the owner-control
route. When both roles share one route, either configured component may
register. In signed mode, that heartbeat must also pass MAVLink 2 signature,
replay validation, and the signing link configured for its role. The station
identity is bound to its full source tuple: system ID, component ID, source
IPv4 address, and source UDP port. An exact match refreshes the entry. A live
identity cannot move to another endpoint, and one live endpoint cannot claim
another identity. The old entry must expire before either change is accepted.

The same authorized `MAV_TYPE_GCS` heartbeat also sustains cue-source
liveness for movement. It must arrive on `MAV_M_INST`, match
`MAV_M_SRC_SYS/CMP` and the learned endpoint, and use the configured signing
link in mode 2. A source does not need to retransmit unchanged task messages.
If that heartbeat and task traffic are both silent for 15 seconds, a pending
movement acceptance is blocked and an active Intercept is permanently aborted
to Hold. A later heartbeat does not resume the aborted Intercept.

Configure the bounded table with:

```text
param set MAV_M_PEERS 4
param set MAV_M_P_TMO 30
param save
reboot
```

`MAV_M_PEERS` defaults to four and accepts `0..8`. Values `1..8` select direct
multi-AAGS mode. The table is fixed at eight entries and performs no dynamic
allocation. `MAV_M_P_TMO` defaults to 30 seconds and accepts `5..300`. A
station must keep sending its normal heartbeat before the timeout. The
separately configured MAVLink partner remains pinned and is never replaced by
this table.

Direct-mode `TRACK_IDENTITY`, `TARGET_CUE`, `TARGET_HANDOVER`, owner
`MAV_CMD_USER_1`, and ESAD control traffic is accepted only when the complete
message source identity and current UDP source tuple match a live learned
entry. An expired, unregistered, moved, or partially matching endpoint is
denied before any generic command or forwarding path. After peer admission,
ordinary arm and mode commands, missions, parameter writes, FTP, timesync, RC
override, manual control, and all other generic MAVLink ingress are restricted
to the exact or wildcard `MAV_M_CTL_SYS/CMP` identity on `MAV_M_CTL_INST`.
Non-owner learned peers are read-only observers. They may register and refresh
with validated GCS heartbeats, receive telemetry and task state, and submit
task frames only when `MAV_M_SRC_SYS/CMP` authorizes them. They cannot reach
ordinary PX4 handlers or forwarding. Serial links and UDP instances where no
MAVLink-M route is selected retain their existing behavior. PX4 also resets
partial MAVLink framing at every UDP datagram boundary, so bytes from one
endpoint can never complete a frame started by another endpoint.

Set `MAV_M_PEERS=0` for gateway mode:

```text
param set MAV_M_PEERS 0
param save
reboot
```

Value zero disables learned-peer admission and fanout, but preserves an
explicitly configured `-t` partner. The `-t` address plus the configured `-o`
port form the one authorized gateway source tuple. Gateway mode without an
explicit `-t` partner fails closed for protected UDP traffic; PX4 never turns
the first raw datagram into a trusted gateway. Use gateway mode when one
vehicle-side MAVLink router or mesh
gateway represents many downstream AAGS identities from one UDP source
endpoint. PX4 then sends one copy to the pinned gateway, and the gateway owns
fanout. Without this mode, the direct peer table would correctly reject the
second GCS identity seen from the gateway's already-registered endpoint as a
live identity conflict.

An invalid `MAV_M_PEERS` value is corrected to zero, not four. This makes a
damaged or out-of-range setting fail closed instead of unexpectedly enabling
direct peer learning.

The selected PX4 instance unicasts its normal outbound stream and
receiver-confirmed AAGS state to every live learned station. This makes the
same accepted or active cue visible at every observer. Learning a peer does
not grant task-source, owner-control, or ordinary PX4 command permission.
`MAV_M_SRC_SYS/CMP` and `MAV_M_CTL_SYS/CMP` are evaluated independently for
every request. Receiver status includes `AAGS_CSYS` and `AAGS_CCMP` so an
observer can show the active state without presenting controls it does not
own.

PX4 continues a discovery heartbeat while `MAV_M_PEERS` is greater than one,
even after the first station connects. Only `HEARTBEAT` uses this broadcast
exception. `TARGET_CUE`, `TARGET_HANDOVER`, acknowledgements, parameter data,
and vehicle telemetry are not broadcast. They are sent only to the pinned
partner and validated learned peers.

Use one unique MAVLink system ID per AAGS station. In production, vehicles on
different IP addresses may all use the same PX4 UDP service port. AAGS must
open one shared fleet UDP socket, not one separately bound Comm Link per
vehicle. It distinguishes vehicles by source IP address and MAVLink system ID.
For example:

```text
Vehicle 44: 10.42.0.44:14556
Vehicle 45: 10.42.0.45:14556
Vehicle 46: 10.42.0.46:14556
```

The bind collision occurs only when separate processes or separate sockets on
the same host try to own the same local address and port. Multiple PX4 SITL
processes on one computer therefore need distinct PX4 service ports. Two AAGS
processes on one test computer likewise need different local receive ports,
such as `14551` and `14552`. On separate computers, AAGS stations may all use
the same conventional local receive port because each computer owns a
different IP address.

Each AAGS sends its component-190 heartbeat through its one fleet socket to
each vehicle endpoint it wants to observe. AAGS should learn an endpoint from
the source address and source port of the vehicle heartbeat, then reply to that
exact tuple. The return heartbeat registers the station's actual IP address
and source port, so neither side needs a preloaded list of peer IP addresses.
If the mesh does not carry IPv4 broadcast between nodes, configure one mesh
gateway or static bootstrap destination; learned traffic remains unicast after
that first contact.

Use `mavlink status` to inspect the live table. It prints the selected
instance's peer count, timeout, GCS identity, endpoint, age, registrations,
expirations, conflicts, table-full rejections, send errors, successful fanout
copies, and aggregate fanout bytes. Conflict and full-table warnings are
rate-limited while their counters continue to record every event. A conflict
is not automatically reassigned. Correct the duplicate GCS ID or endpoint and
wait for `MAV_M_P_TMO`, or reboot the vehicle after correcting the station.

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
param set MAV_M_INT_CLR 30
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
4. After either a hit or a miss, the aircraft recovers to the AMSL altitude
   sampled at local acceptance. A fixed wing first continues straight past the
   target on a climb or descent segment calculated from the same wind, pitch,
   vertical-rate, and airspeed limits. It turns only after reaching the recovery
   waypoint and the acceptance-time altitude. PX4 then establishes a
   target-centered loiter at that recovery altitude. A multicopter establishes
   the same target-centered recovery loiter directly.
5. A hit becomes complete only after Navigator owns that recovery loiter, the
   actual aircraft altitude is within 5 m of the acceptance-time AMSL altitude,
   and the aircraft remains continuously inside the effective arrival radius
   for `MAV_M_INT_DWL`. The receiver never reports completion while the
   aircraft is still at cue altitude or while the straight recovery is active.

`MAV_M_INT_RAD` is only the configured post-crossing dwell radius. It never
changes the fixed 5 m horizontal and vertical fly-through hit tests. For fixed
wing, the effective post-crossing dwell radius is
`max(MAV_M_INT_RAD, abs(NAV_LOITER_RAD) + 10 m)` so an aircraft established on
its commanded loiter circle can satisfy the dwell. Multicopters use
`MAV_M_INT_RAD`. Leaving the radius resets the complete dwell. The mandatory
target-plane crossing and matching internal completion ACK are separate from
this radius and always occur first. The request token remains inside PX4 and
the private command and ACK are not emitted on MAVLink.

If the first bounded crossing misses, Navigator uses the same straight recovery
and target-centered acceptance-altitude loiter used after a hit. PX4 reports the
intercept as `ABORTED`, records the miss, and permanently blocks completion for
that acceptance only after the recovery loiter is owned and the actual aircraft
is within 5 m of its acceptance-time AMSL altitude. The operator must abort it
and accept a fresh cue before another attempt. A safety, policy, source,
terrain, or setpoint failure instead cancels the private navigation request and
commands a hold at the aircraft's current position.
`MAV_M_INT_DZ` is the maximum
permitted absolute difference between acceptance altitude and cue altitude.
A larger difference prevents that intercept.

`MAV_M_INT_CLR` is the minimum terrain clearance for exact Intercept and
defaults to 30 m. Any nonnegative setting requires fresh global terrain altitude
and fresh valid HAGL data. Before acceptance and throughout the level entry,
bounded slope, exact crossing, straight recovery, and recovery loiter, PX4
checks all of the following:

- Aircraft AMSL altitude minus current terrain AMSL is at least
  `MAV_M_INT_CLR`.
- The independent HAGL estimate is at least `MAV_M_INT_CLR`.
- Every active target, approach, recovery, and loiter altitude is at least
  `MAV_M_INT_CLR` above the current terrain estimate.

Missing, stale, nonfinite, or invalid terrain/HAGL data fails closed. A
clearance breach permanently aborts that accepted Intercept. PX4 intentionally
does not infer a safe terrain corridor from the cue altitude. This protection
is reactive against terrain and HAGL under the current aircraft and checks each
active setpoint against the current terrain estimate. It is not a terrain
lookahead planner and does not prove clearance over the entire future corridor.
A ground-level cue altitude is denied before acceptance, or aborts during
execution, unless it satisfies the active clearance policy.

`MAV_M_INT_CLR=-1` is an explicit externally surveyed corridor override for an
aircraft that cannot provide terrain/HAGL. It disables only these terrain-data
and clearance checks and logs the override when used. It does not auto-accept a
cue. Every cue still requires a new local effect-2 acceptance, and every source,
owner, signed-link, flight-state, geofence, vertical-change, exact-hit, and
recovery gate remains active. Use `-1` only when an external process has
surveyed the entire approach and recovery corridor.

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
`MAVLINK_M_ACK RECEIVED` with a movement-blocked reason, and publishes no
motion command.
After the aircraft becomes safe, the operator must review and explicitly
accept the still-pending cue again. It never starts moving automatically.

For a finite-altitude action-2 cue, PX4 continuously rechecks the exact active
cue, expiry, source freshness, action and intercept parameters, armed
airborne-Hold state, failsafe/failure state, global position, vertical limit,
fresh terrain/HAGL, actual and candidate clearance, and ownership of the
level-entry, exact-target, straight-recovery, and recovery-loiter setpoints.
While Intercept is active, Loiter revalidates every active endpoint on every
cycle. PX4 1.16 cannot prove that the whole fixed-wing approach corridor stays
inside a restrictive geofence, so exact Intercept is rejected whenever an
actual polygon, circle, `GF_MAX_HOR_DIST`, or `GF_MAX_VER_DIST` restriction is
configured and `GF_ACTION` is `LOITER`, `RTL`, `TERMINATE`, or `LAND`.
PX4's default `GF_ACTION=LOITER` does not create a fence by itself and does not
block Intercept when all four restrictions are absent. `NONE` and `WARN` allow
Intercept with a configured fence. After crossing PX4 requires ownership of
the promoted target-centered loiter at acceptance-time altitude and actual
altitude within 5 m. Task abort or expiry, mode exit, disarm, failsafe, source
staleness, terrain/HAGL loss, clearance breach, setpoint override, action or
intercept-policy change, or restart permanently changes that acceptance to
intercept phase `ABORTED`.
Restoring a gate does not resume the dwell. Restarted active assignments remain
visible for audit but never resume the intercept.

`listener mavlink_m_target_status` reports `intercept_phase` as `NONE`,
`TRANSIT`, `DWELL`, `COMPLETE`, or `ABORTED`. The owner link sends
the same numeric value as `NAMED_VALUE_INT AAGS_IPHS`.

An accepted cue does not begin moving later after arming, takeoff, entering
Hold, or restart/restore. A newly received cue may remain Pending while the
current cue continues normally. If the operator accepts that new cue, PX4
first durably changes the old active cue to Rejected/Aborted and sends its
authoritative `MAVLINK_M_ACK REJECTED`, then durably accepts the new cue,
sends its `MAVLINK_M_ACK ACCEPTED`, and makes it the sole active assignment.
If the replacement fails any permission or flight-state gate, the new cue
stays Pending and the old active cue continues unchanged. Legacy
accepted-queue state is restored as Pending.

PX4 1.16 Commander requires the
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

Build PX4 SITL with the normal v1.16 Gazebo Harmonic dependencies, then start
the deterministic dual-aircraft lab:

```sh
make px4_sitl_default
Tools/aags_mavlink_m/run_dual_gz.sh start
Tools/aags_mavlink_m/run_dual_gz.sh status
```

The runner starts two `gz_rc_cessna` aircraft in one isolated Gazebo
partition. PX4 instances `43` and `44` become SYS44 and SYS45. Their Fleet UDP
service ports are `18613` and `18614`. Run two AAGS processes with different
MAVLink system IDs and different local UDP ports. The default ownership is:

- AAGS `254/190` owns SYS44.
- AAGS `253/190` owns SYS45.

Gazebo SITL does not always publish the fresh terrain and HAGL estimates
required by the production `MAV_M_INT_CLR=30` default. The lab runner therefore
sets `MAV_M_INT_CLR=-1` explicitly and reports it in its parameter check. This
is a SITL-only externally surveyed corridor override. Do not copy it to a
flight vehicle unless the complete approach and recovery corridor has been
surveyed outside PX4. The override does not accept a cue: the owning AAGS must
still select effect 2 for every Intercept.

Both stations discover and display both vehicles through each vehicle's one
Fleet route. PX4 sends receiver-confirmed cue state to both stations, while
the exact source and control selectors still reject commands from the wrong
station. Set `AAGS_DUAL_OPEN_OWNER=1` only for a test that intentionally gives
both stations cue and owner-control authority.

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

Use `Tools/aags_mavlink_m/run_dual_gz.sh attach` to open both PX4 consoles.
Use `Tools/aags_mavlink_m/run_dual_gz.sh reset` after a crash when the next run
must not reuse parameter or cue state. The runner creates a
`.aags-dual-gz-owned` marker containing the exact canonical run directory,
tmux session, and Gazebo partition. `reset` deletes only a directory with that
matching marker. It refuses `/`, the user's home directory, symlinks, nonempty
unmarked directories, and markers owned by another session or partition.

To validate beside an unrelated running SITL lab, give the new runner a
private session, partition, run root, and matching instance and port offsets:

```sh
AAGS_DUAL_TMUX_SESSION=aags-v116-check \
AAGS_DUAL_GZ_PARTITION=aags-v116-check \
AAGS_DUAL_RUN_ROOT=/tmp/aags-v116-check \
AAGS_DUAL_INSTANCE_OFFSET=20 \
AAGS_DUAL_PORT_OFFSET=20 \
Tools/aags_mavlink_m/run_dual_gz.sh start
```

That example creates SYS64/SYS65 on Fleet ports 18633/18634. PX4 1.16 derives
the Fleet port from the SITL instance, so the two offsets must match. The
runner validates their ranges before starting any process.

If `MAV_M_ACTION=1` or `2`, the aircraft must already be airborne, armed, safe,
and in Hold at the instant Accept is selected. If that motion gate fails, the
cue remains Pending/Received. It becomes eligible only for a later explicit
Accept after the gate is safe and never starts moving on its own.

The repository bench sender can isolate AAGS from the test:

```sh
Tools/aags_mavlink_m/endpoint_tool.py \
  --source-system 254 --source-component 190 \
  --udp-bind 127.0.0.1:14551 \
  --udp-destination 127.0.0.1:18613 \
  cue --instance 731 --cue-type 1 \
  --lat 45.4671 --lon -73.7578 --alt 50 \
  --name ALBATROSS-731
```

Before `TRACK_IDENTITY`, `TARGET_CUE`, or `TARGET_HANDOVER`, the tool sends a
`MAV_TYPE_GCS` heartbeat from the configured source identity and waits 0.3
seconds for Direct Fleet peer admission. Signed operation signs that heartbeat
with the same key and link ID as the task. Use
`--peer-registration-delay SECONDS` only when a slower link needs more time.
Those values target SYS44 in the default runner. To target SYS45, use source
system 253 and destination port 18614.

For a handover, `target_set_id` is the finalized ACK correlation instance:

```sh
Tools/aags_mavlink_m/endpoint_tool.py \
  --source-system 254 --source-component 190 \
  --udp-bind 127.0.0.1:14551 \
  --udp-destination 127.0.0.1:18613 \
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
outbound ACK/status traffic. Each incoming cue-side message must carry
`MAV_M_LNK_ID` in signature byte zero. Each owner decision must carry
`MAV_M_CTL_LNK`. A frame signed with the correct key but the wrong link ID is
denied. The two signing link IDs must differ.

Mode 2 is fail closed during cold start. A selected physical route remains
locked before any decoded frame can reach task, ESAD, command, mission,
parameter, FTP, timesync, RC, manual-control, or forwarding handlers until both
the transmitter and receiver parser own active signing contexts. A missing or
permission-invalid key, or a real-time clock at or before the MAVLink signing
epoch, keeps the route locked. PX4 retries key and clock activation once per
second, so installing a valid owner-only key and obtaining valid UTC unlocks
the signed route without a reboot. It never falls back to unsigned traffic.
The locked route also drops `TIMESYNC`, so cold-start UTC must come from GPS,
an RTC, or a separate MAVLink route that is not protected by this mode-2
endpoint.

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

## ESAD ingress and forwarding

`ESAD_ARMING` and `ESAD_CONFIG` are targetless safety-critical messages. PX4
accepts them only on `MAV_M_INST` from an endpoint authorized by the selected
Direct or Gateway mode, with a source matching `MAV_M_CTL_SYS/CMP`. Signed mode
also requires signature byte zero to equal `MAV_M_CTL_LNK`. Unauthorized forms
are consumed and cannot fall through to generic MAVLink forwarding.

`MAV_M_ESAD_I` selects the outbound MAVLink instance for both messages. The
receiving `MAV_M_INST` and selected output instance must have forwarding
enabled. The forwarded object retains its complete original MAVLink 2
signature. An explicit output must be a distinct running instance. Invalid,
self, unavailable, or forwarding-disabled outputs drop the command. The first
authorized ESAD control frame after boot reaches the explicit output without
requiring prior traffic from the ESAD component. `ESAD_STATE` keeps normal
return forwarding behavior.

## Verification

Run the local policy suites before building firmware:

```sh
python3 Tools/aags_mavlink_m/test_udp_peer_policy.py
python3 Tools/aags_mavlink_m/test_intercept_policy.py
```

For a direct-mode bench check, set `MAV_M_PEERS=4`, connect two AAGS stations
with unique system IDs, and wait for both to send heartbeats. `mavlink status`
must show two live `GCS ...` rows. Send and accept one vehicle cue. Both
stations must receive the receiver-confirmed active state, but only a station
matching `MAV_M_CTL_SYS/CMP` may issue Accept, Reject, or Abort. Stop one AAGS
for longer than `MAV_M_P_TMO`; its row must disappear and the expiration
counter must increment.

For a duplicate check, start a second process with a GCS system/component ID
already live at another address or source port. PX4 must retain the first row,
reject the duplicate, and increment `conflicts`. For a gateway check, set
`MAV_M_PEERS=0` and reboot. `mavlink status` must show `0/0` fleet peers while
the configured pinned partner still receives the normal MAVLink stream.

The conformance source is portable. With the Python 3.9 environment from the
Build section active, run it from the configured host test build:

```sh
cmake --build build/px4_sitl_default \
  --target mavlink_m_conformance_test

build/px4_sitl_default/mavlink_m_conformance_test --self-test
build/px4_sitl_default/mavlink_m_conformance_test --golden
```

The generated cue, handover, and ACK frames must be byte-identical to AAGS
`mavlink-m/golden/workflow-frames.json`.

Run the full SITL acceptance sequence once in unsigned lab mode and once with
MAVLink 2 signing:

```sh
python3 Tools/aags_mavlink_m/run_sitl_acceptance.py \
  --json-output /tmp/mavlink-m-v116-unsigned.json

python3 Tools/aags_mavlink_m/run_sitl_acceptance.py \
  --signed \
  --json-output /tmp/mavlink-m-v116-signed.json
```

The signed run creates an owner-only temporary key inside its isolated rootfs.
Identity conflicts at an already registered UDP tuple must be silently dropped
by the route-wide provenance gate and must not produce `COMMAND_ACK`.
