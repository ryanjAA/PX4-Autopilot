# Private-Inert MAVLink-M Endpoint

This branch adds a destination-side MAVLink-M cue endpoint to
`ark_fmu-v6x_default`. It uses the canonical `mavlink_m` dialect, not an
internal alias:

```
src/modules/mavlink/mavlink/message_definitions/v1.0/mavlink_m.xml
```

The current owner-authorized development profile is
`aags-private-inert-54xxx`, version `private-inert-2026-07-17-v2`, protocol
1.1. The canonical XML SHA-256 is:

```
f8089061a6a216ef5f1bd4ba5b3242f38f3167b81050c9667121f31e46c36d8e
```

The 54xxx message IDs remain private provisional IDs. This integration does
not make them final or partner-approved field-release IDs.

## Safety boundary

The endpoint is application-state-only. It does not publish vehicle commands,
navigation setpoints, flight-mode changes, arming requests, payload commands,
or engagement commands. A cue remains pending until the local vehicle operator
accepts or rejects it.

Accepted cue coordinates are published in `aags_mavlink_m_status` for local
display or application use. The topic is explicitly not a flight-control
input.

## Implemented wire behavior

- Advertises the exact profile every five seconds with a 15-second validity.
- Accepts only exact-profile, fresh, inert capability sources.
- Requires the cue's MAVLink target and `assigned_system` to address this
  vehicle.
- Durably commits a validated cue before returning `MAVLINK_M_ACK/RECEIVED`.
- Preserves a four-record inbox and frozen reply frames in dataman.
- Provides local Accept and Reject decisions.
- Returns addressed Accept/Reject ACKs.
- Reports Active, En Route, Ready, Complete, Aborted, Failed, and Expired task
  states.
- Retries exact frozen decision and status frames five times with bounded
  exponential backoff while the ingress link remains live.
- Treats exact duplicate cues idempotently and rejects an immutable cue-ID
  collision.
- Never replaces an active task with a queued cue.

The PX4 endpoint honestly advertises cue receive, application receipt, local
decision, task status, and inert-only support. It does not advertise handover
or remote task-control support.

## Parameters

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `MVM_ENABLE` | `1` | Enables the private-inert endpoint. |
| `MVM_SIGN_REQ` | `0` | Requires signing when set. See the limitation below. |
| `MVM_RC_CH` | `0` | One-based RC channel for local Reject/Center/Accept; zero disables it. |
| `MVM_RC_REJ` | `-0.65` | Reject threshold. |
| `MVM_RC_CTR` | `0.25` | Absolute center band. |
| `MVM_RC_ACC` | `0.65` | Accept threshold. |

For unsigned private-lab operation:

```
param set MVM_SIGN_REQ 0
param save
```

Signing is therefore not mandatory in this development build and can be
turned off at runtime. PX4 1.14 in this tree does not expose a key-backed
signature-validation result to this endpoint. Setting `MVM_SIGN_REQ=1`
intentionally disables MAVLink-M advertisement and receipt rather than
claiming validation that did not occur.

The configured RC input must first enter the center band after boot and after
every decision. RC signal loss, stale RC input, or an invalid threshold
configuration resets the latch.

## Local operator commands

```
mavlink mavlink_m status
mavlink mavlink_m accept
mavlink mavlink_m reject
mavlink mavlink_m enroute
mavlink mavlink_m ready
mavlink mavlink_m complete
mavlink mavlink_m abort
mavlink mavlink_m fail
```

These commands only update and report the local MAVLink-M application task.

## Build and test

Hardware firmware:

```
make ark_fmu-v6x_default -j8
```

Simulator-free endpoint harness:

```
make px4_sitl_mavlink_m -j8
python3 test/mavlink_m/sitl_endpoint_test.py
```

The SITL test generates its Python bindings from the checked-in canonical XML
and verifies profile advertisement, addressed durable receipt, local
Accept/Reject, exact-frame retries, lifecycle statuses, restart persistence,
and wrong-address filtering.

## Current limits

- The durable inbox holds four cue records. Terminal records are reclaimed
  only after their validity plus a 60-second replay guard has elapsed and
  reply retries are complete.
- Operator presentation is via uORB, MAVLink replies, and the PX4 shell/RC
  decision input. This branch does not add an onboard display.
- Signing-required operation remains fail-closed until key-backed inbound
  signature validation is integrated and tested on this PX4 baseline.
- Handover, task-control reception, payload/ESAD, engagement, and assessment
  messages are not implemented by this inert PX4 endpoint and are not
  advertised.
- Physical radio, multi-GCS, field camera/geolocation, partner-vector, and
  bilateral conformance testing remain separate release gates.
