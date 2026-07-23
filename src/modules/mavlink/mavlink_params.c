/****************************************************************************
 *
 *   Copyright (c) 2012-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * MAVLink system ID
 * @group MAVLink
 * @min 1
 * @max 250
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_SYS_ID, 1);

/**
 * MAVLink component ID
 * @group MAVLink
 * @min 1
 * @max 250
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_COMP_ID, 1);

/**
 * MAVLink protocol version
 * @group MAVLink
 * @value 1 Version 1 with auto-upgrade to v2 if detected
 * @value 2 Version 2
 */
PARAM_DEFINE_INT32(MAV_PROTO_VER, 2);

/**
 * MAVLink SiK Radio ID
 *
 * When non-zero the MAVLink app will attempt to configure the
 * SiK radio to this ID and re-set the parameter to 0. If the value
 * is negative it will reset the complete radio config to
 * factory defaults. Only applies if this mavlink instance is going through a SiK radio
 *
 * @group MAVLink
 * @min -1
 * @max 240
 */
PARAM_DEFINE_INT32(MAV_SIK_RADIO_ID, 0);

/**
 * MAVLink airframe type
 *
 * @min 0
 * @max 22
 * @value 0 Generic micro air vehicle
 * @value 1 Fixed wing aircraft
 * @value 2 Quadrotor
 * @value 3 Coaxial helicopter
 * @value 4 Normal helicopter with tail rotor
 * @value 7 Airship, controlled
 * @value 8 Free balloon, uncontrolled
 * @value 10 Ground rover
 * @value 11 Surface vessel, boat, ship
 * @value 12 Submarine
 * @value 13 Hexarotor
 * @value 14 Octorotor
 * @value 15 Tricopter
 * @value 19 VTOL Two-rotor Tailsitter
 * @value 20 VTOL Quad-rotor Tailsitter
 * @value 21 VTOL Tiltrotor
 * @value 22 VTOL Standard (separate fixed rotors for hover and cruise flight)
 * @value 23 VTOL Tailsitter
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_TYPE, 0);

/**
 * Use/Accept HIL GPS message even if not in HIL mode
 *
 * If set to 1 incoming HIL GPS messages are parsed.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_USEHILGPS, 0);

/**
 * Forward external setpoint messages
 *
 * If set to 1 incoming external setpoint messages will be directly forwarded
 * to the controllers if in offboard control mode
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_FWDEXTSP, 1);

/**
 * Parameter hash check.
 *
 * Disabling the parameter hash check functionality will make the mavlink instance
 * stream parameters continuously.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_HASH_CHK_EN, 1);

/**
 * Heartbeat message forwarding.
 *
 * The mavlink heartbeat message will not be forwarded if this parameter is set to 'disabled'.
 * The main reason for disabling heartbeats to be forwarded is because they confuse dronekit.
 *
 * @boolean
 * @group MAVLink
 */
PARAM_DEFINE_INT32(MAV_HB_FORW_EN, 1);

/**
 * Timeout in seconds for the RADIO_STATUS reports coming in
 *
 * If the connected radio stops reporting RADIO_STATUS for a certain time,
 * a warning is triggered and, if MAV_X_RADIO_CTL is enabled, the software-flow
 * control is reset.
 *
 * @group MAVLink
 * @unit s
 * @min 1
 * @max 250
 */
PARAM_DEFINE_INT32(MAV_RADIO_TOUT, 5);

/**
 * MAVLink-M endpoint mode
 *
 * Finalized TARGET_CUE and TARGET_HANDOVER traffic is accepted only on the
 * selected MAVLink instance and from the configured source selector. The source
 * component is always exact; MAV_M_SRC_SYS may be exact or the documented
 * wildcard. The finalized payload has no destination fields, so the selected
 * vehicle link supplies routing. Mode 2 additionally requires a valid 32-byte
 * MAVLink 2 signing key at PX4_STORAGEDIR/mavlink_m_signing.key and trustworthy
 * UTC from GPS, an RTC, or a separate unprotected route. Until both key and UTC
 * are available, every frame on each selected physical route is dropped,
 * including TIMESYNC. The route retries key and clock setup without falling
 * back to unsigned traffic.
 *
 * @value 0 Disabled (fail closed)
 * @value 1 Lab transport (unsigned)
 * @value 2 Physical transport (signing required)
 * @group MAVLink-M
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_MODE, 0);

/**
 * MAVLink 2 signing link identifier
 *
 * Used only by signed physical mode. It must be unique for this signing key
 * across concurrently transmitting links.
 *
 * @group MAVLink-M
 * @min 0
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_LNK_ID, 0);

/**
 * MAVLink instance carrying MAVLink-M traffic
 *
 * Only this MAVLink instance may consume MAVLink-M assignment messages.
 *
 * @group MAVLink-M
 * @min 0
 * @max 5
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_INST, 0);

/**
 * Maximum learned AAGS UDP peers
 *
 * On the selected MAV_M_INST UDP link, PX4 can learn this many AAGS stations
 * from valid MAV_TYPE_GCS heartbeats. The component and, in signed mode, link
 * ID must match either the configured cue-source role or the valid ESAD
 * owner-control role. Outgoing telemetry and receiver-confirmed cue state are
 * copied to every live learned endpoint. Registration does not grant
 * cue-source, generic-command, or owner-control authority; all exact
 * message-specific identity and signing-link gates remain independently
 * enforced.
 *
 * The table is fixed at eight entries and uses no dynamic allocation. A
 * separately configured MAVLink partner address remains available in addition
 * to the learned table and is never replaced by peer discovery.
 *
 * Set zero when a vehicle-side router or mesh gateway is the one pinned
 * partner. This disables learned-peer admission and fanout while preserving
 * that configured partner, so multiple downstream GCS identities may traverse
 * the gateway without being mistaken for one conflicting direct endpoint. A
 * gateway route must be started with an explicit -t partner address. Without
 * that explicit partner, protected UDP ingress is denied.
 *
 * Values outside zero through eight fail closed to gateway mode rather than
 * enabling direct peer learning.
 *
 * @group MAVLink-M
 * @min 0
 * @max 8
 * @value 0 Pinned gateway only; learned peers disabled
 */
PARAM_DEFINE_INT32(MAV_M_PEERS, 4);

/**
 * Learned AAGS peer timeout
 *
 * A learned UDP endpoint is removed when PX4 has not received a valid GCS
 * heartbeat from its system/component identity for this many seconds. A live
 * identity cannot move to another IP address or UDP port; the old endpoint
 * must expire first. Signed physical mode requires every registration and
 * refresh heartbeat to have a valid MAVLink 2 signature.
 *
 * @group MAVLink-M
 * @unit s
 * @min 5
 * @max 300
 */
PARAM_DEFINE_INT32(MAV_M_P_TMO, 30);

/**
 * MAVLink output instance for ESAD control
 *
 * Selects the one MAVLink output instance used when forwarding an authorized
 * incoming ESAD_ARMING or ESAD_CONFIG command. Minus one preserves standard
 * MAVLink forwarding. Values zero through five restrict both messages to that
 * instance. Standard forwarding may copy the broadcast to every other eligible
 * forwarding-enabled instance; it is not a synonym for one physical telemetry
 * port.
 *
 * The MAV_M_INST instance receiving ESAD control from AAGS and the selected
 * output instance must both be running with their MAV_X_FORWARD parameters
 * enabled. Ingress additionally requires the MAV_M_CTL_SYS/CMP identity,
 * MAV_M_CTL_LNK in signed mode, and authorized direct-peer or pinned-gateway
 * endpoint. This parameter does not start a MAVLink instance or enable
 * forwarding. The selected output must be a distinct running instance. A
 * selected instance equal to MAV_M_INST, outside the supported instance range,
 * absent, or forwarding-disabled fails closed and logs the rejected output.
 * An explicit valid output does not require the ESAD component to transmit
 * first, so the first authorized targetless command after boot is forwarded.
 * ESAD_STATE return traffic and all other messages retain standard forwarding.
 *
 * @value -1 Standard forwarding
 * @value 0 MAVLink instance 0
 * @value 1 MAVLink instance 1
 * @value 2 MAVLink instance 2
 * @value 3 MAVLink instance 3
 * @value 4 MAVLink instance 4
 * @value 5 MAVLink instance 5
 * @group MAVLink-M
 * @min -1
 * @max 5
 */
PARAM_DEFINE_INT32(MAV_M_ESAD_I, -1);

/**
 * Permitted cue-source system ID
 *
 * Selects one exact nonzero MAVLink system ID, or minus one to accept any
 * nonzero system ID on MAV_M_INST. The exact MAV_M_SRC_CMP check still
 * applies. This wildcard changes identity matching only. It does not consume
 * cues from any other MAVLink instance, enable forwarding, or trust observer
 * and telemetry links.
 *
 * @value -1 Any nonzero system on MAV_M_INST
 * @group MAVLink-M
 * @min -1
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_SRC_SYS, 255);

/**
 * Exact permitted cue-source component ID
 *
 * @group MAVLink-M
 * @min 1
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_SRC_CMP, 190);

/**
 * Owner-control MAVLink instance
 *
 * TARGET_CUE is received and acknowledged on MAV_M_INST, while the locally
 * owned AAGS sees the pending cue and may request Accept/Reject on this
 * instance. Minus one disables network decisions; RC and the local PX4
 * console remain available. MAV_M_CTL_SYS=-1 widens the permitted system ID
 * only on this selected instance. It never enables decisions on another
 * physical, observer, or forwarding route.
 *
 * MAV_M_SAME_EP=0 requires this to be a physical route distinct from
 * MAV_M_INST. MAV_M_SAME_EP=1 requires it to equal MAV_M_INST.
 *
 * @group MAVLink-M
 * @min -1
 * @max 5
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_CTL_INST, -1);

/**
 * Owner-control AAGS system ID
 *
 * Selects one exact nonzero MAVLink system ID, or minus one to allow any
 * nonzero system on MAV_M_CTL_INST to request Accept/Reject. Zero disables
 * network decisions. MAV_M_CTL_CMP remains an exact check. The wildcard does
 * not trust any other MAVLink instance and signed mode still requires the
 * configured physical link and signing key.
 *
 * With MAV_M_SAME_EP=0, MAV_M_CTL_INST must remain distinct from MAV_M_INST;
 * wildcard identities may overlap because the configured routes preserve the
 * cue-source and owner-control roles. With MAV_M_SAME_EP=1, source and control
 * share one physical instance but their system/component selectors remain
 * independent. For example, MAV_M_SRC_SYS=-1 can accept cues from any signed
 * AAGS while MAV_M_CTL_SYS=254 reserves Accept/Reject for GCS 254.
 *
 * @value -1 Any nonzero system on MAV_M_CTL_INST
 * @value 0 Network decisions disabled
 * @group MAVLink-M
 * @min -1
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_CTL_SYS, 0);

/**
 * Exact owner-control AAGS component ID
 *
 * This component check always applies, including when MAV_M_CTL_SYS=-1. It is
 * evaluated only on MAV_M_CTL_INST and does not authorize another route.
 *
 * @group MAVLink-M
 * @min 1
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_CTL_CMP, 190);

/**
 * Owner-control MAVLink 2 signing link identifier
 *
 * Used only by signed physical mode. MAV_M_SAME_EP=0 requires a value unique
 * from MAV_M_LNK_ID. MAV_M_SAME_EP=1 requires the same value because cue and
 * control traffic share one physical link. MAV_M_CTL_SYS=-1 does not weaken
 * signing or extend this identifier to other MAVLink instances.
 *
 * @group MAVLink-M
 * @min 0
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_CTL_LNK, 1);

/**
 * Allow one MAVLink instance to carry cue and control roles
 *
 * Zero requires the cue source and owner-control authority to use distinct
 * MAVLink instances. Identical exact system/component identities are rejected;
 * wildcard identities may overlap because the configured routes preserve each
 * role. Value one selects a shared physical route: MAV_M_INST must equal
 * MAV_M_CTL_INST and signed mode requires MAV_M_LNK_ID to equal
 * MAV_M_CTL_LNK. Source and owner-control system/component selectors remain
 * independent and are enforced against each received message, allowing one
 * exact owner to control cues submitted by multiple authorized stations.
 *
 * This setting does not auto-accept a cue. An authorized owner must still make
 * an explicit Accept/Reject decision through AAGS, RC, or the local console
 * before an accepted-cue navigation mode can publish a reposition command.
 *
 * @group MAVLink-M
 * @min 0
 * @max 1
 * @value 0 Separate cue source and owner-control endpoints
 * @value 1 Same physical route with independent source/control selectors
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_SAME_EP, 0);

/**
 * RC channel for Reject/Center/Accept
 *
 * The raw channel is interpreted as low=Reject, center=neutral, high=Accept.
 * Zero disables pilot decisions. The control must pass through center after
 * boot and after every decision before another edge can act.
 *
 * @group MAVLink-M
 * @min 0
 * @max 18
 */
PARAM_DEFINE_INT32(MAV_M_RC_CH, 0);

/**
 * Maximum PWM value for Reject
 *
 * @group MAVLink-M
 * @unit us
 * @min 800
 * @max 1500
 */
PARAM_DEFINE_INT32(MAV_M_RC_REJ, 1300);

/**
 * Minimum PWM value for Accept
 *
 * @group MAVLink-M
 * @unit us
 * @min 1500
 * @max 2200
 */
PARAM_DEFINE_INT32(MAV_M_RC_ACC, 1700);

/**
 * Maximum accepted age of a cue
 *
 * The finalized TARGET_CUE has no expiry field, so PX4 derives its pending
 * lifetime from time_usec plus this value and rejects stale/replayed cues.
 * TARGET_HANDOVER retains its explicit wire deadline and is also bounded by
 * this age. Zero disables the age check and derived cue expiry, but does not
 * disable a TARGET_HANDOVER wire deadline.
 *
 * @group MAVLink-M
 * @unit s
 * @min 0
 * @max 600
 */
PARAM_DEFINE_INT32(MAV_M_MAX_AGE, 30);

/**
 * Accepted-cue navigation permission
 *
 * This is the local movement permission ceiling. Zero keeps the endpoint
 * receipt/status only. Value 1 permits a local acceptance to select level
 * travel. Value 2 permits either level travel or the trusted-source intercept
 * profile for each exact accepted cue. A finite-altitude intercept reaches the
 * cue coordinate at TARGET_CUE.alt. For fixed wing, Navigator first computes a
 * level approach entry from FW_P_LIM_MIN or FW_P_LIM_MAX,
 * FW_T_SINK_MAX or FW_T_CLMB_MAX, and FW_AIRSPD_MAX. If the cue is too close,
 * the entry is placed behind the aircraft so it can fly away level, turn, and
 * establish the bounded slope. Navigator must report a token-matched crossing
 * within the fixed 5 m horizontal and vertical hit bounds before the
 * configured post-crossing dwell can begin. An explicit
 * intercept selection requires a finite cue altitude. Legacy RC and console
 * acceptance select the configured ceiling as their default effect and retain
 * the previous one-phase level behavior for a NaN altitude. MAVLink-M defines
 * no INTERCEPT cue enum, so the per-cue choice is receiver-local and does not
 * add or reinterpret a wire enum.
 *
 * Motion is possible only after the cue has been durably stored and locally
 * accepted by an authorized owner command, RC, or the local console. The vehicle
 * must already be airborne, armed, and in Hold (AUTO_LOITER), with fresh
 * global position and land detector state showing no landed, maybe-landed,
 * ground-contact, or freefall condition, and with no failsafe/failure
 * indication. Reposition is attempted at that explicit acceptance. Intercept
 * completion additionally requires the exact approach setpoints to remain
 * owned, a fresh source, continuous arrival dwell, and a cue-altitude change
 * no larger than MAV_M_INT_DZ. Task abort or expiry, mode exit, disarm,
 * failsafe, source staleness, setpoint override, policy change, or restart
 * permanently aborts that acceptance.
 * An unsafe, restored, or merely pending cue never starts moving later. A new
 * cue remains Pending while another cue is Active. Explicitly accepting it
 * first revalidates every requested-effect gate. A blocked replacement leaves
 * the old cue Active and the new cue Pending. A valid replacement durably
 * aborts the old cue, activates the new cue, and publishes the new navigation
 * request without an intervening hold. PX4 1.16 requires the
 * DO_REPOSITION change-mode flag (param2=1); publication already requires
 * AUTO_LOITER, so the command reasserts the current Hold intention rather than
 * selecting a different mode. This endpoint never arms, controls a payload, or
 * turns a handover into motion.
 *
 * @group MAVLink-M
 * @min 0
 * @max 2
 * @value 0 Deny movement; receipt and local acceptance only
 * @value 1 Permit level travel only
 * @value 2 Permit level travel or trusted-source intercept
 */
PARAM_DEFINE_INT32(MAV_M_ACTION, 0);

/**
 * Intercept post-crossing dwell radius
 *
 * A finite-altitude action-2 cue uses waypoint guidance through the cue
 * coordinate at cue altitude. Crossing the target plane within the fixed 5 m
 * horizontal and vertical hit bounds, plus a token-matched Navigator
 * completion ACK, is the mandatory arrival proof and cannot be replaced or
 * relaxed by this parameter. After that successful crossing, the aircraft
 * must remain within this horizontal distance during MAV_M_INT_DWL before the
 * intercept status becomes complete. For fixed wing,
 * the effective dwell gate is the larger of this value and the absolute
 * NAV_LOITER_RAD plus a 10 m tracking margin. For example, an 80 m loiter
 * radius makes the effective dwell gate at least 90 m even if this parameter
 * is set to 25 m.
 *
 * @group MAVLink-M
 * @unit m
 * @min 1
 * @max 500
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(MAV_M_INT_RAD, 25.0f);

/**
 * Intercept post-crossing dwell
 *
 * Continuous time inside the effective horizontal dwell radius after the
 * successful, token-matched target-plane crossing required before completion
 * is latched. Leaving the radius, losing the owning task, or losing required
 * fresh state resets the full timer. Zero removes the added wait but does not
 * bypass the target-plane crossing or any other safety gate.
 *
 * @group MAVLink-M
 * @unit s
 * @min 0
 * @max 60
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(MAV_M_INT_DWL, 3.0f);

/**
 * Maximum intercept altitude change
 *
 * Maximum absolute vertical difference between the aircraft's
 * acceptance-time AMSL altitude and a finite TARGET_CUE.alt. A larger
 * difference prevents the intercept for that acceptance. This is a
 * permission limit checked before movement. It is not a descent rate, a target
 * altitude, or a terrain-clearance setting.
 *
 * @group MAVLink-M
 * @unit m
 * @min 0
 * @max 1000
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(MAV_M_INT_DZ, 100.0f);

/**
 * Minimum Intercept terrain clearance
 *
 * Minimum required height above terrain during an exact effect-2 Intercept. A
 * nonnegative value requires fresh, valid terrain altitude and HAGL estimates
 * before acceptance and throughout the approach, target crossing, straight
 * recovery, and recovery loiter. PX4 reactively checks actual aircraft
 * clearance and each active candidate setpoint against terrain under the
 * current aircraft. It does not look ahead or prove clearance across the
 * future corridor. Missing, stale, or invalid terrain data, or any clearance
 * breach, fails closed and aborts the Intercept. A ground-level cue altitude
 * is therefore denied unless it satisfies this active clearance policy.
 *
 * Minus one is an explicit externally surveyed corridor override for systems
 * that cannot provide terrain/HAGL data. It disables only the terrain-data and
 * clearance gate. It does not auto-accept a cue or weaken source, owner,
 * flight-state, geofence, altitude-change, exact-hit, or recovery gates. Every
 * cue still requires a local effect-2 acceptance.
 *
 * @group MAVLink-M
 * @unit m
 * @min -1
 * @max 1000
 * @decimal 1
 * @value -1 Externally surveyed corridor override
 */
PARAM_DEFINE_FLOAT(MAV_M_INT_CLR, 30.0f);
