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
 * @value 0 Default to 1, switch to 2 if GCS sends version 2
 * @value 1 Always use version 1
 * @value 2 Always use version 2
 */
PARAM_DEFINE_INT32(MAV_PROTO_VER, 0);

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
 * Private inert-development traffic is accepted only on the selected MAVLink
 * instance, from the exact source endpoint, after a fresh matching capability
 * advertisement, and when the payload names this PX4 system and component.
 * Mode 2 additionally requires a valid 32-byte MAVLink 2 signing key at
 * PX4_STORAGEDIR/mavlink_m_signing.key and rejects every unsigned frame on the
 * selected physical task link.
 *
 * @value 0 Disabled (fail closed)
 * @value 1 Private inert lab transport (unsigned)
 * @value 2 Private inert physical transport (signing required)
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
 * MAVLink instance carrying AAGS traffic
 *
 * Only this MAVLink instance may consume MAVLink-M assignment messages.
 *
 * @group MAVLink-M
 * @min 0
 * @max 7
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_INST, 0);

/**
 * Exact permitted AAGS source system ID
 *
 * @group MAVLink-M
 * @min 1
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_SRC_SYS, 255);

/**
 * Exact permitted AAGS source component ID
 *
 * @group MAVLink-M
 * @min 1
 * @max 255
 * @reboot_required true
 */
PARAM_DEFINE_INT32(MAV_M_SRC_CMP, 190);

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
 * Maximum age and local lifetime of a cue
 *
 * TARGET_CUE has no wire expiry in the supplied profile. This parameter is
 * therefore both the maximum accepted age and the fail-closed local lifetime
 * measured from its UTC timestamp. Zero disables both checks and is not
 * recommended. A valid UTC clock is required when this is nonzero.
 *
 * @group MAVLink-M
 * @unit s
 * @min 0
 * @max 600
 */
PARAM_DEFINE_INT32(MAV_M_MAX_AGE, 30);
