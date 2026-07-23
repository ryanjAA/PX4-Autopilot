/***************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file loiter.h
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 */

#pragma once

#include "navigator_mode.h"
#include "mission_block.h"

#include <px4_platform_common/module_params.h>

class Loiter : public MissionBlock, public ModuleParams
{
public:
	Loiter(Navigator *navigator);
	~Loiter() = default;

	void on_inactive() override;
	void on_inactivation() override;
	void on_activation() override;
	void on_active() override;
	void prepare_fly_through(uint32_t token, hrt_abstime command_timestamp,
				 double target_lat, double target_lon, float target_alt,
				 float recovery_alt, float minimum_clearance,
				 double recovery_lat, double recovery_lon, bool recovery_outbound);
	void cancel_fly_through();

private:
	/**
	 * Use the stored reposition location of the navigator
	 * to move to a new location.
	 */
	void reposition();
	void update_fly_through();
	bool fly_through_endpoints_allowed();
	void promote_fly_through_approach();
	void promote_fly_through_recovery();
	void reset_fly_through_crossing_sample();
	void complete_fly_through(bool target_hit);
	void clear_fly_through_state();

	/**
	 * Set the position to hold based on the current local position
	 */
	void set_loiter_position();

	bool _loiter_pos_set{false};
	uint32_t _fly_through_pending_token{0};
	hrt_abstime _fly_through_pending_command_timestamp{0};
	double _fly_through_pending_target_lat{static_cast<double>(NAN)};
	double _fly_through_pending_target_lon{static_cast<double>(NAN)};
	float _fly_through_pending_target_alt{NAN};
	float _fly_through_pending_recovery_alt{NAN};
	float _fly_through_pending_minimum_clearance{NAN};
	double _fly_through_pending_recovery_lat{static_cast<double>(NAN)};
	double _fly_through_pending_recovery_lon{static_cast<double>(NAN)};
	bool _fly_through_pending_recovery_outbound{false};
	bool _fly_through_active{false};
	uint32_t _fly_through_token{0};
	hrt_abstime _fly_through_started{0};
	double _fly_through_start_lat{static_cast<double>(NAN)};
	double _fly_through_start_lon{static_cast<double>(NAN)};
	double _fly_through_target_lat{static_cast<double>(NAN)};
	double _fly_through_target_lon{static_cast<double>(NAN)};
	float _fly_through_target_alt{NAN};
	float _fly_through_recovery_alt{NAN};
	float _fly_through_minimum_clearance{NAN};
	double _fly_through_recovery_lat{static_cast<double>(NAN)};
	double _fly_through_recovery_lon{static_cast<double>(NAN)};
	bool _fly_through_recovery_outbound{false};
	bool _fly_through_recovery_active{false};
	bool _fly_through_target_hit{false};
	bool _fly_through_approach_active{false};
	double _fly_through_approach_lat{static_cast<double>(NAN)};
	double _fly_through_approach_lon{static_cast<double>(NAN)};
	float _fly_through_approach_alt{NAN};
	float _fly_through_previous_target_north{NAN};
	float _fly_through_previous_target_east{NAN};
	float _fly_through_previous_alt{NAN};
	bool _fly_through_previous_sample_valid{false};
};
