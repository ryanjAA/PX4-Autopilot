/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file manifest.c
 *
 * PAB base-board capabilities for the manifest interface in PX4 v1.14.
 * Sensor revisions are selected independently in spi.cpp. This branch exposes
 * PX4IO, USB, CAN2 and CAN3; newer power-monitor/Ethernet queries are not available.
 */

#include <board_config.h>
#include <systemlib/px4_macros.h>

static const px4_hw_mft_item_t device_unsupported = {0, 0, 0};

static const px4_hw_mft_item_t base_configuration_0[] = {
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_PX4IO
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_USB
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_1[] = {
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_PX4IO
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_USB
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_3[] = {
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_PX4IO
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_USB
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_5[] = {
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_PX4IO
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_USB
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_9[] = {
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_PX4IO
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_USB
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_17[] = {
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_onboard}, // PX4_MFT_PX4IO
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_unknown}, // PX4_MFT_USB
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_18[] = {
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_onboard}, // PX4_MFT_PX4IO
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_USB
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_19[] = {
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_PX4IO
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_USB
	{.present = 1, .mandatory = 1, .connection = px4_hw_con_onboard}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const px4_hw_mft_item_t base_configuration_21[] = {
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_PX4IO
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_USB
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN2
	{.present = 0, .mandatory = 0, .connection = px4_hw_con_unknown}, // PX4_MFT_CAN3
};

static const struct {
	hw_base_id_t base_id;
	const px4_hw_mft_item_t *items;
} base_manifests[] = {
	{HW_BASE_ID(0), base_configuration_0}, // std Base with PX4IO
	{HW_BASE_ID(1), base_configuration_1}, // std Base No PX4IO
	{HW_BASE_ID(2), base_configuration_0}, // CUAV Base
	{HW_BASE_ID(3), base_configuration_3}, // NXP T1 PHY
	{HW_BASE_ID(4), base_configuration_0}, // HB CM4 base
	{HW_BASE_ID(5), base_configuration_5}, // HB Mini
	{HW_BASE_ID(8), base_configuration_0}, // Auterion Skynode ver 8 Aliased to 0
	{HW_BASE_ID(9), base_configuration_9}, // Auterion Skynode ver 9
	{HW_BASE_ID(10), base_configuration_9}, // Auterion Skynode ver 10
	{HW_BASE_ID(16), base_configuration_0}, // Auterion Skynode ver 16
	{HW_BASE_ID(17), base_configuration_17}, // Auterion Skynode ver 17
	{HW_BASE_ID(18), base_configuration_18}, // Auterion Skynode S ver 18
	{HW_BASE_ID(19), base_configuration_19}, // Auterion Skynode N rev 0
	{HW_BASE_ID(20), base_configuration_19}, // Auterion Skynode N rev 1
	{HW_BASE_ID(21), base_configuration_21}, // Auterion CAN IO
	{HW_BASE_ID(22), base_configuration_21}, // Auterion CAN Airspeed
	{HW_BASE_ID(23), base_configuration_21}, // Auterion CAN Power
	{HW_BASE_ID(0x100), base_configuration_0}, // Holybro Pixhawk Jetson Baseboard ver 0x100 Alaised to ID 0
	{HW_BASE_ID(0x150), base_configuration_0}, // ZeroOne Pixhawk Baseboard ver 0x150
	{HW_BASE_ID(0x200), base_configuration_0}, // AmovLab Pixhawk Baseboard ver 0x150
};

__EXPORT px4_hw_mft_item board_query_manifest(px4_hw_mft_item_id_t id)
{
	if ((unsigned)id <= PX4_MFT_CAN3) {
		const hw_base_id_t base_id = GET_HW_BASE_ID();

		for (unsigned i = 0; i < arraySize(base_manifests); ++i) {
			if (base_manifests[i].base_id == base_id) {
				return &base_manifests[i].items[id];
			}
		}
	}

	return &device_unsupported;
}
