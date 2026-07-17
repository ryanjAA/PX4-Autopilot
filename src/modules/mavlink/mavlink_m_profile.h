/****************************************************************************
 *
 * Copyright (c) 2026 Ryan Johnston. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES.
 *
 ****************************************************************************/

#pragma once

#include <stdint.h>

#define AAGS_MAVLINK_M_PROFILE_ID "aags-private-inert-54xxx"
#define AAGS_MAVLINK_M_PROFILE_VERSION "private-inert-2026-07-17-v2"
#define AAGS_MAVLINK_M_CORE_XML_SHA256 "f8089061a6a216ef5f1bd4ba5b3242f38f3167b81050c9667121f31e46c36d8e"
#define AAGS_MAVLINK_M_PROTOCOL_MAJOR 1
#define AAGS_MAVLINK_M_PROTOCOL_MINOR 1

static constexpr uint8_t AAGS_MAVLINK_M_PROFILE_SHA256_BYTES[32] {
	0xf8, 0x08, 0x90, 0x61, 0xa6, 0xa2, 0x16, 0xef,
	0x5f, 0x1b, 0xd4, 0xba, 0x5b, 0x32, 0x42, 0xf3,
	0x8f, 0x31, 0x67, 0xb8, 0x10, 0x50, 0xc9, 0x66,
	0x71, 0x21, 0xf3, 0x1e, 0x46, 0xc3, 0x6d, 0x8e
};
