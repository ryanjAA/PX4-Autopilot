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

#include <mavlink.h>

class Mavlink;

/**
 * Private inert MAVLink-M destination endpoint.
 *
 * This endpoint changes application task state only. It never publishes a
 * flight-control setpoint, vehicle command, arming request, or payload command.
 */
class MavlinkMEndpoint
{
public:
	static void handle_message(Mavlink *link, const mavlink_message_t *message);
	static void update(Mavlink *link);
	static int command(int argc, char *argv[]);
	static void print_status();
};
