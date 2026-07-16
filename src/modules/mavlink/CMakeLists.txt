############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

set(MAVLINK_GIT_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink")
set(MAVLINK_LIBRARY_DIR "${CMAKE_BINARY_DIR}/mavlink")
file(RELATIVE_PATH MAVLINK_GIT_DIR_RELATIVE ${CMAKE_SOURCE_DIR} ${MAVLINK_GIT_DIR})

file(GLOB MAVLINK_GENERATOR_SOURCES
	"${MAVLINK_GIT_DIR}/pymavlink/generator/*.py"
	"${MAVLINK_GIT_DIR}/pymavlink/generator/*.xsd"
	"${MAVLINK_GIT_DIR}/pymavlink/generator/C/include_v2.0/*"
)

file(STRINGS "${MAVLINK_GIT_DIR}/pymavlink/__init__.py" MAVLINK_GENERATOR_VERSION_LINE
	REGEX "^__version__ = '2\\.4\\.49'$"
)

if(NOT MAVLINK_GENERATOR_VERSION_LINE)
	message(FATAL_ERROR "AAGS profile requires vendored pymavlink 2.4.49")
endif()

if("${CONFIG_MAVLINK_DIALECT}" STREQUAL "mavlink_m")
	file(SHA256 "${MAVLINK_GIT_DIR}/message_definitions/v1.0/mavlink_m.xml" MAVLINK_M_CORE_SHA256)

	if(NOT "${MAVLINK_M_CORE_SHA256}" STREQUAL "8ab02215d036f454bf76fee9d73985fa639f2b8ca9509bf24cc51b0cb35d3b4b")
		message(FATAL_ERROR "AAGS MAVLink-M canonical XML hash mismatch: ${MAVLINK_M_CORE_SHA256}")
	endif()
endif()

# This vendor branch tracks the MAVLink generator sources directly rather than
# as a gitlink. Preserve upstream submodule handling when a nested .git marker
# exists, and provide the same dependency target for the vendored layout.
if(EXISTS "${MAVLINK_GIT_DIR}/.git")
	px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")
else()
	add_custom_target(git_mavlink_v2)
endif()

# always generate uAvionix dialect
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")
add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
	COMMAND
		${CMAKE_COMMAND} -E env PYTHONHASHSEED=0
			${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			#--no-validate
			#--strict-units
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${MAVLINK_DIALECT_UAVIONIX}.log
	DEPENDS
		git_mavlink_v2
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_GENERATOR_SOURCES}
		${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml

	COMMENT "Generating Mavlink ${MAVLINK_DIALECT_UAVIONIX}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml"
)
add_custom_target(mavlink_c_generate_uavionix DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h PROPERTIES GENERATED true)

# generate the primary dialect
add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
	COMMAND
		${CMAKE_COMMAND} -E env PYTHONHASHSEED=0
			${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			#--no-validate
			#--strict-units
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${CONFIG_MAVLINK_DIALECT}.log
	DEPENDS
		git_mavlink_v2
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_GENERATOR_SOURCES}
		${MAVLINK_GIT_DIR}/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml
		${BOARD_DEFCONFIG}

		# generate after uAvionix
		mavlink_c_generate_uavionix
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h

	COMMENT "Generating Mavlink ${CONFIG_MAVLINK_DIALECT}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${CONFIG_MAVLINK_DIALECT}.xml"
)
add_custom_target(mavlink_c_generate DEPENDS ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h PROPERTIES GENERATED true)



# mavlink header only library
add_library(mavlink_c INTERFACE)
target_compile_options(mavlink_c INTERFACE -Wno-address-of-packed-member -Wno-cast-align)
target_sources(mavlink_c
	INTERFACE
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
)
target_include_directories(mavlink_c
	INTERFACE
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
)

px4_add_module(
	MODULE modules__mavlink
	MAIN mavlink
	COMPILE_FLAGS
		-Wno-enum-compare # ROTATION <-> MAV_SENSOR_ROTATION
		#-DDEBUG_BUILD
	SRCS
		mavlink.c
		mavlink_command_sender.cpp
		mavlink_events.cpp
		mavlink_ftp.cpp
		mavlink_log_handler.cpp
		mavlink_main.cpp
		mavlink_messages.cpp
		mavlink_mission.cpp
		mavlink_parameters.cpp
		mavlink_rate_limiter.cpp
		mavlink_receiver.cpp
		MavlinkMHandler.cpp
		mavlink_shell.cpp
		mavlink_simple_analyzer.cpp
		mavlink_stream.cpp
		mavlink_timesync.cpp
		mavlink_ulog.cpp
		MavlinkStatustextHandler.cpp
		tune_publisher.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		adsb
		airspeed
		component_general_json # for checksums.h
		drivers_accelerometer
		drivers_gyroscope
		drivers_magnetometer
		conversion
		crc
		sensor_calibration
		geo
		mavlink_c
		timesync
		tunes
		variable_length_ringbuffer
		version
	UNITY_BUILD
	)

if(PX4_TESTING)
	add_subdirectory(mavlink_tests)

	if("${PX4_PLATFORM}" MATCHES "posix")
		add_executable(mavlink_m_conformance_test
			mavlink_tests/MavlinkMConformanceTest.c
		)
		add_dependencies(mavlink_m_conformance_test mavlink_c_generate)
		target_include_directories(mavlink_m_conformance_test PRIVATE
			${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
			${CMAKE_CURRENT_SOURCE_DIR}
		)
		target_compile_options(mavlink_m_conformance_test PRIVATE
			-UNDEBUG
			-Wno-cast-align
			-Wno-unused-function
			-Wno-unused-parameter
		)
		if(BUILD_TESTING)
			add_test(NAME mavlink_m_conformance COMMAND mavlink_m_conformance_test --self-test)
		endif()
	endif()
endif()

px4_add_unit_gtest(SRC MavlinkStatustextHandlerTest.cpp
	INCLUDES
		${MAVLINK_LIBRARY_DIR}
		${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}
	COMPILE_FLAGS
		-Wno-address-of-packed-member # TODO: fix in c_library_v2
		-Wno-cast-align # TODO: fix
	LINKLIBS
		modules__mavlink
	)

if(CONFIG_NET AND "${PX4_PLATFORM}" MATCHES "nuttx")
	target_link_libraries(modules__mavlink PRIVATE nuttx_apps) # netlib_get_ipv4netmask
endif()
