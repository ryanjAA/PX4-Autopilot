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

px4_add_git_submodule(TARGET git_mavlink_v2 PATH "${MAVLINK_GIT_DIR}")

set(MAVLINK_PRIMARY_GENERATOR_DIR "${MAVLINK_GIT_DIR}/pymavlink")
set(MAVLINK_PRIMARY_DEFINITION_DIR "${MAVLINK_GIT_DIR}/message_definitions/v1.0")
set(MAVLINK_PRIMARY_PYTHON_EXECUTABLE "${PYTHON_EXECUTABLE}")
set(MAVLINK_PRIMARY_PYTHONPATH "${MAVLINK_GIT_DIR}")
set(MAVLINK_PRIMARY_DEPENDENCIES git_mavlink_v2)
set(MAVLINK_PRIMARY_GENERATOR_OPTIONS)

if(CONFIG_MAVLINK_M_PRIVATE_PROFILE)
	if(NOT "${CONFIG_MAVLINK_DIALECT}" STREQUAL "mavlink_m")
		message(FATAL_ERROR "CONFIG_MAVLINK_M_PRIVATE_PROFILE requires CONFIG_MAVLINK_DIALECT=mavlink_m")
	endif()

	if("${PX4_PLATFORM}" STREQUAL "nuttx" AND NOT CONFIG_MODULES_UXRCE_DDS_CLIENT)
		message(STATUS
			"AAGS CAM MAVLink-M firmware: ROS 2 uXRCE-DDS client is disabled to preserve the FMUv6 flash budget"
		)
	endif()

	set(MAVLINK_M_GENERATOR_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink_m_generator")
	set(MAVLINK_M_DEFINITION_DIR "${CMAKE_CURRENT_LIST_DIR}/mavlink_m/message_definitions/v1.0")
	# A clean clone needs the normal PX4 initializer. During preparation of the
	# surrounding superproject commit, the checked-out gitlink already exists
	# but is not yet visible from HEAD, so avoid the interactive mismatch prompt.
	if(EXISTS "${MAVLINK_M_GENERATOR_DIR}/tools/mavgen.py")
		add_custom_target(git_mavlink_m_generator)
	else()
		px4_add_git_submodule(TARGET git_mavlink_m_generator PATH "${MAVLINK_M_GENERATOR_DIR}")
	endif()

	execute_process(
		COMMAND git -C "${MAVLINK_M_GENERATOR_DIR}" rev-parse HEAD
		OUTPUT_VARIABLE MAVLINK_M_GENERATOR_COMMIT
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_VARIABLE MAVLINK_M_GENERATOR_GIT_ERROR
		RESULT_VARIABLE MAVLINK_M_GENERATOR_GIT_RESULT
	)
	if(NOT "${MAVLINK_M_GENERATOR_GIT_RESULT}" STREQUAL "0")
		message(FATAL_ERROR
			"Unable to identify the MAVLink-M generator commit: ${MAVLINK_M_GENERATOR_GIT_ERROR}"
		)
	endif()
	if(NOT "${MAVLINK_M_GENERATOR_COMMIT}" STREQUAL "2a500b8acfb507255d02ed8257b6eb132e5b86d7")
		message(FATAL_ERROR "MAVLink-M generator commit mismatch: ${MAVLINK_M_GENERATOR_COMMIT}")
	endif()

	find_program(MAVLINK_M_PYTHON_EXECUTABLE NAMES python3.9)
	if(NOT MAVLINK_M_PYTHON_EXECUTABLE)
		message(FATAL_ERROR "MAVLink-M generation requires Python 3.9")
	endif()
	execute_process(
		COMMAND "${MAVLINK_M_PYTHON_EXECUTABLE}" -c
			"import sys; print(f'{sys.version_info.major}.{sys.version_info.minor}')"
		OUTPUT_VARIABLE MAVLINK_M_PYTHON_VERSION
		OUTPUT_STRIP_TRAILING_WHITESPACE
		ERROR_VARIABLE MAVLINK_M_PYTHON_VERSION_ERROR
		RESULT_VARIABLE MAVLINK_M_PYTHON_VERSION_RESULT
	)
	if(NOT "${MAVLINK_M_PYTHON_VERSION_RESULT}" STREQUAL "0")
		message(FATAL_ERROR
			"Unable to query the MAVLink-M Python version: ${MAVLINK_M_PYTHON_VERSION_ERROR}"
		)
	endif()
	if(NOT "${MAVLINK_M_PYTHON_VERSION}" STREQUAL "3.9")
		message(FATAL_ERROR "MAVLink-M generation requires Python 3.9, found ${MAVLINK_M_PYTHON_VERSION}")
	endif()

	file(STRINGS "${MAVLINK_M_GENERATOR_DIR}/__init__.py" MAVLINK_M_GENERATOR_VERSION_LINE
		REGEX "^__version__ = '2\\.4\\.49'$"
	)
	if(NOT MAVLINK_M_GENERATOR_VERSION_LINE)
		message(FATAL_ERROR "MAVLink-M generation requires pymavlink 2.4.49")
	endif()

	file(SHA256 "${MAVLINK_M_DEFINITION_DIR}/mavlink_m.xml" MAVLINK_M_CORE_SHA256)
	if(NOT "${MAVLINK_M_CORE_SHA256}" STREQUAL "02a88f79b9b4e58b271d9b0012a06087ac4ec975c75a8b1d7ba704c43bb55a8f")
		message(FATAL_ERROR "Dronecode MAVLink-M canonical XML hash mismatch: ${MAVLINK_M_CORE_SHA256}")
	endif()

	set(MAVLINK_PRIMARY_GENERATOR_DIR "${MAVLINK_M_GENERATOR_DIR}")
	set(MAVLINK_PRIMARY_DEFINITION_DIR "${MAVLINK_M_DEFINITION_DIR}")
	set(MAVLINK_PRIMARY_PYTHON_EXECUTABLE "${MAVLINK_M_PYTHON_EXECUTABLE}")
	set(MAVLINK_PRIMARY_PYTHONPATH "${CMAKE_CURRENT_LIST_DIR}/mavlink_m_pythonpath")
	list(APPEND MAVLINK_PRIMARY_DEPENDENCIES git_mavlink_m_generator)
	list(APPEND MAVLINK_PRIMARY_GENERATOR_OPTIONS --no-validate)
endif()

# always generate uAvionix dialect
set(MAVLINK_DIALECT_UAVIONIX "uAvionix")
add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h
	COMMAND
		${PYTHON_EXECUTABLE} ${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
			--lang C --wire-protocol 2.0
			--output ${MAVLINK_LIBRARY_DIR}
			${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${MAVLINK_DIALECT_UAVIONIX}.log
	DEPENDS
		git_mavlink_v2
		${MAVLINK_GIT_DIR}/pymavlink/tools/mavgen.py
		${MAVLINK_GIT_DIR}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml

	COMMENT "Generating Mavlink ${MAVLINK_DIALECT_UAVIONIX}: ${MAVLINK_GIT_DIR_RELATIVE}/message_definitions/v1.0/${MAVLINK_DIALECT_UAVIONIX}.xml"
)
add_custom_target(mavlink_c_generate_uavionix DEPENDS ${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h)
set_source_files_properties(${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h PROPERTIES GENERATED true)

# generate the primary dialect
add_custom_command(
	OUTPUT ${MAVLINK_LIBRARY_DIR}/${CONFIG_MAVLINK_DIALECT}/${CONFIG_MAVLINK_DIALECT}.h
	COMMAND
		${CMAKE_COMMAND} -E env
			PYTHONHASHSEED=0
			PYTHONPATH=${MAVLINK_PRIMARY_PYTHONPATH}
			${MAVLINK_PRIMARY_PYTHON_EXECUTABLE} ${MAVLINK_PRIMARY_GENERATOR_DIR}/tools/mavgen.py
				--lang C --wire-protocol 2.0
				${MAVLINK_PRIMARY_GENERATOR_OPTIONS}
				--output ${MAVLINK_LIBRARY_DIR}
				${MAVLINK_PRIMARY_DEFINITION_DIR}/${CONFIG_MAVLINK_DIALECT}.xml > ${CMAKE_CURRENT_BINARY_DIR}/mavgen_${CONFIG_MAVLINK_DIALECT}.log
	COMMAND
		${MAVLINK_PRIMARY_PYTHON_EXECUTABLE}
			${CMAKE_CURRENT_LIST_DIR}/mavlink_m/normalize_generated_headers.py
			${MAVLINK_LIBRARY_DIR}
	DEPENDS
		${MAVLINK_PRIMARY_DEPENDENCIES}
		${MAVLINK_PRIMARY_GENERATOR_DIR}/tools/mavgen.py
		${MAVLINK_PRIMARY_DEFINITION_DIR}/${CONFIG_MAVLINK_DIALECT}.xml
		${MAVLINK_PRIMARY_DEFINITION_DIR}/common.xml
		${MAVLINK_PRIMARY_DEFINITION_DIR}/standard.xml
		${MAVLINK_PRIMARY_DEFINITION_DIR}/minimal.xml
		${CMAKE_CURRENT_LIST_DIR}/mavlink_m/normalize_generated_headers.py
		${BOARD_DEFCONFIG}

		# generate after uAvionix
		mavlink_c_generate_uavionix
		${MAVLINK_LIBRARY_DIR}/${MAVLINK_DIALECT_UAVIONIX}/${MAVLINK_DIALECT_UAVIONIX}.h

	COMMENT "Generating Mavlink ${CONFIG_MAVLINK_DIALECT}: ${MAVLINK_PRIMARY_DEFINITION_DIR}/${CONFIG_MAVLINK_DIALECT}.xml"
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

set(MAVLINK_M_PRIVATE_SOURCES)
if(CONFIG_MAVLINK_M_PRIVATE_PROFILE)
	list(APPEND MAVLINK_M_PRIVATE_SOURCES MavlinkMHandler.cpp)
endif()

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
		${MAVLINK_M_PRIVATE_SOURCES}
		mavlink_shell.cpp
		mavlink_simple_analyzer.cpp
		mavlink_stream.cpp
		mavlink_timesync.cpp
		mavlink_ulog.cpp
		MavlinkStatustextHandler.cpp
		open_drone_id_translations.cpp
		tune_publisher.cpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		adsb
		airspeed
		component_general_json # for checksums.h
		dataman_client
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
	if(CONFIG_MAVLINK_M_PRIVATE_PROFILE AND "${PX4_PLATFORM}" MATCHES "posix")
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
