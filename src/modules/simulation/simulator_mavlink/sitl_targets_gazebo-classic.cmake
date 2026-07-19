
set(gazebo_classic_external_cmake_args)

if(APPLE)
	# The current OpenCV formula links against a newer Protobuf than Gazebo
	# Classic. Drop unused dylibs so non-camera models do not load both ABIs.
	list(APPEND gazebo_classic_external_cmake_args
		"-DCMAKE_SHARED_LINKER_FLAGS=-Wl,-dead_strip_dylibs")
endif()

if(APPLE AND EXISTS "/usr/local/opt/boost@1.85.0")
	# Gazebo Classic's Intel Homebrew formula uses a keg-only Boost version,
	# which is not discoverable from the default CMake search paths.
	list(PREPEND CMAKE_PREFIX_PATH "/usr/local/opt/boost@1.85.0")
	list(APPEND gazebo_classic_external_cmake_args
		"-DCMAKE_PREFIX_PATH=/usr/local/opt/boost@1.85.0")
endif()

if(APPLE AND EXISTS "/usr/local/opt/ogre1.9-with-boost1.85")
	set(ogre_prefix "/usr/local/opt/ogre1.9-with-boost1.85")
	list(PREPEND CMAKE_MODULE_PATH "${ogre_prefix}/share/OGRE/cmake/modules")
	set(ENV{PKG_CONFIG_PATH} "${ogre_prefix}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
	list(APPEND gazebo_classic_external_cmake_args
		"-DCMAKE_MODULE_PATH=${ogre_prefix}/share/OGRE/cmake/modules"
		"-DOGRE_HOME=${ogre_prefix}")
endif()

if(APPLE AND EXISTS "/usr/local/opt/protobuf@29")
	set(protobuf_prefix "/usr/local/opt/protobuf@29")
	set(Protobuf_DIR "${protobuf_prefix}/lib/cmake/protobuf")
	list(PREPEND CMAKE_PREFIX_PATH "${protobuf_prefix}")
	list(APPEND gazebo_classic_external_cmake_args
		"-DProtobuf_DIR=${Protobuf_DIR}"
		"-DProtobuf_PROTOC_EXECUTABLE=${protobuf_prefix}/bin/protoc"
		"-DProtobuf_INCLUDE_DIR=${protobuf_prefix}/include"
		"-DProtobuf_LIBRARY=${protobuf_prefix}/lib/libprotobuf.dylib"
		"-DCMAKE_CXX_FLAGS=-DTIXML_USE_STL -iquote${protobuf_prefix}/include")
endif()

if(APPLE AND EXISTS "/usr/local/opt/opencv@4")
	list(APPEND gazebo_classic_external_cmake_args
		"-DOpenCV_DIR=/usr/local/opt/opencv@4/lib/cmake/opencv4")
endif()

if(APPLE AND EXISTS "/usr/local/opt/tinyxml1")
	list(APPEND gazebo_classic_external_cmake_args
		"-DTinyXML_ROOT_DIR=/usr/local/opt/tinyxml1"
		"-DTinyXML_INCLUDE_DIR=/usr/local/opt/tinyxml1/include"
		"-DTinyXML_LIBRARY=/usr/local/opt/tinyxml1/lib/libtinyxml.dylib")
endif()

find_package(gazebo
	QUIET
)

if(gazebo_FOUND)

	message(STATUS "Found gazebo-classic ${gazebo_VERSION}, including sitl_gazebo-classic simulator and gazebo-classic targets")

	# Estimate an appropriate number of parallel jobs
	cmake_host_system_information(RESULT AVAILABLE_PHYSICAL_MEMORY QUERY AVAILABLE_PHYSICAL_MEMORY)
	cmake_host_system_information(RESULT NUMBER_OF_LOGICAL_CORES QUERY NUMBER_OF_LOGICAL_CORES)

	set(parallel_jobs 1)

	if(NOT NUMBER_OF_LOGICAL_CORES)
		include(ProcessorCount)
		ProcessorCount(NUMBER_OF_LOGICAL_CORES)
	endif()

	if(NOT AVAILABLE_PHYSICAL_MEMORY AND NUMBER_OF_LOGICAL_CORES GREATER_EQUAL 4)
		# Memory estimate unavailable, use N-2 jobs
		math(EXPR parallel_jobs "${NUMBER_OF_LOGICAL_CORES} - 2")
	endif()

	if(AVAILABLE_PHYSICAL_MEMORY)
		# Allow an additional job for every 1.5GB of available physical memory
		math(EXPR parallel_jobs "${AVAILABLE_PHYSICAL_MEMORY}/(3*1024/2)")
	else()
		set(AVAILABLE_PHYSICAL_MEMORY "?")
	endif()

	if(parallel_jobs GREATER NUMBER_OF_LOGICAL_CORES)
		set(parallel_jobs ${NUMBER_OF_LOGICAL_CORES})
	endif()

	if(parallel_jobs LESS 1)
		set(parallel_jobs 1)
	endif()

	# message(DEBUG "${NUMBER_OF_LOGICAL_CORES} logical cores detected and ${AVAILABLE_PHYSICAL_MEMORY} megabytes of memory available.
	# 		Limiting sitl_gazebo concurrent jobs to ${parallel_jobs}")

	# project to build sitl_gazebo if necessary
	px4_add_git_submodule(TARGET git_sitl_gazebo-classic PATH "${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic")
	include(ExternalProject)
	ExternalProject_Add(sitl_gazebo-classic
		SOURCE_DIR ${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic
		CMAKE_ARGS
			-DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
			-DCMAKE_POLICY_VERSION_MINIMUM=3.5
			-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
			-DSEND_ODOMETRY_DATA=ON
			-DGENERATE_ROS_MODELS=ON
			${gazebo_classic_external_cmake_args}
		BINARY_DIR ${PX4_BINARY_DIR}/build_gazebo-classic
		INSTALL_COMMAND ""
		DEPENDS git_sitl_gazebo-classic
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
		BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -- -j ${parallel_jobs}
	)

	# create targets for each model/world/debugger combination

	set(debuggers
		none
		gdb
		lldb
		valgrind
		callgrind
	)

	set(models
		advanced_plane
		believer
		boat
		cloudship
		glider
		iris
		iris_dual_gps
		iris_foggy_lidar
		iris_irlock
		iris_obs_avoid
		iris_opt_flow
		iris_opt_flow_mockup
		iris_rplidar
		iris_vision
		omnicopter
		plane
		plane_cam
		plane_catapult
		plane_lidar
		px4vision
		quadtailsitter
		r1_rover
		rover
		standard_vtol
		standard_vtol_drop
		tailsitter
		techpod
		tiltrotor
		typhoon_h480
		uuv_bluerov2_heavy
		uuv_hippocampus
	)

	set(worlds
		none
		baylands
		empty
		ksql_airport
		mcmillan_airfield
		sonoma_raceway
		warehouse
		windy
		yosemite
	)

	# find corresponding airframes
	file(GLOB gazebo-classic_airframes
		RELATIVE ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes
		${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/init.d-posix/airframes/*_gazebo-classic_*
	)

	# remove any .post files
	foreach(gazebo-classic_airframe IN LISTS gazebo-classic_airframes)
		if(gazebo-classic_airframe MATCHES ".post")
			list(REMOVE_ITEM gazebo-classic_airframes ${gazebo-classic_airframe})
		endif()
	endforeach()
	list(REMOVE_DUPLICATES gazebo-classic_airframes)

	foreach(gazebo-classic_airframe IN LISTS gazebo-classic_airframes)
		set(model_only)
		string(REGEX REPLACE ".*_gazebo-classic_" "" model_only ${gazebo-classic_airframe})

		if(EXISTS "${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${model_only}")

			if((EXISTS "${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${model_only}/${model_only}.sdf")
			OR (EXISTS "${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${model_only}/${model_only}.sdf.jinja"))
				#message(STATUS "SDF file found for ${model_only}")
			else()
				message(WARNING "No SDF file found for ${model_only}")
			endif()

		else()
			message(WARNING "model directory ${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/${model_only} not found")
		endif()
	endforeach()

	foreach(debugger ${debuggers})
		foreach(model ${models})

			# match model to airframe
			set(airframe_model_only)
			set(airframe_sys_autostart)
			set(gazebo-classic_airframe_found)
			foreach(gazebo-classic_airframe IN LISTS gazebo-classic_airframes)

				string(REGEX REPLACE ".*_gazebo-classic_" "" airframe_model_only ${gazebo-classic_airframe})
				string(REGEX REPLACE "_gazebo-classic_.*" "" airframe_sys_autostart ${gazebo-classic_airframe})

				if(model STREQUAL ${airframe_model_only})
					set(gazebo-classic_airframe_found ${gazebo-classic_airframe})
					break()
				endif()
			endforeach()

			if(gazebo-classic_airframe_found)
				#message(STATUS "gazebo-classic model: ${model} (${airframe_model_only}), airframe: ${gazebo-classic_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
			else()
				message(WARNING "gazebo-classic missing model: ${model} (${airframe_model_only}), airframe: ${gazebo-classic_airframe_found}, SYS_AUTOSTART: ${airframe_sys_autostart}")
			endif()

			foreach(world ${worlds})
				if(world STREQUAL "none")
					if(debugger STREQUAL "none")
						if(model STREQUAL "none")
							set(_targ_name "gazebo-classic")
						else()
							set(_targ_name "gazebo-classic_${model}")
						endif()
					else()
						if(model STREQUAL "none")
							set(_targ_name "gazebo-classic___${debugger}")
						else()
							set(_targ_name "gazebo-classic_${model}_${debugger}")
						endif()
					endif()

					add_custom_target(${_targ_name}
						COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
						WORKING_DIRECTORY ${SITL_WORKING_DIR}
						USES_TERMINAL
						DEPENDS px4 sitl_gazebo-classic
					)

					string(REPLACE "gazebo-classic" "gazebo" _targ_name_compat ${_targ_name})
					add_custom_target(${_targ_name_compat}
						COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "WARNING ${_targ_name_compat} target DEPRECATED, please use ${_targ_name}"
						COMMAND ${CMAKE_COMMAND} --build ${PX4_BINARY_DIR} -- ${_targ_name}
						USES_TERMINAL
						VERBATIM
					)
				else()
					if(debugger STREQUAL "none")
						if(model STREQUAL "none")
							set(_targ_name "gazebo-classic___${world}")
						else()
							set(_targ_name "gazebo-classic_${model}__${world}")
						endif()
					else()
						if(model STREQUAL "none")
							set(_targ_name "gazebo-classic__${debugger}_${world}")
						else()
							set(_targ_name "gazebo-classic_${model}_${debugger}_${world}")
						endif()
					endif()

					add_custom_target(${_targ_name}
						COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/gazebo-classic/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
						WORKING_DIRECTORY ${SITL_WORKING_DIR}
						USES_TERMINAL
						DEPENDS px4 sitl_gazebo-classic
					)

					string(REPLACE "gazebo-classic" "gazebo" _targ_name_compat ${_targ_name})
					add_custom_target(${_targ_name_compat}
						COMMAND ${CMAKE_COMMAND} -E cmake_echo_color --red "WARNING ${_targ_name_compat} target DEPRECATED, please use ${_targ_name}"
						COMMAND ${CMAKE_COMMAND} --build ${PX4_BINARY_DIR} -- ${_targ_name}
						USES_TERMINAL
						VERBATIM
					)
				endif()
			endforeach()
		endforeach()
	endforeach()

	add_custom_target(gazebo-classic DEPENDS gazebo-classic_iris) # alias
	add_custom_target(gazebo DEPENDS gazebo-classic_iris) # alias

	# mavsdk tests currently depend on sitl_gazebo
	ExternalProject_Add(mavsdk_tests
		SOURCE_DIR ${PX4_SOURCE_DIR}/test/mavsdk_tests
		CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
		BINARY_DIR ${PX4_BINARY_DIR}/mavsdk_tests
		INSTALL_COMMAND ""
		USES_TERMINAL_CONFIGURE true
		USES_TERMINAL_BUILD true
		EXCLUDE_FROM_ALL true
		BUILD_ALWAYS 1
	)
endif()
