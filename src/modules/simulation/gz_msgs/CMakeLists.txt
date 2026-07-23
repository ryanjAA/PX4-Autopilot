############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# Find required packages
find_package(Protobuf)

if (Protobuf_FOUND)
    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})

    # Create library
    add_library(px4_gz_msgs STATIC
        ${PROTO_SRCS}
        ${PROTO_HDRS}
    )

    # Current Protobuf and Abseil releases require C++17. Publish the
    # requirement because consumers include the generated Protobuf headers.
    target_compile_features(px4_gz_msgs PUBLIC cxx_std_17)

    target_include_directories(px4_gz_msgs
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
    )
    target_include_directories(px4_gz_msgs SYSTEM
        PUBLIC
            ${Protobuf_INCLUDE_DIRS}
    )
    target_link_libraries(px4_gz_msgs PUBLIC ${PROTOBUF_LIBRARIES})
    target_compile_options(px4_gz_msgs PUBLIC
        -Wno-error=deprecated-declarations
        -Wno-error=double-promotion
    )
    if ("${CMAKE_CXX_COMPILER_ID}" MATCHES "Clang")
        target_compile_options(px4_gz_msgs PUBLIC -Wno-error=unused-private-field)
    endif()
    if (Protobuf_VERSION VERSION_LESS "3.8")
        target_compile_options(px4_gz_msgs PRIVATE -Wno-error=float-equal)
    endif()

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
endif()
