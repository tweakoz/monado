# Copyright 2019, Collabora, Ltd.
# SPDX-License-Identifier: BSL-1.0

######
# Create a test executable for prober functionality.

include_directories(
	${CMAKE_CURRENT_SOURCE_DIR}/../../auxiliary
	${CMAKE_CURRENT_SOURCE_DIR}/../../include
	)

set(SOURCE_FILES
	main.c
	)

# depends on above generated files
add_executable(prober
	${SOURCE_FILES}
	$<TARGET_OBJECTS:aux_util>
	$<TARGET_OBJECTS:aux_math>
	$<TARGET_OBJECTS:st_prober>
	)

target_link_libraries(prober PRIVATE
	${LIBUSB_LIBRARIES}
	${LIBUVC_LIBRARIES}
	)

if(BUILD_WITH_OPENHMD)
	target_sources(prober PRIVATE $<TARGET_OBJECTS:drv_ohmd>)
	target_compile_definitions(prober PRIVATE XRT_HAVE_OHMD)
	target_link_libraries(prober PRIVATE OpenHMD::OpenHMD)
endif()

if(BUILD_WITH_HIDAPI)
	target_sources(prober PRIVATE $<TARGET_OBJECTS:drv_hdk>)
	target_link_libraries(prober PUBLIC ${HIDAPI_LIBRARIES})
	target_compile_definitions(prober PRIVATE XRT_HAVE_HDK)
endif()
