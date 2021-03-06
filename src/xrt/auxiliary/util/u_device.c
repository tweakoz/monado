// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Misc helpers for device drivers.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include "util/u_device.h"


/*
 *
 * Matricies.
 *
 */

const struct xrt_matrix_2x2 u_device_rotation_right = {{
    .vecs =
        {
            {0, 1},
            {-1, 0},
        },
}};


const struct xrt_matrix_2x2 u_device_rotation_left = {{
    .vecs =
        {
            {0, -1},
            {1, 0},
        },
}};

const struct xrt_matrix_2x2 u_device_rotation_ident = {{
    .vecs =
        {
            {1, 0},
            {0, 1},
        },
}};

const struct xrt_matrix_2x2 u_device_rotation_180 = {{
    .vecs =
        {
            {-1, 0},
            {0, -1},
        },
}};

/*
 *
 * Print helpers.
 *
 */

#define PRINT(...) fprintf(stderr, __VA_ARGS__)

#define PRINT_STR(name, val) PRINT("\t%s = %s\n", name, val)

#define PRINT_INT(name, val) PRINT("\t%s = %u\n", name, val)

#define PRINT_MM(name, val)                                                    \
	PRINT("\t%s = %f (%i.%02imm)\n", name, val, (int32_t)(val * 1000.f),   \
	      abs((int32_t)(val * 100000.f)) % 100)

#define PRINT_ANGLE(name, val)                                                 \
	PRINT("\t%s = %f (%i°)\n", name, val, (int32_t)(val * (180 / M_PI)))

#define PRINT_MAT2X2(name, rot)                                                \
	PRINT("\t%s = {%f, %f} {%f, %f}\n", name, rot.v[0], rot.v[1],          \
	      rot.v[2], rot.v[3])

/*!
 * Dump the device config to stderr.
 */
void
u_device_dump_config(struct xrt_device* xdev,
                     const char* prefix,
                     const char* prod)
{
	// clang-format off
	fprintf(stderr, "%s - device_setup\n", prefix);
	PRINT_STR(   "prod", prod);
	PRINT_INT(   "screens[0].w_pixels ", xdev->screens[0].w_pixels);
	PRINT_INT(   "screens[0].h_pixels ", xdev->screens[0].h_pixels);
//	PRINT_MM(    "info.display.w_meters", info.display.w_meters);
//	PRINT_MM(    "info.display.h_meters", info.display.h_meters);
	PRINT_INT(   "views[0].viewport.x_pixels   ", xdev->views[0].viewport.x_pixels);
	PRINT_INT(   "views[0].viewport.y_pixels   ", xdev->views[0].viewport.y_pixels);
	PRINT_INT(   "views[0].viewport.w_pixels   ", xdev->views[0].viewport.w_pixels);
	PRINT_INT(   "views[0].viewport.h_pixels   ", xdev->views[0].viewport.h_pixels);
	PRINT_INT(   "views[0].display.w_pixels    ", xdev->views[0].display.w_pixels);
	PRINT_INT(   "views[0].display.h_pixels    ", xdev->views[0].display.h_pixels);
	PRINT_MM(    "views[0].display.w_meters    ", xdev->views[0].display.w_meters);
	PRINT_MM(    "views[0].display.h_meters    ", xdev->views[0].display.h_meters);
	PRINT_MM(    "views[0].lens_center.x_meters", xdev->views[0].lens_center.x_meters);
	PRINT_MM(    "views[0].lens_center.y_meters", xdev->views[0].lens_center.y_meters);
	PRINT_MAT2X2("views[0].rot            ", xdev->views[0].rot);
	PRINT_ANGLE( "views[0].fov.angle_left ", xdev->views[0].fov.angle_left);
	PRINT_ANGLE( "views[0].fov.angle_right", xdev->views[0].fov.angle_right);
	PRINT_ANGLE( "views[0].fov.angle_up   ", xdev->views[0].fov.angle_up);
	PRINT_ANGLE( "views[0].fov.angle_down ", xdev->views[0].fov.angle_down);
//	PRINT_ANGLE( "info.views[0].fov       ", info.views[0].fov);
	PRINT_INT(   "views[1].viewport.x_pixels   ", xdev->views[1].viewport.x_pixels);
	PRINT_INT(   "views[1].viewport.y_pixels   ", xdev->views[1].viewport.y_pixels);
	PRINT_INT(   "views[1].viewport.w_pixels   ", xdev->views[1].viewport.w_pixels);
	PRINT_INT(   "views[1].viewport.h_pixels   ", xdev->views[1].viewport.h_pixels);
	PRINT_INT(   "views[1].display.w_pixels    ", xdev->views[1].display.w_pixels);
	PRINT_INT(   "views[1].display.h_pixels    ", xdev->views[1].display.h_pixels);
	PRINT_MM(    "views[1].display.w_meters    ", xdev->views[1].display.w_meters);
	PRINT_MM(    "views[1].display.h_meters    ", xdev->views[1].display.h_meters);
	PRINT_MM(    "views[1].lens_center.x_meters", xdev->views[1].lens_center.x_meters);
	PRINT_MM(    "views[1].lens_center.y_meters", xdev->views[1].lens_center.y_meters);
	PRINT_MAT2X2("views[1].rot            ", xdev->views[1].rot);
	PRINT_ANGLE( "views[1].fov.angle_left ", xdev->views[1].fov.angle_left);
	PRINT_ANGLE( "views[1].fov.angle_right", xdev->views[1].fov.angle_right);
	PRINT_ANGLE( "views[1].fov.angle_up   ", xdev->views[1].fov.angle_up);
	PRINT_ANGLE( "views[1].fov.angle_down ", xdev->views[1].fov.angle_down);
//	PRINT_ANGLE( "info.views[1].fov       ", info.views[0].fov);
	// clang-format on
}
