// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Misc helpers for device drivers.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup aux_util
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_device.h"

#ifdef __cplusplus
extern "C" {
#endif


extern const struct xrt_matrix_2x2 u_device_rotation_right;
extern const struct xrt_matrix_2x2 u_device_rotation_left;
extern const struct xrt_matrix_2x2 u_device_rotation_ident;
extern const struct xrt_matrix_2x2 u_device_rotation_180;

struct u_device_simple_info
{
	struct
	{
		uint32_t w_pixels;
		uint32_t h_pixels;
		float w_meters;
		float h_meters;
	} display;

	float lens_horizontal_separation_meters;
	float lens_vertical_position_meters;

	struct
	{
		float fov;
	} views[2];
};

/*!
 * Setup the device information given a very simple info struct.
 *
 * @return true on success.
 * @ingroup aux_util
 */
bool
u_device_setup_split_side_by_side(struct xrt_device* xdev,
                                  const struct u_device_simple_info* info);

/*!
 * Dump the device config to stderr.
 *
 * @ingroup aux_util
 */
void
u_device_dump_config(struct xrt_device* xdev,
                     const char* prefix,
                     const char* prod);


#ifdef __cplusplus
}
#endif
