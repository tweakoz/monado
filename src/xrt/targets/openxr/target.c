// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  The thing that binds all of the OpenXR driver together.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "util/u_meta_prober.h"


struct xrt_prober*
xrt_create_prober()
{
	return u_meta_prober_create();
}
