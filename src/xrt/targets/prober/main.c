// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  A program to help test the probing code in Monado.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 */

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_prober.h"

#include <string.h>
#include <stdio.h>


void
ps3_eye_found(struct xrt_prober2 *xp,
              struct xrt_prober_device **devices,
              size_t index)
{
	printf("Found PS3 Eye!\n");
	devices[index]->usb.suppressed = true;
}

struct xrt_prober_entry_usb quirks_list[] = {
    {0x1415, 0x2000, ps3_eye_found, "PS3 Eye"},
    {0x0000, 0x0000, NULL, NULL}, // Terminate
};

struct xrt_prober_entry_usb *usb_lists[] = {
    quirks_list,
    NULL, // Terminate
};

struct xrt_prober_entry_lists list = {
    usb_lists,
    NULL,
};

int
xrt_prober_create(struct xrt_prober2 **out_xp)
{
	return xrt_prober_create_with_lists(out_xp, &list);
}

int
do_exit(struct xrt_prober2 **xp, int ret)
{
	if (*xp != NULL) {
		(*xp)->destroy(xp);
	}

	printf(" :: Exiting '%i'\n", ret);
	return ret;
}

int
main(int argc, const char **argv)
{
	struct xrt_prober2 *p = NULL;
	int ret = 0;

	printf(" :: Starting\n");

	ret = xrt_prober_create(&p);
	if (ret != 0) {
		return ret;
	}

	printf(" :: Started prober!\n");

	ret = p->probe(p);
	if (ret != 0) {
		return do_exit(&p, ret);
	}

	printf(" :: All ok, shutting down.\n");

	return do_exit(&p, 0);
}
