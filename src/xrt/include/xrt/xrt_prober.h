// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Common interface to probe for devices.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup xrt_iface
 */

#pragma once

#include "xrt/xrt_config.h"
#include "xrt/xrt_device.h"

#ifdef XRT_HAVE_LIBUSB
#include <libusb-1.0/libusb.h>
#endif

#ifdef XRT_HAVE_LIBUVC
#include <libuvc/libuvc.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif


struct xrt_prober2;
struct xrt_prober_device;

struct xrt_prober_entry_usb
{
	uint16_t id_vendor;
	uint16_t id_product;
	void (*found)(struct xrt_prober2 *xp,
	              struct xrt_prober_device **devices,
	              size_t index);
	const char *name;
};

struct xrt_prober_entry_lists
{
	struct xrt_prober_entry_usb **usb;
	struct xrt_prober_entry_lists *next;
};

/*!
 * A probed device, may or may not be opened.
 *
 * @ingroup xrt_iface
 */
struct xrt_prober_device
{
#ifdef XRT_HAVE_LIBUSB
	struct
	{
		libusb_device *dev;
		bool suppressed;
	} usb;
#endif
#ifdef XRT_HAVE_LIBUVC
	struct
	{
		uvc_device_t *dev;
		bool suppressed;
	} uvc;
#endif
#ifdef XRT_OS_LINUX
	struct
	{
		const char **paths;
		bool suppressed;
	} v4l;
#endif

	bool opened;
};

/*!
 * A prober for HMD devices connected to the system.
 *
 * @ingroup xrt_iface
 */
struct xrt_prober2
{
	int (*probe)(struct xrt_prober2 *xp);
	int (*dump)(struct xrt_prober2 *xp);
	int (*select)(struct xrt_prober2 *xp, struct xrt_device **out_xdev);
	void (*destroy)(struct xrt_prober2 **xp);
};

/*!
 * Call this function to create the prober. This function is setup in the the
 * very small target wrapper.c for each binary.
 *
 * @ingroup xrt_iface
 */
int
xrt_prober_create(struct xrt_prober2 **out_prober);

/*!
 * Used by the target binary to create the prober with a list of drivers.
 *
 * @ingroup xrt_iface
 */
int
xrt_prober_create_with_lists(struct xrt_prober2 **out_prober,
                             struct xrt_prober_entry_lists *list);


/*
 *
 * Old
 *
 */

struct xrt_prober
{
	struct xrt_device *(*lelo_dallas_autoprobe)(struct xrt_prober *xdev);
	void (*destroy)(struct xrt_prober *xdev);
};

struct xrt_prober *
xrt_create_prober();


#ifdef __cplusplus
}
#endif
