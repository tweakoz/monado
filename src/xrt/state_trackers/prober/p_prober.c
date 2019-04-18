// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Main prober code.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup st_prober
 */

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_prober.h"
#include "util/u_misc.h"

#include <string.h>
#include <libusb-1.0/libusb.h>

#ifdef XRT_HAVE_LIBUVC
#include <libuvc/libuvc.h>
#endif


struct prober
{
	struct xrt_prober2 base;

	struct
	{
		libusb_context* ctx;
		libusb_device** list;
		ssize_t count;
	} usb;

#ifdef XRT_HAVE_LIBUVC
	struct
	{
		uvc_context_t* ctx;
		uvc_device_t** list;
		ssize_t count;
	} uvc;
#endif
};

struct prober_lookup_entry
{
	uint16_t idVendor;
	uint16_t idProduct;
	const char* name;
};

struct prober_lookup_entry table[] = {
    {0x1415, 0x2000, "PS3 Eye"},
};


/*
 *
 * Pre-declare functions.
 *
 */

static int
print_ports(char* tmp, size_t size, uint8_t* ports, int num);

static int
initialize(struct prober* x);

static void
teardown(struct prober* p);

static int
probe(struct xrt_prober2* xp);

static void
destroy(struct xrt_prober2 **xp);


/*
 *
 * Exported functions.
 *
 */

int
xrt_prober_create_with_lists(struct xrt_prober2** out_xp,
                             struct xrt_prober_entry_lists* lists)
{
	struct prober *p = U_TYPED_CALLOC(struct prober);

	int ret = initialize(p);
	if (ret != 0) {
		free(p);
		return ret;
	}

	*out_xp = &p->base;

	return 0;
}


/*
 *
 * Internal functions.
 *
 */

static int
initialize(struct prober *p)
{
	p->base.destroy = destroy;
	p->base.probe = probe;

	int ret;

	ret = libusb_init(&p->usb.ctx);
	if (ret != 0) {
		teardown(p);
		return -1;
	}

#ifdef XRT_HAVE_LIBUVC
	ret = uvc_init(&p->uvc.ctx, p->usb.ctx);
	if (ret != 0) {
		teardown(p);
		return -1;
	}
#endif

	return 0;
}

static void
teardown(struct prober* p)
{
#ifdef XRT_HAVE_LIBUVC
	// First UVC
	if (p->uvc.list != NULL) {
		uvc_free_device_list(p->uvc.list, 1);
		p->uvc.list = NULL;
	}

	if (p->uvc.ctx != NULL) {
		uvc_exit(p->uvc.ctx);
		p->uvc.ctx = NULL;
	}
#endif

	// Then USB
	if (p->usb.list != NULL) {
		libusb_free_device_list(p->usb.list, 1);
		p->usb.list = NULL;
	}

	if (p->usb.ctx != NULL) {
		libusb_exit(p->usb.ctx);
		p->usb.ctx = NULL;
	}
}


/*
 *
 * Member functions.
 *
 */

static int
probe(struct xrt_prober2* xp)
{
	struct prober *p = (struct prober *)xp;
	char tmp[1024];
	ssize_t i = 0;
	XRT_MAYBE_UNUSED ssize_t k = 0;
	size_t j = 0;
	XRT_MAYBE_UNUSED int ret = 0;

	p->usb.count = libusb_get_device_list(p->usb.ctx, &p->usb.list);
	if (p->usb.count < 0) {
		printf("\tFailed to enumerate usb devices\n");
		return -1;
	}

#ifdef XRT_HAVE_LIBUVC
	ret = uvc_get_device_list(p->uvc.ctx, &p->uvc.list);
	if (ret < 0) {
		printf("\tFailed to enumerate uvc devices\n");
		return -1;
	}

	// Count the number of UVC devices.
	while (p->uvc.list != NULL && p->uvc.list[p->uvc.count] != NULL) {
		p->uvc.count++;
	}

	printf("\tFound %i uvc device%s\n", (int)p->uvc.count,
	       p->uvc.count != 1 ? "s" : "");
	for (i = 0; i < p->uvc.count; i++) {
		uvc_device_t* device = p->uvc.list[i];
		struct uvc_device_descriptor* desc;

		uvc_get_device_descriptor(device, &desc);
		uint8_t bus = uvc_get_bus_number(device);
		uint8_t addr = uvc_get_device_address(device);

		printf("\t% 3i: 0x%04x:0x%04x\n", (int)i, desc->idVendor,
		       desc->idProduct);
		printf("\t\tbus  %3i\n\t\taddr %3i\n", bus, addr);
		if (desc->product != NULL) {
			printf("\t\tproduct: '%s'\n", desc->product);
		}
		if (desc->manufacturer != NULL) {
			printf("\t\tmanufacturer: '%s'\n", desc->manufacturer);
		}
		if (desc->serialNumber != NULL) {
			printf("\t\tserial: '%s'\n", desc->serialNumber);
		}

		uvc_free_device_descriptor(desc);
		desc = NULL;
	}
#endif

	printf("\tFound %i usb device%s\n", (int)p->usb.count,
	       p->usb.count != 1 ? "s" : "");
	for (i = 0; i < p->usb.count; i++) {
		libusb_device* device = p->usb.list[i];
		struct libusb_device_descriptor desc;
		uint8_t ports[8];

		libusb_get_device_descriptor(device, &desc);
		uint8_t bus = libusb_get_bus_number(device);
		uint8_t addr = libusb_get_device_address(device);
		int num =
		    libusb_get_port_numbers(device, ports, ARRAY_SIZE(ports));

		printf("\t% 3i: 0x%04x:0x%04x\n", (int)i, desc.idVendor,
		       desc.idProduct);
		printf("\t\tbus  %3i\n\t\taddr %3i\n", bus, addr);

		for (j = 0; j < ARRAY_SIZE(table); j++) {
			if (table[j].idProduct != desc.idProduct ||
			    table[j].idVendor != desc.idVendor) {
				continue;
			}
			printf("\t\tMatch to: %s\n", table[j].name);
		}

		if (print_ports(tmp, ARRAY_SIZE(tmp), ports, num)) {
			printf("\t\tport%s: %s\n", num > 1 ? "s" : "", tmp);
		}

#ifdef XRT_HAVE_LIBUVC
		for (k = 0; k < p->uvc.count; k++) {
			if (uvc_get_bus_number(p->uvc.list[k]) != bus ||
			    uvc_get_device_address(p->uvc.list[k]) != addr) {
				continue;
			}
			printf("\t\tMatch to UVC device #%i\n", (int)k);
		}
#endif
	}

	return 0;
}

static void
destroy(struct xrt_prober2 **xp)
{
	struct prober *p = (struct prober *)*xp;
	if (p == NULL) {
		return;
	}

	teardown(p);
	free(p);

	*xp = NULL;
}


/*
 *
 * Helper functions.
 *
 */

static int
print_ports(char* tmp, size_t size, uint8_t* ports, int num)
{
	switch (num) {
	case 1: {
		snprintf(tmp, size, "%i", ports[0]);
		return 1;
	}
	case 2: {
		snprintf(tmp, size, "%i.%i", ports[0], ports[1]);
		return 1;
	}
	case 3: {
		snprintf(tmp, size, "%i.%i.%i", ports[0], ports[1], ports[2]);
		return 1;
	}
	case 4: {
		snprintf(tmp, size, "%i.%i.%i.%i", ports[0], ports[1], ports[2],
		         ports[3]);
		return 1;
	}
	case 5: {
		snprintf(tmp, size, "%i.%i.%i.%i.%i", ports[0], ports[1],
		         ports[2], ports[3], ports[4]);
		return 1;
	}
	case 6: {
		snprintf(tmp, size, "%i.%i.%i.%i.%i.%i", ports[0], ports[1],
		         ports[2], ports[3], ports[4], ports[5]);
		return 1;
	}
	case 7: {
		snprintf(tmp, size, "%i.%i.%i.%i.%i.%i.%i", ports[0], ports[1],
		         ports[2], ports[3], ports[4], ports[5], ports[6]);
		return 1;
	}
	default: return 0;
	}
}
