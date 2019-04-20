// Copyright 2016, Joey Ferwerda.
// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  PSVR device implementation, imported from OpenHMD.
 * @author Joey Ferwerda <joeyferweda@gmail.com>
 * @author Philipp Zabel <philipp.zabel@gmail.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup drv_psvr
 */

#include "xrt/xrt_compiler.h"

#include "util/u_misc.h"
#include "util/u_time.h"
#include "util/u_debug.h"
#include "util/u_device.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "psvr_device.h"


/*
 *
 * Structs and defines.
 *
 */

DEBUG_GET_ONCE_BOOL_OPTION(psvr_disco, "PSVR_DISCO", false)

#define FEATURE_BUFFER_SIZE 256

/*!
 * Private struct for the @ref drv_psvr device.
 *
 * @ingroup drv_psvr
 */
struct psvr_device
{
	struct xrt_device base;

	hid_device *hmd_handle;
	hid_device *hmd_control;

	struct psvr_sensor_packet sensor;

	struct
	{
		struct xrt_vec3 gyro;
		struct xrt_vec3 accel;
	} raw;

	uint16_t buttons;

	bool print_spew;
	bool print_debug;
};

static const unsigned char psvr_power_on[8] = {
    0x17, 0x76, 0xaa, 0x04, 0x01, 0x00, 0x00, 0x00,
};

// This command turns it completely off, like you pressed the power button.
XRT_MAYBE_UNUSED static const unsigned char psvr_power_off[8] = {
    0x17, 0x00, 0xaa, 0x04, 0x00, 0x00, 0x00, 0x00,
};

// Alternative way to turn on all of the leds.
static const unsigned char psvr_tracking_on[12] = {
    0x11, 0x00, 0xaa, 0x08, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00,
};


#define PSVR_LED_POWER_OFF 0
#define PSVR_LED_POWER_MAX 100

enum psvr_leds
{
	PSVR_LED_A = (1 << 0),
	PSVR_LED_B = (1 << 1),
	PSVR_LED_C = (1 << 2),
	PSVR_LED_D = (1 << 3),
	PSVR_LED_E = (1 << 4),
	PSVR_LED_F = (1 << 5),
	PSVR_LED_G = (1 << 6),
	PSVR_LED_H = (1 << 7),
	PSVR_LED_I = (1 << 8),

	PSVR_LED_FRONT = PSVR_LED_A | PSVR_LED_B | PSVR_LED_C | PSVR_LED_D |
	                 PSVR_LED_E | PSVR_LED_F | PSVR_LED_G,

	PSVR_LED_BACK = PSVR_LED_H | PSVR_LED_I,

	PSVR_LED_ALL = PSVR_LED_FRONT | PSVR_LED_BACK,
};


/*
 *
 * Helpers and internal functions.
 *
 */

static inline struct psvr_device *
psvr_device(struct xrt_device *p)
{
	return (struct psvr_device *)p;
}

static int
open_hid(struct psvr_device *p,
         struct hid_device_info *dev_info,
         hid_device **out_dev)
{
	hid_device *dev = NULL;
	int ret;

	dev = hid_open_path(dev_info->path);
	if (dev == NULL) {
		PSVR_ERROR(p, "Failed to open '%s'", dev_info->path);
		return -1;
	}

	ret = hid_set_nonblocking(dev, 1);
	if (ret != 0) {
		PSVR_ERROR(p, "Failed to set non-blocking on device");
		hid_close(dev);
		return -1;
	}

	*out_dev = dev;
	return 0;
}

static int
send_to_control(struct psvr_device *psvr, const uint8_t *data, size_t size)
{
	return hid_write(psvr->hmd_control, data, size);
}

inline static uint8_t
read8(const unsigned char **buffer)
{
	uint8_t ret = **buffer;
	*buffer += 1;
	return ret;
}

inline static int16_t
read16(const unsigned char **buffer)
{
	int16_t ret = **buffer | (*(*buffer + 1) << 8);
	*buffer += 2;
	return ret;
}

inline static uint32_t
read32(const unsigned char **buffer)
{
	uint32_t ret = **buffer | (*(*buffer + 1) << 8) |
	               (*(*buffer + 2) << 16) | (*(*buffer + 3) << 24);
	*buffer += 4;
	return ret;
}


/*
 *
 * Packet reading code.
 *
 */

static void
accel_from_psvr_vec(const int16_t smp[3], struct xrt_vec3 *out_vec)
{
	//! @todo Figure out callibration data and use here.

	// clang-format off
	out_vec->x = (float)(smp[1] *  (9.81 / 16384.0));
	out_vec->y = (float)(smp[0] *  (9.81 / 16384.0));
	out_vec->z = (float)(smp[2] * -(9.81 / 16384.0));
	// clang-format on
}

static void
gyro_from_psvr_vec(const int16_t smp[3], struct xrt_vec3 *out_vec)
{
	//! @todo Figure out callibration data and use here.

	out_vec->x = (float)(smp[1] * 0.00105);
	out_vec->y = (float)(smp[0] * 0.00105);
	out_vec->z = (float)(smp[2] * 0.00105 * -1.0);
}

static void
update_fusion(struct psvr_device *psvr,
              struct psvr_sensor_sample *sample,
              uint32_t tick_delta)
{
	struct xrt_vec3 mag = {0.0f, 0.0f, 0.0f};
	float dt = tick_delta * PSVR_TICK_PERIOD;
	(void)mag;
	(void)dt;

	accel_from_psvr_vec(sample->accel, &psvr->raw.accel);
	gyro_from_psvr_vec(sample->gyro, &psvr->raw.gyro);

	// ofusion_update(&psvr->sensor_fusion, dt, &psvr->raw.gyro,
	//                &psvr->raw.accel, &mag);
}

static uint32_t
calc_delta_and_handle_rollover(uint32_t next, uint32_t last)
{
	uint32_t tick_delta = next - last;

	// The 24-bit tick counter has rolled over,
	// adjust the "negative" value to be positive.
	if (tick_delta > 0xffffff) {
		tick_delta += 0x1000000;
	}

	return tick_delta;
}

static void
handle_tracker_sensor_msg(struct psvr_device *psvr,
                          unsigned char *buffer,
                          int size)
{
	uint32_t last_sample_tick = psvr->sensor.samples[1].tick;

	if (!psvr_parse_sensor_packet(&psvr->sensor, buffer, size)) {
		PSVR_ERROR(psvr, "couldn't decode tracker sensor message");
	}

	struct psvr_sensor_packet *s = &psvr->sensor;

	// Simplest is the buttons.
	psvr->buttons = s->buttons;

	uint32_t tick_delta = 500;

	// Startup correction, ignore last_sample_tick if zero.
	if (last_sample_tick > 0) {
		tick_delta = calc_delta_and_handle_rollover(s->samples[0].tick,
		                                            last_sample_tick);

		// The PSVR device can buffer sensor data from previous
		// sessions which we can get at the start of new sessions.
		// @todo Maybe just skip the first 10 sensor packets?
		// @todo Maybe reset sensor fusion?
		if (tick_delta < 400 || tick_delta > 600) {
			PSVR_DEBUG(psvr, "tick_delta = %u", tick_delta);
			tick_delta = 500;
		}
	}

	// Update the fusion with first sample.
	update_fusion(psvr, &s->samples[0], tick_delta);

	// New delta between the two samples.
	tick_delta = calc_delta_and_handle_rollover(s->samples[1].tick,
	                                            s->samples[0].tick);

	// Update the fusion with second sample.
	update_fusion(psvr, &s->samples[1], tick_delta);
}

static void
handle_control_status_msg(struct psvr_device *psvr,
                          unsigned char *buffer,
                          int size)
{
	struct psvr_status_packet packet;

	if (!psvr_parse_status_packet(&packet, buffer, size)) {
		PSVR_ERROR(psvr, "couldn't decode tracker sensor message");
	}
}

static void
read_handle_packets(struct psvr_device *psvr)
{
	uint8_t buffer[FEATURE_BUFFER_SIZE];
	int size = 0;

	do {
		size = hid_read(psvr->hmd_handle, buffer, FEATURE_BUFFER_SIZE);
		if (size == 0) {
			break;
		}

		handle_tracker_sensor_msg(psvr, buffer, size);
	} while (true);
}

static void
read_control_packets(struct psvr_device *psvr)
{
	uint8_t buffer[FEATURE_BUFFER_SIZE];
	int size = 0;

	do {
		size = hid_read(psvr->hmd_control, buffer, FEATURE_BUFFER_SIZE);
		if (size == 0) {
			break;
		}

		if (buffer[0] == PSVR_PKG_STATUS) {
			handle_control_status_msg(psvr, buffer, size);
		} else {
			PSVR_DEBUG(psvr, "Got report, 0x%02x", buffer[0]);
		}

	} while (true);
}


/*
 *
 * Control sending functions.
 *
 */

static int
control_vrmode(struct psvr_device *psvr, bool on)
{
	const uint8_t data[8] = {
	    0x23, 0x00, 0xaa, 0x04, on, 0x00, 0x00, 0x00,
	};

	return send_to_control(psvr, data, sizeof(data));
}

/*!
 * Control the leds on the headset, allowing you to turn on and off different
 * leds with a single call.
 *
 * @param[in] psvr   The PSVR to control leds on.
 * @param[in] adjust The leds to adjust with @p power.
 * @param[in] power  The power level to give to @p adjust leds.
 * @param[in] off    Leds that should be turned off,
 *                   @p adjust has higher priority.
 * @ingroup drv_psvr
 */
static int
control_leds(struct psvr_device *psvr,
             enum psvr_leds adjust,
             uint8_t power,
             enum psvr_leds off)
{
	// Get the leds we should control and remove any extra bits.
	enum psvr_leds all = 0;
	all |= adjust;
	all |= off;
	all &= PSVR_LED_ALL;
	if (all == 0) {
		// Nothing todo.
		return 0;
	}

	// Just in case, if the value is larger
	// then max it will turn the leds off.
	if (power > PSVR_LED_POWER_MAX) {
		power = PSVR_LED_POWER_MAX;
	}

	uint8_t data[20] = {
	    0x15,
	    0x00,
	    0xaa,
	    0x10,
	    all,
	    all >> 8,
	    adjust & PSVR_LED_A ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_B ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_C ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_D ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_E ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_F ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_G ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_H ? power : PSVR_LED_POWER_OFF,
	    adjust & PSVR_LED_I ? power : PSVR_LED_POWER_OFF,
	    0,
	    0,
	    0,
	    0,
	    0,
	};

	return send_to_control(psvr, data, sizeof(data));
}

static int
disco_leds(struct psvr_device *psvr)
{
	static const uint16_t leds[] = {
	    // First loop
	    PSVR_LED_A,
	    PSVR_LED_E,
	    PSVR_LED_B,
	    PSVR_LED_G,
	    PSVR_LED_D,
	    PSVR_LED_C,
	    PSVR_LED_F,
	    // Second loop
	    PSVR_LED_A,
	    PSVR_LED_E,
	    PSVR_LED_B,
	    PSVR_LED_G,
	    PSVR_LED_D,
	    PSVR_LED_C,
	    PSVR_LED_F,
	    // Blink loop
	    PSVR_LED_BACK,
	    PSVR_LED_FRONT,
	    PSVR_LED_BACK,
	    PSVR_LED_FRONT,
	    // All on after loop
	    PSVR_LED_ALL,
	};

	for (size_t i = 0; i < ARRAY_SIZE(leds); i++) {
		int ret = control_leds(psvr, leds[i], PSVR_LED_POWER_MAX,
		                       PSVR_LED_ALL);
		if (ret < 0) {
			return ret;
		}

		usleep(100000);
	}

	return 0;
}

static void
teardown(struct psvr_device *psvr)
{

	if (psvr->hmd_control != NULL) {
		// Restore leds to the default settings.
		control_leds(psvr, PSVR_LED_BACK, PSVR_LED_POWER_MAX,
		             PSVR_LED_FRONT);

		// Turn off VR-mode.
		control_vrmode(psvr, false);

		hid_close(psvr->hmd_control);
		psvr->hmd_control = NULL;
	}

	if (psvr->hmd_handle != NULL) {
		hid_close(psvr->hmd_handle);
		psvr->hmd_handle = NULL;
	}
}


/*
 *
 * xrt_device functions.
 *
 */

static void
psvr_device_get_tracked_pose(struct xrt_device *xdev,
                             struct time_state *timekeeping,
                             int64_t *out_timestamp,
                             struct xrt_space_relation *out_relation)
{
	struct psvr_device *psvr = psvr_device(xdev);

	// Read all packets.
	read_handle_packets(psvr);
	read_control_packets(psvr);

	// Clear out the relation.
	memset(out_relation, 0, sizeof(*out_relation));

	int64_t now = time_state_get_now(timekeeping);
	//! @todo adjust for latency here
	*out_timestamp = now;

	out_relation->pose.orientation.w = 1.0f;

	//! @todo assuming that orientation is actually currently tracked.
	out_relation->relation_flags = (enum xrt_space_relation_flags)(
	    XRT_SPACE_RELATION_ORIENTATION_VALID_BIT |
	    XRT_SPACE_RELATION_ORIENTATION_TRACKED_BIT);

	PSVR_SPEW(psvr, "\n\taccel = %f %f %f\n\tgyro = %f %f %f",
	          psvr->raw.accel.x, psvr->raw.accel.y, psvr->raw.accel.z,
	          psvr->raw.gyro.x, psvr->raw.gyro.y, psvr->raw.gyro.z);
}

static void
psvr_device_get_view_pose(struct xrt_device *xdev,
                          struct xrt_vec3 *eye_relation,
                          uint32_t view_index,
                          struct xrt_pose *out_pose)
{
	struct xrt_pose pose = {{0.0f, 0.0f, 0.0f, 1.0f}, {0.0f, 0.0f, 0.0f}};
	bool adjust = view_index == 0;

	pose.position.x = eye_relation->x / 2.0f;
	pose.position.y = eye_relation->y / 2.0f;
	pose.position.z = eye_relation->z / 2.0f;

	// Adjust for left/right while also making sure there aren't any -0.f.
	if (pose.position.x > 0.0f && adjust) {
		pose.position.x = -pose.position.x;
	}
	if (pose.position.y > 0.0f && adjust) {
		pose.position.y = -pose.position.y;
	}
	if (pose.position.z > 0.0f && adjust) {
		pose.position.z = -pose.position.z;
	}

	*out_pose = pose;
}

static void
psvr_device_destroy(struct xrt_device *xdev)
{
	struct psvr_device *psvr = psvr_device(xdev);
	teardown(psvr);
	free(psvr);
}


/*
 *
 * Exported functions.
 *
 */

struct xrt_device *
psvr_device_create(struct hid_device_info *hmd_handle_info,
                   struct hid_device_info *hmd_control_info,
                   bool print_spew,
                   bool print_debug)
{
	struct psvr_device *psvr = U_TYPED_CALLOC(struct psvr_device);
	int ret;

	psvr->print_spew = print_spew;
	psvr->print_debug = print_debug;
	psvr->base.destroy = psvr_device_destroy;
	psvr->base.get_tracked_pose = psvr_device_get_tracked_pose;
	psvr->base.get_view_pose = psvr_device_get_view_pose;

	ret = open_hid(psvr, hmd_handle_info, &psvr->hmd_handle);
	if (ret != 0) {
		goto cleanup;
	}

	ret = open_hid(psvr, hmd_control_info, &psvr->hmd_control);
	if (ret < 0) {
		goto cleanup;
	}

	ret = send_to_control(psvr, psvr_power_on, sizeof(psvr_power_on));
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to send psvr_power_on! '%i'", ret);
		goto cleanup;
	}

	ret = control_vrmode(psvr, true);
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to switch to VR-mode! '%i'", ret);
		goto cleanup;
	}

	ret = send_to_control(psvr, psvr_tracking_on, sizeof(psvr_tracking_on));
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to send psvr_tracking_on! '%i'", ret);
		goto cleanup;
	}

	if (debug_get_bool_option_psvr_disco()) {
		ret = disco_leds(psvr);
	} else {
		ret = control_leds(psvr, PSVR_LED_ALL, PSVR_LED_POWER_MAX, 0);
	}
	if (ret < 0) {
		PSVR_ERROR(psvr, "Failed to control leds '%i'", ret);
		goto cleanup;
	}


	/*
	 * Device setup.
	 */

	struct u_device_simple_info info;
	info.display.w_pixels = 1980;
	info.display.h_pixels = 1080;
	info.display.w_meters = 0.126; // from calculated specs
	info.display.h_meters = 0.071;
	info.lens_horizontal_separation_meters = 0.0630999878f;
	info.lens_vertical_position_meters = 0.0394899882f;
	info.views[0].fov = 103.57f * M_PI / 180.0f;
	info.views[1].fov = 103.57f * M_PI / 180.0f;

	if (!u_device_setup_split_side_by_side(&psvr->base, &info)) {
		PSVR_ERROR(psvr, "Failed to setup basic device info");
		goto cleanup;
	}


	/*
	 * Finishing touches.
	 */

	if (psvr->print_debug) {
		u_device_dump_config(&psvr->base, __func__, "Sony PSVR");
	}

	PSVR_DEBUG(psvr, "YES!");

	return &psvr->base;


cleanup:
	teardown(psvr);
	free(psvr);

	PSVR_DEBUG(psvr, "NO! :(");

	return NULL;
}
