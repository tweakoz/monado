// Copyright 2018-2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  File for verifing app input into api functions.
 * @author Ryan Pavlik <ryan.pavlik@collabora.com>
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup oxr_main
 * @ingroup oxr_api
 */

#include <cstdio>
#include <cstring>

#include "xrt/xrt_compiler.h"
#include "util/u_debug.h"

#include "oxr_objects.h"
#include "oxr_logger.h"
#include "oxr_api_verify.h"


/*
 *
 * Path verification.
 *
 */

static bool
valid_path_char(const char c)
{
	if ('a' <= c && c <= 'z') {
		return true;
	}

	if ('0' <= c && c <= '9') {
		return true;
	}

	if (c == '-' || c == '_' || c == '.' || c == '/') {
		return true;
	}

	return false;
}

static bool
contains_zero(const char* path, uint32_t size)
{
	for (uint32_t i = 0; i < size; i++) {
		if (path[i] == '\0') {
			return true;
		}
	}

	return false;
}

extern "C" XrResult
oxr_verify_fixed_size_single_level_path(struct oxr_logger* log,
                                        const char* path,
                                        uint32_t array_size,
                                        const char* name)
{
	if (array_size == 0) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
		                 "(%s) internal runtime error", name);
	}

	if (path[0] == '\0') {
		return oxr_error(log, XR_ERROR_NAME_INVALID,
		                 "(%s) can not be empty", name);
	}

	if (!contains_zero(path, array_size)) {
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) must include zero termination '\\0'.",
		                 name);
	}

	size_t length = strlen(path);
	for (size_t i = 0; i < length; i++) {
		const char c = path[i];

		// Slashes are not valid in single level paths.
		if (valid_path_char(c) && c != '/') {
			continue;
		}

		return oxr_error(
		    log, XR_ERROR_PATH_FORMAT_INVALID,
		    "(%s) 0x%02x is not a valid character at position %u", name,
		    c, (uint32_t)i);
	}

	return XR_SUCCESS;
}

extern "C" XrResult
oxr_verify_localized_name(struct oxr_logger* log,
                          const char* string,
                          uint32_t array_size,
                          const char* name)
{
	if (array_size == 0) {
		return oxr_error(log, XR_ERROR_RUNTIME_FAILURE,
		                 "(%s) internal runtime error", name);
	}

	if (string[0] == '\0') {
		return oxr_error(log, XR_ERROR_NAME_INVALID,
		                 "(%s) can not be empty", name);
	}

	if (!contains_zero(string, array_size)) {
		return oxr_error(log, XR_ERROR_NAME_INVALID,
		                 "(%s) must include zero termination '\\0'.",
		                 name);
	}

	// Future work: validate well-formed UTF-8?
	return XR_SUCCESS;
}

enum class State
{
	Start,
	Middle,
	Slash,
	SlashDots,
};

extern "C" XrResult
oxr_verify_full_path_c(struct oxr_logger* log,
                       const char* path,
                       const char* name)
{
	// XR_MAX_PATH_LENGTH is max including null terminator,
	// length will not include null terminator
	size_t length = XR_MAX_PATH_LENGTH;
	for (int i = 0; i < XR_MAX_PATH_LENGTH; i++) {
		if (path[i] == '\0') {
			length = i;
			break;
		}
	}

	return oxr_verify_full_path(log, path, (uint32_t)length, name);
}

extern "C" XrResult
oxr_verify_full_path(struct oxr_logger* log,
                     const char* path,
                     size_t length,
                     const char* name)
{
	State state = State::Start;
	bool valid = true;

	if (length >= XR_MAX_PATH_LENGTH) {
		char formatted_path[XR_MAX_PATH_LENGTH + 6];
		snprintf(formatted_path, XR_MAX_PATH_LENGTH + 6, "%s[...]",
		         path);
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) the string '%s' is too long for a path, "
		                 "must be shorter than %u characters",
		                 name, formatted_path, XR_MAX_PATH_LENGTH);
	}

	for (uint32_t i = 0; i < length; i++) {
		const char c = path[i];
		switch (state) {
		case State::Start:
			if (c != '/') {
				return oxr_error(
				    log, XR_ERROR_PATH_FORMAT_INVALID,
				    "(%s) the string '%s' does not "
				    "start with a forward slash",
				    name, path);
			}
			state = State::Slash;
			break;
		case State::Slash:
			switch (c) {
			case '.':
				// Is valid and starts the SlashDot(s) state.
				state = State::SlashDots;
				break;
			case '/':
				return oxr_error(
				    log, XR_ERROR_PATH_FORMAT_INVALID,
				    "(%s) the string '%s' contains '//[/]*' "
				    "which is not valid in a path",
				    name, path);
			default:
				valid = valid_path_char(c);
				state = State::Middle;
			}
			break;
		case State::Middle:
			switch (c) {
			case '/': state = State::Slash; break;
			default:
				valid = valid_path_char(c);
				state = State::Middle;
			}
			break;
		case State::SlashDots:
			switch (c) {
			case '/':
				return oxr_error(
				    log, XR_ERROR_PATH_FORMAT_INVALID,
				    "(%s) the string '%s' contains '/.[.]*/' "
				    "which is not valid in a path",
				    name, path);
			case '.':
				// It's valid, more ShashDot(s).
				break;
			default:
				valid = valid_path_char(c);
				state = State::Middle;
			}
			break;
		}

		if (!valid) {
			return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
			                 "(%s) the string '%s' contains an "
			                 "invalid character "
			                 "0x%02x at position %u",
			                 name, path, c, (uint32_t)length);
		}
	}

	switch (state) {
	case State::Start:
		// Empty string
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) an empty string is not a valid path",
		                 name);
	case State::Slash:
		// Is this '/foo/' or '/'
		if (length > 1) {
			// It was '/foo/'
			return XR_SUCCESS;
		}
		// It was '/'
		return oxr_error(log, XR_ERROR_PATH_FORMAT_INVALID,
		                 "(%s) the string '%s' is not a valid path",
		                 name, path);
	case State::SlashDots:
		// Does the path ends with '/..'
		return oxr_error(
		    log, XR_ERROR_PATH_FORMAT_INVALID,
		    "(%s) the string '%s' ends with '/.[.]*' which "
		    "is not valid in a path",
		    name, path);

	case State::Middle:
		// '/foo/bar' okay!
		return XR_SUCCESS;
	default:
		// We should not end up here.
		return oxr_error(
		    log, XR_ERROR_RUNTIME_FAILURE,
		    "(%s) internal runtime error validating path '%s'", name,
		    path);
	}
}


/*
 *
 * Other verification.
 *
 */

extern "C" XrResult
oxr_verify_XrSessionCreateInfo(struct oxr_logger* log,
                               const struct oxr_instance* inst,
                               const XrSessionCreateInfo* createInfo)
{
	if (createInfo->type != XR_TYPE_SESSION_CREATE_INFO) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "createInfo->type");
	}

	if (createInfo->next == NULL) {
		if (inst->headless) {
			return XR_SUCCESS;
		}
		return oxr_error(log, XR_ERROR_GRAPHICS_DEVICE_INVALID,
		                 "createInfo->next");
	}

	XrStructureType* next_type = (XrStructureType*)createInfo->next;
#ifdef XR_USE_PLATFORM_XLIB
	if (*next_type == XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR) {
		if (!inst->opengl_enable) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "OpenGL "
			    "requires " XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);
		}
		return oxr_verify_XrGraphicsBindingOpenGLXlibKHR(
		    log, (XrGraphicsBindingOpenGLXlibKHR*)createInfo->next);
	} else
#endif
#ifdef XR_USE_GRAPHICS_API_VULKAN
	    if (*next_type == XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) {
		if (!inst->vulkan_enable) {
			return oxr_error(
			    log, XR_ERROR_VALIDATION_FAILURE,
			    "Vulkan "
			    "requires " XR_KHR_VULKAN_ENABLE_EXTENSION_NAME);
		}
		return oxr_verify_XrGraphicsBindingVulkanKHR(
		    log, (XrGraphicsBindingVulkanKHR*)createInfo->next);
	} else
#endif
	{
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "createInfo->next->type");
	}

	return XR_SUCCESS;
}


#ifdef XR_USE_PLATFORM_XLIB

extern "C" XrResult
oxr_verify_XrGraphicsBindingOpenGLXlibKHR(
    struct oxr_logger* log, const XrGraphicsBindingOpenGLXlibKHR* next)
{
	if (next->type != XR_TYPE_GRAPHICS_BINDING_OPENGL_XLIB_KHR) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "createInfo->next->type");
	}

	if (next->next != NULL) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "createInfo->next->next");
	}

	return XR_SUCCESS;
}

#endif


#ifdef XR_USE_GRAPHICS_API_VULKAN

extern "C" XrResult
oxr_verify_XrGraphicsBindingVulkanKHR(struct oxr_logger* log,
                                      const XrGraphicsBindingVulkanKHR* next)
{
	if (next->type != XR_TYPE_GRAPHICS_BINDING_VULKAN_KHR) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "createInfo->next->type");
	}

	if (next->next != NULL) {
		return oxr_error(log, XR_ERROR_VALIDATION_FAILURE,
		                 "createInfo->next->next");
	}

	return XR_SUCCESS;
}

#endif
