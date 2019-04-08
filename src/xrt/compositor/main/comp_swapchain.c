// Copyright 2019, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Swapchain code for the main compositor.
 * @author Jakob Bornecrantz <jakob@collabora.com>
 * @ingroup comp
 */

#include <stdio.h>
#include <stdlib.h>

#include "util/u_misc.h"

#include "main/comp_compositor.h"


static void
swapchain_destroy(struct xrt_swapchain *xsc)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);
	struct vk_bundle *vk = &sc->c->vk;

	COMP_SPEW(sc->c, "DESTROY");

	for (uint32_t i = 0; i < sc->base.base.num_images; i++) {
		comp_swapchain_image_cleanup(vk, &sc->images[i]);
	}

	free(sc);
}

static bool
swapchain_acquire_image(struct xrt_swapchain *xsc, uint32_t *index)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "ACQUIRE_IMAGE");
	*index = 0;
	return true;
}

static bool
swapchain_wait_image(struct xrt_swapchain *xsc,
                     uint64_t timeout,
                     uint32_t index)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "WAIT_IMAGE");
	return true;
}

static bool
swapchain_release_image(struct xrt_swapchain *xsc, uint32_t index)
{
	struct comp_swapchain *sc = comp_swapchain(xsc);

	COMP_SPEW(sc->c, "RELEASE_IMAGE");
	return true;
}

static VkResult
create_image_fd(struct comp_compositor *c,
                int64_t format,
                uint32_t width,
                uint32_t height,
                uint32_t mip_count,
                VkImage *out_image,
                VkDeviceMemory *out_mem,
                struct xrt_image_fd *out_image_fd)
{
	VkMemoryRequirements memory_requirements;
	VkImageUsageFlags image_usage = (VkImageUsageFlags)0;
	VkDeviceMemory device_memory = NULL;
	uint32_t memory_type_index = UINT32_MAX;
	VkImage image = NULL;
	VkResult ret;
	size_t size;
	int fd;

	COMP_SPEW(c, "->image - vkCreateImage %dx%d", width, height);


	/*
	 * Create the image.
	 */

	VkExternalMemoryImageCreateInfoKHR external_memory_image_create_info = {
	    .sType = VK_STRUCTURE_TYPE_EXTERNAL_MEMORY_IMAGE_CREATE_INFO_KHR,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR,
	};

	image_usage |= VK_IMAGE_USAGE_SAMPLED_BIT;
	image_usage |= VK_IMAGE_USAGE_TRANSFER_SRC_BIT;
	image_usage |= VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;

	VkImageCreateInfo info = {
	    .sType = VK_STRUCTURE_TYPE_IMAGE_CREATE_INFO,
	    .pNext = &external_memory_image_create_info,
	    .imageType = VK_IMAGE_TYPE_2D,
	    .format = (VkFormat)format,
	    .extent = {.width = width, .height = height, .depth = 1},
	    .mipLevels = mip_count,
	    .arrayLayers = 1,
	    .samples = VK_SAMPLE_COUNT_1_BIT,
	    .tiling = VK_IMAGE_TILING_OPTIMAL,
	    .usage = image_usage,
	    .sharingMode = VK_SHARING_MODE_EXCLUSIVE,
	    .initialLayout = VK_IMAGE_LAYOUT_UNDEFINED,
	};

	ret = c->vk.vkCreateImage(c->vk.device, &info, NULL, &image);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(c, "->image - vkCreateImage: %s",
		           vk_result_string(ret));
		// Nothing to cleanup
		return ret;
	}

	/*
	 * Get the size of the buffer.
	 */

	c->vk.vkGetImageMemoryRequirements(c->vk.device, image,
	                                   &memory_requirements);
	size = memory_requirements.size;


	// vkAllocateMemory parameters
	VkMemoryDedicatedAllocateInfoKHR dedicated_memory_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_DEDICATED_ALLOCATE_INFO_KHR,
	    .pNext = NULL,
	    .image = image,
	    .buffer = VK_NULL_HANDLE,
	};

	VkExportMemoryAllocateInfo export_alloc_info = {
	    .sType = VK_STRUCTURE_TYPE_EXPORT_MEMORY_ALLOCATE_INFO_KHR,
	    .pNext = &dedicated_memory_info,
	    .handleTypes = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR,
	};

	VkMemoryAllocateInfo alloc_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO,
	    .pNext = &export_alloc_info,
	    .allocationSize = size,
	};

	// vkGetMemoryFdKHR parameter
	VkMemoryGetFdInfoKHR fd_info = {
	    .sType = VK_STRUCTURE_TYPE_MEMORY_GET_FD_INFO_KHR,
	    .handleType = VK_EXTERNAL_MEMORY_HANDLE_TYPE_OPAQUE_FD_BIT_KHR,
	};

	if (!vk_get_memory_type(&c->vk, memory_requirements.memoryTypeBits,
	                        VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT,
	                        &memory_type_index)) {
		COMP_ERROR(c, "->image - _get_memory_type!");
		ret = VK_ERROR_OUT_OF_DEVICE_MEMORY;
		goto err_image;
	}
	alloc_info.memoryTypeIndex = memory_type_index;

	/*
	 * Create the memory.
	 */

	ret = c->vk.vkAllocateMemory(c->vk.device, &alloc_info, NULL,
	                             &device_memory);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(c, "->image - vkAllocateMemory: %s",
		           vk_result_string(ret));
		goto err_image;
	}
	fd_info.memory = device_memory;

	ret = c->vk.vkBindImageMemory(c->vk.device, image, device_memory, 0);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(c, "->image - vkBindImageMemory: %s",
		           vk_result_string(ret));
		goto err_mem;
	}


	/*
	 * Get the fd.
	 */

	ret = c->vk.vkGetMemoryFdKHR(c->vk.device, &fd_info, &fd);
	if (ret != VK_SUCCESS) {
		COMP_ERROR(c, "->image - vkGetMemoryFdKHR: %s",
		           vk_result_string(ret));
		ret = VK_ERROR_FEATURE_NOT_PRESENT;
		goto err_mem;
	}

	*out_image = image;
	*out_mem = device_memory;
	out_image_fd->fd = fd;
	out_image_fd->size = size;

	return ret;

err_mem:
	c->vk.vkFreeMemory(c->vk.device, device_memory, NULL);
err_image:
	c->vk.vkDestroyImage(c->vk.device, image, NULL);
	return ret;
}

struct xrt_swapchain *
comp_swapchain_create(struct xrt_compositor *xc,
                      enum xrt_swapchain_create_flags create,
                      enum xrt_swapchain_usage_bits bits,
                      int64_t format,
                      uint32_t sample_count,
                      uint32_t width,
                      uint32_t height,
                      uint32_t face_count,
                      uint32_t array_size,
                      uint32_t mip_count)
{
	struct comp_compositor *c = comp_compositor(xc);
	VkCommandBuffer cmd_buffer;
	uint32_t num_images = 3;
	VkResult ret;


	struct comp_swapchain *sc = U_TYPED_CALLOC(struct comp_swapchain);
	sc->base.base.destroy = swapchain_destroy;
	sc->base.base.acquire_image = swapchain_acquire_image;
	sc->base.base.wait_image = swapchain_wait_image;
	sc->base.base.release_image = swapchain_release_image;
	sc->base.base.num_images = num_images;
	sc->c = c;

	COMP_DEBUG(c, "CREATE %p %dx%d", (void *)sc, width, height);

	for (uint32_t i = 0; i < num_images; i++) {
		ret = create_image_fd(
		    c, format, width, height, mip_count, &sc->images[i].image,
		    &sc->images[i].memory, &sc->base.images[i]);
		if (ret != VK_SUCCESS) {
			return NULL;
		}

		vk_create_sampler(&c->vk, &sc->images[i].sampler);
		vk_create_view(&c->vk, sc->images[i].image, (VkFormat)format,
		               &sc->images[i].view);
	}


	/*
	 *
	 * Transition image.
	 *
	 */

	vk_init_cmd_buffer(&c->vk, &cmd_buffer);

	VkImageSubresourceRange subresource_range = {
	    .aspectMask = VK_IMAGE_ASPECT_COLOR_BIT,
	    .baseMipLevel = 0,
	    .levelCount = 1,
	    .baseArrayLayer = 0,
	    .layerCount = 1,
	};

	for (uint32_t i = 0; i < num_images; i++) {
		vk_set_image_layout(&c->vk, cmd_buffer, sc->images[i].image, 0,
		                    VK_ACCESS_SHADER_READ_BIT,
		                    VK_IMAGE_LAYOUT_UNDEFINED,
		                    VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL,
		                    subresource_range);
	}

	vk_submit_cmd_buffer(&c->vk, cmd_buffer);

	return &sc->base.base;
}

void
comp_swapchain_image_cleanup(struct vk_bundle *vk,
                             struct comp_swapchain_image *image)
{
	if (image->view != NULL) {
		vk->vkDestroyImageView(vk->device, image->view, NULL);
		image->view = NULL;
	}

	if (image->sampler != NULL) {
		vk->vkDestroySampler(vk->device, image->sampler, NULL);
		image->sampler = NULL;
	}

	if (image->image != NULL) {
		vk->vkDestroyImage(vk->device, image->image, NULL);
		image->image = NULL;
	}

	if (image->memory != NULL) {
		vk->vkFreeMemory(vk->device, image->memory, NULL);
		image->memory = NULL;
	}
}
