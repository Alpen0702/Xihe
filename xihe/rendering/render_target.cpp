#include "render_target.h"

#include "common/error.h"
#include "common/vk_common.h"
#include "backend/device.h"

namespace xihe::rendering
{
const RenderTarget::CreateFunc RenderTarget::kDefaultCreateFunc = [](backend::Image &&swapchain_image) -> std::unique_ptr<RenderTarget> {
	vk::Format depth_format = get_suitable_depth_format(swapchain_image.get_device().get_gpu().get_handle());

	backend::Image depth_image{swapchain_image.get_device(), swapchain_image.get_extent(),
	                           depth_format,
	                           vk::ImageUsageFlagBits::eDepthStencilAttachment | vk::ImageUsageFlagBits::eTransientAttachment,
	                           VMA_MEMORY_USAGE_GPU_ONLY};

	std::vector<backend::Image> images;
	images.push_back(std::move(swapchain_image));
	images.push_back(std::move(depth_image));

	return std::make_unique<RenderTarget>(std::move(images));
};

RenderTarget::RenderTarget(std::vector<backend::Image> &&images):
    device_{images.front().get_device()}, images_{std::move(images)}
{
	assert(!images_.empty() && "Should specify at least 1 image");

	// check that every image is 2D
	auto it = std::ranges::find_if(images_, [](backend::Image const &image) { return image.get_type() != vk::ImageType::e2D; });
	if (it != images_.end())
	{
		throw VulkanException{VK_ERROR_INITIALIZATION_FAILED, "Image type is not 2D"};
	}

	extent_.width  = images_.front().get_extent().width;
	extent_.height = images_.front().get_extent().height;

	// check that every image has the same extent
	it = std::find_if(std::next(images_.begin()),
	                  images_.end(),
	                  [this](backend::Image const &image) { return (extent_.width != image.get_extent().width) || (extent_.height != image.get_extent().height); });
	if (it != images_.end())
	{
		throw VulkanException{VK_ERROR_INITIALIZATION_FAILED, "Extent size is not unique"};
	}

	for (auto &image : images_)
	{
		image_views_.emplace_back(image, vk::ImageViewType::e2D);
		attachments_.emplace_back(image.get_format(), image.get_sample_count(), image.get_usage());
	}
}
}
