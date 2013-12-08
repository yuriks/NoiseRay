#pragma once
#include <vector>
#include "math/vec.hpp"

namespace yks {
	void tonemap_image(std::vector<yks::vec3>& pixels, int width, int height);
	void save_srgb_image(const std::vector<yks::vec3>& pixels, int width, int height);
}
