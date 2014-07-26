#include "output.hpp"
#include <cmath>
#include <cstdint>
#include <vector>
#include <ctime>
#include <cstdio>
#include "math/vec.hpp"
#include "stb_image_write.h"
#include "util.hpp"
#include "./srgb.hpp"

namespace yks {

static std::vector<uint8_t> encode_srgb(const std::vector<yks::vec3> fdata) {
	std::vector<uint8_t> bdata(fdata.size() * 3);
	for (size_t i = 0; i < fdata.size(); ++i) {
		vec3 p = fdata[i];
		if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
			p = mvec3(1.0f, 0.0f, 1.0f);
		}
		bdata[i*3 + 0] = byte_from_linear(clamp(0.0f, p[0], 1.0f));
		bdata[i*3 + 1] = byte_from_linear(clamp(0.0f, p[1], 1.0f));
		bdata[i*3 + 2] = byte_from_linear(clamp(0.0f, p[2], 1.0f));
	}
	return bdata;
}

void save_srgb_image(const std::vector<yks::vec3>& pixels, int width, int height) {
	std::vector<uint8_t> image_byte_data = encode_srgb(pixels);

	std::time_t t = std::time(nullptr);
	std::tm* tm = std::localtime(&t);

	char buffer[64];
	std::sprintf(buffer, "output/%04d-%02d-%02d_%02d-%02d-%02d_NoiseRay.png", tm->tm_year + 1900, tm->tm_mon + 1, tm->tm_mday, tm->tm_hour, tm->tm_min, tm->tm_sec);

	stbi_write_png(buffer, width, height, 3, image_byte_data.data(), 0);
}

void tonemap_image(std::vector<yks::vec3>& pixels, int width, int height) {
	for (int y = 0; y < height; ++y) {
		for (int x = 0; x < width; x++) {
			vec3 xyY = xyY_from_XYZ(XYZ_from_sRGB * pixels[y*width + x]);
			xyY[2] = xyY[2] / (1.0f + xyY[2]);
			pixels[y*width + x] = sRGB_from_XYZ * XYZ_from_xyY(xyY);
		}
	}
}

}
