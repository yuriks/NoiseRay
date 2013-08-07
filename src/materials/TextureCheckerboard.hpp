#pragma once
#include "./Material.hpp"
#include "shapes/SceneShape.hpp"
#include "math/vec.hpp"
#include <memory>

struct TextureCheckerboard : MaterialTexture {
	std::shared_ptr<MaterialTexture> tex_a;
	std::shared_ptr<MaterialTexture> tex_b;

	TextureCheckerboard(const std::shared_ptr<MaterialTexture>& tex_a, const std::shared_ptr<MaterialTexture>& tex_b)
		: tex_a(tex_a), tex_b(tex_b)
	{}

	virtual yks::vec3 getValue(const Intersection& i) const override {
		bool x = i.uv[0] - std::floor(i.uv[0]) < 0.5f;
		bool y = i.uv[1] - std::floor(i.uv[1]) < 0.5f;
		return x != y ? tex_a->getValue(i) : tex_b->getValue(i);
	}
};
