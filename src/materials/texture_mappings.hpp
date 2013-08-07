#pragma once
#include "./Material.hpp"
#include "shapes/SceneShape.hpp"
#include <memory>
#include "math/vec.hpp"
#include "math/mat.hpp"

struct TexMapScale : MaterialTexture {
	std::shared_ptr<MaterialTexture> tex;
	yks::mat<2, 3> map;

	TexMapScale(const std::shared_ptr<MaterialTexture>& tex, const yks::mat<2, 3>& map)
		: tex(tex), map(map)
	{}

	virtual yks::vec3 getValue(const Intersection& i) const override {
		Intersection mapped_i = i;
		mapped_i.uv = map * yks::mvec3(i.uv[0], i.uv[1], 1.0f);
		return tex->getValue(mapped_i);
	}
};

struct TexMapFromPosition : MaterialTexture {
	std::shared_ptr<MaterialTexture> tex;
	yks::mat<2, 4> map;

	TexMapFromPosition(const std::shared_ptr<MaterialTexture>& tex, const yks::mat<2, 4>& map)
		: tex(tex), map(map)
	{}

	virtual yks::vec3 getValue(const Intersection& i) const override {
		Intersection mapped_i = i;
		mapped_i.uv = map * mvec4(i.position, 1.0f);
		return tex->getValue(mapped_i);
	}
};
