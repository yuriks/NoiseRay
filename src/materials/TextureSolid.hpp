#pragma once
#include "./Material.hpp"
#include "math/vec.hpp"

struct TextureSolid : MaterialTexture {
	yks::vec3 value;

	TextureSolid(const yks::vec3& value)
		: value(value)
	{}

	TextureSolid(float r, float g, float b)
		: value(yks::mvec3(r, g, b))
	{}

	virtual yks::vec3 getValue(const Intersection&) const override {
		return value;
	}
};
