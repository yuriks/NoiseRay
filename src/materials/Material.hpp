#pragma once
#include "math/vec.hpp"
#include <memory>

struct Intersection;

struct MaterialTexture {
	virtual yks::vec3 getValue(const Intersection& i) const = 0;
};

struct Material {
	std::shared_ptr<MaterialTexture> diffuse;
	std::shared_ptr<MaterialTexture> emmision;

	Material(const std::shared_ptr<MaterialTexture>& diffuse, const std::shared_ptr<MaterialTexture>& emmision)
		: diffuse(diffuse), emmision(emmision)
	{}

	yks::vec3 diffuse_brdf(const Intersection& i, const yks::vec3 /*in_dir*/, const yks::vec3 /*out_dir*/) const {
		return diffuse->getValue(i) * (1.0f / yks::pi);
	}
};
