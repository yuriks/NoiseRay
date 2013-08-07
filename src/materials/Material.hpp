#pragma once
#include "math/vec.hpp"
#include <memory>

struct Intersection;

struct MaterialTexture {
	virtual yks::vec3 getValue(const Intersection& i) const = 0;
};

struct Material {
	std::shared_ptr<MaterialTexture> diffuse;
	std::shared_ptr<MaterialTexture> specular;

	Material(const std::shared_ptr<MaterialTexture>& diffuse, const std::shared_ptr<MaterialTexture>& specular)
		: diffuse(diffuse), specular(specular)
	{}
};
