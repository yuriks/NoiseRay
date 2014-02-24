#pragma once
#include "math/vec.hpp"

#include "shapes/ShapeSphere.hpp"

struct Rng;

struct LightSample {
	yks::vec3 point;
	float pdf;
};

struct SceneLight : ShapeSphere {
	yks::vec3 intensity;

	// Emittance = Power / Area = Power / (4*pi*radius^2)
	// Intensity = Emittance / (2*pi) = Power / (8*pi^2*radius^2)
	SceneLight(const ShapeSphere& sphere, const yks::vec3& total_power);
	LightSample samplePoint(Rng& rng) const;
	yks::vec3 calcIntensity(const yks::vec3& point, const yks::vec3& direction) const;
};
