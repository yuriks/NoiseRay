#pragma once
#include "./SceneShape.hpp"

#include "math/Ray.hpp"
#include "math/TransformPair.hpp"
#include "Optional.hpp"

struct ShapeSphere : SceneShape {
	float radius;

	ShapeSphere(const yks::TransformPair& transform, float radius)
		: SceneShape(transform), radius(radius)
	{}

	virtual yks::Optional<float> hasIntersection(const yks::Ray& r) const override;
	virtual yks::Optional<Intersection> intersect(const yks::Ray& r) const override;
};
