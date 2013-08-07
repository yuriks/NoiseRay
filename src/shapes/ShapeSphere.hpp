#pragma once
#include "./SceneShape.hpp"

#include "math/Ray.hpp"
#include "math/TransformPair.hpp"
#include "Optional.hpp"

struct ShapeSphere : SceneShape {
	ShapeSphere(const yks::TransformPair& transform)
		: SceneShape(transform)
	{}

	virtual yks::Optional<float> hasIntersection(const yks::Ray& r) const override;
	virtual yks::Optional<Intersection> intersect(const yks::Ray& r) const override;
};
