#pragma once
#include "math/Ray.hpp"
#include "math/TransformPair.hpp"
#include "math/vec.hpp"
#include "Optional.hpp"
#include <utility>

namespace yks {
	struct Rng;
}

struct SceneObject;
struct SceneShape;

struct Intersection {
	float t;
	yks::vec3 position;
	yks::vec3 normal;
	yks::vec2 uv;

	const SceneObject* object;
	const SceneShape* shape;
};

struct ShapeSample {
	yks::vec3 point;
	float pdf;
};

struct SceneShape {
	yks::TransformPair transform;

	SceneShape(yks::TransformPair transform)
		: transform(std::move(transform))
	{}

	virtual yks::Optional<float> hasIntersection(const yks::Ray& r) const = 0;

	yks::Optional<float> hasIntersection(const yks::Ray& r, float max_t) const {
		const yks::Optional<float> t = hasIntersection(r);
		if (t && *t > max_t) {
			return yks::Optional<float>();
		}
		return t;
	}

	virtual yks::Optional<Intersection> intersect(const yks::Ray& r) const = 0;

	yks::Optional<Intersection> intersect(const yks::Ray& r, float max_t) const {
		const yks::Optional<Intersection> intersection = intersect(r);
		if (intersection && intersection->t > max_t) {
			return yks::Optional<Intersection>();
		}
		return intersection;
	}

	virtual ShapeSample sampleArea(yks::Rng& /* rng */) const {
		assert(false);
		std::abort();
	};
	virtual ShapeSample sampleArea(yks::Rng& /* rng */, const yks::vec3& /* source */) const {
		assert(false);
		std::abort();
	}
};
