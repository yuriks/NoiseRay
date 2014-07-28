#pragma once
#include "./vec.hpp"
#include "./Ray.hpp"
#include "Optional.hpp"

namespace yks {
	Optional<float> intersect_with_sphere(const vec3& origin, float radius, const Ray& r);
}
