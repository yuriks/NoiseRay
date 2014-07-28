#pragma once
#include "vec.hpp"
#include <cmath>

namespace yks {
	/// Generates an uniformly distributed point on the surface of the unit
	/// sphere centered on 0.  a and b are two numbers in [0,1).
	vec3 uniform_point_on_sphere(float a, float b);

	vec2 uniform_point_on_disk(float u1, float u2);

	vec3 cosine_weighted_point_on_hemisphere(float u1, float u2, const vec3 up);

	vec3 uniform_vector_in_cone(float a, float b, float cos_angle);

	inline vec3 uniform_point_on_hemisphere(float a, float b) {
		vec3 p = uniform_point_on_sphere(a, b);
		p[1] = std::abs(p[1]);
		return p;
	}

	// Useful functions for sampling
	vec3 find_orthogonal_vector(const vec3 v);
	vec3 reorient_vector(const vec3 v, const vec3 up);
}