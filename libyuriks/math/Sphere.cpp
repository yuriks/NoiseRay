#include "./Sphere.hpp"
#include "./misc.hpp"
#include <algorithm>
#include <cmath>

namespace yks {

	Optional<float> intersect_with_sphere(const vec3& origin, float radius, const Ray& r) {
		const vec3 o = r.origin - origin;
		const vec3 v = r.direction;

		float t1, t2;
		const int solutions = solve_quadratic(dot(v, v), 2*dot(o, v), dot(o, o) - radius*radius, t1, t2);

		if (solutions == 0 || t1 < 0.0f) {
			return Optional<float>();
		} else {
			return make_optional<float>(t1);
		}
	}

}
