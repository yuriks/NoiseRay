#include "./ShapeSphere.hpp"
#include "math/misc.hpp"
#include <cmath>

using namespace yks;

Optional<float> ShapeSphere::hasIntersection(const Ray& r) const {
	const Ray local_ray = transform.localFromParent * r;
	const vec3 o = local_ray.origin;
	const vec3 v = local_ray.direction;

	float t1, t2;
	const int solutions = solve_quadratic(dot(v, v), 2*dot(o, v), dot(o, o) - 1.0f, t1, t2);

	if (solutions == 0 || t1 < 0.0f) {
		return Optional<float>();
	}
	return make_optional<float>(t1);
}

Optional<Intersection> ShapeSphere::intersect(const Ray& r) const {
	const Ray local_ray = transform.localFromParent * r;
	const vec3 o = local_ray.origin;
	const vec3 v = local_ray.direction;

	float t1, t2;
	const int solutions = solve_quadratic(dot(v, v), 2*dot(o, v), dot(o, o) - 1.0f, t1, t2);

	if (solutions == 0 || t1 < 0.0f) {
		return Optional<Intersection>();
	}

	Intersection i;
	i.t = t1;
	i.position = r(t1);
	vec3 local_pos = local_ray(t1);
	i.uv = mvec2(std::atan2(local_pos[2], local_pos[0]) / (2*pi) + 0.5f, std::acos(local_pos[1]) / pi);
	i.normal = mvec3(transpose(transform.localFromParent) * mvec4(normalized(local_ray(t1)), 0.0f));
	return make_optional<Intersection>(i);
}
