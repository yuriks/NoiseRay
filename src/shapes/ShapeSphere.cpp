#include "./ShapeSphere.hpp"
#include "math/misc.hpp"
#include "math/Sphere.hpp"
#include <cmath>

using namespace yks;

Optional<float> ShapeSphere::hasIntersection(const Ray& r) const {
	const Ray local_ray = transform.localFromParent * r;
	return intersect_with_sphere(vec3_0, radius, local_ray);
}

Optional<Intersection> ShapeSphere::intersect(const Ray& r) const {
	const Ray local_ray = transform.localFromParent * r;
	const Optional<float> intersection = intersect_with_sphere(vec3_0, radius, local_ray);

	if (!intersection) {
		return Optional<Intersection>();
	}
	const float t = *intersection;

	Intersection i;
	i.t = t;
	i.position = r(t);
	vec3 local_pos = local_ray(t);
	i.uv = mvec2(std::atan2(local_pos[2], local_pos[0]) / (2*pi) + 0.5f, std::acos(local_pos[1]) / pi);
	i.normal = normalized(mvec3(transpose(transform.localFromParent) * mvec4(local_ray(t), 0.0f)));
	return make_optional<Intersection>(i);
}
