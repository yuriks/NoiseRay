#include "./ShapePlane.hpp"

using namespace yks;

Optional<float> ShapePlane::hasIntersection(const Ray& r) const {
	const Ray local_ray = transform.localFromParent * r;
	const float t = -local_ray.origin[1] / local_ray.direction[1];
	if (t < 0.0f) {
		return Optional<float>();
	}
	return make_optional<float>(t);
}

Optional<Intersection> ShapePlane::intersect(const Ray& r) const {
	const Ray local_ray = transform.localFromParent * r;
	const float t = -local_ray.origin[1] / local_ray.direction[1];
	if (t < 0.0f) {
		return Optional<Intersection>();
	}
	Intersection i;
	i.t = t;
	i.position = r(t);
	i.uv = mvec2(0.0f, 0.0f);
	i.normal = mvec3(transpose(transform.localFromParent) * mvec4(vec3_y, 0.0f));
	return make_optional<Intersection>(i);
}
