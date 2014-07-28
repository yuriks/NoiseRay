#include "./ShapeSphere.hpp"
#include "math/misc.hpp"
#include "math/Sphere.hpp"
#include "math/sampling.hpp"
#include <cmath>

#include "Rng.hpp"

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

ShapeSample ShapeSphere::sampleArea(Rng& rng) const {
	const float a = rng.canonical();
	const float b = rng.canonical();
	return ShapeSample{
		mvec3(transform.parentFromLocal * mvec4(uniform_point_on_sphere(a, b) * radius, 1.0f)),
		1.0f / (4.0f * pi * sqr(radius)),
	};
}

//! Samples a random point on the surface of the sphere that is visible from \a source.
ShapeSample ShapeSphere::sampleArea(Rng& rng, const yks::vec3& source) const {
	const vec3 origin = mvec3(transform.parentFromLocal * mvec4(vec3_0, 1.0f));
	const float distance = length(source - origin);

	//! If the source point is inside the sphere, we fall back to sampling the
	//! entire sphere area by calling to \c sampleArea.
	if (distance <= radius) {
		return sampleArea(rng);
	}

	const float a = rng.canonical();
	const float b = rng.canonical();

	//! ![Sampling the visible cone on a sphere.](sphere_sampleAreaFromPoint.svg)
	//!
	//! Otherwise, we have to sample a cone of directions that are visible from
	//! the given source point. Notice that the highlighted red cone viewing
	//! from inside the sphere subtends the same points that would be visible
	//! from a cone starting from P and looking at the sphere.
	//!
	//! We need to find the angle \f$\theta\f$ of the cone (highlighted in red)
	//! which subtends the area visible from outside the sphere. The center of
	//! the sampled circle (\f$C\f$), the tangent point and the source point
	//! (\f$P\f$) form a right triangle and thus we can use trigonometry to find
	//! \f$\theta\f$: The sphere radius \f$r\f$ is the adjacent length of the
	//! triangle, while the distance \f$d\f$ between \f$P\f$ and \f$C\f$ is the
	//! hypotenuse. Thus, \f$\cos \theta = \frac{r}{d}\f$.
	const float cos_theta = radius / distance;

	//! The function \c uniform_vector_in_cone is used to sample the cone.
	const vec3 local_vec = uniform_vector_in_cone(a, b, cos_theta) * radius;
	//! Since \c uniform_vector_in_cone returns a vector around the cone
	//! pointing towards the y-axis, the vector is reoriented to point correctly
	//! towards the source point.
	const vec3 world_vec = reorient_vector(local_vec, source - origin) + origin;

	return ShapeSample{
		world_vec,
		//! The pdf is constant across the cone and can be calculated by
		//! integrating the area of its cap:
		/*! \f[
			\int_0^{2\pi} \int_0^{\theta} r^2 \sin \theta' d\theta' d\phi
			= 2\pi r^2 (1 - \cos \theta)
		\f] */
		1.0f / (two_pi * sqr(radius) * (1.0f - cos_theta))
	};
}
