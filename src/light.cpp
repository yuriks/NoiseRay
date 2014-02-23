#include "light.hpp"

#include "math/misc.hpp"
#include "math/Sphere.hpp"

#include "Rng.hpp"

using namespace yks;

vec3 uniform_vector_in_cone(float a, float b, float cos_angle) {
	const float cos_theta = (1.0f - a) + a * cos_angle;
	const float sin_theta = std::sqrt(1.0f - sqr(cos_theta));
	const float phi = b * two_pi;
	return mvec3(std::cos(phi) * sin_theta, cos_theta, std::sin(phi) * sin_theta);
}

vec3 find_orthogonal_vector(const vec3& v) {
	if (std::abs(v[0]) > std::abs(v[1])) {
		float invlen = 1.0f / std::sqrt(v[0] * v[0] + v[2] * v[2]);
		return mvec3(-v[2] * invlen, 0.0f, v[0] * invlen);
	} else {
		float invlen = 1.0f / std::sqrt(v[1] * v[1] + v[2] * v[2]);
		return mvec3(0.0f, v[2] * invlen, -v[1] * invlen);
	}

	// Equivalent version, but which has some precision issues (IIRC)
	//return mvec3(0.0f, v[2], -v[1]);
}

SceneLight::SceneLight(const ShapeSphere& sphere, const vec3& total_power)
: ShapeSphere(sphere), intensity(total_power * (1.0f / (8 * pi*pi)) * (1.0f / sqr(radius)))
{}

LightSample SceneLight::samplePoint(Rng& rng) const {
	const float a = rng.canonical();
	const float b = rng.canonical();
	return LightSample{
		mvec3(transform.parentFromLocal * mvec4(uniform_point_on_sphere(a, b) * radius, 1.0f)),
		1.0f / (4.0f * pi * sqr(radius)),
	};
}

LightSample SceneLight::samplePoint(Rng& rng, const vec3& source) const {
	const vec3 origin = mvec3(transform.parentFromLocal * mvec4(vec3_0, 1.0f));

	const float a = rng.canonical();
	const float b = rng.canonical();

	// theta = angle of cone of directions on subtended sphere
	const float cos_theta = radius / length(origin - source);

	const vec3 new_y = normalized(source - origin);
	const vec3 new_x = normalized(find_orthogonal_vector(new_y));
	const vec3 new_z = normalized(cross(new_x, new_y));

	const mat3 base2 = { new_x, new_y, new_z };
	const mat3 base = transpose(base2);

	const vec3 final = (base * (uniform_vector_in_cone(a, b, cos_theta) * radius)) + origin;

	return LightSample{
		final,
		1.0f / (two_pi * radius * (1.0f - cos_theta)*radius)
	};
}

vec3 SceneLight::calcIntensity(const vec3& point, const vec3& direction) const {
	const vec3 origin = mvec3(transform.parentFromLocal * mvec4(vec3_0, 1.0f));
	return dot(point - origin, direction) >= 0.0f ? intensity : vec3_0;
}
