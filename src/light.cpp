#include "light.hpp"

#include "math/misc.hpp"

#include "Rng.hpp"

using namespace yks;


SceneLight::SceneLight(const ShapeSphere& sphere, const vec3& total_power)
: ShapeSphere(sphere), intensity(total_power * (1.0f / (8 * pi*pi)) * (1.0f / sqr(radius)))
{}

vec3 SceneLight::calcIntensity(const vec3& point, const vec3& direction) const {
	const vec3 origin = mvec3(transform.parentFromLocal * mvec4(vec3_0, 1.0f));
	return dot(point - origin, direction) >= 0.0f ? intensity : vec3_0;
}
