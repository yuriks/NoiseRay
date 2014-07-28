#include "sampling.hpp"

#include "Rng.hpp"
#include "mat.hpp"
#include "misc.hpp"

namespace yks {
	vec3 uniform_point_on_sphere(float a, float b) {
		const float y = 2.0f * a - 1.0f;
		const float longitude = b * two_pi;
		const float cos_latitude = std::sqrt(1.0f - sqr(y));

		return mvec3(cos_latitude * std::cos(longitude), y, cos_latitude * std::sin(longitude));
	}

	vec2 uniform_point_on_disk(const float u1, const float u2) {
		const float theta = two_pi * u1;
		const float r = std::sqrt(u2);
		return mvec2(r * std::cos(theta), r * std::sin(theta));
	}

	vec3 cosine_weighted_point_on_hemisphere(const float u1, const float u2, const vec3 up) {
		const vec2 p = uniform_point_on_disk(u1, u2);
		return reorient_vector(mvec3(p[0], std::sqrt(1.0f - length_sqr(p)), p[1]), up);
	}

	vec3 uniform_vector_in_cone(float a, float b, float cos_angle) {
		const float cos_theta = (1 - a) + a * cos_angle;
		const float sin_theta = std::sqrt(1 - sqr(cos_theta));
		const float phi = b * two_pi;
		return mvec3(std::cos(phi) * sin_theta, cos_theta, std::sin(phi) * sin_theta);
	}

	vec3 find_orthogonal_vector(const vec3 v) {
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

	vec3 reorient_vector(const vec3 v, const vec3 up) {
		const vec3 new_y = normalized(up);
		const vec3 new_x = normalized(find_orthogonal_vector(new_y));
		const vec3 new_z = normalized(cross(new_x, new_y));

		const mat3 base2 = { new_x, new_y, new_z };
		const mat3 base = transpose(base2);

		return base * v;
	}

}