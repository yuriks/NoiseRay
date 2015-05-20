#include "math/MatrixTransform.hpp"
#include "math/Ray.hpp"
#include "math/Sphere.hpp"
#include "math/TransformPair.hpp"
#include "math/mat.hpp"
#include "math/misc.hpp"
#include "math/sampling.hpp"
#include "math/vec.hpp"
#include "srgb.hpp"
#include "Optional.hpp"
#include "noncopyable.hpp"
#include "util.hpp"

#include "materials/Material.hpp"
#include "materials/TextureCheckerboard.hpp"
#include "materials/TextureSolid.hpp"
#include "materials/texture_mappings.hpp"

#include "shapes/SceneShape.hpp"
#include "shapes/ShapeSphere.hpp"
#include "shapes/ShapePlane.hpp"

#include "Rng.hpp"
#include "light.hpp"
#include "output.hpp"

#include <algorithm>
#include <limits>
#include <memory>
#include <vector>
#include <iostream>
#include <cstring>

#include <3ds.h>

using namespace yks;

struct SceneObject {
	Material material;
	std::unique_ptr<SceneShape> shape;

	SceneObject(Material material, std::unique_ptr<SceneShape>&& shape)
		: material(material), shape(std::move(shape))
	{}

	SceneObject(SceneObject&& o)
		: material(std::move(o.material)), shape(std::move(o.shape))
	{}

private:
	NONCOPYABLE(SceneObject);
};

float focal_distance_from_fov(const float fov_degrees) {
	const float half_fov = fov_degrees / 360.0f * pi;
	return std::tan(0.5f*pi - half_fov);
}

struct Camera {
	vec3 origin;
	mat3 orientation;
	float focal_length; // distance from image plane
	float focus_distance;

	Camera(vec3 origin, mat3 orientation, float vertical_fov, float focus_distance)
		: origin(origin), orientation(orientation), focal_length(focal_distance_from_fov(vertical_fov)), focus_distance(focus_distance)
	{}

	Ray createRay(const vec2 film_pos, float separation) const {
		const vec3 cameraspace_ray = mvec3(film_pos[0] - separation, film_pos[1], focal_length);
		vec3 offs = orientation * (vec3_x * separation);
		return Ray{origin + offs, orientation * (cameraspace_ray * (focus_distance / focal_length))};
	}
};

struct Scene {
	Camera camera;
	std::vector<SceneObject> objects;
	std::vector<size_t> lights;

	Scene(Camera camera)
		: camera(camera)
	{}

	Scene(Scene&& o)
		: camera(std::move(o.camera)), objects(std::move(o.objects)), lights(std::move(o.lights))
	{}

private:
	NONCOPYABLE(Scene);
};

Scene setup_scene() {
	const auto black = std::make_shared<TextureSolid>(vec3_0);
	const auto white = std::make_shared<TextureSolid>(vec3_1 * 0.75f);
	const auto red = std::make_shared<TextureSolid>(vec3_x * 0.9f);
	const auto dark_red = std::make_shared<TextureSolid>(vec3_x * 0.45f);

	Scene s(Camera(vec3_y * 0.2, orient(vec3_y, -vec3_z), 45.0f, 4.5f));
	s.objects.push_back(SceneObject(
		Material(std::make_shared<TextureSolid>(vec3_1 * 1.0f), black),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(0.0f, 0.0f, -5.0f)), 1.0f)
		));
	s.objects.push_back(SceneObject(
		Material(white, black),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(-0.5f, 1.5f, -4.0f)), 0.25f)
		));

	const mat<2, 4> plane_tex_mapping = {{
		1, 0, 0, 0,
		0, 0, 1, 0
	}};
	const auto checkerboard = std::make_shared<TextureCheckerboard>(dark_red, red);

	s.objects.push_back(SceneObject(
		Material(std::make_shared<TexMapFromPosition>(checkerboard, plane_tex_mapping), black),
		std::make_unique<ShapePlane>(TransformPair().translate(vec3_y * -1.0f))
		));

	s.objects.push_back(SceneObject(
		Material(black, std::make_shared<TextureSolid>(vec3_1 * 160)),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(-2.0f, 4.0f, -4.0f)), 0.25f)
		));
	s.lights.push_back(s.objects.size() - 1);

#if 0
	s.objects.push_back(SceneObject(
		Material(black, std::make_shared<TextureSolid>(vec3_1 * 500)),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(-4.0f, 0.0f, -5.0f)), 0.1f)
		));
	s.lights.push_back(s.objects.size() - 1);
#endif

	return std::move(s);
}

Optional<Intersection> find_nearest_intersection(const Scene& scene, const Ray ray) {
	Optional<Intersection> nearest_intersection;

	for (const SceneObject& object : scene.objects) {
		const Optional<Intersection> intersection = object.shape->intersect(ray);
		if (intersection && (!nearest_intersection || intersection->t < nearest_intersection->t)) {
			nearest_intersection = intersection;
			nearest_intersection->object = &object;
			nearest_intersection->shape = object.shape.get();
		}
	}
	
	return nearest_intersection;
}

bool find_any_intersection(const Scene& scene, const Ray& ray, float max_t) {
	for (const SceneObject& object : scene.objects) {
		if (object.shape->hasIntersection(ray, max_t)) {
			return true;
		}
	}

	return false;
}

vec3 reflect(const vec3& l, const vec3& n) {
	return l + 2*dot(n, -l) * n;
}

static const float RAY_EPSILON = 1e-6f;

// dw/dA = cos(theta)/r^2
// dw = cos(theta)/r^2 dA
// dA = r^2/cos(theta) dw

static float power_heuristic(float main_pdf, float other_pdf) {
	return sqr(main_pdf) / (sqr(main_pdf) + sqr(other_pdf));
}

static Ray ray_from_surface(const Intersection& hit, const vec3 out_vec) {
	return Ray{ hit.position + hit.normal * RAY_EPSILON, out_vec };
}

vec3 calc_light_incidence(const Scene& scene, Rng& rng, const Ray& ray, int depth) {
	float ray_weight = 1.0f;

	depth += 1;
	if (depth > 2) {
		const float live_probability = 0.75f;

		if (rng.canonical() > live_probability) {
			return vec3_0;
		} else {
			ray_weight = 1.0f / live_probability;
		}
	}

	vec3 color = vec3_0;

	const Optional<Intersection> surface_hit = find_nearest_intersection(scene, ray);
	if (surface_hit) {
		const vec3 out_dir = -normalized(ray.direction);

		const size_t light_index = (size_t)(rng.canonical() * scene.lights.size());
		const SceneObject* light = &scene.objects[scene.lights[light_index]];
		const float light_weight = (float)scene.lights.size();

		// Sample BRDF
		{
			const vec3 in_dir = cosine_weighted_point_on_hemisphere(rng.canonical(), rng.canonical(), surface_hit->normal);
			const vec3 reflectance = surface_hit->object->material.diffuse_brdf(*surface_hit, in_dir, out_dir);
			const float cos_term = dot(in_dir, surface_hit->normal);

			const float brdf_pdf = cos_term / pi;

			const Optional<Intersection> light_hit = light->shape->intersect(ray_from_surface(*surface_hit, in_dir));
			const float light_pdf = (light_hit ? light->shape->areaPdf(surface_hit->position, in_dir) : 0.f);

			if (brdf_pdf != 0.f && reflectance != vec3_0) {
				const vec3 illuminance = calc_light_incidence(scene, rng, ray_from_surface(*surface_hit, in_dir), depth);
				color += (1.0f / brdf_pdf) * cos_term * reflectance * illuminance * power_heuristic(brdf_pdf, light_pdf);
			}
		}

		// Sample lights
		{
			const ShapeSample light_sample = light->shape->sampleArea(rng, surface_hit->position);
			const vec3 light_vec = light_sample.point - surface_hit->position;
			const vec3 in_dir = normalized(light_vec);
			const float cos_term = vmax(0.f, dot(in_dir, surface_hit->normal));

			const Optional<Intersection> light_hit = find_nearest_intersection(scene, ray_from_surface(*surface_hit, light_vec));
			bool occluded = !light_hit || light_hit->object != light;
			const float light_pdf = light_weight * light_sample.pdf;

			const float brdf_pdf = cos_term / pi;

			const vec3 reflectance = surface_hit->object->material.diffuse_brdf(*surface_hit, in_dir, out_dir);
			if (!occluded && light_pdf != 0.f && reflectance != vec3_0) {
				const vec3 illuminance = calc_light_incidence(scene, rng, ray_from_surface(*surface_hit, in_dir), depth);
				const float differential_area = -dot(light_hit->normal, in_dir) / length_sqr(light_vec);
				color += light_weight * (1.0f / light_pdf) * differential_area * cos_term * reflectance * illuminance * power_heuristic(light_pdf, brdf_pdf);
			}
		}

		if (dot(out_dir, surface_hit->normal) >= 0.f) {
			color += surface_hit->object->material.emmision->getValue(*surface_hit);
		}
	} else {
		//color = lerp(mvec3(1.0f, 0.2f, 0.0f), mvec3(0.35f, 0.9f, 1.0f), 1.0f - std::pow(1.0f - vmax(0.0f, dot(ray.direction, vec3_y)), 2)) * 0.5f;
		color = lerp(mvec3(0.02f, 0.06f, 0.36f), mvec3(0.0f, 0.0f, 0.0f), 1.0f - std::pow(1.0f - vmax(0.0f, dot(normalized(ray.direction), vec3_y)), 2.0f)) * 0.5f;
	}

	return color * ray_weight;
}

yks::vec3 tonemap_pixel(yks::vec3 pixel) {
	vec3 xyY = xyY_from_XYZ(XYZ_from_sRGB * pixel);
	xyY[2] = xyY[2] / (1.0f + xyY[2]);
	return sRGB_from_XYZ * XYZ_from_xyY(xyY);
}

u32 wait_input() {
	while (true) {
		hidScanInput();
		u32 keys = hidKeysDown();
		if (keys != 0)
			return keys;
		gspWaitForVBlank();
	}
}

#define CONFIG_3D_SLIDERSTATE (*(volatile	float*)0x1FF81080)

bool use_3d = true;

bool render() {
	u8* fb_l = gfxGetFramebuffer(GFX_TOP, GFX_LEFT, nullptr, nullptr);
	u8* fb_r = gfxGetFramebuffer(GFX_TOP, GFX_RIGHT, nullptr, nullptr);
	std::memset(fb_l, 0, 400 * 240 * 3);
	std::memset(fb_r, 0, 400 * 240 * 3);

	float slider = CONFIG_3D_SLIDERSTATE;
	float separation = slider * 0.5;

	static const int IMAGE_WIDTH = 400;
	static const int IMAGE_HEIGHT = 240;

	const vec2 image_scale = mvec2(2.0f / IMAGE_HEIGHT, -2.0f / IMAGE_HEIGHT);
	const vec2 image_scale_offset = mvec2(-float(IMAGE_WIDTH) / IMAGE_HEIGHT, 1.0f);

	const Scene scene = setup_scene();
	Rng rng;
	rng.seed_with_default();

	std::vector<vec3> image_buffer_l(IMAGE_WIDTH * IMAGE_HEIGHT);
	std::vector<vec3> image_buffer_r(IMAGE_WIDTH * IMAGE_HEIGHT);
	unsigned int num_samples = 0;

	while (true) {
		num_samples += 1;

		for (int y = 0, y_end = IMAGE_HEIGHT; y != y_end; ++y) {
			for (int x = 0, x_end = IMAGE_WIDTH; x != x_end; x++) {
				const vec2 pixel_pos = mvec2(float(x), float(y));

				const vec2 sample_offset = mvec2(rng.canonical(), rng.canonical());
				const vec2 sample_pos = pixel_pos + sample_offset;
				const vec2 film_coord = sample_pos * image_scale + image_scale_offset;

				{
					const Ray camera_ray = scene.camera.createRay(film_coord, -separation);

					vec3 pixel_color = calc_light_incidence(scene, rng, camera_ray, 0) * (1.0f / num_samples);
					vec3 tmp = image_buffer_l[y * IMAGE_WIDTH + x];
					tmp = tmp * (float(num_samples - 1) / float(num_samples)) + pixel_color;
					image_buffer_l[y * IMAGE_WIDTH + x] = tmp;

					vec3 p = tonemap_pixel(tmp);
					if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
						p = mvec3(1.0f, 0.0f, 1.0f);
					}
					size_t pixel_ofs = (x * 240 + (240 - 1 - y)) * 3;
					fb_l[pixel_ofs + 2] = byte_from_linear(clamp(0.0f, p[0], 1.0f));
					fb_l[pixel_ofs + 1] = byte_from_linear(clamp(0.0f, p[1], 1.0f));
					fb_l[pixel_ofs + 0] = byte_from_linear(clamp(0.0f, p[2], 1.0f));
				}

				{
					const Ray camera_ray = scene.camera.createRay(film_coord, separation);

					vec3 pixel_color = calc_light_incidence(scene, rng, camera_ray, 0) * (1.0f / num_samples);
					vec3 tmp = image_buffer_r[y * IMAGE_WIDTH + x];
					tmp = tmp * (float(num_samples - 1) / float(num_samples)) + pixel_color;
					image_buffer_r[y * IMAGE_WIDTH + x] = tmp;

					vec3 p = tonemap_pixel(tmp);
					if (!std::isfinite(p[0]) || !std::isfinite(p[1]) || !std::isfinite(p[2])) {
						p = mvec3(1.0f, 0.0f, 1.0f);
					}
					size_t pixel_ofs = (x * 240 + (240 - 1 - y)) * 3;
					fb_r[pixel_ofs + 2] = byte_from_linear(clamp(0.0f, p[0], 1.0f));
					fb_r[pixel_ofs + 1] = byte_from_linear(clamp(0.0f, p[1], 1.0f));
					fb_r[pixel_ofs + 0] = byte_from_linear(clamp(0.0f, p[2], 1.0f));
				}
			}

			gfxFlushBuffers();
			gfxSwapBuffers();

			hidScanInput();
			u32 down = hidKeysDown();
			if (down & KEY_B) {
				return false;
			}
			if (down & KEY_Y) {
				use_3d = !use_3d;
				gfxSet3D(use_3d);
			}

			if (slider != CONFIG_3D_SLIDERSTATE) {
				return true;
			}
		}
	}
}

u32 __stacksize__ = 256 * 1024;

int main(int, char* []) {
	gfxInitDefault();
	gfxSet3D(true);
	gfxSetDoubleBuffering(GFX_TOP, false);

	while (render());

	wait_input();
	gfxExit();
}