#include "math/MatrixTransform.hpp"
#include "math/Ray.hpp"
#include "math/Sphere.hpp"
#include "math/TransformPair.hpp"
#include "math/mat.hpp"
#include "math/misc.hpp"
#include "math/sampling.hpp"
#include "math/vec.hpp"
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
#include <tbb/tbb.h>

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

	Camera(vec3 origin, mat3 orientation, float vertical_fov)
		: origin(origin), orientation(orientation), focal_length(focal_distance_from_fov(vertical_fov))
	{}

	Ray createRay(const vec2 film_pos) const {
		const vec3 cameraspace_ray = mvec3(film_pos[0], film_pos[1], focal_length);
		return Ray{origin, orientation * cameraspace_ray};
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
	const auto red = std::make_shared<TextureSolid>(vec3_x);

	Scene s(Camera(vec3_y * 0.2, orient(vec3_y, -vec3_z), 75.0f));
	s.objects.push_back(SceneObject(
		Material(std::make_shared<TextureSolid>(vec3_1 * 1.0f), black),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(0.0f, 0.0f, -5.0f)), 1.0f)
		));
	s.objects.push_back(SceneObject(
		Material(white, black),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(-0.5f, 1.5f, -3.0f)), 0.25f)
		));

	const mat<2, 4> plane_tex_mapping = {{
		1, 0, 0, 0,
		0, 0, 1, 0
	}};
	const auto checkerboard = std::make_shared<TextureCheckerboard>(white, red);

	s.objects.push_back(SceneObject(
		Material(std::make_shared<TexMapFromPosition>(checkerboard, plane_tex_mapping), black),
		std::make_unique<ShapePlane>(TransformPair().translate(vec3_y * -1.0f))
		));

	s.objects.push_back(SceneObject(
		Material(black, std::make_shared<TextureSolid>(vec3_1 * 160)),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(-2.0f, 4.0f, -4.0f)), 0.25f)
		));
	s.lights.push_back(s.objects.size() - 1);

#if 1
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
	depth += 1;
	const bool kill_ray = rng.canonical() > (1.5f / depth);
	if (kill_ray) {
		return vec3_0;
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
		color = lerp(mvec3(0.02f, 0.06f, 0.36f), mvec3(0.0f, 0.0f, 0.0f), 1.0f - std::pow(1.0f - vmax(0.0f, dot(ray.direction, vec3_y)), 2)) * 0.5f;
	}

	return color;
}

int main(int, char* []) {
	static const int IMAGE_WIDTH = 320;
	static const int IMAGE_HEIGHT = 240;

	const vec2 image_scale = mvec2(2.0f / IMAGE_HEIGHT, -2.0f / IMAGE_HEIGHT);
	const vec2 image_scale_offset = mvec2(-float(IMAGE_WIDTH) / IMAGE_HEIGHT, 1.0f);

	std::vector<vec3> image_data(IMAGE_WIDTH * IMAGE_HEIGHT);

	const Scene scene = setup_scene();
	Rng global_rng;
	global_rng.seed_with_default();
	tbb::spin_mutex rng_mutex;

	tbb::atomic<size_t> progress;
	progress = 0;

	tbb::parallel_for(tbb::blocked_range2d<int>(0, IMAGE_HEIGHT, 64, 0, IMAGE_WIDTH, 64), [&](const tbb::blocked_range2d<int>& range) {
		Rng rng;
		{
			tbb::spin_mutex::scoped_lock rng_lock;
			rng.seed_with_rng(global_rng);
		}

		for (int y = range.rows().begin(), y_end = range.rows().end(); y != y_end; ++y) {
			for (int x = range.cols().begin(), x_end = range.cols().end(); x != x_end; x++) {
				static const int NUM_IMAGE_SAMPLES = 1024;
				vec3 pixel_color = vec3_0;

				const vec2 pixel_pos = mvec2(float(x), float(y));

				for (int sample = 0; sample < NUM_IMAGE_SAMPLES; ++sample) {
					const vec2 sample_offset = mvec2(rng.canonical(), rng.canonical());
					const vec2 sample_pos = pixel_pos + sample_offset;

					const vec2 film_coord = sample_pos * image_scale + image_scale_offset;
					const Ray camera_ray = scene.camera.createRay(film_coord);

					pixel_color += calc_light_incidence(scene, rng, camera_ray, 0) * (1.0f / NUM_IMAGE_SAMPLES);
				}
				image_data[y*IMAGE_WIDTH + x] = pixel_color;
			}
		}
		size_t new_progress = (progress += range.rows().size() * range.cols().size());
		std::cout << (new_progress * 100.0f / (IMAGE_WIDTH * IMAGE_HEIGHT)) << "%\n";
	});

	tonemap_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
	save_srgb_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
}
