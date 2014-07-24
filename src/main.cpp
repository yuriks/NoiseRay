#include "math/MatrixTransform.hpp"
#include "math/Ray.hpp"
#include "math/Sphere.hpp"
#include "math/TransformPair.hpp"
#include "math/mat.hpp"
#include "math/misc.hpp"
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
	std::vector<SceneLight> lights;

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
		Material(black, std::make_shared<TextureSolid>(1.0f, 0.4f, 0.4f)),
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
		Material(std::make_shared<TexMapFromPosition>(checkerboard, plane_tex_mapping), white),
		std::make_unique<ShapePlane>(TransformPair().translate(vec3_y * -1.0f))
		));

	s.lights.push_back(SceneLight(
		ShapeSphere(TransformPair().translate(mvec3(-2.0f, 4.0f, -4.0f)), 0.25f),
		vec3_1 * 160
		));

	return std::move(s);
}

static vec2 filmspace_from_screenspace(const vec2 screen_pos, const vec2 screen_size) {
	return (screen_pos - (screen_size * 0.5f)) * 2.0f * (1.0f / screen_size[1]);
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

vec3 calc_light_incidence(const Scene& scene, Rng& rng, const Ray& ray, int remaining_depth) {
	vec3 color = vec3_0;

	const Optional<Intersection> surface_hit = find_nearest_intersection(scene, ray);
	if (surface_hit) {
		const vec3 albedo = surface_hit->object->material.diffuse->getValue(*surface_hit);

		for (const SceneLight& light : scene.lights) {
			const int NUM_LIGHT_SAMPLES = 64;
			vec3 light_contribution = vec3_0;

			for (int sample = 0; sample < NUM_LIGHT_SAMPLES; ++sample) {
				const LightSample light_sample = light.samplePoint(rng, surface_hit->position);
				const vec3 light_vec = light_sample.point - surface_hit->position;
				const vec3 light_dir = normalized(light_vec);

				const vec3 reflectance = albedo;

				if (reflectance != vec3_0 && !find_any_intersection(scene, Ray{surface_hit->position + surface_hit->normal * RAY_EPSILON, light_vec}, 1.0f)) {
					const vec3 illuminance = light.calcIntensity(light_sample.point, -light_vec) * (1.0f / length_sqr(light_vec));
					light_contribution += reflectance * illuminance  * vmax(0.0f, dot(light_dir, surface_hit->normal)) * (1.0f / light_sample.pdf);
				}
			}
			color += light_contribution * (1.0f / NUM_LIGHT_SAMPLES);
		}

		const vec3 specular_reflectance = surface_hit->object->material.specular->getValue(*surface_hit);
		if (remaining_depth > 0 && specular_reflectance != vec3_0) {
			color += specular_reflectance * calc_light_incidence(scene, rng, Ray{surface_hit->position + surface_hit->normal * RAY_EPSILON, reflect(normalized(ray.direction), surface_hit->normal)}, remaining_depth-1);
		} else {
			color += specular_reflectance * 0.5f;
		}
	} else {
		color = lerp(mvec3(1.0f, 0.2f, 0.0f), vec3_0, 1.0f - std::pow(1.0f - vmax(0.0f, dot(ray.direction, vec3_y)), 5)) * 0.5f;
	}

	return color;
}

int main(int, char* []) {
	static const int IMAGE_WIDTH = 1280;
	static const int IMAGE_HEIGHT = 720;
	std::vector<vec3> image_data(IMAGE_WIDTH * IMAGE_HEIGHT);

	const Scene scene = setup_scene();
	Rng global_rng;
	global_rng.seed_with_default();
	tbb::spin_mutex rng_mutex;

	tbb::atomic<size_t> progress;
	progress = 0;

	tbb::parallel_for(tbb::blocked_range2d<int>(0, IMAGE_HEIGHT, 256, 0, IMAGE_WIDTH, 256), [&](const tbb::blocked_range2d<int>& range) {
		Rng rng;
		{
			tbb::spin_mutex::scoped_lock rng_lock;
			rng.seed_with_rng(global_rng);
		}

		for (int y = range.rows().begin(), y_end = range.rows().end(); y != y_end; ++y) {
			for (int x = range.cols().begin(), x_end = range.cols().end(); x != x_end; x++) {
				static const int NUM_IMAGE_SAMPLES = 4;
				vec3 pixel_color = vec3_0;

				for (int sample = 0; sample < NUM_IMAGE_SAMPLES; ++sample) {
					const vec2 pixel_offset = mvec2(rng.canonical(), rng.canonical());
					const vec2 pixel_pos = mvec2(float(x), float(y));
					const vec2 film_coord = filmspace_from_screenspace(pixel_pos + pixel_offset, mvec2(float(IMAGE_WIDTH), float(IMAGE_HEIGHT))) * mvec2(1.0f, -1.0f);
					const Ray camera_ray = scene.camera.createRay(film_coord);

					pixel_color += calc_light_incidence(scene, rng, camera_ray, 50);
				}
				image_data[y*IMAGE_WIDTH + x] = pixel_color * (1.0f / NUM_IMAGE_SAMPLES);
			}
		}
		size_t new_progress = (progress += range.rows().size() * range.cols().size());
		std::cout << (new_progress * 100.0f / (IMAGE_WIDTH * IMAGE_HEIGHT)) << "%\n";
	});

	tonemap_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
	save_srgb_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
}
