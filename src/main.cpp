#include "math/vec.hpp"
#include "math/mat.hpp"
#include "math/MatrixTransform.hpp"
#include "math/Sphere.hpp"
#include "math/misc.hpp"
#include "math/Ray.hpp"
#include "math/TransformPair.hpp"
#include "noncopyable.hpp"
#include "output.hpp"
#include "Optional.hpp"
#include <vector>
#include <memory>
#include <algorithm>
#include <limits>

using namespace yks;

struct SceneObject;
struct SceneShape;

struct Intersection {
	float t;
	vec3 position;
	vec3 normal;
	vec2 uv;

	const SceneObject* object;
	const SceneShape* shape;
};

struct MaterialTexture {
	virtual vec3 getValue(const Intersection& i) const = 0;
};

struct TextureSolid : MaterialTexture {
	vec3 value;

	TextureSolid(const vec3& value)
		: value(value)
	{}

	TextureSolid(float r, float g, float b)
		: value(mvec3(r, g, b))
	{}

	virtual vec3 getValue(const Intersection& i) const override {
		(void)i;
		return value;
	}
};

struct TextureCheckerboard : MaterialTexture {
	std::shared_ptr<MaterialTexture> tex_a;
	std::shared_ptr<MaterialTexture> tex_b;

	TextureCheckerboard(const std::shared_ptr<MaterialTexture>& tex_a, const std::shared_ptr<MaterialTexture>& tex_b)
		: tex_a(tex_a), tex_b(tex_b)
	{}

	virtual vec3 getValue(const Intersection& i) const override {
		bool x = i.uv[0] - std::floor(i.uv[0]) < 0.5f;
		bool y = i.uv[1] - std::floor(i.uv[1]) < 0.5f;
		return x != y ? tex_a->getValue(i) : tex_b->getValue(i);
	}
};

struct TexMapScale : MaterialTexture {
	std::shared_ptr<MaterialTexture> tex;
	mat<2,3> map;

	TexMapScale(const std::shared_ptr<MaterialTexture>& tex, const mat<2,3>& map)
		: tex(tex), map(map)
	{}

	virtual vec3 getValue(const Intersection& i) const override {
		Intersection mapped_i = i;
		mapped_i.uv = map * mvec3(i.uv[0], i.uv[1], 1.0f);
		return tex->getValue(mapped_i);
	}
};

struct TexMapFromPosition : MaterialTexture {
	std::shared_ptr<MaterialTexture> tex;
	mat<2,4> map;

	TexMapFromPosition(const std::shared_ptr<MaterialTexture>& tex, const mat<2, 4>& map)
		: tex(tex), map(map)
	{}

	virtual vec3 getValue(const Intersection& i) const override {
		Intersection mapped_i = i;
		mapped_i.uv = map * mvec4(i.position, 1.0f);
		return tex->getValue(mapped_i);
	}
};

struct Material {
	std::shared_ptr<MaterialTexture> diffuse;
	std::shared_ptr<MaterialTexture> specular;

	Material(const std::shared_ptr<MaterialTexture>& diffuse, const std::shared_ptr<MaterialTexture>& specular)
		: diffuse(diffuse), specular(specular)
	{}
};

struct SceneShape {
	TransformPair transform;

	SceneShape(TransformPair transform)
		: transform(std::move(transform))
	{}

	virtual Optional<float> hasIntersection(const Ray& r) const = 0;

	Optional<float> hasIntersection(const Ray& r, float max_t) const {
		const Optional<float> t = hasIntersection(r);
		if (t && *t > max_t) {
			return Optional<float>();
		}
		return t;
	}

	virtual Optional<Intersection> intersect(const Ray& r) const = 0;

	Optional<Intersection> intersect(const Ray& r, float max_t) const {
		const Optional<Intersection> intersection = intersect(r);
		if (intersection && intersection->t > max_t) {
			return Optional<Intersection>();
		}
		return intersection;
	}
};

struct ShapeSphere : SceneShape {
	ShapeSphere(TransformPair transform)
		: SceneShape(transform)
	{}

	virtual Optional<float> hasIntersection(const Ray& r) const override {
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

	virtual Optional<Intersection> intersect(const Ray& r) const override {
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
};

struct ShapePlane : SceneShape {
	ShapePlane(TransformPair transform)
		: SceneShape(std::move(transform))
	{}

	virtual Optional<float> hasIntersection(const Ray& r) const override {
		const Ray local_ray = transform.localFromParent * r;
		const float t = -local_ray.origin[1] / local_ray.direction[1];
		if (t < 0.0f) {
			return Optional<float>();
		}
		return make_optional<float>(t);
	}

	virtual Optional<Intersection> intersect(const Ray& r) const override {
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
};

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

struct SceneLight {
	vec3 origin;
	vec3 intensity;

	SceneLight(const vec3& origin, const vec3& intensity)
		: origin(origin), intensity(intensity)
	{}
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
	const auto white = std::make_shared<TextureSolid>(vec3_1);
	const auto red = std::make_shared<TextureSolid>(vec3_x);

	Scene s(Camera(vec3_y * 0.2, orient(vec3_y, -vec3_z), 75.0f));
	s.objects.push_back(SceneObject(
		Material(black, std::make_shared<TextureSolid>(1.0f, 0.4f, 0.4f)),
		std::make_unique<ShapeSphere>(TransformPair().translate(mvec3(0.0f, 0.0f, -5.0f)))
		));
	s.objects.push_back(SceneObject(
		Material(white, black),
		std::make_unique<ShapeSphere>(TransformPair().scale(0.25f).translate(mvec3(-0.5f, 1.5f, -3.0f)))
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

	s.lights.push_back(SceneLight(mvec3(-2.0f, 4.0f, -4.0f), vec3_1 * 10));

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

vec3 calc_light_incidence(const Scene& scene, const Ray& ray, int remaining_depth) {
	vec3 color = vec3_0;

	const Optional<Intersection> surface_hit = find_nearest_intersection(scene, ray);
	if (surface_hit) {
		for (const SceneLight& light : scene.lights) {
			const vec3 light_dir = light.origin - surface_hit->position;
			if (!find_any_intersection(scene, Ray{surface_hit->position + surface_hit->normal * RAY_EPSILON, light_dir}, 1.0f)) {
				const vec3 albedo = surface_hit->object->material.diffuse->getValue(*surface_hit);
				color += albedo * std::max(0.0f, dot(normalized(light_dir), surface_hit->normal)) * (light.intensity * (1.0f / dot(light_dir, light_dir)));
			}
		}

		const vec3 specular_reflectance = surface_hit->object->material.specular->getValue(*surface_hit);
		if (remaining_depth > 0 && specular_reflectance != vec3_0) {
			color += specular_reflectance * calc_light_incidence(scene, Ray{surface_hit->position + surface_hit->normal * RAY_EPSILON, reflect(normalized(ray.direction), surface_hit->normal)}, remaining_depth-1);
		} else {
			color += specular_reflectance * 0.5f;
		}
	} else {
		color = vec3_0;
	}

	return color;
}

int main(int, char* []) {
	static const int IMAGE_WIDTH = 1280;
	static const int IMAGE_HEIGHT = 720;
	std::vector<vec3> image_data(IMAGE_WIDTH * IMAGE_HEIGHT);

	const Scene scene = setup_scene();

	for (int y = 0; y < IMAGE_HEIGHT; ++y) {
		for (int x = 0; x < IMAGE_WIDTH; x++) {
			const vec2 film_coord = filmspace_from_screenspace(mvec2(float(x), float(y)), mvec2(float(IMAGE_WIDTH), float(IMAGE_HEIGHT))) * mvec2(1.0f, -1.0f);
			const Ray camera_ray = scene.camera.createRay(film_coord);
			
			image_data[y*IMAGE_WIDTH + x] = calc_light_incidence(scene, camera_ray, 5);
		}
	}

	save_srgb_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
}