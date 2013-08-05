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

struct Material {
	vec3 diffuse;

	Material(vec3 diffuse)
		: diffuse(diffuse)
	{}
};

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
		
		if (solutions > 0) {
			return make_optional<float>(t1);
		} else {
			return Optional<float>();
		}
	}

	virtual Optional<Intersection> intersect(const Ray& r) const override {
		const Ray local_ray = transform.localFromParent * r;
		const vec3 o = local_ray.origin;
		const vec3 v = local_ray.direction;

		float t1, t2;
		const int solutions = solve_quadratic(dot(v, v), 2*dot(o, v), dot(o, o) - 1.0f, t1, t2);

		if (solutions == 0) {
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

	Scene(Camera camera)
		: camera(camera)
	{}

	Scene(Scene&& o)
		: camera(std::move(o.camera)), objects(std::move(o.objects))
	{}

private:
	NONCOPYABLE(Scene);
};

Scene setup_scene() {
	Scene s(Camera(vec3_0, orient(vec3_y, vec3_z), 75.0f));
	s.objects.push_back(SceneObject(
		Material(vec3_1),
		std::make_unique<ShapeSphere>(TransformPair().translate(vec3_z * 2))
		));
	s.objects.push_back(SceneObject(
		Material(mvec3(0.5f, 0.0f, 0.0f)),
		std::make_unique<ShapePlane>(TransformPair().translate(vec3_y * -1.5f))
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

int main(int, char* []) {
	static const int IMAGE_WIDTH = 1280;
	static const int IMAGE_HEIGHT = 720;
	std::vector<vec3> image_data(IMAGE_WIDTH * IMAGE_HEIGHT);

	const Scene scene = setup_scene();

	for (int y = 0; y < IMAGE_HEIGHT; ++y) {
		for (int x = 0; x < IMAGE_WIDTH; x++) {
			const vec2 film_coord = filmspace_from_screenspace(mvec2(float(x), float(y)), mvec2(float(IMAGE_WIDTH), float(IMAGE_HEIGHT))) * mvec2(1.0f, -1.0f);
			const Ray camera_ray = scene.camera.createRay(film_coord);

			const Optional<Intersection> hit = find_nearest_intersection(scene, camera_ray);
			const vec3 color = hit ? hit->normal * 0.5f + vec3_1 * 0.5f : vec3_1*0.1f;
			//const vec3 color = hit ? hit->object->material.diffuse : vec3_1*0.1f;
			
			image_data[y*IMAGE_WIDTH + x] = color;
		}
	}

	save_srgb_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
}