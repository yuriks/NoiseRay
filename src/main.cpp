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

struct SceneShape {
	TransformPair transform;

	SceneShape(TransformPair transform)
		: transform(std::move(transform))
	{}

	virtual Optional<float> intersect(const Ray r) const = 0;
};

struct ShapeSphere : SceneShape {
	float radius;

	ShapeSphere(TransformPair transform, float radius)
		: SceneShape(transform), radius(radius)
	{}

	virtual Optional<float> intersect(const Ray r) const {
		const Ray local_ray = transform.localFromParent * r;
		const vec3 o = local_ray.origin;
		const vec3 v = local_ray.direction;

		float t1, t2;
		const int solutions = solve_quadratic(dot(v, v), 2*dot(o, v), dot(o, o) - radius*radius, t1, t2);
		
		if (solutions == 0) {
			return Optional<float>();
		} else if (solutions == 1) {
			return make_optional<float>(t1);
		} else {
			return make_optional<float>(std::min(t1, t2));
		}
	}
};

struct ShapePlane : SceneShape {
	ShapePlane(TransformPair transform)
		: SceneShape(std::move(transform))
	{}

	virtual Optional<float> intersect(const Ray r) const {
		const Ray local_ray = transform.localFromParent * r;
		const float t = -local_ray.origin[1] / local_ray.direction[1];
		return make_optional<float>(t);
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

struct Camera {
	vec3 origin;
	mat3 orientation;
	float focal_distance; // distance from image plane

	Camera(vec3 origin, mat3 orientation, float focal_distance)
		: origin(origin), orientation(orientation), focal_distance(focal_distance)
	{}

	Ray createRay(const vec2 film_pos) const {
		const vec3 cameraspace_ray = mvec3(film_pos[0], film_pos[1], focal_distance);
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
	Scene s(Camera(vec3_0, orient(vec3_y, -vec3_z), 0.5f));
	s.objects.push_back(SceneObject(
		Material(vec3_1),
		std::make_unique<ShapeSphere>(TransformPair().translate(vec3_z * -2), 1.0f)
		));
	s.objects.push_back(SceneObject(
		Material(mvec3(0.5f, 0.0f, 0.0f)),
		std::make_unique<ShapePlane>(TransformPair().translate(vec3_y * -0.5))
		));

	return std::move(s);
}

static vec2 filmspace_from_screenspace(const vec2 screen_pos, const vec2 screen_size) {
	return (screen_pos - (screen_size * 0.5f)) * 2.0f * (1.0f / screen_size[1]);
}

struct Intersection {
	float t;
	const SceneObject* object;
	const SceneShape* shape;
};

Optional<Intersection> find_nearest_intersection(const Scene& scene, const Ray ray) {
	Intersection nearest_intersection;
	nearest_intersection.t = std::numeric_limits<float>::infinity();

	for (const SceneObject& object : scene.objects) {
		const float t = object.shape->intersect(ray).value_or(-1.0f);
		if (t >= 0 && t < nearest_intersection.t) {
			nearest_intersection.t = t;
			nearest_intersection.object = &object;
			nearest_intersection.shape = object.shape.get();
		}
	}
	
	return nearest_intersection.t != std::numeric_limits<float>::infinity() ?
		make_optional<Intersection>(nearest_intersection) : Optional<Intersection>();
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
			const vec3 color = hit ? hit->object->material.diffuse : vec3_1*0.1f;
			
			image_data[y*IMAGE_WIDTH + x] = color;
		}
	}

	save_image(image_data, IMAGE_WIDTH, IMAGE_HEIGHT);
}