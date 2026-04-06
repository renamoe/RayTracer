#pragma once

#include "Vector3f.h"
#include "object3d.hpp"
#include <cstdlib>
#include <vecmath.h>

class Triangle: public Object3D {

public:
	Triangle() = delete;

    // a b c are three vertex positions of the triangle
	Triangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m);

	bool intersect(const Ray& ray,  Hit& hit , float tmin) override;

    float getArea() const override;

	void sample(Vector3f &pos, Vector3f &normal) const override;

	Vector3f normal;
	Vector3f vertices[3];
};