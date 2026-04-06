#pragma once

#include "Vector3f.h"
#include "object3d.hpp"
#include <vecmath.h>

class Sphere : public Object3D {
public:
    Sphere();

    Sphere(const Vector3f &center, float radius, Material *material);

    ~Sphere() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override;

    float getArea() const override;

protected:
    Vector3f center;
    float radius;
};
