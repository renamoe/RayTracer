#pragma once

#include "Vector2f.h"
#include "Vector3f.h"
#include "object3d.hpp"
#include <vecmath.h>

class Plane : public Object3D {
public:
    Plane();

    Plane(const Vector3f &normal, float d, Material *m);

    ~Plane() override = default;

    bool intersect(const Ray &r, Hit &h, float tmin) override;

protected:
    Vector3f normal;
    float d;
};


