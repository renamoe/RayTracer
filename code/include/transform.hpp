#pragma once

#include <vecmath.h>
#include "object3d.hpp"

// transforms a 3D point using a matrix, returning a 3D point
Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point);

// transform a 3D direction using a matrix, returning a direction
Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir);

class Transform : public Object3D {
public:
    Transform() {}

    Transform(const Matrix4f &m, Object3D *obj);

    ~Transform() {}

    virtual bool intersect(const Ray &r, Hit &h, float tmin);

protected:
    Object3D *o; //un-transformed object
    Matrix4f transform;
};
