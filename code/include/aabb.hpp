#pragma once

#include "Vector3f.h"

class Ray;

struct AABB {
    Vector3f min;
    Vector3f max;

    AABB() : min(Vector3f::ZERO), max(Vector3f::ZERO) {}
    explicit AABB(const Vector3f &p) : min(p), max(p) {}
    AABB(const Vector3f &a, const Vector3f &b, const Vector3f &c);

    void expand(const AABB &other);
    void expand(const Vector3f &point);
    int longestAxis() const;
    Vector3f centroid() const;
    bool intersect(const Ray &ray, float tmin, float tmax) const;
};
