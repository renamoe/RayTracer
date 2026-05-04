#include "aabb.hpp"

#include "ray.hpp"

#include <algorithm>
#include <cmath>

namespace {

float minOnDim(const Vector3f &a, const Vector3f &b, const Vector3f &c, int dim) {
    return std::min(a[dim], std::min(b[dim], c[dim]));
}

float maxOnDim(const Vector3f &a, const Vector3f &b, const Vector3f &c, int dim) {
    return std::max(a[dim], std::max(b[dim], c[dim]));
}

} // namespace

AABB::AABB(const Vector3f &a, const Vector3f &b, const Vector3f &c) {
    min = Vector3f(minOnDim(a, b, c, 0), minOnDim(a, b, c, 1), minOnDim(a, b, c, 2));
    max = Vector3f(maxOnDim(a, b, c, 0), maxOnDim(a, b, c, 1), maxOnDim(a, b, c, 2));
}

void AABB::expand(const AABB &other) {
    min = Vector3f(std::min(min.x(), other.min.x()), std::min(min.y(), other.min.y()), std::min(min.z(), other.min.z()));
    max = Vector3f(std::max(max.x(), other.max.x()), std::max(max.y(), other.max.y()), std::max(max.z(), other.max.z()));
}

void AABB::expand(const Vector3f &point) {
    min = Vector3f(std::min(min.x(), point.x()), std::min(min.y(), point.y()), std::min(min.z(), point.z()));
    max = Vector3f(std::max(max.x(), point.x()), std::max(max.y(), point.y()), std::max(max.z(), point.z()));
}

int AABB::longestAxis() const {
    Vector3f diag = max - min;
    if (diag.x() > diag.y() && diag.x() > diag.z()) {
        return 0;
    } else if (diag.y() > diag.z()) {
        return 1;
    } else {
        return 2;
    }
}

Vector3f AABB::centroid() const {
    return (min + max) * 0.5f;
}

bool AABB::intersect(const Ray &ray, float tmin, float tmax) const {
    for (int dim = 0; dim < 3; ++dim) {
        float origin = ray.getOrigin()[dim];
        float dir = ray.getDirection()[dim];
        float slabMin = min[dim] - 1e-5f;
        float slabMax = max[dim] + 1e-5f;

        if (std::abs(dir) < 1e-8f) {
            if (origin < slabMin || origin > slabMax) {
                return false;
            }
            continue;
        }

        float invD = ray.getInverseDirection()[dim];
        float t0 = (slabMin - origin) * invD;
        float t1 = (slabMax - origin) * invD;
        if (invD < 0.0f) {
            std::swap(t0, t1);
        }
        tmin = std::max(tmin, t0);
        tmax = std::min(tmax, t1);
        if (tmax < tmin) {
            return false;
        }
    }
    return true;
}
