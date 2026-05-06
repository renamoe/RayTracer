#include "plane.hpp"
#include "Vector2f.h"

#include <cmath>

Plane::Plane() : normal(0, 0, 1), d(0.0f), Object3D(nullptr) {}

Plane::Plane(const Vector3f &normal, float d, Material *m) 
    : normal(normal), d(d), Object3D(m) {}

bool Plane::intersect(const Ray &r, Hit &h, float tmin) {
    float dn = Vector3f::dot(normal, r.getDirection());
    if (std::abs(dn) < 1e-6) {
        return false;
    }
    float t = (d - Vector3f::dot(normal, r.getOrigin())) / dn;
    if (t < tmin) {
        return false;
    }
    if (t > h.getT()) {
        return false;
    }
    Vector3f n = normal.normalized();
    Vector3f tangent = std::abs(n.x()) > 0.9f
        ? Vector3f::cross(Vector3f(0, 1, 0), n).normalized()
        : Vector3f::cross(Vector3f(1, 0, 0), n).normalized();
    Vector3f bitangent = Vector3f::cross(n, tangent).normalized();
    Vector3f p = r.pointAtParameter(t);
    h.set(t, material, normal, Vector2f(Vector3f::dot(p, tangent), Vector3f::dot(p, bitangent)));
    return true;
}
