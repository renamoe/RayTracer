#include "plane.hpp"

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
    h.set(t, material, normal);
    return true;
} 