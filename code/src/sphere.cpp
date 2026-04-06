#include "sphere.hpp"
#include <cmath>

Sphere::Sphere() : center(Vector3f::ZERO), radius(1.0f), Object3D(nullptr) {}

Sphere::Sphere(const Vector3f &center, float radius, Material *material) : center(center), radius(radius), Object3D(material) {}


bool Sphere::intersect(const Ray &r, Hit &h, float tmin) {
    Vector3f oc = r.getOrigin() - center;
    float b = 2 * Vector3f::dot(oc, r.getDirection());
    float c = oc.squaredLength() - radius * radius;
    float delta = b * b - 4 * c;
    if (delta < 0) {
        return false;
    }
    float t = (-b - std::sqrt(delta)) / 2;
    if (t < tmin) {
        t = (-b + std::sqrt(delta)) / 2;
    }
    if (t < tmin) {
        return false;
    }
    if (t > h.getT()) {
        return false;
    }
    Vector3f normal = (r.pointAtParameter(t) - center).normalized();
    h.set(t, material, normal);
    return true;
}

float Sphere::getArea() const {
    return 4 * M_PI * radius * radius;
}