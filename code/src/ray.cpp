#include "ray.hpp"
#include <iostream>

Ray::Ray(const Vector3f &orig, const Vector3f &dir) {
    origin = orig;
    direction = dir;
}

Ray::Ray(const Ray &r) {
    origin = r.origin;
    direction = r.direction;
}

const Vector3f &Ray::getOrigin() const {
    return origin;
}

const Vector3f &Ray::getDirection() const {
    return direction;
}

Vector3f Ray::pointAtParameter(float t) const {
    return origin + direction * t;
}

std::ostream &operator<<(std::ostream &os, const Ray &r) {
    os << "Ray <" << r.getOrigin() << ", " << r.getDirection() << ">";
    return os;
}