#include "hit.hpp"
#include <iostream>

Hit::Hit() {
    material = nullptr;
    t = 1e38;
}

Hit::Hit(float _t, Material *m, const Vector3f &n) {
    t = _t;
    material = m;
    normal = n;
}

Hit::Hit(const Hit &h) {
    t = h.t;
    material = h.material;
    normal = h.normal;
}

float Hit::getT() const {
    return t;
}

Material *Hit::getMaterial() const {
    return material;
}

const Vector3f &Hit::getNormal() const {
    return normal;
}

void Hit::set(float _t, Material *m, const Vector3f &n) {
    t = _t;
    material = m;
    normal = n;
}

std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}