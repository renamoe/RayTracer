#include "hit.hpp"
#include <iostream>

Hit::Hit() {
    material = nullptr;
    t = 1e38;
    texCoord = Vector2f::ZERO;
    texCoordValid = false;
}

Hit::Hit(float _t, Material *m, const Vector3f &n) {
    t = _t;
    material = m;
    normal = n;
    texCoord = Vector2f::ZERO;
    texCoordValid = false;
}

Hit::Hit(const Hit &h) {
    t = h.t;
    material = h.material;
    normal = h.normal;
    texCoord = h.texCoord;
    texCoordValid = h.texCoordValid;
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

const Vector2f &Hit::getTexCoord() const {
    return texCoord;
}

bool Hit::hasTexCoord() const {
    return texCoordValid;
}

void Hit::set(float _t, Material *m, const Vector3f &n) {
    t = _t;
    material = m;
    normal = n;
    texCoord = Vector2f::ZERO;
    texCoordValid = false;
}

void Hit::set(float _t, Material *m, const Vector3f &n, const Vector2f &uv) {
    t = _t;
    material = m;
    normal = n;
    texCoord = uv;
    texCoordValid = true;
}

std::ostream &operator<<(std::ostream &os, const Hit &h) {
    os << "Hit <" << h.getT() << ", " << h.getNormal() << ">";
    return os;
}
