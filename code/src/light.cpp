#include "light.hpp"

DirectionalLight::DirectionalLight(const Vector3f &d, const Vector3f &c) {
    direction = d.normalized();
    color = c;
}

void DirectionalLight::getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const {
    dir = -direction;
    col = color;
}


PointLight::PointLight(const Vector3f &p, const Vector3f &c) {
    position = p;
    color = c;
}

void PointLight::getIllumination(const Vector3f &p, Vector3f &dir, Vector3f &col) const {
    dir = (position - p);
    dir = dir / dir.length();
    col = color;
}