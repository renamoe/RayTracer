#include "light.hpp"
#include "Vector3f.h"
#include <iostream>
#include <ostream>

Light::SampleResult Light::sample(const Vector3f &p) const {
    return SampleResult();
}

std::ostream &operator<<(std::ostream &os, const Light &light) {
    light.print(os);
    return os;
}

Vector3f DirectionalLight::getDirection() const {
    return direction;
}

Vector3f DirectionalLight::getColor() const {
    return color;
}

DirectionalLight::DirectionalLight(const Vector3f &d, const Vector3f &c) {
    direction = d.normalized();
    color = c;
}

void DirectionalLight::print(std::ostream &os) const {
    os << "DirectionalLight " << getDirection() << " " << getColor();
}

Vector3f PointLight::getPosition() const {
    return position;
}

Vector3f PointLight::getColor() const {
    return color;
}

PointLight::PointLight(const Vector3f &p, const Vector3f &c) {
    position = p;
    color = c;
}

void PointLight::print(std::ostream &os) const {
    os << "PointLight " << getPosition() << " " << getColor();
}

AreaLight::AreaLight(Object3D *obj) {
    object = obj;
    totalArea = object->getArea();
}

Object3D *AreaLight::getObject() const {
    return object;
}

void AreaLight::print(std::ostream &os) const {
    os << "AreaLight " << getObject();
}

AreaLight::SampleResult AreaLight::sample(const Vector3f &p) const {
    SampleResult result;
    Vector3f lightNormal;
    object->sample(result.pos, lightNormal);
    result.dir = (result.pos - p).normalized();
    result.dist = (result.pos - p).length();
    result.col = object->getMaterial()->getEmission();
    float cosThetaLight = Vector3f::dot(lightNormal, -result.dir);
    if (cosThetaLight < 1e-6) {
        result.pdf = 0;
    } else {
        result.pdf = cosThetaLight / totalArea;
    }
    return result;
}