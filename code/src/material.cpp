#include "material.hpp"
#include <algorithm>

Material::Material(const Vector3f &d_color, const Vector3f &s_color = Vector3f::ZERO, float s = 0) :
            diffuseColor(d_color), specularColor(s_color), shininess(s) {
}

Vector3f Material::getDiffuseColor() const {
    return diffuseColor;
}


Vector3f Material::Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
    Vector3f shaded = Vector3f::ZERO;
    // c_ambient = 0, k_a = 0
    Vector3f N = hit.getNormal();
    Vector3f L = dirToLight;
    Vector3f V = -ray.getDirection();
    Vector3f R = (2.0f * Vector3f::dot(N, L)) * N - L;
    shaded += (diffuseColor * std::max(Vector3f::dot(N, L), 0.0f)
                + specularColor * std::pow(std::max(Vector3f::dot(R, V), 0.0f), shininess)) * lightColor;
    return shaded;
}