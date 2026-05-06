#include "material.hpp"
#include "texture.hpp"
#include <algorithm>
#include <utility>

Material::Material(const Vector3f &d_color,
                   const Vector3f &s_color,
                   const Vector3f &e_color,
                   float s,
                   MaterialType type,
                   const Vector3f &t_color,
                   float ior,
                   float r) :
            diffuseColor(d_color), specularColor(s_color), emission(e_color), transmissionColor(t_color), ior(ior), shininess(s), roughness(r), type(type) {
}

Vector3f Material::getEmission() const {
    return emission;
}

Vector3f Material::getDiffuseColor() const {
    return diffuseColor;
}

Vector3f Material::getDiffuseColor(const Hit &hit) const {
    if (diffuseTexture && hit.hasTexCoord()) {
        return diffuseColor * diffuseTexture->sample(hit.getTexCoord());
    }
    return diffuseColor;
}

Vector3f Material::getSpecularColor() const {
    return specularColor;
}

void Material::setDiffuseTexture(std::shared_ptr<Texture> texture) {
    diffuseTexture = std::move(texture);
}

bool Material::hasDiffuseTexture() const {
    return diffuseTexture != nullptr;
}

float Material::getShininess() const {
    return shininess;
}

MaterialType Material::getType() const {
    return type;
}

bool Material::isMirror() const {
    return type == MaterialType::MIRROR;
}

Vector3f Material::getTransmissionColor() const {
    return transmissionColor;
}

float Material::getIOR() const {
    return ior;
}

float Material::getRoughness() const {
    return roughness;
}

bool Material::isGlass() const {
    return type == MaterialType::GLASS;
}

bool Material::isEmissive() const {
    return type == MaterialType::EMISSIVE || emission.length() > 0;
}

Vector3f Material::Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor) {
    Vector3f shaded = Vector3f::ZERO;
    // c_ambient = 0, k_a = 0
    Vector3f N = hit.getNormal();
    Vector3f L = dirToLight;
    Vector3f V = -ray.getDirection();
    Vector3f R = (2.0f * Vector3f::dot(N, L)) * N - L;
    Vector3f albedo = getDiffuseColor(hit);
    shaded += (albedo * std::max(Vector3f::dot(N, L), 0.0f)
                + specularColor * std::pow(std::max(Vector3f::dot(R, V), 0.0f), shininess)) * lightColor;
    return shaded;
}
