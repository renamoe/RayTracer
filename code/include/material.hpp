#pragma once

#include <cassert>
#include <vecmath.h>

#include "Vector3f.h"
#include "ray.hpp"
#include "hit.hpp"

enum class MaterialType {
    DIFFUSE,
    MIRROR,
    GLASS,
    EMISSIVE
};

class Material {
public:

    explicit Material(const Vector3f &d_color,
                      const Vector3f &s_color = Vector3f::ZERO,
                      const Vector3f &e_color = Vector3f::ZERO,
                      float s = 0,
                      MaterialType type = MaterialType::DIFFUSE,
                      const Vector3f &t_color = Vector3f(1, 1, 1),
                      float ior = 1.5f);

    virtual ~Material() = default;

    Vector3f getEmission() const;

    Vector3f getDiffuseColor() const;

    Vector3f getSpecularColor() const;

    float getShininess() const;

    MaterialType getType() const;

    bool isMirror() const;

    Vector3f getTransmissionColor() const;
    
    float getIOR() const;

    bool isGlass() const;

    bool isEmissive() const;

    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor);

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    Vector3f emission;
    Vector3f transmissionColor;
    float ior;
    float shininess;
    MaterialType type;
};

