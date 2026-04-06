#pragma once

#include <cassert>
#include <vecmath.h>

#include "Vector3f.h"
#include "ray.hpp"
#include "hit.hpp"

class Material {
public:

    explicit Material(const Vector3f &d_color, const Vector3f &s_color, float s);

    virtual ~Material() = default;

    virtual Vector3f getDiffuseColor() const;

    Vector3f Shade(const Ray &ray, const Hit &hit,
                   const Vector3f &dirToLight, const Vector3f &lightColor);

protected:
    Vector3f diffuseColor;
    Vector3f specularColor;
    float shininess;
};

