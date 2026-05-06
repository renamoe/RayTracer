#pragma once

#include "Vector2f.h"

#include <vecmath.h>

class Material;

class Hit {
public:
    Hit();
    Hit(float _t, Material *m, const Vector3f &n);
    Hit(const Hit &h);
    ~Hit() = default;

    float getT() const;
    Material *getMaterial() const;
    const Vector3f &getNormal() const;
    const Vector2f &getTexCoord() const;
    bool hasTexCoord() const;
    void set(float _t, Material *m, const Vector3f &n);
    void set(float _t, Material *m, const Vector3f &n, const Vector2f &uv);

private:
    float t;
    Material *material;
    Vector3f normal;
    Vector2f texCoord;
    bool texCoordValid;
};

