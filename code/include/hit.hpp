#pragma once

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
    void set(float _t, Material *m, const Vector3f &n);

private:
    float t;
    Material *material;
    Vector3f normal;
};

