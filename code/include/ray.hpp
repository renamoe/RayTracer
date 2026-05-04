#pragma once

#include <cassert>
#include <Vector3f.h>


// Ray class mostly copied from Peter Shirley and Keith Morley
class Ray {
public:

    Ray() = delete;
    Ray(const Vector3f &orig, const Vector3f &dir);

    Ray(const Ray &r);

    const Vector3f &getOrigin() const;

    const Vector3f &getDirection() const;

    const Vector3f &getInverseDirection() const;

    Vector3f pointAtParameter(float t) const;

private:

    Vector3f origin;
    Vector3f direction;
    Vector3f inverseDirection;

};

