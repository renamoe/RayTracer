#pragma once

#include "object3d.hpp"
#include <Vector3f.h>
#include <iostream>

class Light {
public:

    struct SampleResult {
        Vector3f pos;
        Vector3f dir;
        Vector3f normal;
        Vector3f col;
        float dist;
        float pdf;
    };

    Light() = default;

    virtual ~Light() = default;

    virtual SampleResult sample(const Vector3f &p) const;

    virtual void print(std::ostream &os) const {};
};


class DirectionalLight : public Light {
public:
    DirectionalLight() = delete;

    DirectionalLight(const Vector3f &d, const Vector3f &c);

    ~DirectionalLight() override = default;

    Vector3f getDirection() const;

    Vector3f getColor() const;

    void print(std::ostream &os) const override;

private:

    Vector3f direction;
    Vector3f color;
};

class PointLight : public Light {
public:
    PointLight() = delete;

    PointLight(const Vector3f &p, const Vector3f &c);

    ~PointLight() override = default;

    Vector3f getPosition() const;

    Vector3f getColor() const;

    void print(std::ostream &os) const override;

private:

    Vector3f position;
    Vector3f color;

};

class AreaLight : public Light {
public:
    AreaLight() = delete;

    AreaLight(Object3D *obj);

    ~AreaLight() override = default;

    Object3D *getObject() const;

    void print(std::ostream &os) const override;

    SampleResult sample(const Vector3f &p) const override;

private:

    Object3D *object;
    float totalArea;
};

std::ostream &operator<<(std::ostream &os, const Light &light);