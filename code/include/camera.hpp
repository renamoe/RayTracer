#pragma once

#include "Vector3f.h"
#include "ray.hpp"
#include <vecmath.h>
#include <cmath>


class Camera {
public:
    Camera(const Vector3f &center, const Vector3f &direction, const Vector3f &up, int imgW, int imgH) {
        this->center = center;
        this->direction = direction.normalized();
        this->horizontal = Vector3f::cross(this->direction, up).normalized();
        this->up = Vector3f::cross(this->horizontal, this->direction);
        this->width = imgW;
        this->height = imgH;
    }

    // Generate rays for each screen-space coordinate
    virtual Ray generateRay(const Vector2f &point) = 0;
    virtual bool projectPoint(const Vector3f &point,
                              Vector2f &raster,
                              Vector3f &dirFromCamera,
                              float &dist) const {
        (void)point;
        (void)raster;
        (void)dirFromCamera;
        (void)dist;
        return false;
    }
    virtual float rasterToSolidAnglePdf(const Vector3f &dirFromCamera) const {
        (void)dirFromCamera;
        return 0.0f;
    }
    virtual ~Camera() = default;

    int getWidth() const { return width; }
    int getHeight() const { return height; }
    const Vector3f &getCenter() const { return center; }
    const Vector3f &getDirection() const { return direction; }

protected:
    // Extrinsic parameters
    Vector3f center;
    Vector3f direction;
    Vector3f up;
    Vector3f horizontal;
    // Intrinsic parameters
    int width;
    int height;
};

class PerspectiveCamera : public Camera {

public:
    PerspectiveCamera(const Vector3f &center, const Vector3f &direction,
            const Vector3f &up, int imgW, int imgH, float angle) 
            : Camera(center, direction, up, imgW, imgH), angle(angle) {
        fy =  0.5f * imgH / std::tan(angle / 2);
        // assume fx == fy
    }

    Ray generateRay(const Vector2f &point) override;
    bool projectPoint(const Vector3f &point,
                      Vector2f &raster,
                      Vector3f &dirFromCamera,
                      float &dist) const override;
    float rasterToSolidAnglePdf(const Vector3f &dirFromCamera) const override;

protected:
    float angle;
    float fy;
};

