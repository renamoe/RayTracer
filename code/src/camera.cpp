#include "camera.hpp"

Ray PerspectiveCamera::generateRay(const Vector2f &point) {
    float dx = (point.x() - 0.5f * width) / fy;
    float dy = (0.5f * height - point.y()) / fy;
    Vector3f rayDirection = (horizontal * dx - up * dy + direction).normalized();
    return Ray(center, rayDirection);
}

bool PerspectiveCamera::projectPoint(const Vector3f &point,
                                     Vector2f &raster,
                                     Vector3f &dirFromCamera,
                                     float &dist) const {
    Vector3f toPoint = point - center;
    dist = toPoint.length();
    if (dist <= 0.0f) {
        return false;
    }

    dirFromCamera = toPoint / dist;
    float z = Vector3f::dot(toPoint, direction);
    if (z <= 0.0f) {
        return false;
    }

    float dx = Vector3f::dot(toPoint, horizontal) / z;
    float dy = -Vector3f::dot(toPoint, up) / z;
    raster = Vector2f(dx * fy + 0.5f * width, 0.5f * height - dy * fy);

    return raster.x() >= 0.0f && raster.x() < width &&
        raster.y() >= 0.0f && raster.y() < height;
}

float PerspectiveCamera::rasterToSolidAnglePdf(const Vector3f &dirFromCamera) const {
    float cosTheta = Vector3f::dot(direction, dirFromCamera);
    if (cosTheta <= 0.0f) {
        return 0.0f;
    }

    return fy * fy / (cosTheta * cosTheta * cosTheta);
}
    
