#include "camera.hpp"

Ray PerspectiveCamera::generateRay(const Vector2f &point) {
    float dx = (point.x() - 0.5f * width) / fy;
    float dy = (0.5f * height - point.y()) / fy;
    Vector3f rayDirection = (horizontal * dx - up * dy + direction).normalized();
    return Ray(center, rayDirection);
}
    
