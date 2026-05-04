#include "triangle.hpp"
#include "random.hpp"
#include <cmath>

Triangle::Triangle(const Vector3f& a, const Vector3f& b, const Vector3f& c, Material* m) : Object3D(m) {
    vertices[0] = a;
    vertices[1] = b;
    vertices[2] = c;
    normal = Vector3f::cross(b - a, c - a).normalized();
}

bool Triangle::intersect(const Ray& ray,  Hit& hit , float tmin) {
    Vector3f E1 = vertices[1] - vertices[0];
    Vector3f E2 = vertices[2] - vertices[0];
    Vector3f O = ray.getOrigin();
    Vector3f D = ray.getDirection();
    Vector3f DE2 = Vector3f::cross(D, E2);
    float det = Vector3f::dot(E1, DE2);
    if (std::abs(det) < 1e-6) {
        return false;
    }
    float inv = 1.0f / det;
    Vector3f S = O - vertices[0];
    float u = inv * Vector3f::dot(S, DE2);
    if (u < 0 || u > 1.0f) {
        return false;
    }
    Vector3f SE1 = Vector3f::cross(S, E1);
    float v = inv * Vector3f::dot(D, SE1);
    if (v < 0 || u + v > 1.0f) {
        return false;
    }
    float t = inv * Vector3f::dot(E2, SE1);
    if (t < tmin || t > hit.getT()) {
        return false;
    }
    hit.set(t, material, normal);
    return true;
}

bool Triangle::getBoundingBox(AABB &box) const {
    box = AABB(vertices[0], vertices[1], vertices[2]);
    return true;
}

float Triangle::getArea() const {
    return 0.5 * Vector3f::cross(vertices[1] - vertices[0], vertices[2] - vertices[0]).length();
}

void Triangle::sample(Vector3f &pos, Vector3f &normal) const {
    float s = Random::get_float();
    float t = Random::get_float();
    if (s + t > 1.0f) {
        s = 1.0f - s;
        t = 1.0f - t;
    }
    pos = vertices[0] + s * (vertices[1] - vertices[0]) + t * (vertices[2] - vertices[0]);
    normal = this->normal;
}
