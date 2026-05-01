#include "transform.hpp"

Vector3f transformPoint(const Matrix4f &mat, const Vector3f &point) {
    return (mat * Vector4f(point, 1)).xyz();
}

Vector3f transformDirection(const Matrix4f &mat, const Vector3f &dir) {
    return (mat * Vector4f(dir, 0)).xyz();
}

Transform::Transform(const Matrix4f &m, Object3D *obj) : o(obj) {
    transform = m.inverse();
}

bool Transform::intersect(const Ray &r, Hit &h, float tmin) {
    Vector3f trSource = transformPoint(transform, r.getOrigin());
    Vector3f trDirection = transformDirection(transform, r.getDirection());
    float directionLength = trDirection.length();
    trDirection = trDirection / directionLength;
    Ray tr(trSource, trDirection);
    Hit localHit;
    bool inter = o->intersect(tr, localHit, tmin * directionLength);
    if (!inter) {
        return false;
    }

    float worldT = localHit.getT() / directionLength;
    if (worldT < tmin || worldT >= h.getT()) {
        return false;
    }

    Vector3f worldNormal = transformDirection(transform.transposed(), localHit.getNormal()).normalized();

    h.set(worldT, localHit.getMaterial(), worldNormal);
    return true;
}

float Transform::getArea() const {
    // temporarily assess
    float s = o->getArea();
    float det = transform.determinant();
    return s / pow(std::abs(det), 2.0f / 3.0f);
}