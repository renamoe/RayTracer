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

bool Transform::occluded(const Ray &r, float tmin, float tmax) {
    Vector3f trSource = transformPoint(transform, r.getOrigin());
    Vector3f trDirection = transformDirection(transform, r.getDirection());
    float directionLength = trDirection.length();
    trDirection = trDirection / directionLength;
    Ray tr(trSource, trDirection);
    return o->occluded(tr, tmin * directionLength, tmax * directionLength);
}

bool Transform::getBoundingBox(AABB &box) const {
    AABB childBox;
    if (o == nullptr || !o->getBoundingBox(childBox)) {
        return false;
    }

    Matrix4f world = transform.inverse();
    Vector3f corners[8] = {
        Vector3f(childBox.min.x(), childBox.min.y(), childBox.min.z()),
        Vector3f(childBox.max.x(), childBox.min.y(), childBox.min.z()),
        Vector3f(childBox.min.x(), childBox.max.y(), childBox.min.z()),
        Vector3f(childBox.min.x(), childBox.min.y(), childBox.max.z()),
        Vector3f(childBox.max.x(), childBox.max.y(), childBox.min.z()),
        Vector3f(childBox.max.x(), childBox.min.y(), childBox.max.z()),
        Vector3f(childBox.min.x(), childBox.max.y(), childBox.max.z()),
        Vector3f(childBox.max.x(), childBox.max.y(), childBox.max.z())
    };

    box = AABB(transformPoint(world, corners[0]));
    for (int i = 1; i < 8; ++i) {
        box.expand(transformPoint(world, corners[i]));
    }
    return true;
}

float Transform::getArea() const {
    // temporarily assess
    float s = o->getArea();
    float det = transform.determinant();
    return s / pow(std::abs(det), 2.0f / 3.0f);
}
