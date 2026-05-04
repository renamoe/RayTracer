#include "group.hpp"

bool Group::intersect(const Ray &r, Hit &h, float tmin) {
    bool hit = false;
    for (auto &obj : objects) {
        hit |= obj->intersect(r, h, tmin);
    }
    return hit;
}

bool Group::occluded(const Ray &r, float tmin, float tmax) {
    for (auto &obj : objects) {
        if (obj->occluded(r, tmin, tmax)) {
            return true;
        }
    }
    return false;
}

void Group::addObject(int index, Object3D *obj) {
    objects[index] = obj;
}

int Group::getGroupSize() const {
    return objects.size();
}

Object3D *Group::getGroupObject(int index) const {
    return objects[index];
}

float Group::getArea() const {
    float area = 0;
    for (auto &obj : objects) {
        area += obj->getArea();
    }
    return area;
}
