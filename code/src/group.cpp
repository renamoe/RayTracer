#include "group.hpp"

bool Group::intersect(const Ray &r, Hit &h, float tmin) {
    bool hit = false;
    for (auto &obj : objects) {
        hit |= obj->intersect(r, h, tmin);
    }
    return hit;
}

void Group::addObject(int index, Object3D *obj) {
    objects[index] = obj;
}

int Group::getGroupSize() {
    return objects.size();
}