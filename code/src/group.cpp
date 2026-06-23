#include "group.hpp"

#include <algorithm>
#include <array>
#include <atomic>

namespace {

constexpr int GROUP_BVH_LEAF_SIZE = 2;
constexpr int GROUP_BVH_STACK_SIZE = 128;

} // namespace

bool Group::intersect(const Ray &r, Hit &h, float tmin) {
    bool hit = false;
    ensureBVH();

    for (int objId : unboundedObjectIds) {
        hit |= objects[objId]->intersect(r, h, tmin);
    }
    hit |= intersectBVH(r, h, tmin);
    return hit;
}

bool Group::occluded(const Ray &r, float tmin, float tmax) {
    ensureBVH();

    for (int objId : unboundedObjectIds) {
        if (objects[objId]->occluded(r, tmin, tmax)) {
            return true;
        }
    }
    return occludedBVH(r, tmin, tmax);
}

bool Group::getBoundingBox(AABB &box) const {
    bool hasBox = false;
    for (Object3D *obj : objects) {
        if (obj == nullptr) {
            continue;
        }
        AABB childBox;
        if (!obj->getBoundingBox(childBox)) {
            continue;
        }
        if (!hasBox) {
            box = childBox;
            hasBox = true;
        } else {
            box.expand(childBox);
        }
    }
    return hasBox;
}

void Group::addObject(int index, Object3D *obj) {
    objects[index] = obj;
    bvhDirty.store(true, std::memory_order_release);
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

void Group::ensureBVH() {
    if (!bvhDirty.load(std::memory_order_acquire)) {
        return;
    }

    std::lock_guard<std::mutex> lock(bvhMutex);
    if (!bvhDirty.load(std::memory_order_relaxed)) {
        return;
    }

    boundedObjectIds.clear();
    unboundedObjectIds.clear();
    objectBoxes.assign(objects.size(), AABB());
    objectCentroids.assign(objects.size(), Vector3f::ZERO);
    bvhNodes.clear();
    bvhRoot = -1;

    for (int i = 0; i < (int)objects.size(); ++i) {
        Object3D *obj = objects[i];
        if (obj == nullptr) {
            continue;
        }

        AABB box;
        if (obj->getBoundingBox(box)) {
            objectBoxes[i] = box;
            objectCentroids[i] = box.centroid();
            boundedObjectIds.push_back(i);
        } else {
            unboundedObjectIds.push_back(i);
        }
    }

    if (!boundedObjectIds.empty()) {
        bvhNodes.reserve(boundedObjectIds.size() * 2);
        bvhRoot = buildBVH(0, (int)boundedObjectIds.size());
    }
    bvhDirty.store(false, std::memory_order_release);
}

int Group::buildBVH(int start, int end) {
    int nodeIndex = (int)bvhNodes.size();
    bvhNodes.push_back(GroupBVHNode());

    int firstObjectId = boundedObjectIds[start];
    AABB bounds = objectBoxes[firstObjectId];
    AABB centroidBounds(objectCentroids[firstObjectId]);
    for (int i = start + 1; i < end; ++i) {
        int objectId = boundedObjectIds[i];
        bounds.expand(objectBoxes[objectId]);
        centroidBounds.expand(objectCentroids[objectId]);
    }

    GroupBVHNode &node = bvhNodes[nodeIndex];
    node.box = bounds;

    int count = end - start;
    if (count <= GROUP_BVH_LEAF_SIZE) {
        node.start = start;
        node.count = count;
        return nodeIndex;
    }

    int axis = centroidBounds.longestAxis();
    int mid = (start + end) / 2;
    std::nth_element(
        boundedObjectIds.begin() + start,
        boundedObjectIds.begin() + mid,
        boundedObjectIds.begin() + end,
        [&](int a, int b) {
            return objectCentroids[a][axis] < objectCentroids[b][axis];
        }
    );

    int left = buildBVH(start, mid);
    int right = buildBVH(mid, end);
    bvhNodes[nodeIndex].left = left;
    bvhNodes[nodeIndex].right = right;
    return nodeIndex;
}

bool Group::intersectBVH(const Ray &r, Hit &h, float tmin) const {
    if (bvhRoot < 0) {
        return false;
    }

    bool hit = false;
    std::array<int, GROUP_BVH_STACK_SIZE> stack;
    int stackSize = 0;
    stack[stackSize++] = bvhRoot;

    while (stackSize > 0) {
        const GroupBVHNode &node = bvhNodes[stack[--stackSize]];
        if (!node.box.intersect(r, tmin, h.getT())) {
            continue;
        }

        if (node.count > 0) {
            for (int i = 0; i < node.count; ++i) {
                int objectId = boundedObjectIds[node.start + i];
                hit |= objects[objectId]->intersect(r, h, tmin);
            }
        } else {
            if (node.left >= 0) {
                stack[stackSize++] = node.left;
            }
            if (node.right >= 0) {
                stack[stackSize++] = node.right;
            }
        }
    }
    return hit;
}

bool Group::occludedBVH(const Ray &r, float tmin, float tmax) const {
    if (bvhRoot < 0 || tmax < tmin) {
        return false;
    }

    std::array<int, GROUP_BVH_STACK_SIZE> stack;
    int stackSize = 0;
    stack[stackSize++] = bvhRoot;

    while (stackSize > 0) {
        const GroupBVHNode &node = bvhNodes[stack[--stackSize]];
        if (!node.box.intersect(r, tmin, tmax)) {
            continue;
        }

        if (node.count > 0) {
            for (int i = 0; i < node.count; ++i) {
                int objectId = boundedObjectIds[node.start + i];
                if (objects[objectId]->occluded(r, tmin, tmax)) {
                    return true;
                }
            }
        } else {
            if (node.left >= 0) {
                stack[stackSize++] = node.left;
            }
            if (node.right >= 0) {
                stack[stackSize++] = node.right;
            }
        }
    }
    return false;
}
