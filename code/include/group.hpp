#pragma once

#include "object3d.hpp"
#include <atomic>
#include <array>
#include <mutex>
#include <vector>


class Group : public Object3D {

public:

    Group() {}
    explicit Group (int num_objects) : objects(num_objects) {}
    ~Group() override {}

    bool intersect(const Ray &r, Hit &h, float tmin) override;
    bool occluded(const Ray &r, float tmin, float tmax) override;
    bool getBoundingBox(AABB &box) const override;
    void addObject(int index, Object3D *obj);
    int getGroupSize() const;
    Object3D *getGroupObject(int index) const;
    float getArea() const override;

private:
    struct GroupBVHNode {
        AABB box;
        int left = -1;
        int right = -1;
        int start = 0;
        int count = 0;
    };

    void ensureBVH();
    int buildBVH(int start, int end);
    bool intersectBVH(const Ray &r, Hit &h, float tmin) const;
    bool occludedBVH(const Ray &r, float tmin, float tmax) const;

    std::vector<Object3D*> objects;
    std::vector<int> boundedObjectIds;
    std::vector<int> unboundedObjectIds;
    std::vector<AABB> objectBoxes;
    std::vector<Vector3f> objectCentroids;
    std::vector<GroupBVHNode> bvhNodes;
    std::mutex bvhMutex;
    std::atomic_bool bvhDirty{true};
    int bvhRoot = -1;
};
