#pragma once

#include "object3d.hpp"
#include <vector>


class Group : public Object3D {

public:

    Group() {}
    explicit Group (int num_objects) : objects(num_objects) {}
    ~Group() override {}

    bool intersect(const Ray &r, Hit &h, float tmin) override;
    void addObject(int index, Object3D *obj);
    int getGroupSize() const;
    Object3D *getGroupObject(int index) const;
    float getArea() const override;

private:
    std::vector<Object3D*> objects;
};
	
