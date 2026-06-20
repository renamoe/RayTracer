#pragma once

#include "Vector3f.h"
#include "material.hpp"

struct PhotonVertex {
    Vector3f pos;
    Vector3f normal;
    Vector3f throughput;
    Vector3f diffuseColor = Vector3f(1, 1, 1);
    Material *material = nullptr;
    Vector3f wi;
    int depth = 0;
    bool isDelta = false;
};