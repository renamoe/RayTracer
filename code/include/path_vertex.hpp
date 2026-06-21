#pragma once

#include "Vector3f.h"
#include "material.hpp"

enum class VCMPathVertexType {
    Camera,
    Light,
    Surface
};

struct VCMPathVertex {
    Vector3f pos;
    Vector3f normal;
    Vector3f throughput;
    Vector3f diffuseColor = Vector3f(1, 1, 1);

    Material *material = nullptr;

    Vector3f wo;
    Vector3f wi;

    float pdfForwardArea = 0.0f;
    float pdfReverseArea = 0.0f;
    float pdfForwardSolidAngle = 0.0f;

    float dVCM = 0.0f;
    float dVC = 0.0f;
    float dVM = 0.0f;

    bool isDelta = false;
    bool isLight = false;

    VCMPathVertexType type = VCMPathVertexType::Surface;
};