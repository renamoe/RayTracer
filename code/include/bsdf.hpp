#pragma once

#include "Vector3f.h"

struct GlossySample {
    Vector3f dir;
    float pdf;
};

Vector3f reflect(const Vector3f &I, const Vector3f &N);
bool refract(const Vector3f &I,
             const Vector3f &N,
             float etaI,
             float etaT,
             Vector3f &T);
float fresnelSchlick(const Vector3f &I, const Vector3f &N, float etaI, float etaT);
float powerHeuristic(float pdfA, float pdfB);
bool isFiniteColor(const Vector3f &v);
float diffusePdf(const Vector3f &N, const Vector3f &wi);
float ggxPdf(const Vector3f &N, const Vector3f &V, const Vector3f &L, float roughness);
float areaPdfToSolidAnglePdf(float pdfArea, float dist, float cosLight);
Vector3f evaluateCookTorranceGGX(const Vector3f &N,
                                 const Vector3f &V,
                                 const Vector3f &L,
                                 const Vector3f &F0,
                                 float roughness);
GlossySample sampleGGXDirection(const Vector3f &N,
                                const Vector3f &V,
                                float roughness);
