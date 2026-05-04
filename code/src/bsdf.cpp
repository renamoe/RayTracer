#include "bsdf.hpp"

#include "random.hpp"

#include <algorithm>
#include <cmath>

namespace {

float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

Vector3f fresnelSchlickColor(float cosTheta, const Vector3f &F0) {
    float f = std::pow(1.0f - clamp01(cosTheta), 5.0f);
    return F0 + (Vector3f(1, 1, 1) - F0) * f;
}

float distributionGGX(const Vector3f &N, const Vector3f &H, float roughness) {
    float alpha = std::max(0.001f, roughness * roughness);
    float a2 = alpha * alpha;
    float NoH = clamp01(Vector3f::dot(N, H));
    float NoH2 = NoH * NoH;

    float denom = NoH2 * (a2 - 1.0f) + 1.0f;
    return a2 / (M_PI * denom * denom);
}

float geometrySmithGGX(const Vector3f &N, const Vector3f &V, const Vector3f &L, float roughness) {
    float alpha = std::max(0.001f, roughness * roughness);
    float a2 = alpha * alpha;

    auto G1 = [a2](float NoX) {
        NoX = std::max(0.0f, NoX);
        return 2.0f * NoX / (NoX + std::sqrt(a2 + (1.0f - a2) * NoX * NoX));
    };

    return G1(Vector3f::dot(N, V)) * G1(Vector3f::dot(N, L));
}

} // namespace

Vector3f reflect(const Vector3f &I, const Vector3f &N) {
    return (I - 2 * Vector3f::dot(I, N) * N).normalized();
}

bool refract(const Vector3f &I,
             const Vector3f &N,
             float etaI,
             float etaT,
             Vector3f &T) {
    float cosI = std::max(-1.0f, std::min(1.0f, Vector3f::dot(I, N)));
    Vector3f n = N;
    if (cosI < 0.0f) {
        cosI = -cosI;
    } else {
        std::swap(etaI, etaT);
        n = -N;
    }

    float eta = etaI / etaT;
    float k = 1.0f - eta * eta * (1.0f - cosI * cosI);

    if (k < 0.0f) {
        return false;
    }
    T = (eta * I + (eta * cosI - std::sqrt(k)) * n).normalized();
    return true;
}

float fresnelSchlick(const Vector3f &I, const Vector3f &N, float etaI, float etaT) {
    float cosTheta = std::max(-1.0f, std::min(1.0f, Vector3f::dot(I, N)));
    if (cosTheta > 0.0f) {
        std::swap(etaI, etaT);
    }

    float r0 = (etaI - etaT) / (etaI + etaT);
    r0 = r0 * r0;
    return r0 + (1.0f - r0) * std::pow(1.0f - std::abs(cosTheta), 5.0f);
}

float powerHeuristic(float pdfA, float pdfB) {
    if (!std::isfinite(pdfA) || !std::isfinite(pdfB) || pdfA <= 0.0f) {
        return 0.0f;
    }
    pdfB = std::max(0.0f, pdfB);
    float a2 = pdfA * pdfA;
    float b2 = pdfB * pdfB;
    float denom = a2 + b2;
    if (!std::isfinite(denom) || denom <= 1e-8f) {
        return 0.0f;
    }
    return a2 / denom;
}

bool isFiniteColor(const Vector3f &v) {
    return std::isfinite(v.x()) && std::isfinite(v.y()) && std::isfinite(v.z());
}

float diffusePdf(const Vector3f &N, const Vector3f &wi) {
    float cosTheta = std::max(0.0f, Vector3f::dot(N, wi));
    return cosTheta / M_PI;
}

float ggxPdf(const Vector3f &N,
             const Vector3f &V,
             const Vector3f &L,
             float roughness) {
    float NoL = clamp01(Vector3f::dot(N, L));
    if (NoL <= 0.0f) {
        return 0.0f;
    }

    Vector3f halfVector = V + L;
    if (halfVector.squaredLength() <= 1e-12f) {
        return 0.0f;
    }
    Vector3f H = halfVector.normalized();
    float NoH = clamp01(Vector3f::dot(N, H));
    float VoH = clamp01(Vector3f::dot(V, H));

    if (NoH <= 0.0f || VoH <= 0.0f) {
        return 0.0f;
    }

    float pdfH = distributionGGX(N, H, roughness) * NoH;
    return pdfH / std::max(4.0f * VoH, 1e-6f);
}

float areaPdfToSolidAnglePdf(float pdfArea, float dist, float cosLight) {
    if (!std::isfinite(pdfArea) || !std::isfinite(dist) || !std::isfinite(cosLight) ||
        pdfArea <= 0.0f || dist <= 0.0f || cosLight <= 0.0f) {
        return 0.0f;
    }
    return pdfArea * dist * dist / cosLight;
}

Vector3f evaluateCookTorranceGGX(const Vector3f &N,
                                 const Vector3f &V,
                                 const Vector3f &L,
                                 const Vector3f &F0,
                                 float roughness) {
    float NoV = clamp01(Vector3f::dot(N, V));
    float NoL = clamp01(Vector3f::dot(N, L));
    if (NoV <= 0.0f || NoL <= 0.0f) {
        return Vector3f::ZERO;
    }

    Vector3f halfVector = V + L;
    if (halfVector.squaredLength() <= 1e-12f) {
        return Vector3f::ZERO;
    }
    Vector3f H = halfVector.normalized();
    float VoH = clamp01(Vector3f::dot(V, H));

    float D = distributionGGX(N, H, roughness);
    float G = geometrySmithGGX(N, V, L, roughness);
    Vector3f F = fresnelSchlickColor(VoH, F0);

    return F * (D * G / std::max(4.0f * NoV * NoL, 1e-6f));
}

GlossySample sampleGGXDirection(const Vector3f &N,
                                const Vector3f &V,
                                float roughness) {
    Vector3f T;
    if (std::abs(N.x()) > 0.9f) {
        T = Vector3f::cross(Vector3f(0, 1, 0), N).normalized();
    } else {
        T = Vector3f::cross(Vector3f(1, 0, 0), N).normalized();
    }
    Vector3f B = Vector3f::cross(N, T).normalized();

    float u1 = Random::get_float();
    float u2 = Random::get_float();
    float alpha = std::max(0.001f, roughness * roughness);
    float a2 = alpha * alpha;

    float phi = 2.0f * M_PI * u1;
    float cosTheta = std::sqrt((1.0f - u2) / std::max(1.0f + (a2 - 1.0f) * u2, 1e-6f));
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

    Vector3f H = (T * std::cos(phi) * sinTheta +
                  B * std::sin(phi) * sinTheta +
                  N * cosTheta).normalized();
    if (Vector3f::dot(V, H) <= 0.0f) {
        H = -H;
    }

    Vector3f wi = reflect(-V, H);
    float NoL = clamp01(Vector3f::dot(N, wi));
    float NoH = clamp01(Vector3f::dot(N, H));
    float VoH = clamp01(Vector3f::dot(V, H));
    if (NoL <= 0.0f || NoH <= 0.0f || VoH <= 0.0f) {
        return {Vector3f::ZERO, 0.0f};
    }

    float pdfH = distributionGGX(N, H, roughness) * NoH;
    float pdfWi = pdfH / std::max(4.0f * VoH, 1e-6f);
    return {wi.normalized(), pdfWi};
}
