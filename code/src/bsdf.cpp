#include "bsdf.hpp"

#include "random.hpp"

#include <algorithm>
#include <cmath>

namespace {

float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

float maxComponent(const Vector3f &v) {
    return std::max(v.x(), std::max(v.y(), v.z()));
}

float diffuseScaleFromSpecular(const Vector3f &specularColor) {
    return 1.0f - std::min(maxComponent(specularColor), 0.95f);
}

float specularSamplingProbability(const Vector3f &specularColor) {
    float specularStrength = maxComponent(specularColor);
    if (specularStrength <= 1e-4f) {
        return 0.0f;
    }
    return std::max(0.05f, std::min(specularStrength, 0.75f));
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

float solidAngleToAreaPdf(float pdfW,
                          const Vector3f &from,
                          const Vector3f &to,
                          const Vector3f &targetNormal) {
    Vector3f edge = to - from;
    float dist2 = edge.squaredLength();
    if (pdfW <= 0.0f || dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    float cosTarget = std::max(0.0f, Vector3f::dot(targetNormal, -dir));
    return pdfW * cosTarget / dist2;
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

Vector3f cosineSampleHemisphere(const Vector3f &N) {
    Vector3f T, B;
    if (std::abs(N.x()) > 0.9f) {
        T = Vector3f::cross(Vector3f(0, 1, 0), N).normalized();
    } else {
        T = Vector3f::cross(Vector3f(1, 0, 0), N).normalized();
    }
    B = Vector3f::cross(N, T).normalized();
    float r1 = 2 * M_PI * Random::get_float();
    float r2 = Random::get_float();
    float r2s = std::sqrt(r2);
    return (T * std::cos(r1) * r2s +
            B * std::sin(r1) * r2s +
            N * std::sqrt(1 - r2)).normalized();
}

Vector3f evaluateBSDF(Material *mat,
                      const Vector3f &normal,
                      const Vector3f &wo,
                      const Vector3f &wi) {
    return evaluateBSDF(mat, mat->getDiffuseColor(), normal, wo, wi);
}

Vector3f evaluateBSDF(Material *mat,
                      const Vector3f &diffuseColor,
                      const Vector3f &normal,
                      const Vector3f &wo,
                      const Vector3f &wi) {
    if (mat->isMirror()) {
        if (mat->getRoughness() < DELTA_MIRROR_ROUGHNESS) {
            return Vector3f::ZERO;
        }
        return evaluateCookTorranceGGX(
            normal,
            wo,
            wi,
            mat->getSpecularColor(),
            mat->getRoughness()
        );
    } else if (mat->isGlass()) {
        return Vector3f::ZERO;
    } else {
        if (Vector3f::dot(normal, wo) <= 0.0f ||
            Vector3f::dot(normal, wi) <= 0.0f) {
            return Vector3f::ZERO;
        }

        Vector3f specularColor = mat->getSpecularColor();
        Vector3f result = diffuseColor * diffuseScaleFromSpecular(specularColor) / M_PI;
        if (maxComponent(specularColor) > 1e-4f) {
            result += evaluateCookTorranceGGX(
                normal,
                wo,
                wi,
                specularColor,
                mat->getRoughness()
            );
        }
        return result;
    }
}

float bsdfPdf(Material *mat,
              const Vector3f &normal,
              const Vector3f &wo,
              const Vector3f &wi) {
    if (mat->isMirror()) {
        if (mat->getRoughness() < DELTA_MIRROR_ROUGHNESS) {
            return 0.0f;
        }
        return ggxPdf(normal, wo, wi, mat->getRoughness());
    } else if (mat->isGlass()) {
        return 0.0f;
    } else {
        if (Vector3f::dot(normal, wo) <= 0.0f ||
            Vector3f::dot(normal, wi) <= 0.0f) {
            return 0.0f;
        }
        float specProb = specularSamplingProbability(mat->getSpecularColor());
        float diffuseProb = 1.0f - specProb;
        return diffuseProb * diffusePdf(normal, wi) +
               specProb * ggxPdf(normal, wo, wi, mat->getRoughness());
    }
}

BSDFSample sampleBSDF(Material *mat,
                      const Vector3f &normal,
                      const Vector3f &wo) {
    return sampleBSDF(mat, mat->getDiffuseColor(), normal, wo);
}

BSDFSample sampleBSDF(Material *mat,
                      const Vector3f &diffuseColor,
                      const Vector3f &normal,
                      const Vector3f &wo) {
    BSDFSample result{Vector3f::ZERO, Vector3f::ZERO, 0.0f, false};
    if (mat->isMirror()) {
        if (mat->getRoughness() < DELTA_MIRROR_ROUGHNESS) {
            result.wi = reflect(-wo, normal);
            result.throughputWeight = mat->getSpecularColor();
            result.pdf = 1.0f;
            result.isDelta = true;

        } else {
            GlossySample sample = sampleGGXDirection(normal, wo, mat->getRoughness());
            if (sample.pdf <= 0.0f) {
                result.wi = Vector3f::ZERO;
                result.throughputWeight = Vector3f::ZERO;
                result.pdf = 0.0f;
                result.isDelta = false;
                return result;
            }

            float cosTheta = std::max(Vector3f::dot(sample.dir, normal), 0.0f);
            if (cosTheta <= 0.0f) {
                result.wi = Vector3f::ZERO;
                result.throughputWeight = Vector3f::ZERO;
                result.pdf = 0.0f;
                result.isDelta = false;
                return result;
            }

            Vector3f glossyBRDF = evaluateCookTorranceGGX(
                normal,
                wo,
                sample.dir,
                mat->getSpecularColor(),
                mat->getRoughness()
            );
            
            result.wi = sample.dir;
            result.throughputWeight = glossyBRDF * cosTheta / std::max(sample.pdf, 1e-6f);
            result.pdf = sample.pdf;
            result.isDelta = false;
        }

    } else if (mat->isGlass()) {
        Vector3f rayDir = -wo;
        Vector3f refracted;
        float etaI = 1.0f;
        float etaT = mat->getIOR();
        bool canRefract = refract(rayDir, normal, etaI, etaT, refracted);
        float kr = canRefract 
            ? fresnelSchlick(rayDir, normal, etaI, etaT) 
            : 1.0f;

        if (!canRefract || Random::get_float() < kr) {
            result.wi = reflect(rayDir, normal);
            result.throughputWeight = mat->getSpecularColor();
            result.pdf = kr;
        } else {
            float kt = 1.0f - kr;
            result.wi = refracted;
            result.throughputWeight = mat->getTransmissionColor();
            result.pdf = kt;
        }
        result.isDelta = true;

    } else {
        float specProb = specularSamplingProbability(mat->getSpecularColor());
        Vector3f wi = Vector3f::ZERO;
        if (specProb > 0.0f && Random::get_float() < specProb) {
            GlossySample glossy = sampleGGXDirection(normal, wo, mat->getRoughness());
            wi = glossy.pdf > 0.0f ? glossy.dir : cosineSampleHemisphere(normal);
        } else {
            wi = cosineSampleHemisphere(normal);
        }

        float cosTheta = std::max(0.0f, Vector3f::dot(normal, wi));
        float pdf = bsdfPdf(mat, normal, wo, wi);
        Vector3f f = evaluateBSDF(mat, diffuseColor, normal, wo, wi);
        if (cosTheta <= 0.0f || pdf <= 0.0f || f.squaredLength() <= 0.0f) {
            return result;
        }

        result.wi = wi;
        result.throughputWeight = f * cosTheta / std::max(pdf, 1e-6f);
        result.pdf = pdf;
        result.isDelta = false;
    }
    return result;
}
