#include <cassert>
#include <chrono>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>

#include "Vector3f.h"
#include "scene_parser.hpp"
#include "image.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "light.hpp"
#include "random.hpp"

#include <cmath>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

constexpr float P_RR = 0.8f;
constexpr int NUM_SAMPLES = 32;
constexpr float DEFAULT_EXPOSURE = 1.5f;
constexpr float DELTA_MIRROR_ROUGHNESS = 0.0015f;
constexpr int MAX_DEPTH = 8;

SceneParser *sceneParser;

Vector3f toneMap(Vector3f color, float exposure) {
    color = color * exposure;
    color = Vector3f(
        1.0f - std::exp(-std::max(0.0f, color.x())),
        1.0f - std::exp(-std::max(0.0f, color.y())),
        1.0f - std::exp(-std::max(0.0f, color.z()))
    );
    return Vector3f(
        std::pow(color.x(), 1.0f / 2.2f),
        std::pow(color.y(), 1.0f / 2.2f),
        std::pow(color.z(), 1.0f / 2.2f)
    );
}

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

struct GlossySample {
    Vector3f dir;
    float pdf;
};

float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

Vector3f fresnelSchlickColor(float cosTheta, const Vector3f &F0) {
    float f = std::pow(1.0f - clamp01(cosTheta), 5.0f);
    return F0 + (Vector3f(1, 1, 1) - F0) * f;
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

float distributionGGX(const Vector3f &N, const Vector3f &H, float roughness) {
    float alpha = std::max(0.001f, roughness * roughness);
    float a2 = alpha * alpha;
    float NoH = clamp01(Vector3f::dot(N, H));
    float NoH2 = NoH * NoH;

    float denom = NoH2 * (a2 - 1.0f) + 1.0f;
    return a2 / (M_PI * denom * denom);
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

float geometrySmithGGX(const Vector3f &N, const Vector3f &V, const Vector3f &L, float roughness) {
    float alpha = std::max(0.001f, roughness * roughness);
    float a2 = alpha * alpha;

    auto G1 = [a2](float NoX) {
        NoX = std::max(0.0f, NoX);
        return 2.0f * NoX / (NoX + std::sqrt(a2 + (1.0f - a2) * NoX * NoX));
    };

    return G1(Vector3f::dot(N, V)) * G1(Vector3f::dot(N, L));
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

Vector3f estimateGlossyDirectLight(const Vector3f &pos,
                                   const Vector3f &normal,
                                   const Vector3f &rayDir,
                                   Material *material) {
    Light::SampleResult lightSample = sceneParser->sampleLight(pos);
    if (lightSample.pdf <= 0.0f || lightSample.dist <= 0.0f) {
        return Vector3f::ZERO;
    }

    float cosThetaX = std::max(0.0f, Vector3f::dot(normal, lightSample.dir));
    float cosThetaY = std::max(0.0f, Vector3f::dot(lightSample.normal, -lightSample.dir));
    if (cosThetaX <= 0.0f || cosThetaY <= 0.0f) {
        return Vector3f::ZERO;
    }

    Vector3f shadowOrigin = pos + normal * 1e-6;
    Ray shadowRay(shadowOrigin, lightSample.dir);
    Hit shadowHit;
    sceneParser->getGroup()->intersect(shadowRay, shadowHit, 1e-6);
    if (shadowHit.getT() <= lightSample.dist - 1e-4) {
        return Vector3f::ZERO;
    }

    Vector3f f_r = evaluateCookTorranceGGX(
        normal,
        (-rayDir).normalized(),
        lightSample.dir.normalized(),
        material->getSpecularColor(),
        material->getRoughness()
    );

    Vector3f V = (-rayDir).normalized();

    float pdfLight = areaPdfToSolidAnglePdf(lightSample.pdf, lightSample.dist, cosThetaY);
    float pdfBrdf = ggxPdf(normal, V, lightSample.dir.normalized(), material->getRoughness());
    if (pdfLight <= 0.0f) {
        return Vector3f::ZERO;
    }
    float wLight = powerHeuristic(pdfLight, pdfBrdf);

    return lightSample.col * f_r * cosThetaX / pdfLight * wLight;
}

Vector3f tracePath(Ray ray, int depth, bool fromSpecular = false) {
    Hit hit;
    if (!sceneParser->getGroup()->intersect(ray, hit, 0)) {
        return sceneParser->getBackgroundColor();
    }
    Material *material = hit.getMaterial();
    Vector3f emission = Vector3f::ZERO;
    if (depth == 0 || fromSpecular) {
        emission = material->getEmission();
    }

    if (depth >= MAX_DEPTH) {
        return emission;
    }

    Vector3f pos = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal();

    if (material->isMirror()) {
        Vector3f shadingNormal = normal;
        if (Vector3f::dot(ray.getDirection(), shadingNormal) > 0.0f) {
            shadingNormal = -shadingNormal;
        }
        Vector3f R = reflect(ray.getDirection(), shadingNormal);

        if (material->getRoughness() <= DELTA_MIRROR_ROUGHNESS) {
            if (Random::get_float() > P_RR) {
                return emission;
            }
            Vector3f offsetNormal = Vector3f::dot(R, normal) > 0 ? normal : -normal;
            Ray nextRay(pos + offsetNormal * 1e-6, R);
            return emission + tracePath(nextRay, depth + 1, true) * material->getSpecularColor() / P_RR;
        }

        Vector3f direct = estimateGlossyDirectLight(pos, shadingNormal, ray.getDirection(), material);
        if (Random::get_float() > P_RR) {
            return emission + direct;
        }

        Vector3f V = (-ray.getDirection()).normalized();
        GlossySample sample = sampleGGXDirection(shadingNormal, V, material->getRoughness());
        float cosTheta = Vector3f::dot(sample.dir, shadingNormal);
        if (cosTheta <= 0.0f || sample.pdf <= 0.0f) {
            return emission + direct;
        }
        Vector3f wi = sample.dir;
        Vector3f glossyBRDF = evaluateCookTorranceGGX(
            shadingNormal,
            V,
            wi.normalized(),
            material->getSpecularColor(),
            material->getRoughness()
        );
        Vector3f offsetNormal = Vector3f::dot(wi, normal) > 0 ? normal : -normal;
        Ray nextRay(pos + offsetNormal * 1e-6, wi);
        Hit nextHit;
        sceneParser->getGroup()->intersect(nextRay, nextHit, 1e-6f);

        bool hitLight = nextHit.getMaterial() != nullptr && nextHit.getMaterial()->isEmissive();
        Vector3f brdfLight = Vector3f::ZERO;
        if (hitLight) {
            float pdfLight = sceneParser->lightPdf(pos + offsetNormal * 1e-6, wi);
            float wBrdf = powerHeuristic(sample.pdf, pdfLight);

            brdfLight = nextHit.getMaterial()->getEmission() * glossyBRDF * cosTheta / std::max(sample.pdf, 1e-6f) * wBrdf / P_RR;
        }

        Vector3f indirect = Vector3f::ZERO;
        if (!hitLight) {
            indirect = tracePath(nextRay, depth + 1) * glossyBRDF * cosTheta / (sample.pdf * P_RR);
        }

        return emission + direct + brdfLight + indirect;
    }

    if (material->isGlass()) {
        if (Random::get_float() > P_RR) {
            return emission;
        }
        
        Vector3f reflected = reflect(ray.getDirection(), normal);

        Vector3f refracted;
        float etaI = 1.0f;
        float etaT = material->getIOR();
        bool canRefract = refract(ray.getDirection(), normal, etaI, etaT, refracted);
        float kr = canRefract ? fresnelSchlick(ray.getDirection(), normal, etaI, etaT) : 1.0f;
        
        if (Random::get_float() < kr) {
            Vector3f offsetNormal = Vector3f::dot(reflected, normal) > 0 ? normal : -normal;
            Ray nextRay(pos + offsetNormal * 1e-6, reflected);
            return emission + tracePath(nextRay, depth + 1, true) * material->getSpecularColor() / P_RR;
        } else {
            Vector3f offsetNormal = Vector3f::dot(refracted, normal) > 0 ? normal : -normal;
            Ray nextRay(pos + offsetNormal * 1e-6, refracted);
            return emission + tracePath(nextRay, depth + 1, true) * material->getTransmissionColor() / P_RR;
        }
    }
    
    Light::SampleResult sample = sceneParser->sampleLight(pos);
    Vector3f direct = Vector3f::ZERO;
    if (sample.pdf > 0.0f && sample.dist > 0.0f) {
        Vector3f shadowOrigin = pos + normal * 1e-6;
        Ray shadowRay(shadowOrigin, sample.dir);
        Hit shadowHit;
        sceneParser->getGroup()->intersect(shadowRay, shadowHit, 1e-6);
        if (shadowHit.getT() > sample.dist - 1e-4) {
            float cosThetaX = std::max(0.0f, Vector3f::dot(normal, sample.dir));
            float cosThetaY = std::max(0.0f, Vector3f::dot(sample.normal, -sample.dir));
            Vector3f f_r = material->getDiffuseColor() / M_PI;
            float pdfLight = areaPdfToSolidAnglePdf(sample.pdf, sample.dist, cosThetaY);
            float pdfBrdf = diffusePdf(normal, sample.dir);
            if (pdfLight > 0.0f) {
                float wLight = powerHeuristic(pdfLight, pdfBrdf);
                direct = sample.col * f_r * cosThetaX / pdfLight * wLight;
            }
        }
    }
    
    if (Random::get_float() > P_RR) {
        return emission + direct;
    }

    Vector3f T, B;
    if (std::abs(normal.x()) > 0.9f) {
        T = Vector3f::cross(Vector3f(0, 1, 0), normal).normalized();
    } else {
        T = Vector3f::cross(Vector3f(1, 0, 0), normal).normalized();
    }
    B = Vector3f::cross(normal, T).normalized();
    float r1 = 2 * M_PI * Random::get_float();
    float r2 = Random::get_float();
    float r2s = std::sqrt(r2);
    Vector3f wi = (T * std::cos(r1) * r2s + 
                   B * std::sin(r1) * r2s + 
                   normal * std::sqrt(1 - r2)).normalized();
    Ray nextRay(pos + normal * 1e-6, wi);

    Hit nextHit;
    sceneParser->getGroup()->intersect(nextRay, nextHit, 1e-6f);

    bool hitLight = nextHit.getMaterial() != nullptr && nextHit.getMaterial()->isEmissive();
    Vector3f brdfLight = Vector3f::ZERO;
    if (hitLight) {
        float cosTheta = std::max(0.0f, Vector3f::dot(normal, wi));
        float pdfBrdf = diffusePdf(normal, wi);
        float pdfLight = sceneParser->lightPdf(pos + normal * 1e-6, wi);
        float wBrdf = powerHeuristic(pdfBrdf, pdfLight);

        Vector3f f_r = material->getDiffuseColor() / M_PI;
        brdfLight = nextHit.getMaterial()->getEmission() * f_r * cosTheta / std::max(pdfBrdf, 1e-6f) * wBrdf / P_RR;
    }
    
    Vector3f indirect = Vector3f::ZERO;
    if (!hitLight) {
        indirect = tracePath(nextRay, depth + 1) * material->getDiffuseColor() / P_RR;
    }
    
    Vector3f result = emission + direct + brdfLight + indirect;
    if (!isFiniteColor(result)) {
        std::cout << "nan or inf" << std::endl;
        return Vector3f::ZERO;
    }
    return result;
}

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc < 3 || argc > 5) {
        std::cout << "Usage: ./bin/RayTracer <input scene file> <output bmp file> [num_samples] [exposure]" << std::endl;
        return 1;
    }
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];  // only bmp is allowed.
    int numSamples = argc >= 4 ? std::atoi(argv[3]) : NUM_SAMPLES;
    if (numSamples <= 0) {
        std::cout << "num samples must be a positive integer" << std::endl;
        return 1;
    }
    float exposure = argc >= 5 ? std::atof(argv[4]) : DEFAULT_EXPOSURE;
    if (!std::isfinite(exposure) || exposure <= 0.0f) {
        std::cout << "exposure must be a positive number" << std::endl;
        return 1;
    }
    std::cout << "num samples per pixel: " << numSamples << std::endl;
    std::cout << "exposure: " << exposure << std::endl;

    std::cout << "Hello! Computer Graphics!" << std::endl;

    sceneParser = new SceneParser(inputFile.c_str());
    std::cout << "num lights: " << sceneParser->getNumLights() << "\n";

    int width = sceneParser->getCamera()->getWidth();
    int height = sceneParser->getCamera()->getHeight();
    Image image(width, height);

    const auto renderStart = std::chrono::steady_clock::now();
    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            Vector3f color = Vector3f::ZERO;
            for (int i = 0; i < numSamples; ++i) {
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = sceneParser->getCamera()->generateRay(Vector2f(x + dx, y + dy));
                color += tracePath(ray, 0);
            }
            color = color / (float)numSamples;
            color = toneMap(color, exposure);
            image.SetPixel(x, y, color);
        }
    }
    const auto renderEnd = std::chrono::steady_clock::now();

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    const double statsSeconds = renderSeconds > 0.0 ? renderSeconds : 1e-9;
    const long long numPixels = static_cast<long long>(width) * height;
    const long long primarySamples = numPixels * static_cast<long long>(numSamples);

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[render stats] resolution: " << width << "x" << height
              << ", spp: " << numSamples;
#ifdef _OPENMP
    std::cout << ", threads: " << omp_get_max_threads();
#endif
    std::cout << "\n";
    std::cout << "[render stats] total render time: " << renderSeconds << " s\n";
    std::cout << "[render stats] avg time per full-image spp: "
              << statsSeconds / static_cast<double>(numSamples) << " s\n";
    std::cout << "[render stats] avg time per primary sample: "
              << statsSeconds * 1000000.0 / static_cast<double>(primarySamples) << " us\n";
    std::cout << "[render stats] throughput: "
              << numPixels / statsSeconds << " pixels/s, "
              << primarySamples / statsSeconds << " primary samples/s\n";
    
    image.SaveBMP(outputFile.c_str());

    delete sceneParser;
    return 0;
}

