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
constexpr float EXPOSURE = 2.0f;
constexpr float DELTA_MIRROR_ROUGHNESS = 0.0015f;
constexpr int MAX_DEPTH = 8;

SceneParser *sceneParser;

Vector3f toneMap(Vector3f color) {
    color = color * EXPOSURE;
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
    float cosAlpha;
    float exponent;
};

float glossyExponentFromRoughness(float roughness) {
    roughness = std::max(0.001f, roughness);
    return std::max(1.0f, 2.0f / (roughness * roughness) - 2.0f);
}

Vector3f evaluateGlossyBRDF(const Vector3f &specularColor,
                            const Vector3f &R,
                            const Vector3f &wi,
                            float exponent) {
    float cosAlpha = std::max(0.0f, Vector3f::dot(R.normalized(), wi.normalized()));
    if (cosAlpha <= 0.0f) {
        return Vector3f::ZERO;
    }
    return specularColor * ((exponent + 2.0f) * std::pow(cosAlpha, exponent) / (2.0f * M_PI));
}

GlossySample sampleGlossyDirection(const Vector3f &R, float roughness) {
    float exponent = glossyExponentFromRoughness(roughness);

    Vector3f W = R.normalized();
    Vector3f U;
    if (std::abs(W.x()) > 0.9f) {
        U = Vector3f::cross(Vector3f(0, 1, 0), W).normalized();
    } else {
        U = Vector3f::cross(Vector3f(1, 0, 0), W).normalized();
    }
    Vector3f V = Vector3f::cross(W, U).normalized();

    float r1 = 2.0f * M_PI * Random::get_float();
    float r2 = Random::get_float();

    float cosTheta = std::pow(r2, 1.0f / (exponent + 1.0f));
    float sinTheta = std::sqrt(std::max(0.0f, 1.0f - cosTheta * cosTheta));

    Vector3f wi = (U * std::cos(r1) * sinTheta +
                   V * std::sin(r1) * sinTheta +
                   W * cosTheta).normalized();
    float pdf = (exponent + 1.0f) * std::pow(cosTheta, exponent) / (2.0f * M_PI);
    return {wi, pdf, cosTheta, exponent};
}

Vector3f estimateGlossyDirectLight(const Vector3f &pos,
                                   const Vector3f &normal,
                                   const Vector3f &R,
                                   Material *material,
                                   float exponent) {
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

    Vector3f f_r = evaluateGlossyBRDF(material->getSpecularColor(), R, lightSample.dir, exponent);
    return lightSample.col * f_r * cosThetaX * cosThetaY /
           (lightSample.pdf * lightSample.dist * lightSample.dist);
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

        float exponent = glossyExponentFromRoughness(material->getRoughness());
        Vector3f direct = estimateGlossyDirectLight(pos, shadingNormal, R, material, exponent);
        if (Random::get_float() > P_RR) {
            return emission + direct;
        }

        GlossySample sample = sampleGlossyDirection(R, material->getRoughness());
        float cosTheta = Vector3f::dot(sample.dir, shadingNormal);
        if (cosTheta <= 0.0f || sample.pdf <= 0.0f) {
            return emission + direct;
        }
        Vector3f glossyBRDF = evaluateGlossyBRDF(material->getSpecularColor(), R, sample.dir, sample.exponent);
        Vector3f wi = sample.dir;
        Vector3f offsetNormal = Vector3f::dot(wi, normal) > 0 ? normal : -normal;
        Ray nextRay(pos + offsetNormal * 1e-6, wi);
        return emission + direct + tracePath(nextRay, depth + 1) * glossyBRDF * cosTheta / (sample.pdf * P_RR);
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
            direct = sample.col * f_r * cosThetaX * cosThetaY / (sample.pdf * sample.dist * sample.dist);
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
    Vector3f indirect = tracePath(nextRay, depth + 1) * material->getDiffuseColor() / P_RR;
    Vector3f result = emission + direct + indirect;
    if (std::isnan(result.x()) || std::isinf(result.x())) {
        std::cout << "nan or inf" << std::endl;
        return Vector3f::ZERO;
    }
    return result;
}

int main(int argc, char *argv[]) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc != 3 && argc != 4) {
        std::cout << "Usage: ./bin/RayTracer <input scene file> <output bmp file> [num_samples]" << std::endl;
        return 1;
    }
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];  // only bmp is allowed.
    int numSamples = argc == 4 ? std::atoi(argv[3]) : NUM_SAMPLES;
    if (numSamples <= 0) {
        std::cout << "num samples must be a positive integer" << std::endl;
        return 1;
    }
    std::cout << "num samples per pixel: " << numSamples << std::endl;

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
            color = toneMap(color);
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

