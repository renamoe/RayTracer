#include <cassert>
#include <algorithm>
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

#include <cmath>
#include <string>

#ifdef _OPENMP
#include <omp.h>
#endif

constexpr int NUM_SAMPLES = 1;
constexpr float EXPOSURE = 1.0f;
constexpr int MAX_DEPTH = 8;
constexpr float RAY_EPS = 1e-5f;

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

Vector3f offsetRayOrigin(const Vector3f &pos, const Vector3f &normal, const Vector3f &dir) {
    Vector3f offsetNormal = Vector3f::dot(dir, normal) > 0.0f ? normal : -normal;
    return pos + offsetNormal * RAY_EPS;
}

bool isOccluded(const Vector3f &pos, const Vector3f &normal, const Vector3f &dir, float maxDist) {
    Ray shadowRay(offsetRayOrigin(pos, normal, dir), dir);
    Hit shadowHit;
    sceneParser->getGroup()->intersect(shadowRay, shadowHit, RAY_EPS);
    return shadowHit.getT() < maxDist - RAY_EPS;
}

Vector3f phongDirectLighting(const Ray &ray, const Hit &hit, const Vector3f &pos) {
    Vector3f result = Vector3f::ZERO;
    Material *material = hit.getMaterial();
    Vector3f normal = hit.getNormal();

    for (int i = 0; i < sceneParser->getNumLights(); ++i) {
        Light *light = sceneParser->getLight(i);
        Vector3f dirToLight;
        Vector3f lightColor;
        float maxDist = 1e38f;

        if (auto *pointLight = dynamic_cast<PointLight *>(light)) {
            Vector3f lightVec = pointLight->getPosition() - pos;
            float dist2 = lightVec.squaredLength();
            if (dist2 <= RAY_EPS * RAY_EPS) {
                continue;
            }
            float dist = std::sqrt(dist2);
            dirToLight = lightVec / dist;
            lightColor = pointLight->getColor() / dist2;
            maxDist = dist;
        } else if (auto *directionalLight = dynamic_cast<DirectionalLight *>(light)) {
            dirToLight = -directionalLight->getDirection();
            lightColor = directionalLight->getColor();
        } else {
            continue;
        }

        if (Vector3f::dot(normal, dirToLight) <= 0.0f) {
            continue;
        }
        if (isOccluded(pos, normal, dirToLight, maxDist)) {
            continue;
        }
        result += material->Shade(ray, hit, dirToLight, lightColor);
    }

    return result;
}

Vector3f traceWhitted(Ray ray, int depth) {
    Hit hit;
    if (!sceneParser->getGroup()->intersect(ray, hit, 0)) {
        return sceneParser->getBackgroundColor();
    }
    Material *material = hit.getMaterial();
    Vector3f pos = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal();

    if (depth >= MAX_DEPTH) {
        return Vector3f::ZERO;
    }

    Vector3f color = phongDirectLighting(ray, hit, pos);

    if (material->isMirror()) {
        Vector3f reflected = reflect(ray.getDirection(), normal);
        Ray reflectedRay(offsetRayOrigin(pos, normal, reflected), reflected);
        color += traceWhitted(reflectedRay, depth + 1) * material->getSpecularColor();
        return color;
    }

    if (material->isGlass()) {
        Vector3f reflected = reflect(ray.getDirection(), normal);
        Vector3f refracted;
        float etaI = 1.0f;
        float etaT = material->getIOR();
        bool canRefract = refract(ray.getDirection(), normal, etaI, etaT, refracted);
        float kr = canRefract ? fresnelSchlick(ray.getDirection(), normal, etaI, etaT) : 1.0f;

        Ray reflectedRay(offsetRayOrigin(pos, normal, reflected), reflected);
        color += traceWhitted(reflectedRay, depth + 1) * material->getSpecularColor() * kr;
        if (canRefract) {
            Ray refractedRay(offsetRayOrigin(pos, normal, refracted), refracted);
            color += traceWhitted(refractedRay, depth + 1) * material->getTransmissionColor() * (1.0f - kr);
        }
    }

    return color;
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
    int sampleGridSize = static_cast<int>(std::ceil(std::sqrt(static_cast<float>(numSamples))));
    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            Vector3f color = Vector3f::ZERO;
            for (int i = 0; i < numSamples; ++i) {
                int sx = i % sampleGridSize;
                int sy = i / sampleGridSize;
                float dx = (sx + 0.5f) / static_cast<float>(sampleGridSize);
                float dy = (sy + 0.5f) / static_cast<float>(sampleGridSize);
                Ray ray = sceneParser->getCamera()->generateRay(Vector2f(x + dx, y + dy));
                color += traceWhitted(ray, 0);
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

