#include <cassert>
#include <cstdlib>
#include <cstring>
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

constexpr float P_RR = 0.8f;
constexpr int NUM_SAMPLES = 16;

SceneParser *sceneParser;

Vector3f tracePath(Ray ray, int depth) {
    Hit hit;
    if (!sceneParser->getGroup()->intersect(ray, hit, 0)) {
        return sceneParser->getBackgroundColor();
    }
    Vector3f emission = Vector3f::ZERO;
    if (depth == 0) {
        emission = hit.getMaterial()->getEmission();
    }

    Vector3f pos = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal();
    
    Light::SampleResult sample = sceneParser->sampleLight(pos);
    Vector3f direct = Vector3f::ZERO;
    Vector3f shadowOrigin = pos + normal * 1e-6;
    Ray shadowRay(shadowOrigin, sample.dir);
    Hit shadowHit;
    sceneParser->getGroup()->intersect(shadowRay, shadowHit, 1e-6);
    if (shadowHit.getT() > sample.dist - 1e-4) {
        float cosThetaX = std::max(0.0f, Vector3f::dot(normal, sample.dir));
        float cosThetaY = std::max(0.0f, Vector3f::dot(sample.normal, -sample.dir));
        Vector3f f_r = hit.getMaterial()->getDiffuseColor() / M_PI;
        direct = sample.col * f_r * cosThetaX * cosThetaY * sample.pdf / (sample.dist * sample.dist);
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
    Vector3f indirect = tracePath(nextRay, depth + 1) * hit.getMaterial()->getDiffuseColor() / P_RR;
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

    if (argc != 3) {
        std::cout << "Usage: ./bin/RayTracer <input scene file> <output bmp file>" << std::endl;
        return 1;
    }
    std::string inputFile = argv[1];
    std::string outputFile = argv[2];  // only bmp is allowed.

    std::cout << "Hello! Computer Graphics!" << std::endl;

    sceneParser = new SceneParser(inputFile.c_str());
    std::cout << "lights: " << sceneParser->getNumLights() << "\n";
    for (int i = 0; i < sceneParser->getNumLights(); ++i) {
        std::cout << *sceneParser->getLight(i) << "\n";
    }

    int width = sceneParser->getCamera()->getWidth();
    int height = sceneParser->getCamera()->getHeight();
    Image image(width, height);
    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            Vector3f color = Vector3f::ZERO;
            for (int i = 0; i < NUM_SAMPLES; ++i) {
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = sceneParser->getCamera()->generateRay(Vector2f(x + dx, y + dy));
                color += tracePath(ray, 0);
            }
            color = color / (float)NUM_SAMPLES;
            image.SetPixel(x, y, color);
        }
    }
    image.SaveBMP(outputFile.c_str());

    delete sceneParser;
    return 0;
}

