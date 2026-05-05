#pragma once

#include "Vector3f.h"
#include "material.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"

#include <vector>

constexpr int MAX_CAMERA_PATH_DEPTH = 5;
constexpr int MAX_LIGHT_PATH_DEPTH = 3;

struct PathVertex {
    Vector3f pos;
    Vector3f normal;
    Vector3f throughput;

    Material *material = nullptr;

    Vector3f wo;
    Vector3f wi;

    float pdfForwardArea = 0.0f;
    float pdfForwardSolidAngle = 0.0f;

    bool isDelta = false;
    bool isLight = false;
};

class SceneParser;

class BDPT {
public:
    explicit BDPT(SceneParser &scene);

    Vector3f trace(const Ray &cameraRay);

private:
    int generateCameraPath(const Ray &cameraRay,
                           std::vector<PathVertex> &path,
                           int maxDepth);

    int generateLightPath(std::vector<PathVertex> &path, int maxDepth);

    Vector3f connectVertices(const PathVertex &eye,
                               const PathVertex &light);

    float bdptMisWeight(int ci, int li) const;

    SceneParser &scene;
    std::vector<PathVertex> cameraPath;
    std::vector<PathVertex> lightPath;
};