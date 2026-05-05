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
    explicit BDPT(SceneParser &scene,
                  int primaryDirectLightSamples = 1,
                  int secondaryDirectLightSamples = 1);

    Vector3f trace(const Ray &cameraRay);

private:
    struct ConnectionGeometry {
        Vector3f eyeToLight;
        Vector3f lightToEye;
        float dist2 = 0.0f;
        float dist = 0.0f;
        float cosThetaEye = 0.0f;
        float cosThetaLight = 0.0f;
    };

    int generateCameraPath(const Ray &cameraRay,
                           std::vector<PathVertex> &path,
                           int maxDepth);

    int generateLightPath(std::vector<PathVertex> &path, int maxDepth);

    bool makeConnectionGeometry(const PathVertex &eye,
                                const PathVertex &light,
                                ConnectionGeometry &connection) const;

    Vector3f connectVertices(const PathVertex &eye,
                             const PathVertex &light,
                             const ConnectionGeometry &connection);

    Vector3f estimateDirectLight(const PathVertex &eye, int numSamples) const;

    float bdptMisWeight(int ci, int li, const ConnectionGeometry &connection) const;

    SceneParser &scene;
    int primaryDirectLightSamples;
    int secondaryDirectLightSamples;
    std::vector<PathVertex> cameraPath;
    std::vector<PathVertex> lightPath;
};
