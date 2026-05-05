#pragma once

#include "Vector3f.h"
#include "material.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"

#include <vector>

constexpr int MAX_CAMERA_PATH_DEPTH = 8;
constexpr int MAX_LIGHT_PATH_DEPTH = 5;

enum class PathVertexType {
    Camera,
    Light,
    Surface
};

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
    PathVertexType type = PathVertexType::Surface;
};

class SceneParser;

class BDPT {
public:
    struct FilmSplat {
        int x = 0;
        int y = 0;
        Vector3f contribution;
    };

    explicit BDPT(SceneParser &scene,
                  int primaryDirectLightSamples = 1,
                  int secondaryDirectLightSamples = 1);

    Vector3f trace(const Ray &cameraRay);
    Vector3f trace(const Ray &cameraRay,
                   std::vector<FilmSplat> *splats,
                   float splatScale);

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

    Vector3f connectBDPT(int s,
                         int t,
                         std::vector<FilmSplat> *splats,
                         float splatScale);

    Vector3f connectLightToCamera(int s,
                                  std::vector<FilmSplat> *splats,
                                  float splatScale);

    Vector3f estimateDirectLight(const PathVertex &eye, int ci, int numSamples) const;

    Vector3f estimateCameraHitLight(int ci, bool includeLightTracingMis) const;

    float cameraAreaPdf(const PathVertex &vertex,
                        const Vector3f &dirFromCamera,
                        float dist2) const;

    float bdptMisWeight(int s, int t, const ConnectionGeometry &connection) const;
    float bdptMisWeightLightToCamera(int s,
                                     const Vector3f &vertexToCamera,
                                     float cameraPdfArea) const;

    SceneParser &scene;
    int primaryDirectLightSamples;
    int secondaryDirectLightSamples;
    std::vector<PathVertex> cameraPath;
    std::vector<PathVertex> lightPath;
};
