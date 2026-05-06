#pragma once

#include "Vector3f.h"
#include "material.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"

#include <vector>

constexpr int MAX_VCM_CAMERA_PATH_DEPTH = 8;
constexpr int MAX_VCM_LIGHT_PATH_DEPTH = 5;

enum class VCMPathVertexType {
    Camera,
    Light,
    Surface
};

struct VCMPathVertex {
    Vector3f pos;
    Vector3f normal;
    Vector3f throughput;
    Vector3f diffuseColor = Vector3f(1, 1, 1);

    Material *material = nullptr;

    Vector3f wo;
    Vector3f wi;

    float pdfForwardArea = 0.0f;
    float pdfReverseArea = 0.0f;
    float pdfForwardSolidAngle = 0.0f;

    bool isDelta = false;
    bool isLight = false;
    VCMPathVertexType type = VCMPathVertexType::Surface;
};

class SceneParser;

class VCM {
public:
    struct FilmSplat {
        int x = 0;
        int y = 0;
        Vector3f contribution;
    };

    explicit VCM(SceneParser &scene,
                 int primaryDirectLightSamples = 1,
                 int secondaryDirectLightSamples = 1);

    void beginIteration(int iteration, int width, int height);

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
                           std::vector<VCMPathVertex> &path,
                           int maxDepth);

    int generateLightPath(std::vector<VCMPathVertex> &path, int maxDepth);

    bool makeConnectionGeometry(const VCMPathVertex &eye,
                                const VCMPathVertex &light,
                                ConnectionGeometry &connection) const;

    Vector3f connectVertices(const VCMPathVertex &eye,
                             const VCMPathVertex &light,
                             const ConnectionGeometry &connection) const;

    Vector3f connectVCM(int s,
                        int t,
                        std::vector<FilmSplat> *splats,
                        float splatScale);

    Vector3f connectLightToCamera(int s,
                                  std::vector<FilmSplat> *splats,
                                  float splatScale);

    Vector3f estimateDirectLight(const VCMPathVertex &eye, int cameraIndex, int numSamples) const;

    Vector3f estimateCameraHitLight(int ci, bool includeLightTracingMis) const;

    float cameraAreaPdf(const VCMPathVertex &vertex,
                        const Vector3f &dirFromCamera,
                        float dist2) const;

    float pdfAreaFromVertexTo(const VCMPathVertex &from,
                              const VCMPathVertex *prev,
                              const VCMPathVertex &to) const;

    float pdfAreaFromVertexToDirection(const VCMPathVertex &from,
                                       const Vector3f &wo,
                                       const VCMPathVertex &to) const;

    float vcmMisWeight(const std::vector<VCMPathVertex> &lightVertices,
                       const std::vector<VCMPathVertex> &cameraVertices,
                       int s,
                       int t,
                       float cameraPdfArea = 0.0f) const;

    SceneParser &scene;
    int primaryDirectLightSamples;
    int secondaryDirectLightSamples;
    std::vector<VCMPathVertex> cameraPath;
    std::vector<VCMPathVertex> lightPath;
};
