#pragma once

#include "Vector3f.h"
#include "hash_grid.hpp"
#include "material.hpp"
#include "ray.hpp"
#include "scene_parser.hpp"
#include "path_vertex.hpp"

#include <cstddef>
#include <vector>

constexpr int MAX_VCM_CAMERA_PATH_DEPTH = 5;
constexpr int MAX_VCM_LIGHT_PATH_DEPTH = 3;


class VCM {
public:
    struct FilmSplat {
        int x = 0;
        int y = 0;
        Vector3f contribution;
    };

    explicit VCM(SceneParser &scene,
                 int primaryDirectLightSamples,
                 int secondaryDirectLightSamples,
                 float baseRadius);

    void beginIteration(int iteration, int width, int height);

    Vector3f trace(size_t pathIdx, const Ray &cameraRay);
    Vector3f trace(size_t pathIdx, 
                   const Ray &cameraRay,
                   std::vector<FilmSplat> *splats,
                   float splatScale);
    Vector3f traceVMOnly(size_t pathIdx, const Ray &cameraRay);
    Vector3f traceVCOnly(size_t pathIdx,
                         const Ray &cameraRay,
                         std::vector<FilmSplat> *splats = nullptr,
                         float splatScale = 0.0f);
    Vector3f traceVCMNoMIS(size_t pathIdx,
                           const Ray &cameraRay,
                           std::vector<FilmSplat> *splats = nullptr,
                           float splatScale = 0.0f);

private:
    struct ConnectionGeometry {
        Vector3f eyeToLight;
        Vector3f lightToEye;
        float dist2 = 0.0f;
        float dist = 0.0f;
        float cosThetaEye = 0.0f;
        float cosThetaLight = 0.0f;
    };

    void generateLightPath(size_t pathIdx, int maxDepth);

    Vector3f gatherPhotons(const VCMPathVertex &v,
                           const Vector3f &cameraThroughput) const;

    Vector3f generateCameraPath(const Ray &cameraRay,
                     std::vector<VCMPathVertex> &cameraPath, 
                     int maxDepth,
                     bool includeHitLight,
                     bool includeMerging);

    bool makeConnectionGeometry(const VCMPathVertex &eye,
                                const VCMPathVertex &light,
                                ConnectionGeometry &connection) const;

    Vector3f connectVertices(const VCMPathVertex &eye,
                             const VCMPathVertex &light,
                             const ConnectionGeometry &connection) const;

    Vector3f connectVCM(int s,
                        int t,
                        std::vector<FilmSplat> *splats,
                        float splatScale,
                        const std::vector<VCMPathVertex> &cameraPath,
                        const std::vector<VCMPathVertex> &lightPath,
                        bool useMis = true);

    Vector3f connectLightToCamera(int s,
                                  std::vector<FilmSplat> *splats,
                                  float splatScale,
                                  const std::vector<VCMPathVertex> &lightPath,
                                  const std::vector<VCMPathVertex> &cameraPath,
                                  bool useMis = true);

    Vector3f estimateDirectLight(const VCMPathVertex &eye, 
                                int cameraIndex, 
                                int numSamples,
                                const std::vector<VCMPathVertex> &cameraPath,
                                const std::vector<VCMPathVertex> &lightPath,
                                bool useMis = true) const;

    Vector3f estimateCameraHitLight(int ci, 
                                    bool includeLightTracingMis,
                                    const std::vector<VCMPathVertex> &cameraPath,
                                    const std::vector<VCMPathVertex> &lightPath,
                                    bool useMis = true) const;

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

    int lightPathCount;
    float baseRadius;
    float pmRadius;
    std::vector<VCMPathVertex> lightPhotons;
    HashGrid photonHashGrid;
    std::vector<size_t> pathHeads;
};
