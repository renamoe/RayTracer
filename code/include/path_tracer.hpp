#pragma once

#include "bsdf.hpp"
#include "Vector3f.h"
#include "hit.hpp"
#include "ray.hpp"

class Material;
class PathGuideGrid;
class SceneParser;

class PathTracer {
public:
    explicit PathTracer(SceneParser &scene,
                        float specularClamp = 0.0f,
                        PathGuideGrid *pathGuideGrid = nullptr,
                        bool trainPathGuiding = false,
                        bool usePathGuiding = false,
                        float pathGuidingProbability = 0.5f);

    Vector3f trace(const Ray &ray, int depth = 0, bool fromSpecular = false);

private:
    Vector3f traceFromHit(const Ray &ray, const Hit &hit, int depth, bool fromSpecular = false);
    Vector3f estimateGlossyDirectLight(const Vector3f &pos,
                                       const Vector3f &normal,
                                       const Vector3f &rayDir,
                                       Material *material);
    float activeGuideProbability(Material *material,
                                 const Vector3f &pos,
                                 const Vector3f &normal) const;
    float continuationPdf(Material *material,
                          const Vector3f &pos,
                          const Vector3f &normal,
                          const Vector3f &wo,
                          const Vector3f &wi) const;
    BSDFSample sampleGuidedBSDF(Material *material,
                                const Vector3f &diffuseColor,
                                const Vector3f &pos,
                                const Vector3f &normal,
                                const Vector3f &wo) const;
    void recordGuidingSample(Material *material,
                             const Vector3f &pos,
                             const Vector3f &normal,
                             const Vector3f &wi,
                             const Vector3f &incoming) const;

    SceneParser &scene;
    float specularClamp;
    PathGuideGrid *pathGuideGrid;
    bool trainPathGuiding;
    bool usePathGuiding;
    float pathGuidingProbability;
};
