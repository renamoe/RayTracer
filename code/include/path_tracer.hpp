#pragma once

#include "Vector3f.h"
#include "hit.hpp"
#include "ray.hpp"

class Material;
class SceneParser;

class PathTracer {
public:
    explicit PathTracer(SceneParser &scene);

    Vector3f trace(const Ray &ray, int depth = 0, bool fromSpecular = false);

private:
    Vector3f traceFromHit(const Ray &ray, const Hit &hit, int depth, bool fromSpecular = false);
    Vector3f estimateGlossyDirectLight(const Vector3f &pos,
                                       const Vector3f &normal,
                                       const Vector3f &rayDir,
                                       Material *material);

    SceneParser &scene;
};
