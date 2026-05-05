#include "bdpt.hpp"
#include "bsdf.hpp"
#include "group.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float CONNECT_EPS = 1e-4f;

Vector3f offsetRayOrigin(const Vector3f &pos,
                         const Vector3f &normal,
                         const Vector3f &dir) {
    return pos + (Vector3f::dot(dir, normal) > 0.0f ? normal : -normal) * CONNECT_EPS;
}

} // namespace

BDPT::BDPT(SceneParser &scene) : scene(scene) {}

int BDPT::generateCameraPath(const Ray &cameraRay,
                       std::vector<PathVertex> &path,
                       int maxDepth) {
    path.clear();
    
    Ray ray = cameraRay;
    Vector3f beta(1, 1, 1);
    float lastPdfFwd = 1.0f;
    bool lastDelta = false;

    for (int depth = 0; depth < maxDepth; ++depth) {
        Hit hit;
        if (!scene.getGroup()->intersect(ray, hit, 1e-6f)) {
            break;
        }

        Material *mat = hit.getMaterial();
        if (mat == nullptr) {
            break;
        }
        
        Vector3f pos = ray.pointAtParameter(hit.getT());
        Vector3f normal = hit.getNormal();
        if (Vector3f::dot(normal, ray.getDirection()) > 0.0f) {
            normal = -normal;
        }

        PathVertex v;
        v.pos = pos;
        v.normal = normal;
        v.throughput = beta;
        v.material = mat;
        v.wo = -ray.getDirection();
        v.pdfForward = lastPdfFwd;
        v.isDelta = mat->isGlass() || 
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();

        path.push_back(v);
        
        if (v.isLight) {
            break;
        }

        BSDFSample sample = sampleBSDF(mat, normal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.length() <= 0.0f) {
            break;
        }
        
        beta = beta * sample.throughputWeight;

        // TODO: convert to area pdf
        lastPdfFwd = sample.pdf;
        lastDelta = sample.isDelta;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, normal) > 0.0f ? normal : -normal;

        ray = Ray(pos + offsetNormal * 1e-6f, sample.wi);
    }

    return path.size();
}

int BDPT::generateLightPath(std::vector<PathVertex> &path, int maxDepth) {
    path.clear();
    
    LightEmitSample emit = scene.sampleEmitLight();
    if (emit.pdfPos <= 0.0f || emit.pdfDir <= 0.0f ||
        emit.material == nullptr || emit.emission.squaredLength() <= 0.0f) {
        return 0;
    }

    float cosEmit = Vector3f::dot(emit.normal, emit.dir);
    if (cosEmit <= 0.0f) {
        return 0;
    }

    Vector3f sourceThroughput = emit.emission / emit.pdfPos;
    Vector3f beta = emit.emission * cosEmit / (emit.pdfPos * emit.pdfDir);

    PathVertex lightVertex;
    lightVertex.pos = emit.pos;
    lightVertex.normal = emit.normal;
    lightVertex.throughput = sourceThroughput;
    lightVertex.material = emit.material;
    lightVertex.wo = -emit.dir;
    lightVertex.wi = emit.dir;
    lightVertex.pdfForward = emit.pdfPos;
    lightVertex.isDelta = false;
    lightVertex.isLight = true;

    path.push_back(lightVertex);

    Ray ray(emit.pos + emit.normal * 1e-4f, emit.dir);
    float lastPdfFwd = emit.pdfDir;

    for (int depth = 1; depth < maxDepth; ++depth) {
        Hit hit;
        if (!scene.getGroup()->intersect(ray, hit, 1e-6f)) {
            break;
        }

        Material *mat = hit.getMaterial();
        if (mat == nullptr) {
            break;
        }

        Vector3f pos = ray.pointAtParameter(hit.getT());
        Vector3f normal = hit.getNormal();
        if (Vector3f::dot(normal, ray.getDirection()) > 0.0f) {
            normal = -normal;
        }

        PathVertex v;
        v.pos = pos;
        v.normal = normal;
        v.throughput = beta;
        v.material = mat;
        v.wo = -ray.getDirection();
        v.pdfForward = lastPdfFwd;
        v.isDelta = mat->isGlass() ||
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();

        path.push_back(v);

        if (v.isLight) {
            break;
        }

        BSDFSample sample = sampleBSDF(mat, normal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        beta = beta * sample.throughputWeight;
        lastPdfFwd = sample.pdf;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, normal) > 0.0f ? normal : -normal;
        ray = Ray(pos + offsetNormal * 1e-6f, sample.wi);
    }

    return path.size();
}

Vector3f BDPT::connectVertices(const PathVertex &eye,
                               const PathVertex &light) {
    if (eye.material == nullptr || light.material == nullptr ||
        eye.isDelta || light.isDelta) {
        return Vector3f::ZERO;
    }
    
    Vector3f edge = light.pos - eye.pos;
    float dist2 = edge.squaredLength();
    if (dist2 < 1e-12f) {
        return Vector3f::ZERO;
    }

    float dist = std::sqrt(dist2);
    Vector3f eyeToLight = edge / dist;
    Vector3f lightToEye = -eyeToLight;
    
    float cosThetaEye = std::max(0.0f, Vector3f::dot(eye.normal, eyeToLight));
    float cosThetaLight = std::max(0.0f, Vector3f::dot(light.normal, lightToEye));
    if (cosThetaEye <= 0.0f || cosThetaLight <= 0.0f) {
        return Vector3f::ZERO;
    }

    Ray shadowRay(offsetRayOrigin(eye.pos, eye.normal, eyeToLight), eyeToLight);
    if (scene.getGroup()->occluded(shadowRay, 1e-6f, dist - CONNECT_EPS)) {
        return Vector3f::ZERO;
    }

    Vector3f fEye = evaluateBSDF(eye.material, eye.normal, eye.wo, eyeToLight);
    if (fEye.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    Vector3f fLight = light.isLight
        ? Vector3f(1, 1, 1)
        : evaluateBSDF(light.material, light.normal, light.wo, lightToEye);
    if (fLight.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    float geometryTerm = cosThetaEye * cosThetaLight / dist2;
    float misWeight = 1.0f; // TODO: use BDPT MIS after tracking reverse/area PDFs.

    return eye.throughput * fEye * geometryTerm * fLight * light.throughput * misWeight;
}

Vector3f BDPT::trace(const Ray &cameraRay) {
    cameraPath.clear();
    lightPath.clear();

    generateCameraPath(cameraRay, cameraPath, MAX_CAMERA_PATH_DEPTH);
    generateLightPath(lightPath, MAX_LIGHT_PATH_DEPTH);

    Vector3f L = Vector3f::ZERO;

    for (const auto &eye : cameraPath) {
        if (eye.isLight) {
            L += eye.throughput * eye.material->getEmission();
            continue;
        }

        for (const auto &light : lightPath) {
            L += connectVertices(eye, light);
        }
    }

    return L;
}
