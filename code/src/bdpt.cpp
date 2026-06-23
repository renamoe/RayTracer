#include "bdpt.hpp"
#include "Vector3f.h"
#include "bsdf.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "random.hpp"

#include <algorithm>
#include <cmath>

#ifndef BDPT_DEBUG_MIS
#define BDPT_DEBUG_MIS 0
#endif

#if BDPT_DEBUG_MIS
#include <iostream>
#endif

namespace {

constexpr float CONNECT_EPS = 1e-4f;
constexpr float MIN_CONNECT_DIST = 1e-3f;
constexpr float MAX_CONNECTION_GEOMETRY_TERM = 1e4f;
constexpr int RR_START_DEPTH = 2;
constexpr float RR_MIN_CONTINUE = 0.05f;
constexpr float RR_MAX_CONTINUE = 0.95f;
constexpr float DELTA_HIT_LIGHT_MAX_LUMINANCE = 20.0f;

Vector3f clampLuminance(const Vector3f &c, float maxLum) {
    float lum = 0.2126f * c.x() + 0.7152f * c.y() + 0.0722f * c.z();
    if (lum > maxLum && lum > 0.0f) {
        return c * (maxLum / lum);
    }
    return c;
}

float rrContinueProbability(const Vector3f &beta) {
    float maxComp = std::max(beta.x(), std::max(beta.y(), beta.z()));
    if (!std::isfinite(maxComp) || maxComp <= 0.0f) {
        return 0.0f;
    }
    return std::min(RR_MAX_CONTINUE, std::max(RR_MIN_CONTINUE, maxComp));
}

Vector3f offsetRayOrigin(const Vector3f &pos,
                         const Vector3f &normal,
                         const Vector3f &dir) {
    return pos + (Vector3f::dot(dir, normal) > 0.0f ? normal : -normal) * CONNECT_EPS;
}

float remap0(float pdf) {
    return pdf > 0.0f && std::isfinite(pdf) ? pdf : 1.0f;
}

float pdfAreaFromDirection(const PathVertex &from,
                           const Vector3f &wo,
                           const PathVertex &to,
                           const Vector3f &dir,
                           float dist2) {
    if ((!from.isLight && from.material == nullptr) || from.isDelta) {
        return 0.0f;
    }

    float pdfW = from.isLight
        ? std::max(0.0f, Vector3f::dot(from.normal, dir)) / M_PI
        : bsdfPdf(from.material, from.normal, wo, dir);

    if (pdfW <= 0.0f || dist2 <= 1e-12f) {
        return 0.0f;
    }

    float cosTarget = std::max(0.0f, Vector3f::dot(to.normal, -dir));
    return pdfW * cosTarget / dist2;
}

} // namespace

BDPT::BDPT(SceneParser &scene,
           int primaryDirectLightSamples,
           int secondaryDirectLightSamples)
    : scene(scene),
      primaryDirectLightSamples(std::max(0, primaryDirectLightSamples)),
      secondaryDirectLightSamples(std::max(0, secondaryDirectLightSamples)) {}

int BDPT::generateCameraPath(const Ray &cameraRay,
                       std::vector<PathVertex> &path,
                       int maxDepth) {
    path.clear();

    Camera *camera = scene.getCamera();
    PathVertex cameraVertex;
    cameraVertex.pos = camera->getCenter();
    cameraVertex.normal = camera->getDirection();
    cameraVertex.throughput = Vector3f(1, 1, 1);
    cameraVertex.pdfForwardArea = 1.0f;
    cameraVertex.type = PathVertexType::Camera;
    cameraVertex.wi = cameraRay.getDirection();
    path.push_back(cameraVertex);
    
    Ray ray = cameraRay;
    Vector3f beta(1, 1, 1);
    float pendingPdfW = 0.0f;

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
        Vector3f geometryNormal = hit.getNormal();
        Vector3f normal = geometryNormal;
        if (Vector3f::dot(normal, ray.getDirection()) > 0.0f) {
            normal = -normal;
        }

        PathVertex v;
        v.pos = pos;
        v.normal = normal;
        v.throughput = beta;
        v.diffuseColor = mat->getDiffuseColor(hit);
        v.material = mat;
        v.wo = -ray.getDirection();
        v.isDelta = mat->isGlass() || 
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();
        v.type = PathVertexType::Surface;

        PathVertex &p = path.back();
        if (p.type == PathVertexType::Camera) {
            v.pdfForwardSolidAngle =
                scene.getCamera()->rasterToSolidAnglePdf(ray.getDirection());
            v.pdfForwardArea = cameraAreaPdf(
                v,
                ray.getDirection(),
                (v.pos - p.pos).squaredLength()
            );
        } else {
            v.pdfForwardSolidAngle = pendingPdfW;
            v.pdfForwardArea = solidAngleToAreaPdf(
                v.pdfForwardSolidAngle,
                p.pos,
                v.pos,
                v.normal
            );
        }

        path.push_back(v);
        
        if (v.isLight) {
            break;
        }

        if (depth + 1 >= maxDepth) {
            break;
        }

        float rrProb = 1.0f;
        if (depth >= RR_START_DEPTH) {
            rrProb = rrContinueProbability(beta);
            if (rrProb <= 0.0f || Random::get_float() > rrProb) {
                break;
            }
            beta = beta / rrProb;
        }

        Vector3f bsdfNormal = mat->isGlass() ? geometryNormal : normal;
        BSDFSample sample = sampleBSDF(mat, v.diffuseColor, bsdfNormal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        PathVertex &curr = path.back();
        curr.wi = sample.wi;
        if (path.size() >= 2) {
            PathVertex &prev = path[path.size() - 2];
            prev.pdfReverseArea =
                pdfAreaFromVertexToDirection(curr, sample.wi, prev) * rrProb;
        }
        pendingPdfW = sample.pdf * rrProb;
        
        beta = beta * sample.throughputWeight;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, geometryNormal) > 0.0f
            ? geometryNormal
            : -geometryNormal;

        ray = Ray(pos + offsetNormal * CONNECT_EPS, sample.wi);
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

    PathVertex lightVertex;
    lightVertex.pos = emit.pos;
    lightVertex.normal = emit.normal;
    lightVertex.throughput = sourceThroughput;
    lightVertex.material = emit.material;
    lightVertex.wo = -emit.dir;
    lightVertex.wi = emit.dir;
    lightVertex.pdfForwardArea = emit.pdfPos;
    lightVertex.isDelta = false;
    lightVertex.isLight = true;
    lightVertex.type = PathVertexType::Light;

    path.push_back(lightVertex);

    Vector3f beta = emit.emission * cosEmit / (emit.pdfPos * emit.pdfDir);

    Ray ray(emit.pos + emit.normal * 1e-4f, emit.dir);
    float pendingPdfW = emit.pdfDir;

    for (int depth = 1; depth < maxDepth; ++depth) {
        Hit hit;
        if (!scene.getGroup()->intersect(ray, hit, CONNECT_EPS)) {
            break;
        }

        Material *mat = hit.getMaterial();
        if (mat == nullptr) {
            break;
        }

        Vector3f pos = ray.pointAtParameter(hit.getT());
        Vector3f geometryNormal = hit.getNormal();
        Vector3f normal = geometryNormal;
        if (Vector3f::dot(normal, ray.getDirection()) > 0.0f) {
            normal = -normal;
        }

        PathVertex v;
        v.pos = pos;
        v.normal = normal;
        v.throughput = beta;
        v.diffuseColor = mat->getDiffuseColor(hit);
        v.material = mat;
        v.wo = -ray.getDirection();
        v.isDelta = mat->isGlass() ||
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();
        v.type = PathVertexType::Surface;

        PathVertex &p = path.back();
        v.pdfForwardSolidAngle = pendingPdfW;
        v.pdfForwardArea = solidAngleToAreaPdf(
            v.pdfForwardSolidAngle,
            p.pos,
            v.pos,
            v.normal
        );

        path.push_back(v);

        if (v.isLight) {
            break;
        }

        if (depth + 1 >= maxDepth) {
            break;
        }

        float rrProb = 1.0f;
        if (depth >= RR_START_DEPTH) {
            rrProb = rrContinueProbability(beta);
            if (rrProb <= 0.0f || Random::get_float() > rrProb) {
                break;
            }
            beta = beta / rrProb;
        }

        Vector3f bsdfNormal = mat->isGlass() ? geometryNormal : normal;
        BSDFSample sample = sampleBSDF(mat, v.diffuseColor, bsdfNormal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        PathVertex &curr = path.back();
        curr.wi = sample.wi;
        if (path.size() >= 2) {
            PathVertex &prev = path[path.size() - 2];
            prev.pdfReverseArea =
                pdfAreaFromVertexToDirection(curr, sample.wi, prev) * rrProb;
        }
        pendingPdfW = sample.pdf * rrProb;

        beta = beta * sample.throughputWeight;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, geometryNormal) > 0.0f
            ? geometryNormal
            : -geometryNormal;
        ray = Ray(pos + offsetNormal * CONNECT_EPS, sample.wi);
    }

    return path.size();
}

bool BDPT::makeConnectionGeometry(const PathVertex &eye,
                                  const PathVertex &light,
                                  ConnectionGeometry &connection) const {
    Vector3f edge = light.pos - eye.pos;
    connection.dist2 = edge.squaredLength();
    if (connection.dist2 < MIN_CONNECT_DIST * MIN_CONNECT_DIST) {
        return false;
    }

    connection.dist = std::sqrt(connection.dist2);
    connection.eyeToLight = edge / connection.dist;
    connection.lightToEye = -connection.eyeToLight;
    connection.cosThetaEye = std::max(
        0.0f,
        Vector3f::dot(eye.normal, connection.eyeToLight)
    );
    connection.cosThetaLight = std::max(
        0.0f,
        Vector3f::dot(light.normal, connection.lightToEye)
    );
    return connection.cosThetaEye > 0.0f && connection.cosThetaLight > 0.0f;
}

Vector3f BDPT::connectVertices(const PathVertex &eye,
                               const PathVertex &light,
                               const ConnectionGeometry &connection) const {
    if (eye.material == nullptr || (!light.isLight && light.material == nullptr) ||
        eye.isDelta || light.isDelta) {
        return Vector3f::ZERO;
    }

    Vector3f shadowOrigin = offsetRayOrigin(eye.pos, eye.normal, connection.eyeToLight);
    Vector3f shadowEdge = light.pos - shadowOrigin;
    float shadowDist = shadowEdge.length();
    if (shadowDist <= CONNECT_EPS) {
        return Vector3f::ZERO;
    }
    Ray shadowRay(shadowOrigin, shadowEdge / shadowDist);
    if (scene.getGroup()->occluded(shadowRay, CONNECT_EPS, shadowDist - CONNECT_EPS)) {
        return Vector3f::ZERO;
    }

    Vector3f fEye = evaluateBSDF(
        eye.material,
        eye.diffuseColor,
        eye.normal,
        eye.wo,
        connection.eyeToLight
    );
    if (fEye.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    Vector3f fLight = light.isLight
        ? Vector3f(1, 1, 1)
        : evaluateBSDF(
            light.material,
            light.diffuseColor,
            light.normal,
            light.wo,
            connection.lightToEye
        );
    if (fLight.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    float geometryTerm =
        connection.cosThetaEye * connection.cosThetaLight / connection.dist2;
    geometryTerm = std::min(geometryTerm, MAX_CONNECTION_GEOMETRY_TERM);

    return eye.throughput * fEye * geometryTerm * fLight * light.throughput;
}

float BDPT::cameraAreaPdf(const PathVertex &vertex,
                          const Vector3f &dirFromCamera,
                          float dist2) const {
    if (vertex.type == PathVertexType::Camera || dist2 <= 1e-12f) {
        return 0.0f;
    }

    float cosSurface = std::max(0.0f, Vector3f::dot(vertex.normal, -dirFromCamera));
    float pdfW = scene.getCamera()->rasterToSolidAnglePdf(dirFromCamera);
    if (cosSurface <= 0.0f || pdfW <= 0.0f) {
        return 0.0f;
    }

    return pdfW * cosSurface / dist2;
}

float BDPT::pdfAreaFromVertexToDirection(const PathVertex &from,
                                         const Vector3f &wo,
                                         const PathVertex &to) const {
    Vector3f edge = to.pos - from.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    if (from.type == PathVertexType::Camera) {
        return cameraAreaPdf(to, dir, dist2);
    }
    return pdfAreaFromDirection(from, wo, to, dir, dist2);
}

float BDPT::pdfAreaFromVertexTo(const PathVertex &from,
                                const PathVertex *prev,
                                const PathVertex &to) const {
    Vector3f edge = to.pos - from.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    if (from.type == PathVertexType::Camera) {
        return cameraAreaPdf(to, dir, dist2);
    }

    Vector3f wo = from.wo;
    if (prev != nullptr) {
        Vector3f prevEdge = prev->pos - from.pos;
        if (prevEdge.squaredLength() > 1e-12f) {
            wo = prevEdge.normalized();
        }
    }

    return pdfAreaFromDirection(from, wo, to, dir, dist2);
}

float BDPT::bdptMisWeight(const std::vector<PathVertex> &lightVertices,
                          const std::vector<PathVertex> &cameraVertices,
                          int s,
                          int t,
                          float cameraPdfArea,
                          bool includeLightTracingMis) const {
    if (s + t == 2) {
        return 1.0f;
    }
    if (s < 0 || t <= 0 ||
        s > (int)lightVertices.size() ||
        t > (int)cameraVertices.size()) {
        return 0.0f;
    }

    std::vector<PathVertex> light(lightVertices.begin(), lightVertices.begin() + s);
    std::vector<PathVertex> camera(cameraVertices.begin(), cameraVertices.begin() + t);

    PathVertex *qs = s > 0 ? &light[s - 1] : nullptr;
    PathVertex *pt = t > 0 ? &camera[t - 1] : nullptr;
    PathVertex *qsMinus = s > 1 ? &light[s - 2] : nullptr;
    PathVertex *ptMinus = t > 1 ? &camera[t - 2] : nullptr;

    if (pt != nullptr) {
        pt->isDelta = false;
    }
    if (qs != nullptr) {
        qs->isDelta = false;
    }

    if (s == 0) {
        if (pt == nullptr || !pt->isLight || ptMinus == nullptr) {
            return 1.0f;
        }

        pt->pdfReverseArea = scene.lightAreaPdf();
        ptMinus->pdfReverseArea = pdfAreaFromVertexTo(*pt, nullptr, *ptMinus);
    } else if (t == 1) {
        if (qs == nullptr || cameraPdfArea <= 0.0f) {
            return 0.0f;
        }

        qs->pdfReverseArea = cameraPdfArea;
        if (qsMinus != nullptr && pt != nullptr) {
            qsMinus->pdfReverseArea = pdfAreaFromVertexTo(*qs, pt, *qsMinus);
        }
    } else {
        if (qs == nullptr || pt == nullptr) {
            return 0.0f;
        }

        pt->pdfReverseArea = pdfAreaFromVertexTo(*qs, qsMinus, *pt);
        if (ptMinus != nullptr) {
            ptMinus->pdfReverseArea = pdfAreaFromVertexTo(*pt, qs, *ptMinus);
        }

        qs->pdfReverseArea = pdfAreaFromVertexTo(*pt, ptMinus, *qs);
        if (qsMinus != nullptr) {
            qsMinus->pdfReverseArea = pdfAreaFromVertexTo(*qs, pt, *qsMinus);
        }
    }

    float sumRi = 0.0f;
    float ri = 1.0f;
    for (int i = t - 1; i > 0; --i) {
        ri *= remap0(camera[i].pdfReverseArea) / remap0(camera[i].pdfForwardArea);
        if (!std::isfinite(ri)) {
            return 0.0f;
        }
        bool isLightTracingStrategy = i == 1;
        if ((includeLightTracingMis || !isLightTracingStrategy) &&
            !camera[i].isDelta && !camera[i - 1].isDelta) {
            sumRi += ri;
        }
    }

    ri = 1.0f;
    for (int i = s - 1; i >= 0; --i) {
        ri *= remap0(light[i].pdfReverseArea) / remap0(light[i].pdfForwardArea);
        if (!std::isfinite(ri)) {
            return 0.0f;
        }

        bool previousDelta = i > 0 && light[i - 1].isDelta;
        if (!light[i].isDelta && !previousDelta) {
            sumRi += ri;
        }
    }

    if (!std::isfinite(sumRi) || sumRi < 0.0f) {
        return 0.0f;
    }
    return 1.0f / (1.0f + sumRi);
}

Vector3f BDPT::connectLightToCamera(int s,
                                    std::vector<FilmSplat> *splats,
                                    float splatScale) {
    if (splats == nullptr || s <= 1 || s > (int)lightPath.size() ||
        splatScale <= 0.0f) {
        return Vector3f::ZERO;
    }

    const PathVertex &light = lightPath[s - 1];
    if (light.material == nullptr || light.isDelta) {
        return Vector3f::ZERO;
    }

    Vector2f raster;
    Vector3f dirFromCamera;
    float dist = 0.0f;
    if (!scene.getCamera()->projectPoint(light.pos, raster, dirFromCamera, dist)) {
        return Vector3f::ZERO;
    }

    float dist2 = dist * dist;
    float pdfCameraArea = cameraAreaPdf(light, dirFromCamera, dist2);
    if (pdfCameraArea <= 0.0f) {
        return Vector3f::ZERO;
    }

    Ray visibilityRay(scene.getCamera()->getCenter(), dirFromCamera);
    if (scene.getGroup()->occluded(visibilityRay, CONNECT_EPS, dist - CONNECT_EPS)) {
        return Vector3f::ZERO;
    }

    Vector3f vertexToCamera = -dirFromCamera;
    Vector3f contribution;
    if (s == 1) {
        contribution = light.throughput * pdfCameraArea;
    } else {
        Vector3f fLight = evaluateBSDF(
            light.material,
            light.diffuseColor,
            light.normal,
            light.wo,
            vertexToCamera
        );
        if (fLight.squaredLength() <= 0.0f) {
            return Vector3f::ZERO;
        }
        contribution = light.throughput * fLight * pdfCameraArea;
    }

    std::vector<PathVertex> lightVertices(lightPath.begin(), lightPath.begin() + s);
    std::vector<PathVertex> cameraVertices(1, cameraPath[0]);
    float w = bdptMisWeight(
        lightVertices,
        cameraVertices,
        s,
        1,
        pdfCameraArea,
        true
    );
    contribution = contribution * (w * splatScale);
    if (contribution.squaredLength() <= 0.0f || !isFiniteColor(contribution)) {
        return Vector3f::ZERO;
    }

    FilmSplat splat;
    splat.x = static_cast<int>(std::floor(raster.x()));
    splat.y = static_cast<int>(std::floor(raster.y()));
    splat.contribution = contribution;
    splats->push_back(splat);

    return Vector3f::ZERO;
}

Vector3f BDPT::connectBDPT(int s,
                           int t,
                           std::vector<FilmSplat> *splats,
                           float splatScale) {
    if ((s == 0 && t == 1) || (s == 1 && t == 1)) {
        return Vector3f::ZERO;
    }
    if (t < 1 || t > (int)cameraPath.size() ||
        s < 0 || s > (int)lightPath.size()) {
        return Vector3f::ZERO;
    }

    if (t == 1) {
        return connectLightToCamera(s, splats, splatScale);
    }

    int ci = t - 1;
    const PathVertex &eye = cameraPath[ci];
    bool includeLightTracingMis = splats != nullptr && splatScale > 0.0f;
    if (eye.isLight) {
        return s == 0
            ? estimateCameraHitLight(ci, includeLightTracingMis)
            : Vector3f::ZERO;
    }

    if (s == 0) {
        return estimateCameraHitLight(ci, includeLightTracingMis);
    }

    if (s == 1) {
        int surfaceDepth = t - 2;
        int directSamples = surfaceDepth == 0
            ? primaryDirectLightSamples
            : secondaryDirectLightSamples;
        return estimateDirectLight(eye, ci, directSamples, includeLightTracingMis);
    }

    int li = s - 1;
    const PathVertex &light = lightPath[li];

    ConnectionGeometry connection;
    if (!makeConnectionGeometry(eye, light, connection)) {
        return Vector3f::ZERO;
    }

    Vector3f c = connectVertices(eye, light, connection);
    if (c.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    std::vector<PathVertex> lightVertices(lightPath.begin(), lightPath.begin() + s);
    std::vector<PathVertex> cameraVertices(cameraPath.begin(), cameraPath.begin() + t);
    float w = bdptMisWeight(
        lightVertices,
        cameraVertices,
        s,
        t,
        0.0f,
        includeLightTracingMis
    );
#if BDPT_DEBUG_MIS
    if (!std::isfinite(w) || w <= 0.0f || w > 1.0f) {
        std::cout << "bad mis weight: " << w << "\n";
    }
#endif

    return c * w;
}

Vector3f BDPT::estimateDirectLight(const PathVertex &eye,
                                   int cameraIndex,
                                   int numSamples,
                                   bool includeLightTracingMis) const {
    if (numSamples <= 0 || eye.material == nullptr || eye.isDelta || eye.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f result = Vector3f::ZERO;
    for (int i = 0; i < numSamples; ++i) {
        Light::SampleResult sample = scene.sampleLight(eye.pos);
        if (sample.pdf <= 0.0f || sample.dist <= 0.0f) {
            continue;
        }

        PathVertex lightVertex;
        lightVertex.pos = sample.pos;
        lightVertex.normal = sample.normal;
        lightVertex.throughput = sample.col / sample.pdf;
        lightVertex.wo = -sample.dir;
        lightVertex.pdfForwardArea = sample.pdf;
        lightVertex.isDelta = false;
        lightVertex.isLight = true;
        lightVertex.type = PathVertexType::Light;

        ConnectionGeometry connection;
        if (!makeConnectionGeometry(eye, lightVertex, connection)) {
            continue;
        }

        Vector3f c = connectVertices(eye, lightVertex, connection);
        if (c.squaredLength() <= 0.0f) {
            continue;
        }

        std::vector<PathVertex> lightVertices(1, lightVertex);
        std::vector<PathVertex> cameraVertices(
            cameraPath.begin(),
            cameraPath.begin() + cameraIndex + 1
        );
        float wLight = bdptMisWeight(
            lightVertices,
            cameraVertices,
            1,
            cameraIndex + 1,
            0.0f,
            includeLightTracingMis
        );

        result += c * wLight;
    }

    return result / static_cast<float>(numSamples);
}

Vector3f BDPT::estimateCameraHitLight(int ci, bool includeLightTracingMis) const {
    const PathVertex &light = cameraPath[ci];
    if (ci <= 0 || light.material == nullptr || !light.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f contribution = light.throughput * light.material->getEmission();
    if (ci == 1) {
        return contribution;
    }

    bool hasDeltaAncestor = false;
    for (int i = 1; i < ci; ++i) {
        if (cameraPath[i].isDelta) {
            hasDeltaAncestor = true;
            break;
        }
    }

    std::vector<PathVertex> lightVertices;
    std::vector<PathVertex> cameraVertices(
        cameraPath.begin(),
        cameraPath.begin() + ci + 1
    );

    float wBsdf = bdptMisWeight(
        lightVertices,
        cameraVertices,
        0,
        ci + 1,
        0.0f,
        includeLightTracingMis
    );
    Vector3f result = contribution * wBsdf;

    if (hasDeltaAncestor) {
        result = clampLuminance(result, DELTA_HIT_LIGHT_MAX_LUMINANCE);
    }

    return result;
}

Vector3f BDPT::trace(const Ray &cameraRay) {
    return trace(cameraRay, nullptr, 0.0f);
}

Vector3f BDPT::trace(const Ray &cameraRay,
                     std::vector<FilmSplat> *splats,
                     float splatScale) {
    cameraPath.clear();
    lightPath.clear();

    generateCameraPath(cameraRay, cameraPath, MAX_CAMERA_PATH_DEPTH);
    generateLightPath(lightPath, MAX_LIGHT_PATH_DEPTH);

    Vector3f L = Vector3f::ZERO;

    for (int t = 1; t <= (int)cameraPath.size(); ++t) {
        for (int s = 0; s <= (int)lightPath.size(); ++s) {
            L += connectBDPT(s, t, splats, splatScale);
        }
    }

    return L;
}
