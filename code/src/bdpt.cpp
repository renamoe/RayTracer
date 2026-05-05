#include "bdpt.hpp"
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

float rrContinueProbability(const Vector3f &beta) {
    float maxComp = std::max(beta.x(), std::max(beta.y(), beta.z()));
    if (!std::isfinite(maxComp) || maxComp <= 0.0f) {
        return 0.0f;
    }
    return std::min(RR_MAX_CONTINUE, std::max(RR_MIN_CONTINUE, maxComp));
}

float rrContinueProbabilityAtDepth(const Vector3f &beta, int depth) {
    return depth >= RR_START_DEPTH ? rrContinueProbability(beta) : 1.0f;
}

Vector3f offsetRayOrigin(const Vector3f &pos,
                         const Vector3f &normal,
                         const Vector3f &dir) {
    return pos + (Vector3f::dot(dir, normal) > 0.0f ? normal : -normal) * CONNECT_EPS;
}

bool isDeltaMaterial(Material *mat) {
    return mat->isGlass() ||
        (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
}

float pdfAreaFromDirection(const PathVertex &from,
                           const Vector3f &wo,
                           const PathVertex &to,
                           const Vector3f &dir,
                           float dist2) {
    if (from.material == nullptr || from.isDelta) {
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

float pdfAreaFromTo(const PathVertex &from,
                    const Vector3f &wo,
                    const PathVertex &to) {
    Vector3f edge = to.pos - from.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    return pdfAreaFromDirection(from, wo, to, dir, dist2);
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
    path.push_back(cameraVertex);
    
    Ray ray = cameraRay;
    Vector3f beta(1, 1, 1);
    bool hasPrev = false;
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
        v.material = mat;
        v.wo = -ray.getDirection();
        v.isDelta = mat->isGlass() || 
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();
        v.type = PathVertexType::Surface;

        if (hasPrev) {
            PathVertex &p = path.back();
            v.pdfForwardSolidAngle = pendingPdfW;
            v.pdfForwardArea = solidAngleToAreaPdf(
                v.pdfForwardSolidAngle,
                p.pos,
                v.pos,
                v.normal
            );
        } else {
            v.pdfForwardArea = 1.0f;
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
        BSDFSample sample = sampleBSDF(mat, bsdfNormal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        PathVertex &curr = path.back();
        curr.wi = sample.wi;
        pendingPdfW = sample.pdf * rrProb;
        
        beta = beta * sample.throughputWeight;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, geometryNormal) > 0.0f
            ? geometryNormal
            : -geometryNormal;

        ray = Ray(pos + offsetNormal * CONNECT_EPS, sample.wi);
        hasPrev = true;
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
        BSDFSample sample = sampleBSDF(mat, bsdfNormal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        PathVertex &curr = path.back();
        curr.wi = sample.wi;
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
                               const ConnectionGeometry &connection) {
    if (eye.material == nullptr || light.material == nullptr ||
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

    Vector3f fEye = evaluateBSDF(eye.material, eye.normal, eye.wo, connection.eyeToLight);
    if (fEye.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    Vector3f fLight = light.isLight
        ? Vector3f(1, 1, 1)
        : evaluateBSDF(light.material, light.normal, light.wo, connection.lightToEye);
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

float BDPT::bdptMisWeightLightToCamera(int s,
                                       const Vector3f &vertexToCamera,
                                       float cameraPdfArea) const {
    constexpr float EPS = 1e-8f;

    int li = s - 1;
    if (li < 0 || li >= (int)lightPath.size()) {
        return 0.0f;
    }

    const PathVertex &light = lightPath[li];
    float sum = 1.0f;
    float ratio = cameraPdfArea / std::max(light.pdfForwardArea, EPS);
    sum += ratio * ratio;

    Vector3f incoming = vertexToCamera;
    for (int k = li - 1; k >= 1; --k) {
        const PathVertex &from = lightPath[k + 1];
        const PathVertex &to = lightPath[k];

        float pdfRev = pdfAreaFromTo(from, incoming, to);
        float pdfFwd = std::max(to.pdfForwardArea, EPS);

        ratio *= pdfRev / pdfFwd;
        sum += ratio * ratio;

        incoming = (from.pos - to.pos).normalized();
    }

    return 1.0f / sum;
}

Vector3f BDPT::connectLightToCamera(int s,
                                    std::vector<FilmSplat> *splats,
                                    float splatScale) {
    if (splats == nullptr || s <= 0 || s > (int)lightPath.size() ||
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
            light.normal,
            light.wo,
            vertexToCamera
        );
        if (fLight.squaredLength() <= 0.0f) {
            return Vector3f::ZERO;
        }
        contribution = light.throughput * fLight * pdfCameraArea;
    }

    float w = bdptMisWeightLightToCamera(s, vertexToCamera, pdfCameraArea);
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
    if (s == 0 && t == 1) {
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
        return estimateDirectLight(eye, surfaceDepth, directSamples);
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

    float w = bdptMisWeight(s, t, connection);
#if BDPT_DEBUG_MIS
    if (!std::isfinite(w) || w <= 0.0f || w > 1.0f) {
        std::cout << "bad mis weight: " << w << "\n";
    }
#endif

    return c * w;
}

Vector3f BDPT::estimateDirectLight(const PathVertex &eye,
                                   int ci,
                                   int numSamples) const {
    if (numSamples <= 0 || eye.material == nullptr || eye.isDelta || eye.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f result = Vector3f::ZERO;
    for (int i = 0; i < numSamples; ++i) {
        Light::SampleResult sample = scene.sampleLight(eye.pos);
        if (sample.pdf <= 0.0f || sample.dist <= 0.0f) {
            continue;
        }

        float cosEye = std::max(0.0f, Vector3f::dot(eye.normal, sample.dir));
        float cosLight = std::max(0.0f, Vector3f::dot(sample.normal, -sample.dir));
        if (cosEye <= 0.0f || cosLight <= 0.0f) {
            continue;
        }

        Vector3f shadowOrigin = offsetRayOrigin(eye.pos, eye.normal, sample.dir);
        Vector3f shadowEdge = sample.pos - shadowOrigin;
        float shadowDist = shadowEdge.length();
        if (shadowDist <= CONNECT_EPS) {
            continue;
        }
        Ray shadowRay(shadowOrigin, shadowEdge / shadowDist);
        if (scene.getGroup()->occluded(shadowRay, CONNECT_EPS, shadowDist - CONNECT_EPS)) {
            continue;
        }

        Vector3f fEye = evaluateBSDF(eye.material, eye.normal, eye.wo, sample.dir);
        if (fEye.squaredLength() <= 0.0f) {
            continue;
        }

        float pdfLightW = areaPdfToSolidAnglePdf(sample.pdf, sample.dist, cosLight);
        if (pdfLightW <= 0.0f) {
            continue;
        }

        float pdfBsdfW = bsdfPdf(eye.material, eye.normal, eye.wo, sample.dir) *
            rrContinueProbabilityAtDepth(eye.throughput, ci);
        float wLight = powerHeuristic(pdfLightW, pdfBsdfW);

        result += eye.throughput * sample.col * fEye * cosEye / pdfLightW * wLight;
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
        if (!includeLightTracingMis) {
            return contribution;
        }

        Vector2f raster;
        Vector3f dirFromCamera;
        float dist = 0.0f;
        if (!scene.getCamera()->projectPoint(light.pos, raster, dirFromCamera, dist)) {
            return contribution;
        }

        float pdfCameraArea = cameraAreaPdf(light, dirFromCamera, dist * dist);
        float pdfLightArea = scene.lightAreaPdf();
        if (pdfCameraArea <= 0.0f || pdfLightArea <= 0.0f) {
            return contribution;
        }

        return contribution * powerHeuristic(pdfCameraArea, pdfLightArea);
    }

    const PathVertex &prev = cameraPath[ci - 1];
    if (prev.isDelta) {
        return contribution;
    }

    Vector3f edge = light.pos - prev.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return Vector3f::ZERO;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    float pdfBsdfW = light.pdfForwardSolidAngle;
    if (pdfBsdfW <= 0.0f) {
        pdfBsdfW = bsdfPdf(prev.material, prev.normal, prev.wo, dir) *
            rrContinueProbabilityAtDepth(prev.throughput, ci - 2);
    }

    float pdfLightW = scene.lightPdf(prev.pos, dir);
    float wBsdf = powerHeuristic(pdfBsdfW, pdfLightW);
    return contribution * wBsdf;
}

float BDPT::bdptMisWeight(int s,
                          int t,
                          const ConnectionGeometry &connection) const {
    constexpr float EPS = 1e-8f;

    int li = s - 1;
    int ci = t - 1;
    const PathVertex &eye = cameraPath[ci];
    const PathVertex &light = lightPath[li];

    float sum = 1.0f;

    // Move light vertices to camera side.
    float ratio = 1.0f;
    if (li >= 0) {
        float pdfEyeToLight = pdfAreaFromDirection(
            eye,
            eye.wo,
            light,
            connection.eyeToLight,
            connection.dist2
        );
        float denom = std::max(light.pdfForwardArea, EPS);

        ratio *= pdfEyeToLight / denom;
        sum += ratio * ratio;

        Vector3f incoming = connection.lightToEye;

        for (int k = li - 1; k >= 1; --k) {
            const PathVertex &from = lightPath[k + 1];
            const PathVertex &to = lightPath[k];

            float pdfRev = pdfAreaFromTo(from, incoming, to);
            float pdfFwd = std::max(to.pdfForwardArea, EPS);

            ratio *= pdfRev / pdfFwd;
            sum += ratio * ratio;

            incoming = (from.pos - to.pos).normalized();
        }
    }

    // Move camera vertices to light side.
    ratio = 1.0f;
    if (ci > 0) {
        float pdfLightToEye = pdfAreaFromDirection(
            light,
            light.wo,
            eye,
            connection.lightToEye,
            connection.dist2
        );
        float denom = std::max(eye.pdfForwardArea, EPS);

        ratio *= pdfLightToEye / denom;
        sum += ratio * ratio;

        Vector3f incoming = connection.eyeToLight;

        for (int k = ci - 1; k >= 1; --k) {
            const PathVertex &from = cameraPath[k + 1];
            const PathVertex &to = cameraPath[k];

            float pdfRev = pdfAreaFromTo(from, incoming, to);
            float pdfFwd = std::max(to.pdfForwardArea, EPS);

            ratio *= pdfRev / pdfFwd;
            sum += ratio * ratio;

            incoming = (from.pos - to.pos).normalized();
        }
    }

    return 1.0f / sum;
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
