#include "vcm.hpp"
#include "bsdf.hpp"
#include "group.hpp"
#include "random.hpp"

#include <algorithm>
#include <cmath>

#ifndef VCM_DEBUG_MIS
#define VCM_DEBUG_MIS 0
#endif

#if VCM_DEBUG_MIS
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

float pdfAreaFromDirection(const VCMPathVertex &from,
                           const Vector3f &wo,
                           const VCMPathVertex &to,
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

float pdfAreaFromTo(const VCMPathVertex &from,
                    const Vector3f &wo,
                    const VCMPathVertex &to) {
    Vector3f edge = to.pos - from.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    return pdfAreaFromDirection(from, wo, to, dir, dist2);
}

} // namespace

VCM::VCM(SceneParser &scene,
         int primaryDirectLightSamples,
         int secondaryDirectLightSamples)
    : scene(scene),
      primaryDirectLightSamples(std::max(0, primaryDirectLightSamples)),
      secondaryDirectLightSamples(std::max(0, secondaryDirectLightSamples)) {}

void VCM::beginIteration(int iteration, int width, int height) {
    (void)iteration;
    (void)width;
    (void)height;
}

int VCM::generateCameraPath(const Ray &cameraRay,
                            std::vector<VCMPathVertex> &path,
                            int maxDepth) {
    path.clear();

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

        VCMPathVertex v;
        v.pos = pos;
        v.normal = normal;
        v.throughput = beta;
        v.material = mat;
        v.wo = -ray.getDirection();
        v.isDelta = mat->isGlass() ||
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();

        if (hasPrev) {
            VCMPathVertex &p = path.back();
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

        VCMPathVertex &curr = path.back();
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

int VCM::generateLightPath(std::vector<VCMPathVertex> &path, int maxDepth) {
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

    VCMPathVertex lightVertex;
    lightVertex.pos = emit.pos;
    lightVertex.normal = emit.normal;
    lightVertex.throughput = sourceThroughput;
    lightVertex.material = emit.material;
    lightVertex.wo = -emit.dir;
    lightVertex.wi = emit.dir;
    lightVertex.pdfForwardArea = emit.pdfPos;
    lightVertex.isDelta = false;
    lightVertex.isLight = true;

    path.push_back(lightVertex);

    Vector3f beta = emit.emission * cosEmit / (emit.pdfPos * emit.pdfDir);

    Ray ray(emit.pos + emit.normal * CONNECT_EPS, emit.dir);
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

        VCMPathVertex v;
        v.pos = pos;
        v.normal = normal;
        v.throughput = beta;
        v.material = mat;
        v.wo = -ray.getDirection();
        v.isDelta = mat->isGlass() ||
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();

        VCMPathVertex &p = path.back();
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

        VCMPathVertex &curr = path.back();
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

bool VCM::makeConnectionGeometry(const VCMPathVertex &eye,
                                 const VCMPathVertex &light,
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

Vector3f VCM::connectVertices(const VCMPathVertex &eye,
                              const VCMPathVertex &light,
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

Vector3f VCM::estimateDirectLight(const VCMPathVertex &eye,
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

Vector3f VCM::estimateCameraHitLight(int ci) const {
    const VCMPathVertex &light = cameraPath[ci];
    if (light.material == nullptr || !light.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f contribution = light.throughput * light.material->getEmission();
    if (ci == 0) {
        return contribution;
    }

    const VCMPathVertex &prev = cameraPath[ci - 1];
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
            rrContinueProbabilityAtDepth(prev.throughput, ci - 1);
    }

    float pdfLightW = scene.lightPdf(prev.pos, dir);
    float wBsdf = powerHeuristic(pdfBsdfW, pdfLightW);
    return contribution * wBsdf;
}

float VCM::vcmMisWeight(int ci,
                        int li,
                        const ConnectionGeometry &connection) const {
    constexpr float EPS = 1e-8f;

    const VCMPathVertex &eye = cameraPath[ci];
    const VCMPathVertex &light = lightPath[li];

    float sum = 1.0f;

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
            const VCMPathVertex &from = lightPath[k + 1];
            const VCMPathVertex &to = lightPath[k];

            float pdfRev = pdfAreaFromTo(from, incoming, to);
            float pdfFwd = std::max(to.pdfForwardArea, EPS);

            ratio *= pdfRev / pdfFwd;
            sum += ratio * ratio;

            incoming = (from.pos - to.pos).normalized();
        }
    }

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
            const VCMPathVertex &from = cameraPath[k + 1];
            const VCMPathVertex &to = cameraPath[k];

            float pdfRev = pdfAreaFromTo(from, incoming, to);
            float pdfFwd = std::max(to.pdfForwardArea, EPS);

            ratio *= pdfRev / pdfFwd;
            sum += ratio * ratio;

            incoming = (from.pos - to.pos).normalized();
        }
    }

    return 1.0f / sum;
}

Vector3f VCM::trace(const Ray &cameraRay) {
    cameraPath.clear();
    lightPath.clear();

    generateCameraPath(cameraRay, cameraPath, MAX_VCM_CAMERA_PATH_DEPTH);
    generateLightPath(lightPath, MAX_VCM_LIGHT_PATH_DEPTH);

    Vector3f L = Vector3f::ZERO;

    for (int ci = 0; ci < (int)cameraPath.size(); ++ci) {
        const auto &eye = cameraPath[ci];

        if (eye.isLight) {
            L += estimateCameraHitLight(ci);
            continue;
        }

        int directSamples = ci == 0
            ? primaryDirectLightSamples
            : secondaryDirectLightSamples;
        L += estimateDirectLight(eye, ci, directSamples);

        for (int li = 1; li < (int)lightPath.size(); ++li) {
            const auto &light = lightPath[li];

            ConnectionGeometry connection;
            if (!makeConnectionGeometry(eye, light, connection)) {
                continue;
            }

            Vector3f c = connectVertices(eye, light, connection);
            if (c.squaredLength() <= 0.0f) {
                continue;
            }

            float w = vcmMisWeight(ci, li, connection);
#if VCM_DEBUG_MIS
            if (!std::isfinite(w) || w <= 0.0f || w > 1.0f) {
                std::cout << "bad mis weight: " << w << "\n";
            }
#endif

            L += c * w;
        }
    }

    return L;
}
