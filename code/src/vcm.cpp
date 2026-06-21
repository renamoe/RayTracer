#include "Vector3f.h"
#include "vcm.hpp"
#include "bsdf.hpp"
#include "camera.hpp"
#include "group.hpp"
#include "random.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace {

constexpr float CONNECT_EPS = 1e-4f;
constexpr float MIN_CONNECT_DIST = 1e-3f;
constexpr float MAX_CONNECTION_GEOMETRY_TERM = 1e4f;
constexpr int RR_START_DEPTH = 2;
constexpr float RR_MIN_CONTINUE = 0.05f;
constexpr float RR_MAX_CONTINUE = 0.95f;
constexpr float ALPHA = 0.7f;

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

float pdfAreaFromDirection(const VCMPathVertex &from,
                           const Vector3f &wo,
                           const VCMPathVertex &to,
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

VCM::VCM(SceneParser &scene,
         int primaryDirectLightSamples,
         int secondaryDirectLightSamples,
         float baseRadius)
    : scene(scene),
      primaryDirectLightSamples(std::max(0, primaryDirectLightSamples)),
      secondaryDirectLightSamples(std::max(0, secondaryDirectLightSamples)),
      baseRadius(baseRadius) {
}

void VCM::beginIteration(int iteration, int width, int height) {
    lightPathCount = width * height;

    // pmRadius = baseRadius / pow(iteration + 1, 0.5f * (1.0f - ALPHA));
    pmRadius = baseRadius;

    radius2 = pmRadius * pmRadius;
    etaVCM = M_PI * radius2 * lightPathCount;
    misVmWeightFactor = etaVCM;
    misVcWeightFactor = 1.0f / etaVCM;
    vmNormalization = 1.0f / etaVCM;

    lightPhotons.clear();
    lightPhotons.reserve(lightPathCount * 3);

    pathHeads.resize(lightPathCount + 1);
    for (int i = 0; i < lightPathCount; ++i) {
        generateLightPath(i, MAX_VCM_LIGHT_PATH_DEPTH);
    }
    pathHeads[lightPathCount] = lightPhotons.size();

    photonHashGrid.build(pmRadius, lightPhotons);
}

void VCM::generateLightPath(size_t pathIdx, int maxDepth) {
    pathHeads[pathIdx] = lightPhotons.size();

    LightEmitSample emit = scene.sampleEmitLight();
    if (emit.pdfPos <= 0.0f || emit.pdfDir <= 0.0f ||
        emit.material == nullptr || emit.emission.squaredLength() <= 0.0f) {
        return;
    }

    float cosEmit = Vector3f::dot(emit.normal, emit.dir);
    if (cosEmit <= 0.0f) {
        return;
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
    lightVertex.type = VCMPathVertexType::Light;

    lightPhotons.push_back(lightVertex);

    float directPdfA = emit.pdfPos;
    float emissionPdfW = emit.pdfDir;
    float dVCM = directPdfA / emissionPdfW;
    float dVC = cosEmit / emissionPdfW;
    float dVM = dVC * misVcWeightFactor;

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
        v.wo = -ray.getDirection();
        v.diffuseColor = mat->getDiffuseColor(hit);
        v.material = mat;
        v.isDelta = mat->isDelta();
        v.isLight = mat->isEmissive();
        v.type = VCMPathVertexType::Surface;

        VCMPathVertex &p = lightPhotons.back();
        v.pdfForwardSolidAngle = pendingPdfW;
        v.pdfForwardArea = solidAngleToAreaPdf(
            v.pdfForwardSolidAngle,
            p.pos,
            v.pos,
            v.normal
        );

        float dist2 = (v.pos - p.pos).squaredLength();
        float cosIn = std::max(1e-6f, std::abs(Vector3f::dot(v.normal, v.wo)));

        dVCM *= dist2 / cosIn;
        dVC /= cosIn;
        dVM /= cosIn;

        v.dVCM = dVCM;
        v.dVC = dVC;
        v.dVM = dVM;

        lightPhotons.push_back(v);

        if (mat->isEmissive()) {
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
        BSDFSample sample = sampleBSDF(mat, mat->getDiffuseColor(hit), bsdfNormal, -ray.getDirection());
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        float pdfFwd = sample.pdf * rrProb;
        float pdfRev = bsdfPdf(mat, bsdfNormal, sample.wi, v.wo) * rrProb;
        float cosOut = std::max(0.0f, std::abs(Vector3f::dot(bsdfNormal, sample.wi)));

        if (sample.isDelta) {
            dVCM = 0.0f;
            dVC *= cosOut;
            dVM *= cosOut;
        } else {
            dVC = (cosOut / pdfFwd) * (dVC * pdfRev + dVCM + misVmWeightFactor);
            dVM = (cosOut / pdfFwd) * (dVM * pdfRev + dVCM * misVcWeightFactor + 1.0f);
            dVCM = 1.0f / pdfFwd;
        }

        VCMPathVertex &curr = lightPhotons.back();
        curr.wi = sample.wi;
        if (lightPhotons.size() >= 2) {
            VCMPathVertex &prev = lightPhotons[lightPhotons.size() - 2];
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
}

Vector3f VCM::gatherPhotons(const VCMPathVertex &v,
                            const Vector3f &cameraThroughput,
                            bool useMis) const {
    if (v.material == nullptr || v.isDelta || v.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f sum = Vector3f::ZERO;

    photonHashGrid.query(v.pos, [&](size_t idx, const VCMPathVertex &photon) {
        Vector3f wi = photon.wo;

        if (Vector3f::dot(v.normal, wi) <= 0.0f) {
            return;
        }

        Vector3f f = evaluateBSDF(v.material, v.diffuseColor, v.normal, v.wo, wi);
        if (f.squaredLength() <= 0.0f) {
            return;
        }

        if (useMis) {
            float cameraPdfW = bsdfPdf(v.material, v.normal, v.wo, photon.wo);
            float cameraRevPdfW = bsdfPdf(v.material, v.normal, photon.wo, v.wo);

            float wLight =
                photon.dVCM * misVcWeightFactor +
                photon.dVM * cameraPdfW;

            float wCamera =
                v.dVCM * misVcWeightFactor +
                v.dVM * cameraRevPdfW;

            float misWeight = 1.0f / (1.0f + wLight + wCamera);
            sum += photon.throughput * f * misWeight;
        } else {
            sum += photon.throughput * f;
        }
    });

    return cameraThroughput * sum * vmNormalization;
}

Vector3f VCM::generateCameraPath(const Ray &cameraRay,
                                 std::vector<VCMPathVertex> &path,
                                 int maxDepth,
                                 bool includeHitLight,
                                 bool includeMerging,
                                 bool useVmMis) {
    path.clear();

    Camera *camera = scene.getCamera();
    VCMPathVertex cameraVertex;
    cameraVertex.pos = camera->getCenter();
    cameraVertex.normal = camera->getDirection();
    cameraVertex.throughput = Vector3f(1, 1, 1);
    cameraVertex.pdfForwardArea = 1.0f;
    cameraVertex.type = VCMPathVertexType::Camera;
    cameraVertex.wi = cameraRay.getDirection();
    path.push_back(cameraVertex);
    
    Ray ray = cameraRay;

    Vector3f L = Vector3f::ZERO;
    Vector3f beta(1, 1, 1);
    float pendingPdfW = 0.0f;

    float cameraPdfW = camera->rasterToSolidAnglePdf(cameraRay.getDirection());

    float dVCM = lightPathCount / cameraPdfW;
    float dVC = 0.0f;
    float dVM = 0.0f;

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
        v.diffuseColor = mat->getDiffuseColor(hit);
        v.material = mat;
        v.wo = -ray.getDirection();
        v.isDelta = mat->isDelta();
        v.isLight = mat->isEmissive();
        v.type = VCMPathVertexType::Surface;

        VCMPathVertex &p = path.back();
        float dist2 = (v.pos - p.pos).squaredLength();
        float cosIn = std::max(1e-6f, std::abs(Vector3f::dot(v.normal, v.wo)));

        if (p.type == VCMPathVertexType::Camera) {
            v.pdfForwardSolidAngle =
                scene.getCamera()->rasterToSolidAnglePdf(ray.getDirection());
            v.pdfForwardArea = cameraAreaPdf(
                v,
                ray.getDirection(),
                dist2
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
        
        dVCM *= dist2 / cosIn;
        dVC /= cosIn;
        dVM /= cosIn;

        v.dVCM = dVCM;
        v.dVC = dVC;
        v.dVM = dVM;

        path.push_back(v);

        if (mat->isEmissive()) {
            if (!includeHitLight) {
                break;
            }
            L += beta * mat->getEmission();
            break;
        }

        if (includeMerging && !mat->isDelta()) {
            L += gatherPhotons(v, beta, useVmMis);
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

        float pdfFwd = sample.pdf * rrProb;
        float pdfRev = bsdfPdf(mat, bsdfNormal, sample.wi, v.wo) * rrProb;
        float cosOut = std::max(0.0f, std::abs(Vector3f::dot(bsdfNormal, sample.wi)));

        if (sample.isDelta) {
            dVCM = 0.0f;
            dVC *= cosOut;
            dVM *= cosOut;
        } else {
            dVC = (cosOut / pdfFwd) * (dVC * pdfRev + dVCM + misVmWeightFactor);
            dVM = (cosOut / pdfFwd) * (dVM * pdfRev + dVCM * misVcWeightFactor + 1.0f);
            dVCM = 1.0f / pdfFwd;
        }

        VCMPathVertex &curr = path.back();
        curr.wi = sample.wi;
        if (path.size() >= 2) {
            VCMPathVertex &prev = path[path.size() - 2];
            prev.pdfReverseArea =
                pdfAreaFromVertexToDirection(curr, sample.wi, prev) * rrProb;
        }
        pendingPdfW = sample.pdf * rrProb;

        beta = beta * sample.throughputWeight;

        Vector3f offsetNormal =
            Vector3f::dot(sample.wi, geometryNormal) > 0.0f
                ? geometryNormal
                : -geometryNormal;

        ray = Ray(pos + offsetNormal * CONNECT_EPS, sample.wi);
    }

    return isFiniteColor(L) ? L : Vector3f::ZERO;
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

float VCM::cameraAreaPdf(const VCMPathVertex &vertex,
                         const Vector3f &dirFromCamera,
                         float dist2) const {
    if (vertex.type == VCMPathVertexType::Camera || dist2 <= 1e-12f) {
        return 0.0f;
    }

    float cosSurface = std::max(0.0f, Vector3f::dot(vertex.normal, -dirFromCamera));
    float pdfW = scene.getCamera()->rasterToSolidAnglePdf(dirFromCamera);
    if (cosSurface <= 0.0f || pdfW <= 0.0f) {
        return 0.0f;
    }

    return pdfW * cosSurface / dist2;
}

float VCM::pdfAreaFromVertexToDirection(const VCMPathVertex &from,
                                        const Vector3f &wo,
                                        const VCMPathVertex &to) const {
    Vector3f edge = to.pos - from.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    if (from.type == VCMPathVertexType::Camera) {
        return cameraAreaPdf(to, dir, dist2);
    }
    return pdfAreaFromDirection(from, wo, to, dir, dist2);
}

float VCM::pdfAreaFromVertexTo(const VCMPathVertex &from,
                               const VCMPathVertex *prev,
                               const VCMPathVertex &to) const {
    Vector3f edge = to.pos - from.pos;
    float dist2 = edge.squaredLength();
    if (dist2 <= 1e-12f) {
        return 0.0f;
    }

    Vector3f dir = edge / std::sqrt(dist2);
    if (from.type == VCMPathVertexType::Camera) {
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

float VCM::vcmMisWeight(const std::vector<VCMPathVertex> &lightVertices,
                        const std::vector<VCMPathVertex> &cameraVertices,
                        int s,
                        int t,
                        float cameraPdfArea) const {
    if (s + t == 2) {
        return 1.0f;
    }
    if (s < 0 || t <= 0 ||
        s > (int)lightVertices.size() ||
        t > (int)cameraVertices.size()) {
        return 0.0f;
    }

    std::vector<VCMPathVertex> light(lightVertices.begin(), lightVertices.begin() + s);
    std::vector<VCMPathVertex> camera(cameraVertices.begin(), cameraVertices.begin() + t);

    VCMPathVertex *qs = s > 0 ? &light[s - 1] : nullptr;
    VCMPathVertex *pt = t > 0 ? &camera[t - 1] : nullptr;
    VCMPathVertex *qsMinus = s > 1 ? &light[s - 2] : nullptr;
    VCMPathVertex *ptMinus = t > 1 ? &camera[t - 2] : nullptr;

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
        if (!camera[i].isDelta && !camera[i - 1].isDelta) {
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

float VCM::vcMisWeight(const VCMPathVertex &eye,
                       const VCMPathVertex &light,
                       ConnectionGeometry &connection) const {
    float cameraPdfW = bsdfPdf(eye.material, eye.normal, eye.wo, connection.eyeToLight);
    float cameraRevPdfW = bsdfPdf(eye.material, eye.normal, connection.eyeToLight, eye.wo);

    float lightPdfW = bsdfPdf(light.material, light.normal, light.wo, connection.lightToEye);
    float lightRevPdfW = bsdfPdf(light.material, light.normal, connection.lightToEye, light.wo);

    float cameraPdfA = cameraPdfW * connection.cosThetaLight / connection.dist2;
    float lightPdfA = lightPdfW * connection.cosThetaEye / connection.dist2;

    float wLight = cameraPdfA *
        (misVmWeightFactor + light.dVCM + light.dVC * lightRevPdfW);

    float wCamera = lightPdfA *
        (misVmWeightFactor + eye.dVCM + eye.dVC * cameraRevPdfW);

    float misWeight = 1.0f / (1.0f + wLight + wCamera);
    return misWeight;
}

float VCM::cameraHitLightMisWeight(
        int cameraIndex,
        const std::vector<VCMPathVertex> &cameraPath) const {
    if (cameraIndex <= 1 || cameraIndex >= (int)cameraPath.size()) {
        return 1.0f;
    }

    const VCMPathVertex &light = cameraPath[cameraIndex];
    const VCMPathVertex &prev = cameraPath[cameraIndex - 1];
    if (prev.isDelta) {
        return 1.0f;
    }

    float directPdfArea = scene.lightAreaPdf();
    float cosEmit = std::max(0.0f, Vector3f::dot(light.normal, light.wo));
    float emissionPdfW = cosEmit / M_PI;
    float wCamera = directPdfArea * light.dVCM + emissionPdfW * light.dVC;

    if (!std::isfinite(wCamera) || wCamera < 0.0f) {
        return 0.0f;
    }
    return 1.0f / (1.0f + wCamera);
}

float VCM::directLightMisWeight(const VCMPathVertex &eye,
                                const ConnectionGeometry &connection,
                                float directPdfArea) const {
    if (eye.material == nullptr || directPdfArea <= 0.0f ||
        connection.dist2 <= 0.0f || connection.cosThetaLight <= 0.0f) {
        return 0.0f;
    }

    float bsdfPdfW = bsdfPdf(
        eye.material,
        eye.normal,
        eye.wo,
        connection.eyeToLight
    );
    float bsdfRevPdfW = bsdfPdf(
        eye.material,
        eye.normal,
        connection.eyeToLight,
        eye.wo
    );

    float bsdfPdfArea = bsdfPdfW * connection.cosThetaLight / connection.dist2;
    float wLight = bsdfPdfArea / directPdfArea;

    float emissionPdfW = connection.cosThetaLight / M_PI;
    float emissionAreaOverDirectArea =
        emissionPdfW * connection.cosThetaEye /
        (connection.dist2 * directPdfArea);
    float wCamera = emissionAreaOverDirectArea *
        (misVmWeightFactor + eye.dVCM + eye.dVC * bsdfRevPdfW);

    float weightSum = wLight + wCamera;
    if (!std::isfinite(weightSum) || weightSum < 0.0f) {
        return 0.0f;
    }
    return 1.0f / (1.0f + weightSum);
}

float VCM::lightToCameraMisWeight(const VCMPathVertex &light,
                                  float cameraPdfArea,
                                  const Vector3f &vertexToCamera) const {
    if (light.material == nullptr || cameraPdfArea <= 0.0f ||
        lightPathCount <= 0) {
        return 0.0f;
    }

    float bsdfRevPdfW = bsdfPdf(
        light.material,
        light.normal,
        vertexToCamera,
        light.wo
    );
    float wLight = (cameraPdfArea / static_cast<float>(lightPathCount)) *
        (misVmWeightFactor + light.dVCM + light.dVC * bsdfRevPdfW);

    if (!std::isfinite(wLight) || wLight < 0.0f) {
        return 0.0f;
    }
    return 1.0f / (1.0f + wLight);
}

Vector3f VCM::connectLightToCamera(int s,
                                   std::vector<FilmSplat> *splats,
                                   float splatScale,
                                   const std::vector<VCMPathVertex> &lightPath,
                                   const std::vector<VCMPathVertex> &cameraPath,
                                   bool useMis) {
    if (splats == nullptr || s <= 1 || s > (int)lightPath.size() ||
        splatScale <= 0.0f) {
        return Vector3f::ZERO;
    }

    const VCMPathVertex &light = lightPath[s - 1];
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

    float w = useMis
        ? lightToCameraMisWeight(light, pdfCameraArea, vertexToCamera)
        : 1.0f;
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

Vector3f VCM::connectVCM(int s,
                         int t,
                         std::vector<FilmSplat> *splats,
                         float splatScale,
                         const std::vector<VCMPathVertex> &cameraPath,
                         const std::vector<VCMPathVertex> &lightPath,
                         bool useVcMis) {
    if ((s == 0 && t == 1) || (s == 1 && t == 1)) {
        return Vector3f::ZERO;
    }
    if (t < 1 || t > (int)cameraPath.size() ||
        s < 0 || s > (int)lightPath.size()) {
        return Vector3f::ZERO;
    }

    if (t == 1) {
        return connectLightToCamera(s, splats, splatScale, lightPath, cameraPath, useVcMis);
    }

    int ci = t - 1;
    const VCMPathVertex &eye = cameraPath[ci];
    bool includeLightTracingMis = splats != nullptr && splatScale > 0.0f;
    if (eye.isLight) {
        return s == 0
            ? estimateCameraHitLight(ci, includeLightTracingMis, cameraPath, lightPath, useVcMis)
            : Vector3f::ZERO;
    }

    if (s == 0) {
        return estimateCameraHitLight(ci, includeLightTracingMis, cameraPath, lightPath, useVcMis);
    }

    if (s == 1) {
        int surfaceDepth = t - 2;
        int directSamples = surfaceDepth == 0
            ? primaryDirectLightSamples
            : secondaryDirectLightSamples;
        return estimateDirectLight(eye, ci, directSamples, cameraPath, lightPath, useVcMis);
    }

    int li = s - 1;
    const VCMPathVertex &light = lightPath[li];

    ConnectionGeometry connection;
    if (!makeConnectionGeometry(eye, light, connection)) {
        return Vector3f::ZERO;
    }

    Vector3f c = connectVertices(eye, light, connection);
    if (c.squaredLength() <= 0.0f) {
        return Vector3f::ZERO;
    }

    if (useVcMis) {
        return c * vcMisWeight(eye, light, connection);
    }

    return c;
}

Vector3f VCM::estimateDirectLight(const VCMPathVertex &eye,
                                  int cameraIndex,
                                  int numSamples,
                                  const std::vector<VCMPathVertex> &cameraPath,
                                  const std::vector<VCMPathVertex> &lightPath,
                                  bool useMis) const {
    if (numSamples <= 0 || eye.material == nullptr || eye.isDelta || eye.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f result = Vector3f::ZERO;
    for (int i = 0; i < numSamples; ++i) {
        Light::SampleResult sample = scene.sampleLight(eye.pos);
        if (sample.pdf <= 0.0f || sample.dist <= 0.0f) {
            continue;
        }

        VCMPathVertex lightVertex;
        lightVertex.pos = sample.pos;
        lightVertex.normal = sample.normal;
        lightVertex.throughput = sample.col / sample.pdf;
        lightVertex.wo = -sample.dir;
        lightVertex.pdfForwardArea = sample.pdf;
        lightVertex.isDelta = false;
        lightVertex.isLight = true;
        lightVertex.type = VCMPathVertexType::Light;

        ConnectionGeometry connection;
        if (!makeConnectionGeometry(eye, lightVertex, connection)) {
            continue;
        }

        Vector3f c = connectVertices(eye, lightVertex, connection);
        if (c.squaredLength() <= 0.0f) {
            continue;
        }

        float wLight = useMis
            ? directLightMisWeight(eye, connection, sample.pdf)
            : 1.0f;

        result += c * wLight;
    }

    return result / static_cast<float>(numSamples);
}

Vector3f VCM::estimateCameraHitLight(int ci, 
                                     bool includeLightTracingMis,
                                     const std::vector<VCMPathVertex> &cameraPath,
                                     const std::vector<VCMPathVertex> &lightPath,
                                     bool useMis) const {
    (void)includeLightTracingMis;

    const VCMPathVertex &light = cameraPath[ci];
    if (ci <= 0 || light.material == nullptr || !light.isLight) {
        return Vector3f::ZERO;
    }

    Vector3f contribution = light.throughput * light.material->getEmission();
    if (ci == 1) {
        return contribution;
    }

    const VCMPathVertex &prev = cameraPath[ci - 1];
    if (prev.isDelta) {
        return contribution;
    }

    float wBsdf = useMis ? cameraHitLightMisWeight(ci, cameraPath) : 1.0f;
    return contribution * wBsdf;
}

Vector3f VCM::trace(size_t pathIdx, const Ray &cameraRay) {
    return trace(pathIdx, cameraRay, nullptr, 0.0f);
}

Vector3f VCM::trace(size_t pathIdx,
                    const Ray &cameraRay,
                    std::vector<FilmSplat> *splats,
                    float splatScale) {
    return traceVCMWithMIS(pathIdx, cameraRay, splats, splatScale);
}

Vector3f VCM::traceVMOnly(size_t pathIdx, const Ray &cameraRay) {
    (void)pathIdx;

    std::vector<VCMPathVertex> cameraPath;
    Vector3f L = generateCameraPath(
        cameraRay,
        cameraPath,
        MAX_VCM_CAMERA_PATH_DEPTH,
        true,
        true,
        true
    );
    return isFiniteColor(L) ? L : Vector3f::ZERO;
}

Vector3f VCM::traceVCOnly(size_t pathIdx,
                          const Ray &cameraRay,
                          std::vector<FilmSplat> *splats,
                          float splatScale) {
    if (pathIdx + 1 >= pathHeads.size()) {
        return Vector3f::ZERO;
    }

    std::vector<VCMPathVertex> cameraPath;
    std::vector<VCMPathVertex> lightPath(
        lightPhotons.begin() + pathHeads[pathIdx],
        lightPhotons.begin() + pathHeads[pathIdx + 1]
    );

    generateCameraPath(
        cameraRay,
        cameraPath,
        MAX_VCM_CAMERA_PATH_DEPTH,
        false,
        false,
        true
    );

    Vector3f L = Vector3f::ZERO;

    for (int t = 1; t <= (int)cameraPath.size(); ++t) {
        for (int s = 0; s <= (int)lightPath.size(); ++s) {
            L += connectVCM(s, t, splats, splatScale, cameraPath, lightPath, true);
        }
    }

    return isFiniteColor(L) ? L : Vector3f::ZERO;
}

Vector3f VCM::traceVCMNoMIS(size_t pathIdx,
                            const Ray &cameraRay,
                            std::vector<FilmSplat> *splats,
                            float splatScale) {
    if (pathIdx + 1 >= pathHeads.size()) {
        return Vector3f::ZERO;
    }

    std::vector<VCMPathVertex> cameraPath;
    std::vector<VCMPathVertex> lightPath(
        lightPhotons.begin() + pathHeads[pathIdx],
        lightPhotons.begin() + pathHeads[pathIdx + 1]
    );

    Vector3f L = generateCameraPath(
        cameraRay,
        cameraPath,
        MAX_VCM_CAMERA_PATH_DEPTH,
        false,
        true,
        false
    );

    for (int t = 1; t <= (int)cameraPath.size(); ++t) {
        for (int s = 0; s <= (int)lightPath.size(); ++s) {
            L += connectVCM(s, t, splats, splatScale, cameraPath, lightPath, false);
        }
    }

    return isFiniteColor(L) ? L : Vector3f::ZERO;
}

Vector3f VCM::traceVCMWithMIS(size_t pathIdx,
                              const Ray &cameraRay,
                              std::vector<FilmSplat> *splats,
                              float splatScale) {
    if (pathIdx + 1 >= pathHeads.size()) {
        return Vector3f::ZERO;
    }

    std::vector<VCMPathVertex> cameraPath;
    std::vector<VCMPathVertex> lightPath(
        lightPhotons.begin() + pathHeads[pathIdx],
        lightPhotons.begin() + pathHeads[pathIdx + 1]
    );

    Vector3f L = generateCameraPath(
        cameraRay,
        cameraPath,
        MAX_VCM_CAMERA_PATH_DEPTH,
        false,
        true,
        true
    );

    for (int t = 1; t <= (int)cameraPath.size(); ++t) {
        for (int s = 0; s <= (int)lightPath.size(); ++s) {
            L += connectVCM(s, t, splats, splatScale, cameraPath, lightPath, true);
        }
    }

    return isFiniteColor(L) ? L : Vector3f::ZERO;
}
