#include "bdpt.hpp"
#include "bsdf.hpp"
#include "group.hpp"
#include "random.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float CONNECT_EPS = 1e-4f;
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

Vector3f offsetRayOrigin(const Vector3f &pos,
                         const Vector3f &normal,
                         const Vector3f &dir) {
    return pos + (Vector3f::dot(dir, normal) > 0.0f ? normal : -normal) * CONNECT_EPS;
}

bool isDeltaMaterial(Material *mat) {
    return mat->isGlass() ||
        (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
}

float pdfAreaFromTo(const PathVertex &from,
                    const Vector3f &wo,
                    const PathVertex &to) {
    if (from.material == nullptr || from.isDelta) {
        return 0.0f;
    }

    Vector3f dir = (to.pos - from.pos).normalized();

    float pdfW = from.isLight
        ? std::max(0.0f, Vector3f::dot(from.normal, dir)) / M_PI
        : bsdfPdf(from.material, from.normal, wo, dir);

    return solidAngleToAreaPdf(pdfW, from.pos, to.pos, to.normal);
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
        v.isDelta = mat->isGlass() || 
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();

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

        BSDFSample sample = sampleBSDF(mat, normal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.length() <= 0.0f) {
            break;
        }

        PathVertex &curr = path.back();
        curr.wi = sample.wi;
        pendingPdfW = sample.pdf * rrProb;
        
        beta = beta * sample.throughputWeight;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, normal) > 0.0f ? normal : -normal;

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
        v.isDelta = mat->isGlass() ||
            (mat->isMirror() && mat->getRoughness() < DELTA_MIRROR_ROUGHNESS);
        v.isLight = mat->isEmissive();

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

        BSDFSample sample = sampleBSDF(mat, normal, v.wo);
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            break;
        }

        PathVertex &curr = path.back();
        curr.wi = sample.wi;
        pendingPdfW = sample.pdf * rrProb;

        beta = beta * sample.throughputWeight;

        Vector3f offsetNormal = Vector3f::dot(sample.wi, normal) > 0.0f ? normal : -normal;
        ray = Ray(pos + offsetNormal * CONNECT_EPS, sample.wi);
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

    Vector3f shadowOrigin = offsetRayOrigin(eye.pos, eye.normal, eyeToLight);
    Vector3f shadowEdge = light.pos - shadowOrigin;
    float shadowDist = shadowEdge.length();
    if (shadowDist <= CONNECT_EPS) {
        return Vector3f::ZERO;
    }
    Ray shadowRay(shadowOrigin, shadowEdge / shadowDist);
    if (scene.getGroup()->occluded(shadowRay, CONNECT_EPS, shadowDist - CONNECT_EPS)) {
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

    return eye.throughput * fEye * geometryTerm * fLight * light.throughput;
}

Vector3f BDPT::estimateDirectLight(const PathVertex &eye, int numSamples) const {
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

        result += eye.throughput * sample.col * fEye * cosEye / pdfLightW;
    }

    return result / static_cast<float>(numSamples);
}

float BDPT::bdptMisWeight(int ci, int li) const {
    constexpr float EPS = 1e-8f;

    const PathVertex &eye = cameraPath[ci];
    const PathVertex &light = lightPath[li];

    Vector3f eyeToLight = (light.pos - eye.pos).normalized();
    Vector3f lightToEye = -eyeToLight;

    float sum = 1.0f;

    // Move light vertices to camera side.
    float ratio = 1.0f;
    if (li >= 0) {
        float pdfEyeToLight = pdfAreaFromTo(eye, eye.wo, light);
        float denom = std::max(light.pdfForwardArea, EPS);

        ratio *= pdfEyeToLight / denom;
        sum += ratio * ratio;

        Vector3f incoming = lightToEye;

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
        float pdfLightToEye = pdfAreaFromTo(light, light.wo, eye);
        float denom = std::max(eye.pdfForwardArea, EPS);

        ratio *= pdfLightToEye / denom;
        sum += ratio * ratio;

        Vector3f incoming = eyeToLight;

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
    cameraPath.clear();
    lightPath.clear();

    generateCameraPath(cameraRay, cameraPath, MAX_CAMERA_PATH_DEPTH);
    generateLightPath(lightPath, MAX_LIGHT_PATH_DEPTH);

    Vector3f L = Vector3f::ZERO;

    for (int ci = 0; ci < (int)cameraPath.size(); ++ci) {
        const auto &eye = cameraPath[ci];

        if (eye.isLight) {
            if (ci == 0 || cameraPath[ci - 1].isDelta) {
                L += eye.throughput * eye.material->getEmission();
            }
            continue;
        }

        int directSamples = ci == 0
            ? primaryDirectLightSamples
            : secondaryDirectLightSamples;
        L += estimateDirectLight(eye, directSamples);

        for (int li = 1; li < (int)lightPath.size(); ++li) {
            const auto &light = lightPath[li];

            Vector3f c = connectVertices(eye, light);
            if (c.squaredLength() <= 0.0f) {
                continue;
            }

            float w = bdptMisWeight(ci, li);
            if (!std::isfinite(w) || w <= 0.0f || w > 1.0f) {
                std::cout << "bad mis weight: " << w << "\n";
            }

            L += c * w;
        }
    }

    return L;
}
