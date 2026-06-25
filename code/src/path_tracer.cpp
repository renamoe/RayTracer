#include "path_tracer.hpp"

#include "bsdf.hpp"
#include "group.hpp"
#include "light.hpp"
#include "material.hpp"
#include "path_guiding.hpp"
#include "random.hpp"
#include "scene_parser.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

constexpr float P_RR = 0.95f;
constexpr float RAY_EPS = 1e-6f;
constexpr float SHADOW_END_EPS = 1e-4f;
constexpr int RR_START_DEPTH = 2;
constexpr int MAX_DEPTH = 8;

float rrProbability(int depth) {
    return depth >= RR_START_DEPTH ? P_RR : 1.0f;
}

Vector3f clampLuminance(const Vector3f &c, float maxLum) {
    if (maxLum <= 0.0f) {
        return c;
    }
    float lum = 0.2126f * c.x() + 0.7152f * c.y() + 0.0722f * c.z();
    if (lum > maxLum && lum > 0.0f) {
        return c * (maxLum / lum);
    }
    return c;
}

float luminance(const Vector3f &c) {
    if (!std::isfinite(c.x()) ||
        !std::isfinite(c.y()) ||
        !std::isfinite(c.z())) {
        return 0.0f;
    }
    return 0.2126f * c.x() + 0.7152f * c.y() + 0.0722f * c.z();
}

Vector3f offsetRayOrigin(const Vector3f &pos,
                         const Vector3f &normal,
                         const Vector3f &dir) {
    return pos + (Vector3f::dot(dir, normal) > 0.0f ? normal : -normal) * RAY_EPS;
}

} // namespace

PathTracer::PathTracer(SceneParser &scene,
                       float specularClamp,
                       PathGuideGrid *pathGuideGrid,
                       bool trainPathGuiding,
                       bool usePathGuiding,
                       float pathGuidingProbability)
    : scene(scene),
      specularClamp(specularClamp),
      pathGuideGrid(pathGuideGrid),
      trainPathGuiding(trainPathGuiding),
      usePathGuiding(usePathGuiding),
      pathGuidingProbability(std::max(0.0f, std::min(1.0f, pathGuidingProbability))) {}

Vector3f PathTracer::estimateGlossyDirectLight(const Vector3f &pos,
                                               const Vector3f &normal,
                                               const Vector3f &rayDir,
                                               Material *material) {
    Light::SampleResult lightSample = scene.sampleLight(pos);
    if (lightSample.pdf <= 0.0f || lightSample.dist <= 0.0f) {
        return Vector3f::ZERO;
    }

    float cosThetaX = std::max(0.0f, Vector3f::dot(normal, lightSample.dir));
    float cosThetaY = std::max(0.0f, Vector3f::dot(lightSample.normal, -lightSample.dir));
    if (cosThetaX <= 0.0f || cosThetaY <= 0.0f) {
        return Vector3f::ZERO;
    }

    Vector3f shadowOrigin = offsetRayOrigin(pos, normal, lightSample.dir);
    Ray shadowRay(shadowOrigin, lightSample.dir);
    if (scene.getGroup()->occluded(shadowRay, RAY_EPS, lightSample.dist - SHADOW_END_EPS)) {
        return Vector3f::ZERO;
    }

    Vector3f V = (-rayDir).normalized();
    Vector3f f_r = evaluateCookTorranceGGX(
        normal,
        V,
        lightSample.dir.normalized(),
        material->getSpecularColor(),
        material->getRoughness()
    );

    float pdfLight = areaPdfToSolidAnglePdf(lightSample.pdf, lightSample.dist, cosThetaY);
    float pdfBrdf = ggxPdf(normal, V, lightSample.dir.normalized(), material->getRoughness());
    if (pdfLight <= 0.0f) {
        return Vector3f::ZERO;
    }
    float wLight = powerHeuristic(pdfLight, pdfBrdf);

    return lightSample.col * f_r * cosThetaX / pdfLight * wLight;
}

float PathTracer::activeGuideProbability(Material *material,
                                         const Vector3f &pos,
                                         const Vector3f &normal) const {
    if (!usePathGuiding || pathGuideGrid == nullptr || !pathGuideGrid->isValid() ||
        material == nullptr || material->isDelta()) {
        return 0.0f;
    }
    if (!pathGuideGrid->hasSamples(pos, normal)) {
        return 0.0f;
    }
    return pathGuidingProbability;
}

float PathTracer::continuationPdf(Material *material,
                                  const Vector3f &pos,
                                  const Vector3f &normal,
                                  const Vector3f &wo,
                                  const Vector3f &wi) const {
    float pdfBsdf = bsdfPdf(material, normal, wo, wi);
    float guideProb = activeGuideProbability(material, pos, normal);
    if (guideProb <= 0.0f) {
        return pdfBsdf;
    }

    float pdfGuide = pathGuideGrid->pdf(pos, normal, wi);
    return (1.0f - guideProb) * pdfBsdf + guideProb * pdfGuide;
}

BSDFSample PathTracer::sampleGuidedBSDF(Material *material,
                                        const Vector3f &diffuseColor,
                                        const Vector3f &pos,
                                        const Vector3f &normal,
                                        const Vector3f &wo) const {
    BSDFSample result{Vector3f::ZERO, Vector3f::ZERO, 0.0f, false};

    float guideProb = activeGuideProbability(material, pos, normal);
    if (guideProb <= 0.0f) {
        return sampleBSDF(material, diffuseColor, normal, wo);
    }

    Vector3f wi = Vector3f::ZERO;
    float pdfBsdf = 0.0f;
    float pdfGuide = 0.0f;

    if (Random::get_float() < guideProb) {
        PathGuideSample guideSample = pathGuideGrid->sample(pos, normal);
        if (!guideSample.valid || guideSample.pdf <= 0.0f) {
            return result;
        }
        wi = guideSample.dir;
        pdfGuide = guideSample.pdf;
        pdfBsdf = bsdfPdf(material, normal, wo, wi);
    } else {
        BSDFSample bsdfSample = sampleBSDF(material, diffuseColor, normal, wo);
        if (bsdfSample.pdf <= 0.0f ||
            bsdfSample.throughputWeight.squaredLength() <= 0.0f) {
            return result;
        }
        wi = bsdfSample.wi;
        pdfBsdf = bsdfSample.pdf;
        pdfGuide = pathGuideGrid->pdf(pos, normal, wi);
    }

    float pdfMix = (1.0f - guideProb) * pdfBsdf + guideProb * pdfGuide;
    float cosTheta = std::max(0.0f, Vector3f::dot(normal, wi));
    Vector3f f = evaluateBSDF(material, diffuseColor, normal, wo, wi);
    if (pdfMix <= 0.0f || cosTheta <= 0.0f ||
        f.squaredLength() <= 0.0f || !isFiniteColor(f)) {
        return result;
    }

    result.wi = wi;
    result.throughputWeight = f * cosTheta / std::max(pdfMix, 1e-6f);
    result.pdf = pdfMix;
    result.isDelta = false;
    return result;
}

void PathTracer::recordGuidingSample(Material *material,
                                     const Vector3f &pos,
                                     const Vector3f &normal,
                                     const Vector3f &wi,
                                     const Vector3f &incoming) const {
    if (!trainPathGuiding || pathGuideGrid == nullptr || !pathGuideGrid->isValid() ||
        material == nullptr || material->isDelta()) {
        return;
    }

    float value = luminance(incoming);
    if (value <= 0.0f) {
        return;
    }
    pathGuideGrid->record(pos, normal, wi, value);
}

Vector3f PathTracer::traceFromHit(const Ray &ray, const Hit &hit, int depth, bool fromSpecular) {
    Material *material = hit.getMaterial();
    Vector3f emission = Vector3f::ZERO;
    if (depth == 0 || fromSpecular) {
        emission = material->getEmission();
    }

    if (depth >= MAX_DEPTH) {
        return emission;
    }

    Vector3f pos = ray.pointAtParameter(hit.getT());
    Vector3f normal = hit.getNormal();

    if (material->isMirror()) {
        Vector3f shadingNormal = normal;
        if (Vector3f::dot(ray.getDirection(), shadingNormal) > 0.0f) {
            shadingNormal = -shadingNormal;
        }
        Vector3f R = reflect(ray.getDirection(), shadingNormal);

        if (material->getRoughness() <= DELTA_MIRROR_ROUGHNESS) {
            float rrProb = rrProbability(depth);
            if (Random::get_float() > rrProb) {
                return emission;
            }
            Ray nextRay(offsetRayOrigin(pos, normal, R), R);
            Vector3f bounced =
                trace(nextRay, depth + 1, true) * material->getSpecularColor() / rrProb;
            return emission + clampLuminance(bounced, specularClamp);
        }

        Vector3f direct = estimateGlossyDirectLight(pos, shadingNormal, ray.getDirection(), material);
        float rrProb = rrProbability(depth);
        if (Random::get_float() > rrProb) {
            return emission + direct;
        }

        Vector3f V = (-ray.getDirection()).normalized();
        GlossySample sample = sampleGGXDirection(shadingNormal, V, material->getRoughness());
        float cosTheta = Vector3f::dot(sample.dir, shadingNormal);
        if (cosTheta <= 0.0f || sample.pdf <= 0.0f) {
            return emission + direct;
        }
        Vector3f wi = sample.dir;
        Vector3f glossyBRDF = evaluateCookTorranceGGX(
            shadingNormal,
            V,
            wi.normalized(),
            material->getSpecularColor(),
            material->getRoughness()
        );
        Ray nextRay(offsetRayOrigin(pos, normal, wi), wi);
        Hit nextHit;
        bool nextIntersects = scene.getGroup()->intersect(nextRay, nextHit, RAY_EPS);

        bool hitLight = nextIntersects && nextHit.getMaterial() != nullptr && nextHit.getMaterial()->isEmissive();
        Vector3f brdfLight = Vector3f::ZERO;
        if (hitLight) {
            float pdfLight = scene.lightPdfFromHit(nextHit, wi);
            float wBrdf = powerHeuristic(sample.pdf, pdfLight);

            brdfLight = nextHit.getMaterial()->getEmission() * glossyBRDF * cosTheta / std::max(sample.pdf, 1e-6f) * wBrdf / rrProb;
        }

        Vector3f indirect = Vector3f::ZERO;
        if (!hitLight) {
            Vector3f incoming = nextIntersects
                ? traceFromHit(nextRay, nextHit, depth + 1)
                : scene.getBackgroundColor();
            indirect = incoming * glossyBRDF * cosTheta / (sample.pdf * rrProb);
        }

        return emission + direct + brdfLight + indirect;
    }

    if (material->isGlass()) {
        float rrProb = rrProbability(depth);
        if (Random::get_float() > rrProb) {
            return emission;
        }

        BSDFSample sample = sampleBSDF(material, normal, (-ray.getDirection()).normalized());
        if (sample.pdf <= 0.0f || sample.throughputWeight.squaredLength() <= 0.0f) {
            return emission;
        }
        Ray nextRay(offsetRayOrigin(pos, normal, sample.wi), sample.wi);
        Vector3f bounced =
            trace(nextRay, depth + 1, true) * sample.throughputWeight / rrProb;
        return emission + clampLuminance(bounced, specularClamp);
    }

    Vector3f shadingNormal = normal;
    if (Vector3f::dot(ray.getDirection(), shadingNormal) > 0.0f) {
        shadingNormal = -shadingNormal;
    }
    Vector3f wo = (-ray.getDirection()).normalized();
    Vector3f albedo = material->getDiffuseColor(hit);
    Light::SampleResult sample = scene.sampleLight(pos);
    Vector3f direct = Vector3f::ZERO;
    if (sample.pdf > 0.0f && sample.dist > 0.0f) {
        Vector3f shadowOrigin = offsetRayOrigin(pos, normal, sample.dir);
        Ray shadowRay(shadowOrigin, sample.dir);
        if (!scene.getGroup()->occluded(shadowRay, RAY_EPS, sample.dist - SHADOW_END_EPS)) {
            Vector3f lightDir = sample.dir.normalized();
            float cosThetaX = std::max(0.0f, Vector3f::dot(shadingNormal, lightDir));
            float cosThetaY = std::max(0.0f, Vector3f::dot(sample.normal, -sample.dir));
            Vector3f f_r = evaluateBSDF(material, albedo, shadingNormal, wo, lightDir);
            float pdfLight = areaPdfToSolidAnglePdf(sample.pdf, sample.dist, cosThetaY);
            float pdfPath = continuationPdf(material, pos, shadingNormal, wo, lightDir);
            if (pdfLight > 0.0f && f_r.squaredLength() > 0.0f) {
                float wLight = powerHeuristic(pdfLight, pdfPath);
                direct = sample.col * f_r * cosThetaX / pdfLight * wLight;
            }
        }
    }

    float rrProb = rrProbability(depth);
    if (Random::get_float() > rrProb) {
        return emission + direct;
    }

    BSDFSample bsdfSample = sampleGuidedBSDF(material, albedo, pos, shadingNormal, wo);
    if (bsdfSample.pdf <= 0.0f || bsdfSample.throughputWeight.squaredLength() <= 0.0f) {
        return emission + direct;
    }

    Ray nextRay(offsetRayOrigin(pos, normal, bsdfSample.wi), bsdfSample.wi);

    Hit nextHit;
    bool nextIntersects = scene.getGroup()->intersect(nextRay, nextHit, RAY_EPS);

    bool hitLight = nextIntersects && nextHit.getMaterial() != nullptr && nextHit.getMaterial()->isEmissive();
    Vector3f brdfLight = Vector3f::ZERO;
    if (hitLight) {
        float pdfLight = scene.lightPdfFromHit(nextHit, bsdfSample.wi);
        float wBrdf = powerHeuristic(bsdfSample.pdf, pdfLight);
        Vector3f lightEmission = nextHit.getMaterial()->getEmission();
        recordGuidingSample(material, pos, shadingNormal, bsdfSample.wi, lightEmission);
        brdfLight = lightEmission * bsdfSample.throughputWeight * wBrdf / rrProb;
    }

    Vector3f indirect = Vector3f::ZERO;
    if (!hitLight) {
        Vector3f incoming = nextIntersects
            ? traceFromHit(nextRay, nextHit, depth + 1)
            : scene.getBackgroundColor();
        recordGuidingSample(material, pos, shadingNormal, bsdfSample.wi, incoming);
        indirect = incoming * bsdfSample.throughputWeight / rrProb;
    }

    Vector3f result = emission + direct + brdfLight + indirect;
    if (!isFiniteColor(result)) {
        std::cout << "nan or inf" << std::endl;
        return Vector3f::ZERO;
    }
    return result;
}

Vector3f PathTracer::trace(const Ray &ray, int depth, bool fromSpecular) {
    Hit hit;
    if (!scene.getGroup()->intersect(ray, hit, RAY_EPS)) {
        return scene.getBackgroundColor();
    }
    return traceFromHit(ray, hit, depth, fromSpecular);
}
