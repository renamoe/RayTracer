#include "path_tracer.hpp"

#include "bsdf.hpp"
#include "group.hpp"
#include "light.hpp"
#include "material.hpp"
#include "random.hpp"
#include "scene_parser.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

namespace {

constexpr float P_RR = 0.95f;
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

} // namespace

PathTracer::PathTracer(SceneParser &scene, float specularClamp)
    : scene(scene), specularClamp(specularClamp) {}

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

    Vector3f shadowOrigin = pos + normal * 1e-6;
    Ray shadowRay(shadowOrigin, lightSample.dir);
    if (scene.getGroup()->occluded(shadowRay, 1e-6f, lightSample.dist - 1e-4f)) {
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
            Vector3f offsetNormal = Vector3f::dot(R, normal) > 0 ? normal : -normal;
            Ray nextRay(pos + offsetNormal * 1e-6, R);
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
        Vector3f offsetNormal = Vector3f::dot(wi, normal) > 0 ? normal : -normal;
        Ray nextRay(pos + offsetNormal * 1e-6, wi);
        Hit nextHit;
        bool nextIntersects = scene.getGroup()->intersect(nextRay, nextHit, 1e-6f);

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
        Vector3f offsetNormal = Vector3f::dot(sample.wi, normal) > 0 ? normal : -normal;
        Ray nextRay(pos + offsetNormal * 1e-6, sample.wi);
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
        Vector3f shadowOffsetNormal = Vector3f::dot(sample.dir, normal) > 0.0f ? normal : -normal;
        Vector3f shadowOrigin = pos + shadowOffsetNormal * 1e-6;
        Ray shadowRay(shadowOrigin, sample.dir);
        if (!scene.getGroup()->occluded(shadowRay, 1e-6f, sample.dist - 1e-4f)) {
            Vector3f lightDir = sample.dir.normalized();
            float cosThetaX = std::max(0.0f, Vector3f::dot(shadingNormal, lightDir));
            float cosThetaY = std::max(0.0f, Vector3f::dot(sample.normal, -sample.dir));
            Vector3f f_r = evaluateBSDF(material, albedo, shadingNormal, wo, lightDir);
            float pdfLight = areaPdfToSolidAnglePdf(sample.pdf, sample.dist, cosThetaY);
            float pdfBrdf = bsdfPdf(material, shadingNormal, wo, lightDir);
            if (pdfLight > 0.0f && f_r.squaredLength() > 0.0f) {
                float wLight = powerHeuristic(pdfLight, pdfBrdf);
                direct = sample.col * f_r * cosThetaX / pdfLight * wLight;
            }
        }
    }

    float rrProb = rrProbability(depth);
    if (Random::get_float() > rrProb) {
        return emission + direct;
    }

    BSDFSample bsdfSample = sampleBSDF(material, albedo, shadingNormal, wo);
    if (bsdfSample.pdf <= 0.0f || bsdfSample.throughputWeight.squaredLength() <= 0.0f) {
        return emission + direct;
    }

    Vector3f offsetNormal = Vector3f::dot(bsdfSample.wi, normal) > 0.0f ? normal : -normal;
    Ray nextRay(pos + offsetNormal * 1e-6, bsdfSample.wi);

    Hit nextHit;
    bool nextIntersects = scene.getGroup()->intersect(nextRay, nextHit, 1e-6f);

    bool hitLight = nextIntersects && nextHit.getMaterial() != nullptr && nextHit.getMaterial()->isEmissive();
    Vector3f brdfLight = Vector3f::ZERO;
    if (hitLight) {
        float pdfLight = scene.lightPdfFromHit(nextHit, bsdfSample.wi);
        float wBrdf = powerHeuristic(bsdfSample.pdf, pdfLight);
        brdfLight = nextHit.getMaterial()->getEmission() * bsdfSample.throughputWeight * wBrdf / rrProb;
    }

    Vector3f indirect = Vector3f::ZERO;
    if (!hitLight) {
        Vector3f incoming = nextIntersects
            ? traceFromHit(nextRay, nextHit, depth + 1)
            : scene.getBackgroundColor();
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
    if (!scene.getGroup()->intersect(ray, hit, 0)) {
        return scene.getBackgroundColor();
    }
    return traceFromHit(ray, hit, depth, fromSpecular);
}
