#include "renderer.hpp"

#include "Vector2f.h"
#include "bdpt.hpp"
#include "camera.hpp"
#include "path_tracer.hpp"
#include "progress.hpp"
#include "random.hpp"
#include "scene_parser.hpp"
#include "tone_mapping.hpp"
#include "vcm.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

const char *integratorName(IntegratorType integrator) {
    switch (integrator) {
    case IntegratorType::PT:
        return "pt";
    case IntegratorType::BDPT:
        return "bdpt";
    case IntegratorType::VCM:
        return "vcm";
    }
    return "unknown";
}

} // namespace

Renderer::Renderer(SceneParser &scene, const RenderConfig &config)
    : scene(scene), config(config) {}

Image Renderer::render() {
    if (config.integrator == IntegratorType::VCM) {
        return renderVCM();
    }
    if (config.integrator == IntegratorType::BDPT) {
        return renderBDPT();
    }

    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();
    Image image(width, height);

    const long long numPixels = static_cast<long long>(width) * height;
    ProgressBar progress(numPixels);

    const auto renderStart = std::chrono::steady_clock::now();
    progress.start();
    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < width; ++x) {
        PathTracer pathTracer(scene);
        for (int y = 0; y < height; ++y) {
            Vector3f color = Vector3f::ZERO;
            for (int i = 0; i < config.numSamples; ++i) {
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));
                color += pathTracer.trace(ray);
            }
            color = color / static_cast<float>(config.numSamples);
            color = toneMap(color, config.exposure);
            image.SetPixel(x, y, color);
        }
        progress.advance(height);
    }
    const auto renderEnd = std::chrono::steady_clock::now();
    progress.finish();

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    const long long primarySamples = numPixels * static_cast<long long>(config.numSamples);
    printStats(renderSeconds, numPixels, primarySamples);

    return image;
}

Image Renderer::renderBDPT() {
    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();
    Image image(width, height);
    std::vector<Vector3f> accumulated(
        static_cast<size_t>(width) * static_cast<size_t>(height),
        Vector3f::ZERO
    );

    const long long numPixels = static_cast<long long>(width) * height;
    ProgressBar progress(numPixels);

    const auto renderStart = std::chrono::steady_clock::now();
    progress.start();
    const float splatScale = 1.0f / static_cast<float>(numPixels);

    #pragma omp parallel for schedule(dynamic, 1)
    for (int x = 0; x < width; ++x) {
        BDPT bdpt(
            scene,
            config.bdptPrimaryDirectLightSamples,
            config.bdptSecondaryDirectLightSamples
        );
        std::vector<BDPT::FilmSplat> splats;
        for (int y = 0; y < height; ++y) {
            Vector3f color = Vector3f::ZERO;
            for (int i = 0; i < config.numSamples; ++i) {
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));

                splats.clear();
                color += bdpt.trace(ray, &splats, splatScale);
                for (const BDPT::FilmSplat &splat : splats) {
                    if (splat.x < 0 || splat.x >= width ||
                        splat.y < 0 || splat.y >= height) {
                        continue;
                    }
                    size_t index = static_cast<size_t>(splat.y) * width + splat.x;
                    #pragma omp critical(bdpt_splat_accumulate)
                    {
                        accumulated[index] += splat.contribution;
                    }
                }
            }

            size_t index = static_cast<size_t>(y) * width + x;
            #pragma omp critical(bdpt_splat_accumulate)
            {
                accumulated[index] += color;
            }
        }
        progress.advance(height);
    }

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            Vector3f color = accumulated[static_cast<size_t>(y) * width + x] /
                static_cast<float>(config.numSamples);
            color = toneMap(color, config.exposure);
            image.SetPixel(x, y, color);
        }
    }

    const auto renderEnd = std::chrono::steady_clock::now();
    progress.finish();

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    const long long primarySamples = numPixels * static_cast<long long>(config.numSamples);
    printStats(renderSeconds, numPixels, primarySamples);

    return image;
}

Image Renderer::renderVCM() {
    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();
    Image image(width, height);
    std::vector<Vector3f> accumulated(
        static_cast<size_t>(width) * static_cast<size_t>(height),
        Vector3f::ZERO
    );

    const long long numPixels = static_cast<long long>(width) * height;
    const long long primarySamples = numPixels * static_cast<long long>(config.numSamples);
    ProgressBar progress(primarySamples);

    const auto renderStart = std::chrono::steady_clock::now();
    progress.start();
    VCM vcm(
        scene,
        config.bdptPrimaryDirectLightSamples,
        config.bdptSecondaryDirectLightSamples,
        config.vcmRadius
    );
    for (int sampleIndex = 0; sampleIndex < config.numSamples; ++sampleIndex) {
        vcm.beginIteration(sampleIndex, width, height);

        #pragma omp parallel for schedule(dynamic, 1)
        for (int x = 0; x < width; ++x) {
            for (int y = 0; y < height; ++y) {
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));

                Vector3f color = vcm.trace(ray);

                size_t index = static_cast<size_t>(y) * width + x;
                accumulated[index] += color;
            }
            progress.advance(height);
        }
    }

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            Vector3f color = accumulated[static_cast<size_t>(y) * width + x] /
                static_cast<float>(config.numSamples);
            color = toneMap(color, config.exposure);
            image.SetPixel(x, y, color);
        }
    }

    const auto renderEnd = std::chrono::steady_clock::now();
    progress.finish();

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    printStats(renderSeconds, numPixels, primarySamples);

    return image;
}

void Renderer::printStats(double renderSeconds, long long numPixels, long long primarySamples) const {
    const double statsSeconds = renderSeconds > 0.0 ? renderSeconds : 1e-9;
    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[render stats] resolution: " << width << "x" << height
              << ", spp: " << config.numSamples
              << ", integrator: " << integratorName(config.integrator);
    if (config.integrator == IntegratorType::BDPT ||
        config.integrator == IntegratorType::VCM) {
        std::cout << ", direct-light samples: primary="
                  << config.bdptPrimaryDirectLightSamples
                  << ", secondary="
                  << config.bdptSecondaryDirectLightSamples;
    }
#ifdef _OPENMP
    std::cout << ", threads: " << omp_get_max_threads();
#endif
    std::cout << "\n";
    std::cout << "[render stats] total render time: " << renderSeconds << " s\n";
    std::cout << "[render stats] avg time per full-image spp: "
              << statsSeconds / static_cast<double>(config.numSamples) << " s\n";
    std::cout << "[render stats] avg time per primary sample: "
              << statsSeconds * 1000000.0 / static_cast<double>(primarySamples) << " us\n";
    std::cout << "[render stats] throughput: "
              << numPixels / statsSeconds << " pixels/s, "
              << primarySamples / statsSeconds << " primary samples/s\n";
}
