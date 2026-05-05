#include "renderer.hpp"

#include "Vector2f.h"
#include "bdpt.hpp"
#include "camera.hpp"
#include "path_tracer.hpp"
#include "progress.hpp"
#include "random.hpp"
#include "scene_parser.hpp"
#include "tone_mapping.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

const char *integratorName(IntegratorType integrator) {
    return integrator == IntegratorType::BDPT ? "bdpt" : "pt";
}

} // namespace

Renderer::Renderer(SceneParser &scene, const RenderConfig &config)
    : scene(scene), config(config) {}

Image Renderer::render() {
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
        BDPT bdpt(scene);
        for (int y = 0; y < height; ++y) {
            Vector3f color = Vector3f::ZERO;
            for (int i = 0; i < config.numSamples; ++i) {
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));
                if (config.integrator == IntegratorType::BDPT) {
                    color += bdpt.trace(ray);
                } else {
                    color += pathTracer.trace(ray);
                }
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

void Renderer::printStats(double renderSeconds, long long numPixels, long long primarySamples) const {
    const double statsSeconds = renderSeconds > 0.0 ? renderSeconds : 1e-9;
    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[render stats] resolution: " << width << "x" << height
              << ", spp: " << config.numSamples
              << ", integrator: " << integratorName(config.integrator);
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
