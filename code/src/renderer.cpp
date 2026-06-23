#include "renderer.hpp"

#include "Vector2f.h"
#include "bdpt.hpp"
#include "camera.hpp"
#include "path_tracer.hpp"
#include "preview_window.hpp"
#include "progress.hpp"
#include "random.hpp"
#include "scene_parser.hpp"
#include "tone_mapping.hpp"
#include "vcm.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <memory>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

volatile std::sig_atomic_t gSignalInterrupted = 0;

void handleRenderSignal(int) {
    gSignalInterrupted = 1;
}

bool signalInterrupted() {
    return gSignalInterrupted != 0;
}

class RenderInterruptGuard {
public:
    RenderInterruptGuard() {
        gSignalInterrupted = 0;
        previousInt = std::signal(SIGINT, handleRenderSignal);
        previousTerm = std::signal(SIGTERM, handleRenderSignal);
    }

    ~RenderInterruptGuard() {
        if (previousInt != SIG_ERR) {
            std::signal(SIGINT, previousInt);
        }
        if (previousTerm != SIG_ERR) {
            std::signal(SIGTERM, previousTerm);
        }
    }

private:
    using SignalHandler = void (*)(int);

    SignalHandler previousInt = SIG_DFL;
    SignalHandler previousTerm = SIG_DFL;
};

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

double elapsedSeconds(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start
    ).count();
}

bool hasTimeLimit(const RenderConfig &config) {
    return config.timeLimitSeconds > 0.0;
}

bool shouldStartIteration(const RenderConfig &config,
                          int completedIterations,
                          std::chrono::steady_clock::time_point renderStart) {
    if (!hasTimeLimit(config)) {
        return completedIterations < config.numSamples;
    }
    if (completedIterations <= 0) {
        return true;
    }

    double elapsed = elapsedSeconds(renderStart);
    return elapsed < config.timeLimitSeconds;
}

void printTimedIteration(int completedIterations,
                         double iterationSeconds,
                         double elapsed,
                         double timeLimitSeconds) {
    std::cout << "[render] completed iteration " << completedIterations
              << " in " << iterationSeconds << " s"
              << ", elapsed " << elapsed << "/" << timeLimitSeconds << " s\n";
}

std::unique_ptr<PreviewWindow> createPreviewWindow(const RenderConfig &config,
                                                   int width,
                                                   int height) {
    if (!config.preview) {
        return nullptr;
    }

    std::unique_ptr<PreviewWindow> preview =
        std::make_unique<PreviewWindow>(width, height);
    if (!preview->isOpen()) {
        return nullptr;
    }
    return preview;
}

void updatePreview(PreviewWindow *preview,
                   const RenderConfig &config,
                   const std::vector<Vector3f> &accumulated,
                   const std::vector<Vector3f> *splatAccumulated,
                   int completedIterations) {
    if (preview == nullptr || !preview->isOpen()) {
        return;
    }
    preview->update(
        accumulated,
        splatAccumulated,
        completedIterations,
        config.exposure
    );
}

void updatePreviewIfNeeded(PreviewWindow *preview,
                           const RenderConfig &config,
                           const std::vector<Vector3f> &accumulated,
                           const std::vector<Vector3f> *splatAccumulated,
                           int completedIterations) {
    const int interval = config.previewEveryIterations > 0
        ? config.previewEveryIterations
        : 1;
    if (completedIterations % interval != 0) {
        return;
    }
    updatePreview(
        preview,
        config,
        accumulated,
        splatAccumulated,
        completedIterations
    );
}

bool cancellationRequested(std::atomic_bool &cancelRequested) {
    if (signalInterrupted()) {
        cancelRequested.store(true);
    }
    return cancelRequested.load();
}

bool refreshCancellationFromPreview(PreviewWindow *preview,
                                    std::atomic_bool &cancelRequested) {
    if (preview != nullptr && !preview->isOpen()) {
        cancelRequested.store(true);
    }
    return cancellationRequested(cancelRequested);
}

bool pollPreviewEventsDuringWork(PreviewWindow *preview,
                                 std::atomic_bool &cancelRequested) {
    if (cancellationRequested(cancelRequested)) {
        return true;
    }
#ifdef _OPENMP
    if (omp_in_parallel() && omp_get_thread_num() != 0) {
        return cancelRequested.load();
    }
#endif
    if (preview != nullptr && preview->isOpen()) {
        preview->pollEvents();
    }
    return refreshCancellationFromPreview(preview, cancelRequested);
}

Vector3f clampLuminance(const Vector3f &color, float maxLum) {
    if (!std::isfinite(color.x()) ||
        !std::isfinite(color.y()) ||
        !std::isfinite(color.z())) {
        return Vector3f::ZERO;
    }
    if (maxLum <= 0.0f) {
        return color;
    }

    float lum = 0.2126f * color.x() + 0.7152f * color.y() + 0.0722f * color.z();
    if (lum > maxLum && lum > 0.0f) {
        return color * (maxLum / lum);
    }
    return color;
}

void writeToneMappedImage(Image &image,
                          const std::vector<Vector3f> &accumulated,
                          const std::vector<Vector3f> *splatAccumulated,
                          int width,
                          int height,
                          int completedIterations,
                          float exposure) {
    if (completedIterations <= 0) {
        image.SetAllPixels(Vector3f::ZERO);
        return;
    }

    const float invIterations = 1.0f / static_cast<float>(completedIterations);
    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            size_t index = static_cast<size_t>(y) * width + x;
            Vector3f color = accumulated[index];
            if (splatAccumulated != nullptr) {
                color += (*splatAccumulated)[index];
            }
            color = toneMap(color * invIterations, exposure);
            image.SetPixel(x, y, color);
        }
    }
}

} // namespace

Renderer::Renderer(SceneParser &scene, const RenderConfig &config)
    : scene(scene), config(config) {}

Image Renderer::render() {
    RenderInterruptGuard interruptGuard;

    if (config.integrator == IntegratorType::VCM) {
        return renderVCM();
    }
    if (config.integrator == IntegratorType::BDPT) {
        return renderBDPT();
    }

    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();
    Image image(width, height);
    std::vector<Vector3f> accumulated(
        static_cast<size_t>(width) * static_cast<size_t>(height),
        Vector3f::ZERO
    );

    const long long numPixels = static_cast<long long>(width) * height;
    const bool timed = hasTimeLimit(config);
    const long long requestedPrimarySamples =
        numPixels * static_cast<long long>(config.numSamples);
    ProgressBar progress(requestedPrimarySamples);

    const auto renderStart = std::chrono::steady_clock::now();
    if (timed) {
        std::cout << "[render] time-limited mode: " << config.timeLimitSeconds
                  << " s\n";
    } else {
        progress.start();
    }
    std::unique_ptr<PreviewWindow> preview =
        createPreviewWindow(config, width, height);
    std::atomic_bool cancelRequested(false);

    int completedIterations = 0;
    double lastIterationSeconds = 0.0;
    while (!cancellationRequested(cancelRequested) &&
           shouldStartIteration(config, completedIterations, renderStart)) {
        const auto iterationStart = std::chrono::steady_clock::now();

        #pragma omp parallel for schedule(dynamic, 1)
        for (int x = 0; x < width; ++x) {
            if (cancellationRequested(cancelRequested)) {
                continue;
            }
            PathTracer pathTracer(scene);
            int renderedRows = 0;
            for (int y = 0; y < height; ++y) {
                if ((y & 15) == 0 && cancellationRequested(cancelRequested)) {
                    break;
                }
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));

                size_t index = static_cast<size_t>(y) * width + x;
                accumulated[index] += clampLuminance(pathTracer.trace(ray), config.sampleClamp);
                ++renderedRows;
            }
            if (!timed && renderedRows > 0) {
                progress.advance(renderedRows);
            }
            pollPreviewEventsDuringWork(preview.get(), cancelRequested);
        }

        if (cancellationRequested(cancelRequested)) {
            break;
        }

        ++completedIterations;
        lastIterationSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - iterationStart
        ).count();
        if (timed) {
            printTimedIteration(
                completedIterations,
                lastIterationSeconds,
                elapsedSeconds(renderStart),
                config.timeLimitSeconds
            );
        }
        updatePreviewIfNeeded(
            preview.get(),
            config,
            accumulated,
            nullptr,
            completedIterations
        );
        if (refreshCancellationFromPreview(preview.get(), cancelRequested)) {
            break;
        }
    }

    if (!cancellationRequested(cancelRequested)) {
        updatePreview(preview.get(), config, accumulated, nullptr, completedIterations);
        refreshCancellationFromPreview(preview.get(), cancelRequested);
    }
    writeToneMappedImage(
        image,
        accumulated,
        nullptr,
        width,
        height,
        completedIterations,
        config.exposure
    );
    const auto renderEnd = std::chrono::steady_clock::now();
    const bool cancelled = cancellationRequested(cancelRequested);
    if (!timed && !cancelled) {
        progress.finish();
    }
    if (cancelled) {
        std::cout << "\n[render] cancelled after " << completedIterations
                  << " completed iteration(s)\n";
    }

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    const long long primarySamples =
        numPixels * static_cast<long long>(completedIterations);
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
    std::vector<Vector3f> splatAccumulated(
        static_cast<size_t>(width) * static_cast<size_t>(height),
        Vector3f::ZERO
    );

    const long long numPixels = static_cast<long long>(width) * height;
    const bool timed = hasTimeLimit(config);
    const long long requestedPrimarySamples =
        numPixels * static_cast<long long>(config.numSamples);
    ProgressBar progress(requestedPrimarySamples);

    const auto renderStart = std::chrono::steady_clock::now();
    if (timed) {
        std::cout << "[render] time-limited mode: " << config.timeLimitSeconds
                  << " s\n";
    } else {
        progress.start();
    }
    std::unique_ptr<PreviewWindow> preview =
        createPreviewWindow(config, width, height);
    std::atomic_bool cancelRequested(false);
    const float splatScale = 1.0f / static_cast<float>(numPixels);

    int completedIterations = 0;
    double lastIterationSeconds = 0.0;
    while (!cancellationRequested(cancelRequested) &&
           shouldStartIteration(config, completedIterations, renderStart)) {
        const auto iterationStart = std::chrono::steady_clock::now();

        #pragma omp parallel for schedule(dynamic, 1)
        for (int x = 0; x < width; ++x) {
            if (cancellationRequested(cancelRequested)) {
                continue;
            }
            BDPT bdpt(
                scene,
                config.bdptPrimaryDirectLightSamples,
                config.bdptSecondaryDirectLightSamples
            );
            std::vector<BDPT::FilmSplat> splats;
            int renderedRows = 0;
            for (int y = 0; y < height; ++y) {
                if ((y & 15) == 0 && cancellationRequested(cancelRequested)) {
                    break;
                }
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));

                splats.clear();
                Vector3f color = bdpt.trace(ray, &splats, splatScale);
                for (const BDPT::FilmSplat &splat : splats) {
                    if (splat.x < 0 || splat.x >= width ||
                        splat.y < 0 || splat.y >= height) {
                        continue;
                    }
                    size_t splatIndex = static_cast<size_t>(splat.y) * width + splat.x;
                    #pragma omp critical(bdpt_splat_accumulate)
                    {
                        splatAccumulated[splatIndex] +=
                            clampLuminance(splat.contribution, config.sampleClamp);
                    }
                }

                size_t index = static_cast<size_t>(y) * width + x;
                accumulated[index] += clampLuminance(color, config.sampleClamp);
                ++renderedRows;
            }
            if (!timed && renderedRows > 0) {
                progress.advance(renderedRows);
            }
            pollPreviewEventsDuringWork(preview.get(), cancelRequested);
        }

        if (cancellationRequested(cancelRequested)) {
            break;
        }

        ++completedIterations;
        lastIterationSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - iterationStart
        ).count();
        if (timed) {
            printTimedIteration(
                completedIterations,
                lastIterationSeconds,
                elapsedSeconds(renderStart),
                config.timeLimitSeconds
            );
        }
        updatePreviewIfNeeded(
            preview.get(),
            config,
            accumulated,
            &splatAccumulated,
            completedIterations
        );
        if (refreshCancellationFromPreview(preview.get(), cancelRequested)) {
            break;
        }
    }

    if (!cancellationRequested(cancelRequested)) {
        updatePreview(
            preview.get(),
            config,
            accumulated,
            &splatAccumulated,
            completedIterations
        );
        refreshCancellationFromPreview(preview.get(), cancelRequested);
    }
    writeToneMappedImage(
        image,
        accumulated,
        &splatAccumulated,
        width,
        height,
        completedIterations,
        config.exposure
    );

    const auto renderEnd = std::chrono::steady_clock::now();
    const bool cancelled = cancellationRequested(cancelRequested);
    if (!timed && !cancelled) {
        progress.finish();
    }
    if (cancelled) {
        std::cout << "\n[render] cancelled after " << completedIterations
                  << " completed iteration(s)\n";
    }

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    const long long primarySamples =
        numPixels * static_cast<long long>(completedIterations);
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
    std::vector<Vector3f> splatAccumulated(
        static_cast<size_t>(width) * static_cast<size_t>(height),
        Vector3f::ZERO
    );

    const long long numPixels = static_cast<long long>(width) * height;
    const bool timed = hasTimeLimit(config);
    const long long requestedPrimarySamples =
        numPixels * static_cast<long long>(config.numSamples);
    ProgressBar progress(requestedPrimarySamples);

    const auto renderStart = std::chrono::steady_clock::now();
    if (timed) {
        std::cout << "[render] time-limited mode: " << config.timeLimitSeconds
                  << " s\n";
    } else {
        progress.start();
    }
    std::unique_ptr<PreviewWindow> preview =
        createPreviewWindow(config, width, height);
    std::atomic_bool cancelRequested(false);
    const float splatScale = 1.0f / static_cast<float>(numPixels);
    VCM vcm(
        scene,
        config.bdptPrimaryDirectLightSamples,
        config.bdptSecondaryDirectLightSamples,
        config.vcmRadius,
        config.vcmCameraPathDepth,
        config.vcmLightPathDepth,
        config.vcmCausticOnlyMerging
    );
    int completedIterations = 0;
    double lastIterationSeconds = 0.0;
    while (!cancellationRequested(cancelRequested) &&
           shouldStartIteration(config, completedIterations, renderStart)) {
        const auto iterationStart = std::chrono::steady_clock::now();
        bool iterationReady = vcm.beginIteration(
            completedIterations,
            width,
            height,
            config.vcmLightPathCount,
            [&preview, &cancelRequested]() {
                return pollPreviewEventsDuringWork(preview.get(), cancelRequested);
            }
        );
        if (!iterationReady || cancellationRequested(cancelRequested)) {
            cancelRequested.store(true);
            break;
        }

        #pragma omp parallel for schedule(dynamic, 1)
        for (int x = 0; x < width; ++x) {
            if (cancellationRequested(cancelRequested)) {
                continue;
            }
            std::vector<VCM::FilmSplat> splats;
            int renderedRows = 0;
            for (int y = 0; y < height; ++y) {
                if ((y & 15) == 0 && cancellationRequested(cancelRequested)) {
                    break;
                }
                float dx = Random::get_float() - 0.5f;
                float dy = Random::get_float() - 0.5f;
                Ray ray = scene.getCamera()->generateRay(Vector2f(x + dx, y + dy));

                size_t pathIdx = static_cast<size_t>(y) * width + x;
                int lightPathCount = vcm.getLightPathCount();
                size_t lightPathIdx = lightPathCount > 0
                    ? (pathIdx +
                       static_cast<size_t>(completedIterations) *
                       static_cast<size_t>(numPixels)) %
                       static_cast<size_t>(lightPathCount)
                    : pathIdx;
                // Vector3f color = vcm.traceVMOnly(pathIdx, ray);
                // Vector3f color = vcm.traceVCOnly(pathIdx, ray);
                // Vector3f color = vcm.traceVCMNoMIS(pathIdx, ray);
                splats.clear();
                Vector3f color = vcm.trace(lightPathIdx, ray, &splats, splatScale);
                for (const VCM::FilmSplat &splat : splats) {
                    if (splat.x < 0 || splat.x >= width ||
                        splat.y < 0 || splat.y >= height) {
                        continue;
                    }
                    size_t splatIndex = static_cast<size_t>(splat.y) * width + splat.x;
                    #pragma omp critical(vcm_splat_accumulate)
                    {
                        splatAccumulated[splatIndex] +=
                            clampLuminance(splat.contribution, config.sampleClamp);
                    }
                }


                size_t index = static_cast<size_t>(y) * width + x;
                accumulated[index] += clampLuminance(color, config.sampleClamp);
                ++renderedRows;
            }
            if (!timed && renderedRows > 0) {
                progress.advance(renderedRows);
            }
            pollPreviewEventsDuringWork(preview.get(), cancelRequested);
        }

        if (cancellationRequested(cancelRequested)) {
            break;
        }

        ++completedIterations;
        lastIterationSeconds = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - iterationStart
        ).count();
        if (timed) {
            printTimedIteration(
                completedIterations,
                lastIterationSeconds,
                elapsedSeconds(renderStart),
                config.timeLimitSeconds
            );
        }
        updatePreviewIfNeeded(
            preview.get(),
            config,
            accumulated,
            &splatAccumulated,
            completedIterations
        );
        if (refreshCancellationFromPreview(preview.get(), cancelRequested)) {
            break;
        }
    }

    if (!cancellationRequested(cancelRequested)) {
        updatePreview(
            preview.get(),
            config,
            accumulated,
            &splatAccumulated,
            completedIterations
        );
        refreshCancellationFromPreview(preview.get(), cancelRequested);
    }
    writeToneMappedImage(
        image,
        accumulated,
        &splatAccumulated,
        width,
        height,
        completedIterations,
        config.exposure
    );

    const auto renderEnd = std::chrono::steady_clock::now();
    const bool cancelled = cancellationRequested(cancelRequested);
    if (!timed && !cancelled) {
        progress.finish();
    }
    if (cancelled) {
        std::cout << "\n[render] cancelled after " << completedIterations
                  << " completed iteration(s)\n";
    }

    const double renderSeconds =
        std::chrono::duration<double>(renderEnd - renderStart).count();
    const long long primarySamples =
        numPixels * static_cast<long long>(completedIterations);
    printStats(renderSeconds, numPixels, primarySamples);

    return image;
}

void Renderer::printStats(double renderSeconds, long long numPixels, long long primarySamples) const {
    const double statsSeconds = renderSeconds > 0.0 ? renderSeconds : 1e-9;
    const long long completedSpp = numPixels > 0 ? primarySamples / numPixels : 0;
    const double completedSppForStats =
        completedSpp > 0 ? static_cast<double>(completedSpp) : 1.0;
    int width = scene.getCamera()->getWidth();
    int height = scene.getCamera()->getHeight();

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "[render stats] resolution: " << width << "x" << height
              << ", spp: " << completedSpp
              << ", integrator: " << integratorName(config.integrator);
    if (hasTimeLimit(config)) {
        std::cout << ", time limit: " << config.timeLimitSeconds << " s";
    }
    if (config.integrator == IntegratorType::BDPT ||
        config.integrator == IntegratorType::VCM) {
        std::cout << ", direct-light samples: primary="
                  << config.bdptPrimaryDirectLightSamples
                  << ", secondary="
                  << config.bdptSecondaryDirectLightSamples;
    }
    if (config.integrator == IntegratorType::VCM) {
        std::cout << ", vcm base radius: " << config.vcmRadius
                  << ", vcm depth: camera=" << config.vcmCameraPathDepth
                  << ", light=" << config.vcmLightPathDepth
                  << ", light paths="
                  << (config.vcmLightPathCount > 0
                      ? config.vcmLightPathCount
                      : static_cast<int>(numPixels))
                  << ", vcm merging: "
                  << (config.vcmCausticOnlyMerging ? "caustic-only" : "all");
    }
    if (config.sampleClamp > 0.0f) {
        std::cout << ", sample clamp: " << config.sampleClamp;
    }
#ifdef _OPENMP
    std::cout << ", threads: " << omp_get_max_threads();
#endif
    std::cout << "\n";
    std::cout << "[render stats] total render time: " << renderSeconds << " s\n";
    std::cout << "[render stats] avg time per full-image spp: "
              << statsSeconds / completedSppForStats << " s\n";
    if (primarySamples > 0) {
        std::cout << "[render stats] avg time per primary sample: "
                  << statsSeconds * 1000000.0 / static_cast<double>(primarySamples) << " us\n";
        std::cout << "[render stats] throughput: "
                  << numPixels / statsSeconds << " pixels/s, "
                  << primarySamples / statsSeconds << " primary samples/s\n";
    } else {
        std::cout << "[render stats] avg time per primary sample: n/a\n";
        std::cout << "[render stats] throughput: "
                  << numPixels / statsSeconds << " pixels/s, 0.000 primary samples/s\n";
    }
}
