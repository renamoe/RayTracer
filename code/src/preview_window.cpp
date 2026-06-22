#include "preview_window.hpp"

#include "tone_mapping.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#ifdef RAYTRACER_ENABLE_PREVIEW
#define SDL_MAIN_HANDLED
#include <SDL.h>
#endif

namespace {

unsigned char toByte(float value) {
    if (!std::isfinite(value)) {
        return 0;
    }
    value = std::max(0.0f, std::min(1.0f, value));
    return static_cast<unsigned char>(value * 255.0f + 0.5f);
}

} // namespace

PreviewWindow::PreviewWindow(int width, int height)
    : width(width), height(height) {
#ifdef RAYTRACER_ENABLE_PREVIEW
    SDL_SetMainReady();
    SDL_SetHint(SDL_HINT_NO_SIGNAL_HANDLERS, "1");
    if (SDL_InitSubSystem(SDL_INIT_VIDEO) != 0) {
        std::cerr << "[preview] failed to initialize SDL video: "
                  << SDL_GetError() << "\n";
        return;
    }
    sdlInitialized = true;

    window = SDL_CreateWindow(
        "RayTracer Preview",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        width,
        height,
        SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE
    );
    if (window == nullptr) {
        std::cerr << "[preview] failed to create window: "
                  << SDL_GetError() << "\n";
        return;
    }

    renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC
    );
    if (renderer == nullptr) {
        renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_SOFTWARE);
    }
    if (renderer == nullptr) {
        std::cerr << "[preview] failed to create renderer: "
                  << SDL_GetError() << "\n";
        return;
    }

    texture = SDL_CreateTexture(
        renderer,
        SDL_PIXELFORMAT_RGB24,
        SDL_TEXTUREACCESS_STREAMING,
        width,
        height
    );
    if (texture == nullptr) {
        std::cerr << "[preview] failed to create texture: "
                  << SDL_GetError() << "\n";
        return;
    }

    pixels.resize(static_cast<size_t>(width) * static_cast<size_t>(height) * 3);
    open = true;
#else
    (void)width;
    (void)height;
    std::cerr << "[preview] SDL2 support was not enabled at build time; "
              << "continuing without a preview window.\n";
#endif
}

PreviewWindow::~PreviewWindow() {
#ifdef RAYTRACER_ENABLE_PREVIEW
    if (texture != nullptr) {
        SDL_DestroyTexture(texture);
    }
    if (renderer != nullptr) {
        SDL_DestroyRenderer(renderer);
    }
    if (window != nullptr) {
        SDL_DestroyWindow(window);
    }
    if (sdlInitialized) {
        SDL_QuitSubSystem(SDL_INIT_VIDEO);
    }
#endif
}

bool PreviewWindow::isOpen() const {
    return open;
}

void PreviewWindow::update(const std::vector<Vector3f> &accumulated,
                           const std::vector<Vector3f> *splatAccumulated,
                           int completedIterations,
                           float exposure) {
#ifdef RAYTRACER_ENABLE_PREVIEW
    if (!open || completedIterations <= 0) {
        return;
    }

    pollEvents();
    if (!open) {
        return;
    }

    const float invIterations = 1.0f / static_cast<float>(completedIterations);
    const size_t expectedPixels =
        static_cast<size_t>(width) * static_cast<size_t>(height);
    if (accumulated.size() < expectedPixels ||
        (splatAccumulated != nullptr && splatAccumulated->size() < expectedPixels)) {
        return;
    }

    for (int y = 0; y < height; ++y) {
        const int displayY = height - 1 - y;
        for (int x = 0; x < width; ++x) {
            const size_t index = static_cast<size_t>(y) * width + x;
            Vector3f color = accumulated[index];
            if (splatAccumulated != nullptr) {
                color += (*splatAccumulated)[index];
            }
            color = toneMap(color * invIterations, exposure);

            const size_t pixelIndex =
                (static_cast<size_t>(displayY) * width + x) * 3;
            pixels[pixelIndex] = toByte(color.x());
            pixels[pixelIndex + 1] = toByte(color.y());
            pixels[pixelIndex + 2] = toByte(color.z());
        }
    }

    if (SDL_UpdateTexture(texture, nullptr, pixels.data(), width * 3) != 0) {
        std::cerr << "[preview] failed to update texture: "
                  << SDL_GetError() << "\n";
        open = false;
        return;
    }

    SDL_RenderClear(renderer);
    SDL_RenderCopy(renderer, texture, nullptr, nullptr);
    SDL_RenderPresent(renderer);
    pollEvents();
#else
    (void)accumulated;
    (void)splatAccumulated;
    (void)completedIterations;
    (void)exposure;
#endif
}

void PreviewWindow::pollEvents() {
#ifdef RAYTRACER_ENABLE_PREVIEW
    SDL_Event event;
    const Uint32 windowId = window != nullptr ? SDL_GetWindowID(window) : 0;
    while (SDL_PollEvent(&event)) {
        if (event.type == SDL_QUIT) {
            open = false;
        } else if (event.type == SDL_WINDOWEVENT &&
                   event.window.windowID == windowId &&
                   event.window.event == SDL_WINDOWEVENT_CLOSE) {
            open = false;
        }
    }
#endif
}
