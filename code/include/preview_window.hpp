#pragma once

#include "Vector3f.h"

#include <vector>

#ifdef RAYTRACER_ENABLE_PREVIEW
struct SDL_Renderer;
struct SDL_Texture;
struct SDL_Window;
#endif

class PreviewWindow {
public:
    PreviewWindow(int width, int height);
    ~PreviewWindow();

    PreviewWindow(const PreviewWindow &) = delete;
    PreviewWindow &operator=(const PreviewWindow &) = delete;

    bool isOpen() const;
    void pollEvents();

    void update(const std::vector<Vector3f> &accumulated,
                const std::vector<Vector3f> *splatAccumulated,
                int completedIterations,
                float exposure);

private:
    int width = 0;
    int height = 0;
    bool open = false;

#ifdef RAYTRACER_ENABLE_PREVIEW
    bool sdlInitialized = false;
    SDL_Window *window = nullptr;
    SDL_Renderer *renderer = nullptr;
    SDL_Texture *texture = nullptr;
    std::vector<unsigned char> pixels;
#endif
};
