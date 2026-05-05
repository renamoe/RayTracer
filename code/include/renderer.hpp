#pragma once

#include "image.hpp"
#include "render_config.hpp"

class SceneParser;

class Renderer {
public:
    Renderer(SceneParser &scene, const RenderConfig &config);

    Image render();

private:
    Image renderVCM();
    void printStats(double renderSeconds, long long numPixels, long long primarySamples) const;

    SceneParser &scene;
    RenderConfig config;
};
