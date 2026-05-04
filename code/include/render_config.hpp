#pragma once

#include <string>

constexpr int DEFAULT_NUM_SAMPLES = 32;
constexpr float DEFAULT_EXPOSURE = 1.5f;

struct RenderConfig {
    std::string inputFile;
    std::string outputFile;
    int numSamples = DEFAULT_NUM_SAMPLES;
    float exposure = DEFAULT_EXPOSURE;
};

bool parseRenderConfig(int argc, char *argv[], RenderConfig &config);
