#pragma once

#include <string>

constexpr int DEFAULT_NUM_SAMPLES = 32;
constexpr float DEFAULT_EXPOSURE = 1.5f;

constexpr int DEFAULT_BDPT_PRIMARY_DIRECT_LIGHT_SAMPLES = 1;
constexpr int DEFAULT_BDPT_SECONDARY_DIRECT_LIGHT_SAMPLES = 1;

constexpr float DEFAULT_VCM_RADIUS = 0.1f;

enum class IntegratorType {
    PT,
    BDPT,
    VCM
};

struct RenderConfig {
    std::string inputFile;
    std::string outputFile;
    int numSamples = DEFAULT_NUM_SAMPLES;
    float exposure = DEFAULT_EXPOSURE;

    int bdptPrimaryDirectLightSamples = DEFAULT_BDPT_PRIMARY_DIRECT_LIGHT_SAMPLES;
    int bdptSecondaryDirectLightSamples = DEFAULT_BDPT_SECONDARY_DIRECT_LIGHT_SAMPLES;

    float vcmRadius = DEFAULT_VCM_RADIUS;

    IntegratorType integrator = IntegratorType::PT;
};

bool parseRenderConfig(int argc, char *argv[], RenderConfig &config);
