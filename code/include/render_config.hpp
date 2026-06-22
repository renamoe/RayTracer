#pragma once

#include <string>

constexpr int DEFAULT_NUM_SAMPLES = 32;
constexpr float DEFAULT_EXPOSURE = 1.5f;

constexpr int DEFAULT_BDPT_PRIMARY_DIRECT_LIGHT_SAMPLES = 1;
constexpr int DEFAULT_BDPT_SECONDARY_DIRECT_LIGHT_SAMPLES = 1;

constexpr float DEFAULT_VCM_RADIUS = 0.01f;
constexpr int DEFAULT_VCM_CAMERA_PATH_DEPTH = 8;
constexpr int DEFAULT_VCM_LIGHT_PATH_DEPTH = 5;

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
    double timeLimitSeconds = 0.0;

    int bdptPrimaryDirectLightSamples = DEFAULT_BDPT_PRIMARY_DIRECT_LIGHT_SAMPLES;
    int bdptSecondaryDirectLightSamples = DEFAULT_BDPT_SECONDARY_DIRECT_LIGHT_SAMPLES;

    float vcmRadius = DEFAULT_VCM_RADIUS;
    int vcmCameraPathDepth = DEFAULT_VCM_CAMERA_PATH_DEPTH;
    int vcmLightPathDepth = DEFAULT_VCM_LIGHT_PATH_DEPTH;
    bool vcmCausticOnlyMerging = true;

    IntegratorType integrator = IntegratorType::PT;
};

bool parseRenderConfig(int argc, char *argv[], RenderConfig &config);
