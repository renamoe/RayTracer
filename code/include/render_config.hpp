#pragma once

#include <string>

constexpr int DEFAULT_NUM_SAMPLES = 32;
constexpr float DEFAULT_EXPOSURE = 1.5f;

constexpr int DEFAULT_BDPT_PRIMARY_DIRECT_LIGHT_SAMPLES = 1;
constexpr int DEFAULT_BDPT_SECONDARY_DIRECT_LIGHT_SAMPLES = 1;

constexpr float DEFAULT_VCM_RADIUS = 0.01f;
constexpr int DEFAULT_VCM_CAMERA_PATH_DEPTH = 8;
constexpr int DEFAULT_VCM_LIGHT_PATH_DEPTH = 5;
constexpr float DEFAULT_SAMPLE_CLAMP = 0.0f;
constexpr int DEFAULT_PATH_GUIDING_TRAINING_SPP = 32;
constexpr int DEFAULT_PATH_GUIDING_GRID_RESOLUTION = 8;
constexpr int DEFAULT_PATH_GUIDING_MAP_RESOLUTION = 32;
constexpr float DEFAULT_PATH_GUIDING_PROBABILITY = 0.5f;
constexpr float DEFAULT_PATH_GUIDING_FORGET = 0.98f;

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
    float sampleClamp = DEFAULT_SAMPLE_CLAMP;
    bool preview = false;
    int previewEveryIterations = 1;

    int bdptPrimaryDirectLightSamples = DEFAULT_BDPT_PRIMARY_DIRECT_LIGHT_SAMPLES;
    int bdptSecondaryDirectLightSamples = DEFAULT_BDPT_SECONDARY_DIRECT_LIGHT_SAMPLES;

    float vcmRadius = DEFAULT_VCM_RADIUS;
    int vcmCameraPathDepth = DEFAULT_VCM_CAMERA_PATH_DEPTH;
    int vcmLightPathDepth = DEFAULT_VCM_LIGHT_PATH_DEPTH;
    int vcmLightPathCount = 0;
    bool vcmCausticOnlyMerging = true;

    bool pathGuiding = false;
    int pathGuidingTrainingSpp = DEFAULT_PATH_GUIDING_TRAINING_SPP;
    int pathGuidingGridResolution = DEFAULT_PATH_GUIDING_GRID_RESOLUTION;
    int pathGuidingMapResolution = DEFAULT_PATH_GUIDING_MAP_RESOLUTION;
    float pathGuidingProbability = DEFAULT_PATH_GUIDING_PROBABILITY;
    float pathGuidingForget = DEFAULT_PATH_GUIDING_FORGET;

    IntegratorType integrator = IntegratorType::PT;
};

bool parseRenderConfig(int argc, char *argv[], RenderConfig &config);
