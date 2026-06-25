#include "render_config.hpp"

#include <cmath>
#include <climits>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

namespace {

void printUsage() {
    std::cout << "Usage: ./bin/RayTracer <input scene file> <output bmp file> "
              << "[-n num_samples] [-e exposure] [-pt|-bdpt|-vcm] "
              << "[-t duration] "
              << "[--sample-clamp luminance] "
              << "[--preview] [--preview-every iterations] "
              << "[-vcm-radius radius] "
              << "[-vcm-camera-depth depth] "
              << "[-vcm-light-depth depth] "
              << "[--vcm-light-paths count] "
              << "[-vcm-caustic-only|-vcm-all-merging] "
              << "[-bdpt-direct samples] "
              << "[-bdpt-direct-primary samples] "
              << "[-bdpt-direct-secondary samples] "
              << "[--path-guiding] "
              << "[--pg-train-spp spp] "
              << "[--pg-prob probability] "
              << "[--pg-grid resolution] "
              << "[--pg-map resolution] "
              << "[--pg-forget factor]" << std::endl;
}

bool parsePositiveInt(const char *text, int &value) {
    char *end = nullptr;
    long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed <= 0 || parsed > INT_MAX) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool parseNonNegativeInt(const char *text, int &value) {
    char *end = nullptr;
    long parsed = std::strtol(text, &end, 10);
    if (end == text || *end != '\0' || parsed < 0 || parsed > INT_MAX) {
        return false;
    }
    value = static_cast<int>(parsed);
    return true;
}

bool parsePositiveFloat(const char *text, float &value) {
    char *end = nullptr;
    float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0f) {
        return false;
    }
    value = parsed;
    return true;
}

bool parseNonNegativeFloat(const char *text, float &value) {
    char *end = nullptr;
    float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(parsed) || parsed < 0.0f) {
        return false;
    }
    value = parsed;
    return true;
}

bool parseUnitFloat(const char *text, float &value) {
    char *end = nullptr;
    float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(parsed) ||
        parsed < 0.0f || parsed > 1.0f) {
        return false;
    }
    value = parsed;
    return true;
}

bool parsePositiveDurationSeconds(const char *text, double &seconds) {
    char *end = nullptr;
    double parsed = std::strtod(text, &end);
    if (end == text || !std::isfinite(parsed) || parsed <= 0.0) {
        return false;
    }

    double scale = 1.0;
    if (*end != '\0') {
        std::string unit(end);
        if (unit == "s" || unit == "sec" || unit == "secs") {
            scale = 1.0;
        } else if (unit == "m" || unit == "min" || unit == "mins") {
            scale = 60.0;
        } else if (unit == "h" || unit == "hr" || unit == "hrs") {
            scale = 3600.0;
        } else {
            return false;
        }
    }

    seconds = parsed * scale;
    return std::isfinite(seconds) && seconds > 0.0;
}

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

bool parseRenderConfig(int argc, char *argv[], RenderConfig &config) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    std::vector<const char *> positionalArgs;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "-n") {
            if (i + 1 >= argc || !parsePositiveInt(argv[++i], config.numSamples)) {
                std::cout << "num samples must be a positive integer" << std::endl;
                return false;
            }
        } else if (arg == "-e") {
            if (i + 1 >= argc || !parsePositiveFloat(argv[++i], config.exposure)) {
                std::cout << "exposure must be a positive number" << std::endl;
                return false;
            }
        } else if (arg == "--sample-clamp" || arg == "-sample-clamp") {
            if (i + 1 >= argc || !parseNonNegativeFloat(argv[++i], config.sampleClamp)) {
                std::cout << "sample clamp must be a non-negative number" << std::endl;
                return false;
            }
        } else if (arg == "-t" || arg == "--time") {
            if (i + 1 >= argc ||
                !parsePositiveDurationSeconds(argv[++i], config.timeLimitSeconds)) {
                std::cout << "time limit must be a positive duration, e.g. 100s, 2m, or 1.5h" << std::endl;
                return false;
            }
        } else if (arg == "--preview" || arg == "-preview") {
            config.preview = true;
        } else if (arg == "--preview-every" || arg == "-preview-every") {
            if (i + 1 >= argc ||
                !parsePositiveInt(argv[++i], config.previewEveryIterations)) {
                std::cout << "preview update interval must be a positive integer" << std::endl;
                return false;
            }
            config.preview = true;
        } else if (arg == "-pt") {
            config.integrator = IntegratorType::PT;
        } else if (arg == "-bdpt") {
            config.integrator = IntegratorType::BDPT;
        } else if (arg == "-vcm") {
            config.integrator = IntegratorType::VCM;
        } else if (arg == "--path-guiding" || arg == "-path-guiding") {
            config.pathGuiding = true;
        } else if (arg == "--pg-train-spp" || arg == "-pg-train-spp") {
            if (i + 1 >= argc ||
                !parseNonNegativeInt(argv[++i], config.pathGuidingTrainingSpp)) {
                std::cout << "path guiding training spp must be a non-negative integer" << std::endl;
                return false;
            }
            config.pathGuiding = true;
        } else if (arg == "--pg-prob" || arg == "-pg-prob") {
            if (i + 1 >= argc ||
                !parseUnitFloat(argv[++i], config.pathGuidingProbability)) {
                std::cout << "path guiding probability must be in [0, 1]" << std::endl;
                return false;
            }
            config.pathGuiding = true;
        } else if (arg == "--pg-grid" || arg == "-pg-grid") {
            if (i + 1 >= argc ||
                !parsePositiveInt(argv[++i], config.pathGuidingGridResolution)) {
                std::cout << "path guiding grid resolution must be a positive integer" << std::endl;
                return false;
            }
            config.pathGuiding = true;
        } else if (arg == "--pg-map" || arg == "-pg-map") {
            if (i + 1 >= argc ||
                !parsePositiveInt(argv[++i], config.pathGuidingMapResolution)) {
                std::cout << "path guiding map resolution must be a positive integer" << std::endl;
                return false;
            }
            config.pathGuiding = true;
        } else if (arg == "--pg-forget" || arg == "-pg-forget") {
            if (i + 1 >= argc ||
                !parseUnitFloat(argv[++i], config.pathGuidingForget)) {
                std::cout << "path guiding forget factor must be in [0, 1]" << std::endl;
                return false;
            }
            config.pathGuiding = true;
        } else if (arg == "-vcm-radius" || arg == "--vcm-radius") {
            if (i + 1 >= argc || !parsePositiveFloat(argv[++i], config.vcmRadius)) {
                std::cout << "vcm radius must be a positive number" << std::endl;
                return false;
            }
        } else if (arg == "-vcm-camera-depth" || arg == "--vcm-camera-depth") {
            if (i + 1 >= argc || !parsePositiveInt(argv[++i], config.vcmCameraPathDepth)) {
                std::cout << "vcm camera depth must be a positive integer" << std::endl;
                return false;
            }
        } else if (arg == "-vcm-light-depth" || arg == "--vcm-light-depth") {
            if (i + 1 >= argc || !parsePositiveInt(argv[++i], config.vcmLightPathDepth)) {
                std::cout << "vcm light depth must be a positive integer" << std::endl;
                return false;
            }
        } else if (arg == "-vcm-light-paths" || arg == "--vcm-light-paths") {
            if (i + 1 >= argc ||
                !parseNonNegativeInt(argv[++i], config.vcmLightPathCount)) {
                std::cout << "vcm light paths must be a non-negative integer" << std::endl;
                return false;
            }
        } else if (arg == "-vcm-caustic-only" || arg == "--vcm-caustic-only") {
            config.vcmCausticOnlyMerging = true;
        } else if (arg == "-vcm-all-merging" || arg == "--vcm-all-merging") {
            config.vcmCausticOnlyMerging = false;
        } else if (arg == "-bdpt-direct") {
            int samples = 0;
            if (i + 1 >= argc || !parseNonNegativeInt(argv[++i], samples)) {
                std::cout << "bdpt direct light samples must be a non-negative integer" << std::endl;
                return false;
            }
            config.bdptPrimaryDirectLightSamples = samples;
            config.bdptSecondaryDirectLightSamples = samples;
        } else if (arg == "-bdpt-direct-primary") {
            if (i + 1 >= argc ||
                !parseNonNegativeInt(argv[++i], config.bdptPrimaryDirectLightSamples)) {
                std::cout << "bdpt primary direct light samples must be a non-negative integer" << std::endl;
                return false;
            }
        } else if (arg == "-bdpt-direct-secondary") {
            if (i + 1 >= argc ||
                !parseNonNegativeInt(argv[++i], config.bdptSecondaryDirectLightSamples)) {
                std::cout << "bdpt secondary direct light samples must be a non-negative integer" << std::endl;
                return false;
            }
        } else {
            if (!arg.empty() && arg[0] == '-') {
                std::cout << "unknown option: " << arg << std::endl;
                printUsage();
                return false;
            }
            positionalArgs.push_back(argv[i]);
        }
    }

    if (positionalArgs.size() != 2) {
        printUsage();
        return false;
    }

    config.inputFile = positionalArgs[0];
    config.outputFile = positionalArgs[1];

    if (config.timeLimitSeconds > 0.0) {
        std::cout << "num samples per pixel: time-limited" << std::endl;
        std::cout << "time limit: " << config.timeLimitSeconds << " s" << std::endl;
    } else {
        std::cout << "num samples per pixel: " << config.numSamples << std::endl;
    }
    std::cout << "exposure: " << config.exposure << std::endl;
    if (config.sampleClamp > 0.0f) {
        std::cout << "sample clamp luminance: " << config.sampleClamp << std::endl;
    }
    std::cout << "integrator: " << integratorName(config.integrator) << std::endl;
    if (config.pathGuiding) {
        std::cout << "path guiding: enabled for PT, training spp="
                  << config.pathGuidingTrainingSpp
                  << ", probability=" << config.pathGuidingProbability
                  << ", grid=" << config.pathGuidingGridResolution
                  << "^3, map=" << config.pathGuidingMapResolution
                  << "x" << config.pathGuidingMapResolution
                  << ", forget=" << config.pathGuidingForget << std::endl;
    }
    if (config.preview) {
        std::cout << "preview: every " << config.previewEveryIterations
                  << " iteration(s)" << std::endl;
    }
    if (config.integrator == IntegratorType::BDPT ||
        config.integrator == IntegratorType::VCM) {
        std::cout << integratorName(config.integrator)
                  << " direct light samples: primary="
                  << config.bdptPrimaryDirectLightSamples
                  << ", secondary="
                  << config.bdptSecondaryDirectLightSamples << std::endl;
    }
    if (config.integrator == IntegratorType::VCM) {
        std::cout << "vcm base radius: " << config.vcmRadius << std::endl;
        std::cout << "vcm depth: camera=" << config.vcmCameraPathDepth
                  << ", light=" << config.vcmLightPathDepth << std::endl;
        std::cout << "vcm light paths: ";
        if (config.vcmLightPathCount > 0) {
            std::cout << config.vcmLightPathCount;
        } else {
            std::cout << "auto";
        }
        std::cout << std::endl;
        std::cout << "vcm merging: "
                  << (config.vcmCausticOnlyMerging ? "caustic-only" : "all")
                  << std::endl;
    }
    return true;
}
