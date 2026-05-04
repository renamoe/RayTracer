#include "render_config.hpp"

#include <cmath>
#include <cstdlib>
#include <iostream>

bool parseRenderConfig(int argc, char *argv[], RenderConfig &config) {
    for (int argNum = 1; argNum < argc; ++argNum) {
        std::cout << "Argument " << argNum << " is: " << argv[argNum] << std::endl;
    }

    if (argc < 3 || argc > 5) {
        std::cout << "Usage: ./bin/RayTracer <input scene file> <output bmp file> [num_samples] [exposure]" << std::endl;
        return false;
    }

    config.inputFile = argv[1];
    config.outputFile = argv[2];
    config.numSamples = argc >= 4 ? std::atoi(argv[3]) : DEFAULT_NUM_SAMPLES;
    if (config.numSamples <= 0) {
        std::cout << "num samples must be a positive integer" << std::endl;
        return false;
    }

    config.exposure = argc >= 5 ? std::atof(argv[4]) : DEFAULT_EXPOSURE;
    if (!std::isfinite(config.exposure) || config.exposure <= 0.0f) {
        std::cout << "exposure must be a positive number" << std::endl;
        return false;
    }

    std::cout << "num samples per pixel: " << config.numSamples << std::endl;
    std::cout << "exposure: " << config.exposure << std::endl;
    return true;
}
