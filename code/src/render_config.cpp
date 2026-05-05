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
              << "[-n num_samples] [-e exposure] [-pt|-bdpt]" << std::endl;
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

bool parsePositiveFloat(const char *text, float &value) {
    char *end = nullptr;
    float parsed = std::strtof(text, &end);
    if (end == text || *end != '\0' || !std::isfinite(parsed) || parsed <= 0.0f) {
        return false;
    }
    value = parsed;
    return true;
}

const char *integratorName(IntegratorType integrator) {
    return integrator == IntegratorType::BDPT ? "bdpt" : "pt";
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
        } else if (arg == "-pt") {
            config.integrator = IntegratorType::PT;
        } else if (arg == "-bdpt") {
            config.integrator = IntegratorType::BDPT;
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

    std::cout << "num samples per pixel: " << config.numSamples << std::endl;
    std::cout << "exposure: " << config.exposure << std::endl;
    std::cout << "integrator: " << integratorName(config.integrator) << std::endl;
    return true;
}
