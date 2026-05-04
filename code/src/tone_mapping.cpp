#include "tone_mapping.hpp"

#include <algorithm>
#include <cmath>

Vector3f toneMap(Vector3f color, float exposure) {
    color = color * exposure;
    color = Vector3f(
        1.0f - std::exp(-std::max(0.0f, color.x())),
        1.0f - std::exp(-std::max(0.0f, color.y())),
        1.0f - std::exp(-std::max(0.0f, color.z()))
    );
    return Vector3f(
        std::pow(color.x(), 1.0f / 2.2f),
        std::pow(color.y(), 1.0f / 2.2f),
        std::pow(color.z(), 1.0f / 2.2f)
    );
}
