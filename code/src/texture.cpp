#include "texture.hpp"

#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>

namespace {

float clamp01(float v) {
    return std::max(0.0f, std::min(1.0f, v));
}

float srgbToLinear(float c) {
    c = clamp01(c);
    if (c <= 0.04045f) {
        return c / 12.92f;
    }
    return std::pow((c + 0.055f) / 1.055f, 2.4f);
}

float wrap01(float v) {
    v = v - std::floor(v);
    return v < 0.0f ? v + 1.0f : v;
}

bool isAbsolutePath(const std::string &path) {
    if (path.empty()) {
        return false;
    }
    if (path[0] == '/') {
        return true;
    }
    return path.size() > 2 &&
           std::isalpha(static_cast<unsigned char>(path[0])) &&
           path[1] == ':';
}

std::string directoryOf(const std::string &filename) {
    std::string::size_type pos = std::string::npos;
    for (std::string::size_type i = filename.size(); i > 0; --i) {
        char c = filename[i - 1];
        if (c == '/' || c == '\\') {
            pos = i - 1;
            break;
        }
    }
    if (pos == std::string::npos) {
        return "";
    }
    return filename.substr(0, pos + 1);
}

} // namespace

Texture::Texture(int width, int height, std::vector<Vector3f> pixels)
    : width(width), height(height), pixels(std::move(pixels)) {}

Texture::Texture(int width, int height, std::vector<Vector3f> pixels, std::string filename)
    : width(width), height(height), pixels(std::move(pixels)), filename(std::move(filename)) {}

std::shared_ptr<Texture> Texture::load(const std::string &filename) {
    std::string normalized = normalizeTexturePath(filename);

    int w = 0;
    int h = 0;
    int channels = 0;
    unsigned char *data = stbi_load(normalized.c_str(), &w, &h, &channels, 3);
    if (data == nullptr || w <= 0 || h <= 0) {
        std::cerr << "Failed to load texture '" << normalized
                  << "': " << stbi_failure_reason() << std::endl;
        if (data != nullptr) {
            stbi_image_free(data);
        }
        return nullptr;
    }

    std::vector<Vector3f> pixels;
    pixels.reserve(static_cast<size_t>(w) * static_cast<size_t>(h));
    for (int i = 0; i < w * h; ++i) {
        float r = data[3 * i + 0] / 255.0f;
        float g = data[3 * i + 1] / 255.0f;
        float b = data[3 * i + 2] / 255.0f;
        pixels.emplace_back(srgbToLinear(r), srgbToLinear(g), srgbToLinear(b));
    }
    stbi_image_free(data);

    return std::make_shared<Texture>(w, h, std::move(pixels), normalized);
}

bool Texture::isValid() const {
    return width > 0 && height > 0 &&
           pixels.size() == static_cast<size_t>(width) * static_cast<size_t>(height);
}

Vector3f Texture::sample(const Vector2f &uv) const {
    if (!isValid()) {
        return Vector3f(1, 1, 1);
    }

    float u = wrap01(uv.x());
    float v = wrap01(uv.y());

    float x = u * static_cast<float>(width - 1);
    float y = (1.0f - v) * static_cast<float>(height - 1);

    int x0 = static_cast<int>(std::floor(x));
    int y0 = static_cast<int>(std::floor(y));
    int x1 = std::min(x0 + 1, width - 1);
    int y1 = std::min(y0 + 1, height - 1);

    float tx = x - static_cast<float>(x0);
    float ty = y - static_cast<float>(y0);

    const Vector3f &c00 = pixels[static_cast<size_t>(y0) * width + x0];
    const Vector3f &c10 = pixels[static_cast<size_t>(y0) * width + x1];
    const Vector3f &c01 = pixels[static_cast<size_t>(y1) * width + x0];
    const Vector3f &c11 = pixels[static_cast<size_t>(y1) * width + x1];

    Vector3f cx0 = c00 * (1.0f - tx) + c10 * tx;
    Vector3f cx1 = c01 * (1.0f - tx) + c11 * tx;
    return cx0 * (1.0f - ty) + cx1 * ty;
}

int Texture::getWidth() const {
    return width;
}

int Texture::getHeight() const {
    return height;
}

const std::string &Texture::getFilename() const {
    return filename;
}

std::string normalizeTexturePath(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    return path;
}

std::string resolveRelativePath(const std::string &baseFile, const std::string &relativePath) {
    std::string normalized = normalizeTexturePath(relativePath);
    if (normalized.empty() || isAbsolutePath(normalized)) {
        return normalized;
    }
    return normalizeTexturePath(directoryOf(baseFile) + normalized);
}
