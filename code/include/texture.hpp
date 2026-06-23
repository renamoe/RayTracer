#pragma once

#include "Vector2f.h"
#include "Vector3f.h"

#include <memory>
#include <string>
#include <vector>

class Texture {
public:
    static std::shared_ptr<Texture> load(const std::string &filename);

    Texture() = default;
    Texture(int width, int height, std::vector<unsigned char> pixels);
    Texture(int width, int height, std::vector<unsigned char> pixels, std::string filename);

    bool isValid() const;
    Vector3f sample(const Vector2f &uv) const;

    int getWidth() const;
    int getHeight() const;
    const std::string &getFilename() const;

private:
    int width = 0;
    int height = 0;
    std::vector<unsigned char> pixels;
    std::string filename;
};

std::string normalizeTexturePath(std::string path);
std::string resolveRelativePath(const std::string &baseFile, const std::string &relativePath);
