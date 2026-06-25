#include "path_guiding.hpp"

#include "random.hpp"

#include <algorithm>
#include <cmath>

namespace {

constexpr float GUIDE_EPSILON_WEIGHT = 1e-4f;
constexpr int NORMAL_FACET_COUNT = 6;

struct GuideUV {
    float u = 0.0f;
    float v = 0.0f;
};

float clamp01(float x) {
    return std::max(0.0f, std::min(1.0f, x));
}

int clampInt(int x, int lo, int hi) {
    return std::max(lo, std::min(hi, x));
}

float luminanceLikeWeight(float x) {
    if (!std::isfinite(x) || x <= 0.0f) {
        return 0.0f;
    }
    return x;
}

GuideUV octEncode(Vector3f dir) {
    float norm = std::abs(dir.x()) + std::abs(dir.y()) + std::abs(dir.z());
    if (norm <= 1e-12f) {
        return {0.5f, 0.5f};
    }

    dir = dir / norm;
    float x = dir.x();
    float y = dir.y();
    float z = dir.z();

    if (z < 0.0f) {
        float oldX = x;
        float oldY = y;
        x = (1.0f - std::abs(oldY)) * (oldX >= 0.0f ? 1.0f : -1.0f);
        y = (1.0f - std::abs(oldX)) * (oldY >= 0.0f ? 1.0f : -1.0f);
    }

    return {clamp01(x * 0.5f + 0.5f), clamp01(y * 0.5f + 0.5f)};
}

Vector3f octDecode(float u, float v) {
    float x = u * 2.0f - 1.0f;
    float y = v * 2.0f - 1.0f;
    float z = 1.0f - std::abs(x) - std::abs(y);

    if (z < 0.0f) {
        float oldX = x;
        float oldY = y;
        x = (1.0f - std::abs(oldY)) * (oldX >= 0.0f ? 1.0f : -1.0f);
        y = (1.0f - std::abs(oldX)) * (oldY >= 0.0f ? 1.0f : -1.0f);
    }

    Vector3f dir(x, y, z);
    if (dir.squaredLength() <= 1e-12f) {
        return Vector3f(0, 0, 1);
    }
    return dir.normalized();
}

Vector3f uniformSampleSphere(float u1, float u2) {
    float z = 1.0f - 2.0f * u1;
    float r = std::sqrt(std::max(0.0f, 1.0f - z * z));
    float phi = 2.0f * M_PI * u2;
    return Vector3f(r * std::cos(phi), r * std::sin(phi), z);
}

float sphericalTriangleArea(const Vector3f &a, const Vector3f &b, const Vector3f &c) {
    float det = Vector3f::dot(a, Vector3f::cross(b, c));
    float denom = 1.0f + Vector3f::dot(a, b) +
                  Vector3f::dot(b, c) + Vector3f::dot(c, a);
    float area = 2.0f * std::atan2(std::abs(det), denom);
    return std::isfinite(area) ? std::abs(area) : 0.0f;
}

float texelSolidAngle(int x, int y, int width, int height) {
    float u0 = static_cast<float>(x) / static_cast<float>(width);
    float v0 = static_cast<float>(y) / static_cast<float>(height);
    float u1 = static_cast<float>(x + 1) / static_cast<float>(width);
    float v1 = static_cast<float>(y + 1) / static_cast<float>(height);

    Vector3f d00 = octDecode(u0, v0);
    Vector3f d10 = octDecode(u1, v0);
    Vector3f d01 = octDecode(u0, v1);
    Vector3f d11 = octDecode(u1, v1);

    return sphericalTriangleArea(d00, d10, d11) +
           sphericalTriangleArea(d00, d11, d01);
}

} // namespace

void GuideMap::initialize(int resolution) {
    width = std::max(4, resolution);
    height = width;

    std::size_t texelCount = static_cast<std::size_t>(width) * height;
    mean.assign(texelCount, 0.0f);
    count.assign(texelCount, 0.0f);
    tempSum.assign(texelCount, 0.0f);
    tempCount.assign(texelCount, 0.0f);
    totalCount = 0.0f;
    buildMip();
}

PathGuideSample GuideMap::sample(const std::vector<float> &solidAngle) const {
    if (mipLevels.empty() || width <= 0 || height <= 0) {
        float u1 = Random::get_float();
        float u2 = Random::get_float();
        return {uniformSampleSphere(u1, u2), 1.0f / (4.0f * M_PI), true};
    }

    const MipLevel &root = mipLevels.back();
    float total = root.weights.empty() ? 0.0f : root.weights[0];
    if (!std::isfinite(total) || total <= 0.0f) {
        float u1 = Random::get_float();
        float u2 = Random::get_float();
        return {uniformSampleSphere(u1, u2), 1.0f / (4.0f * M_PI), true};
    }

    int x = 0;
    int y = 0;
    for (int level = static_cast<int>(mipLevels.size()) - 2; level >= 0; --level) {
        const MipLevel &childLevel = mipLevels[level];

        int childX[4];
        int childY[4];
        float childWeight[4];
        int childCount = 0;
        float childSum = 0.0f;

        for (int dy = 0; dy < 2; ++dy) {
            for (int dx = 0; dx < 2; ++dx) {
                int cx = 2 * x + dx;
                int cy = 2 * y + dy;
                if (cx >= childLevel.width || cy >= childLevel.height) {
                    continue;
                }
                float w = childLevel.weights[static_cast<std::size_t>(cy) *
                                             childLevel.width + cx];
                childX[childCount] = cx;
                childY[childCount] = cy;
                childWeight[childCount] = std::max(0.0f, w);
                childSum += childWeight[childCount];
                ++childCount;
            }
        }

        if (childCount <= 0) {
            break;
        }

        int chosen = 0;
        if (childSum > 0.0f) {
            float r = Random::get_float() * childSum;
            float acc = 0.0f;
            for (int i = 0; i < childCount; ++i) {
                acc += childWeight[i];
                if (r <= acc) {
                    chosen = i;
                    break;
                }
            }
        } else {
            chosen = clampInt(static_cast<int>(Random::get_float() * childCount),
                              0,
                              childCount - 1);
        }

        x = childX[chosen];
        y = childY[chosen];
    }

    float u = (static_cast<float>(x) + Random::get_float()) / static_cast<float>(width);
    float v = (static_cast<float>(y) + Random::get_float()) / static_cast<float>(height);
    Vector3f dir = octDecode(u, v);
    float samplePdf = pdf(dir, solidAngle);
    return {dir, samplePdf, samplePdf > 0.0f};
}

float GuideMap::pdf(const Vector3f &dir, const std::vector<float> &solidAngle) const {
    if (mipLevels.empty() || width <= 0 || height <= 0 ||
        solidAngle.size() != static_cast<std::size_t>(width) * height) {
        return 1.0f / (4.0f * M_PI);
    }
    if (dir.squaredLength() <= 1e-12f) {
        return 1.0f / (4.0f * M_PI);
    }

    const MipLevel &root = mipLevels.back();
    float total = root.weights.empty() ? 0.0f : root.weights[0];
    if (!std::isfinite(total) || total <= 0.0f) {
        return 1.0f / (4.0f * M_PI);
    }

    GuideUV uv = octEncode(dir);
    int x = clampInt(static_cast<int>(uv.u * width), 0, width - 1);
    int y = clampInt(static_cast<int>(uv.v * height), 0, height - 1);
    std::size_t idx = static_cast<std::size_t>(y) * width + x;

    float texelWeight = std::max(0.0f, mean[idx]) + GUIDE_EPSILON_WEIGHT;
    float omega = std::max(solidAngle[idx], 1e-8f);
    return (texelWeight / total) / omega;
}

void GuideMap::record(const Vector3f &dir, float value) {
    value = luminanceLikeWeight(value);
    if (value <= 0.0f || width <= 0 || height <= 0 ||
        dir.squaredLength() <= 1e-12f) {
        return;
    }

    GuideUV uv = octEncode(dir);
    int x = clampInt(static_cast<int>(uv.u * width), 0, width - 1);
    int y = clampInt(static_cast<int>(uv.v * height), 0, height - 1);
    std::size_t idx = static_cast<std::size_t>(y) * width + x;

    tempSum[idx] += value;
    tempCount[idx] += 1.0f;
}

void GuideMap::mergeTemp(float forget) {
    if (width <= 0 || height <= 0) {
        return;
    }

    forget = std::max(0.0f, std::min(0.999f, forget));
    totalCount = 0.0f;

    for (std::size_t i = 0; i < mean.size(); ++i) {
        if (tempCount[i] > 0.0f) {
            float observed = tempSum[i] / tempCount[i];
            if (count[i] <= 0.0f) {
                mean[i] = observed;
            } else {
                mean[i] = forget * mean[i] + (1.0f - forget) * observed;
            }
            count[i] = std::min(forget * count[i] + tempCount[i], 1024.0f);
        } else {
            count[i] *= forget;
        }

        totalCount += count[i];
        tempSum[i] = 0.0f;
        tempCount[i] = 0.0f;
    }

    buildMip();
}

void GuideMap::buildMip() {
    mipLevels.clear();
    if (width <= 0 || height <= 0) {
        return;
    }

    MipLevel base;
    base.width = width;
    base.height = height;
    base.weights.resize(static_cast<std::size_t>(width) * height);
    for (std::size_t i = 0; i < base.weights.size(); ++i) {
        base.weights[i] = std::max(0.0f, mean[i]) + GUIDE_EPSILON_WEIGHT;
    }
    mipLevels.push_back(base);

    int currentLevel = 0;
    while (mipLevels[currentLevel].width > 1 || mipLevels[currentLevel].height > 1) {
        const MipLevel &child = mipLevels[currentLevel];
        MipLevel parent;
        parent.width = std::max(1, (child.width + 1) / 2);
        parent.height = std::max(1, (child.height + 1) / 2);
        parent.weights.assign(
            static_cast<std::size_t>(parent.width) * parent.height,
            0.0f
        );

        for (int y = 0; y < parent.height; ++y) {
            for (int x = 0; x < parent.width; ++x) {
                float sum = 0.0f;
                for (int dy = 0; dy < 2; ++dy) {
                    for (int dx = 0; dx < 2; ++dx) {
                        int cx = 2 * x + dx;
                        int cy = 2 * y + dy;
                        if (cx >= child.width || cy >= child.height) {
                            continue;
                        }
                        sum += child.weights[static_cast<std::size_t>(cy) *
                                             child.width + cx];
                    }
                }
                parent.weights[static_cast<std::size_t>(y) * parent.width + x] = sum;
            }
        }

        mipLevels.push_back(parent);
        ++currentLevel;
    }
}

PathGuideGrid::PathGuideGrid(const AABB &sceneBounds,
                             int gridResolution,
                             int mapResolution) {
    initialize(sceneBounds, gridResolution, mapResolution);
}

void PathGuideGrid::initialize(const AABB &sceneBounds,
                               int gridResolution,
                               int mapResolution) {
    nx = ny = nz = std::max(1, gridResolution);
    mapRes = std::max(4, mapResolution);
    bounds = sceneBounds;

    Vector3f diag = bounds.max - bounds.min;
    for (int dim = 0; dim < 3; ++dim) {
        float extent = diag[dim];
        if (!std::isfinite(extent) || extent <= 1e-4f) {
            bounds.min[dim] -= 0.5f;
            bounds.max[dim] += 0.5f;
        } else {
            float pad = std::max(1e-4f, extent * 1e-4f);
            bounds.min[dim] -= pad;
            bounds.max[dim] += pad;
        }
    }

    computeSolidAngles();

    std::size_t totalMaps = static_cast<std::size_t>(nx) * ny * nz * NORMAL_FACET_COUNT;
    maps.resize(totalMaps);
    for (GuideMap &map : maps) {
        map.initialize(mapRes);
    }

    valid = !maps.empty() && !solidAngle.empty();
}

bool PathGuideGrid::hasSamples(const Vector3f &pos, const Vector3f &normal) const {
    if (!valid) {
        return false;
    }
    int idx = mapIndex(pos, normal);
    return idx >= 0 && idx < static_cast<int>(maps.size()) && maps[idx].hasSamples();
}

PathGuideSample PathGuideGrid::sample(const Vector3f &pos, const Vector3f &normal) const {
    if (!valid) {
        float u1 = Random::get_float();
        float u2 = Random::get_float();
        return {uniformSampleSphere(u1, u2), 1.0f / (4.0f * M_PI), true};
    }

    int idx = mapIndex(pos, normal);
    if (idx < 0 || idx >= static_cast<int>(maps.size())) {
        float u1 = Random::get_float();
        float u2 = Random::get_float();
        return {uniformSampleSphere(u1, u2), 1.0f / (4.0f * M_PI), true};
    }
    return maps[idx].sample(solidAngle);
}

float PathGuideGrid::pdf(const Vector3f &pos,
                         const Vector3f &normal,
                         const Vector3f &dir) const {
    if (!valid) {
        return 1.0f / (4.0f * M_PI);
    }

    int idx = mapIndex(pos, normal);
    if (idx < 0 || idx >= static_cast<int>(maps.size())) {
        return 1.0f / (4.0f * M_PI);
    }
    return maps[idx].pdf(dir, solidAngle);
}

void PathGuideGrid::record(const Vector3f &pos,
                           const Vector3f &normal,
                           const Vector3f &dir,
                           float value) {
    if (!valid || !std::isfinite(value) || value <= 0.0f) {
        return;
    }

    int idx = mapIndex(pos, normal);
    if (idx < 0 || idx >= static_cast<int>(maps.size())) {
        return;
    }

    std::lock_guard<std::mutex> lock(recordMutex);
    maps[idx].record(dir, value);
}

void PathGuideGrid::mergeTemp(float forget) {
    if (!valid) {
        return;
    }

    std::lock_guard<std::mutex> lock(recordMutex);
    for (GuideMap &map : maps) {
        map.mergeTemp(forget);
    }
}

int PathGuideGrid::mapIndex(const Vector3f &pos, const Vector3f &normal) const {
    if (nx <= 0 || ny <= 0 || nz <= 0) {
        return -1;
    }

    Vector3f diag = bounds.max - bounds.min;
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (diag.x() > 0.0f) {
        ix = clampInt(static_cast<int>(((pos.x() - bounds.min.x()) / diag.x()) * nx), 0, nx - 1);
    }
    if (diag.y() > 0.0f) {
        iy = clampInt(static_cast<int>(((pos.y() - bounds.min.y()) / diag.y()) * ny), 0, ny - 1);
    }
    if (diag.z() > 0.0f) {
        iz = clampInt(static_cast<int>(((pos.z() - bounds.min.z()) / diag.z()) * nz), 0, nz - 1);
    }

    int facet = normalFacet(normal);
    return (((iz * ny + iy) * nx + ix) * NORMAL_FACET_COUNT) + facet;
}

int PathGuideGrid::normalFacet(const Vector3f &normal) const {
    Vector3f n = normal;
    if (n.squaredLength() <= 1e-12f) {
        return 4;
    }
    n = n.normalized();
    Vector3f a(std::abs(n.x()), std::abs(n.y()), std::abs(n.z()));

    if (a.x() > a.y() && a.x() > a.z()) {
        return n.x() >= 0.0f ? 0 : 1;
    }
    if (a.y() > a.z()) {
        return n.y() >= 0.0f ? 2 : 3;
    }
    return n.z() >= 0.0f ? 4 : 5;
}

void PathGuideGrid::computeSolidAngles() {
    solidAngle.assign(static_cast<std::size_t>(mapRes) * mapRes, 0.0f);
    for (int y = 0; y < mapRes; ++y) {
        for (int x = 0; x < mapRes; ++x) {
            solidAngle[static_cast<std::size_t>(y) * mapRes + x] =
                std::max(texelSolidAngle(x, y, mapRes, mapRes), 1e-8f);
        }
    }
}
