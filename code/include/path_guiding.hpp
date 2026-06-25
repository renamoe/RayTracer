#pragma once

#include "Vector3f.h"
#include "aabb.hpp"

#include <atomic>
#include <cstddef>
#include <memory>
#include <mutex>
#include <vector>

struct PathGuideSample {
    Vector3f dir = Vector3f::ZERO;
    float pdf = 0.0f;
    bool valid = false;
};

class GuideMap {
public:
    void initialize(int resolution);

    PathGuideSample sample(const std::vector<float> &solidAngle) const;
    float pdf(const Vector3f &dir, const std::vector<float> &solidAngle) const;
    void record(const Vector3f &dir, float value);
    void mergeTemp(float forget);

    bool hasSamples() const {
        return totalCount > 0.0f;
    }

    int resolution() const {
        return width;
    }

private:
    struct MipLevel {
        int width = 0;
        int height = 0;
        std::vector<float> weights;
    };

    void buildMip();

    int width = 0;
    int height = 0;
    float totalCount = 0.0f;

    std::vector<float> mean;
    std::vector<float> count;
    std::vector<float> tempSum;
    std::vector<float> tempCount;
    std::vector<MipLevel> mipLevels;
};

class PathGuideGrid {
public:
    PathGuideGrid() = default;
    PathGuideGrid(const AABB &sceneBounds, int gridResolution, int mapResolution);

    void initialize(const AABB &sceneBounds, int gridResolution, int mapResolution);

    bool isValid() const {
        return valid;
    }

    bool hasSamples(const Vector3f &pos, const Vector3f &normal) const;
    PathGuideSample sample(const Vector3f &pos, const Vector3f &normal) const;
    float pdf(const Vector3f &pos, const Vector3f &normal, const Vector3f &dir) const;
    void record(const Vector3f &pos,
                const Vector3f &normal,
                const Vector3f &dir,
                float value);
    void mergeTemp(float forget);

    int gridResolution() const {
        return nx;
    }

    int mapResolution() const {
        return mapRes;
    }

    std::size_t mapCount() const {
        return maps.size();
    }

private:
    int mapIndex(const Vector3f &pos, const Vector3f &normal) const;
    int normalFacet(const Vector3f &normal) const;
    void computeSolidAngles();
    void markDirty(int mapIdx);

    bool valid = false;
    AABB bounds;
    int nx = 0;
    int ny = 0;
    int nz = 0;
    int mapRes = 0;

    std::vector<float> solidAngle;
    std::vector<GuideMap> maps;
    std::vector<std::unique_ptr<std::mutex>> mapMutexes;
    std::vector<std::unique_ptr<std::atomic_bool>> dirtyFlags;
    std::vector<int> dirtyMapIndices;
    mutable std::mutex dirtyMapsMutex;
};
