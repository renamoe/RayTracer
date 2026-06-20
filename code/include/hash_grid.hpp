#pragma once

#include "Vector3f.h"
#include "path_vertex.hpp"

#include <cstddef>
#include <vector>
#include <unordered_map>

struct Int3 {
    int x, y, z;

    bool operator==(const Int3 &other) const {
        return x == other.x && y == other.y && z == other.z;
    }
};

struct Int3Hash {
    static uint64_t splitmix64(uint64_t x) {
        x += 0x9e3779b97f4a7c15ull;
        x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ull;
        x = (x ^ (x >> 27)) * 0x94d049bb133111ebull;
        return x ^ (x >> 31);
    }

    size_t operator()(const Int3& c) const {
        uint64_t hx = splitmix64(static_cast<uint64_t>(static_cast<int64_t>(c.x)));
        uint64_t hy = splitmix64(static_cast<uint64_t>(static_cast<int64_t>(c.y)));
        uint64_t hz = splitmix64(static_cast<uint64_t>(static_cast<int64_t>(c.z)));
        return static_cast<size_t>(hx ^ (hy << 1) ^ (hz << 2));
    }
};

class HashGrid {
public:
    HashGrid() = default;
    ~HashGrid() = default;

    void build(float radius, const std::vector<VCMPathVertex> &vertices);

    template<class Callback>
    void query(const Vector3f &p, Callback &&callback) const {
        Int3 cell = cellOf(p);
        for (int dx = -1; dx <= 1; ++dx) {
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dz = -1; dz <= 1; ++dz) {
                    Int3 c = {cell.x + dx, cell.y + dy, cell.z + dz};
                    auto it = buckets.find(c);
                    if (it == buckets.end()) {
                        continue;
                    }
                    for (size_t i : it->second) {
                        const VCMPathVertex &v = (*vertices)[i];
                        float dist2 = (v.pos - p).squaredLength();
                        if (dist2 < radius2) {
                            callback(i, v);
                        }
                    }
                }
            }
        }
    }

private:
    Int3 cellOf(const Vector3f &p) const;
    bool isMergeable(const VCMPathVertex &p) const;

    float radius2;
    float cellSize;
    float invCellSize;
    const std::vector<VCMPathVertex> *vertices = nullptr;
    std::unordered_map<Int3, std::vector<size_t>, Int3Hash> buckets;
};
