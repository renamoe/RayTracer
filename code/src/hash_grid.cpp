#include "hash_grid.hpp"

#include <cmath>

void HashGrid::build(float radius, const std::vector<PhotonVertex> &photons) {
    radius2 = radius * radius;
    cellSize = radius;
    invCellSize = 1.0f / radius;

    vertices = &photons;
    buckets.clear();
    buckets.reserve(photons.size() * 2);

    for (size_t i = 0; i < photons.size(); ++i) {
        const PhotonVertex &v = photons[i];
        if (!isMergeable(v)) {
            continue;
        }
        Int3 cell = cellOf(v.pos);
        buckets[cell].push_back(i);
    }
}


Int3 HashGrid::cellOf(const Vector3f &p) const {
    int x = static_cast<int>(std::floor(p.x() * invCellSize));
    int y = static_cast<int>(std::floor(p.y() * invCellSize));
    int z = static_cast<int>(std::floor(p.z() * invCellSize));
    return {x, y, z};
}

bool HashGrid::isMergeable(const PhotonVertex &p) const {
    return !p.isDelta;
}

