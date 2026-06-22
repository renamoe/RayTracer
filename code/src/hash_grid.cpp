#include "hash_grid.hpp"
#include "path_vertex.hpp"

#include <cmath>

void HashGrid::build(float radius,
                     const std::vector<VCMPathVertex> &lightPhotons,
                     bool causticOnly) {
    radius2 = radius * radius;
    cellSize = radius;
    invCellSize = 1.0f / radius;

    vertices = &lightPhotons;
    buckets.clear();
    buckets.reserve(lightPhotons.size() * 2);

    for (size_t i = 0; i < lightPhotons.size(); ++i) {
        const VCMPathVertex &v = lightPhotons[i];
        if (!isMergeable(v, causticOnly)) {
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

bool HashGrid::isMergeable(const VCMPathVertex &p, bool causticOnly) const {
    return !p.isDelta && !p.isLight && (!causticOnly || p.isCaustic);
}
