#include "hash_grid.hpp"
#include "path_vertex.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <functional>
#include <unordered_map>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace {

using BucketMap = std::unordered_map<Int3, std::vector<size_t>, Int3Hash>;

} // namespace

bool HashGrid::build(float radius,
                     const std::vector<VCMPathVertex> &lightPhotons,
                     bool causticOnly,
                     const std::function<bool()> &progressCallback) {
    radius2 = radius * radius;
    cellSize = radius;
    invCellSize = 1.0f / radius;

    vertices = &lightPhotons;
    buckets.clear();
    buckets.reserve(lightPhotons.size() * 2);

    int threadCount = 1;
#ifdef _OPENMP
    threadCount = std::max(1, omp_get_max_threads());
#endif
    std::vector<BucketMap> localBuckets(threadCount);
    std::atomic_bool cancelled(false);

    const long long photonCount = static_cast<long long>(lightPhotons.size());
    #pragma omp parallel
    {
        int threadId = 0;
#ifdef _OPENMP
        threadId = omp_get_thread_num();
#endif
        BucketMap &local = localBuckets[threadId];
        if (threadCount > 0) {
            local.reserve(lightPhotons.size() / static_cast<size_t>(threadCount));
        }

        #pragma omp for schedule(static)
        for (long long i = 0; i < photonCount; ++i) {
            if (cancelled.load(std::memory_order_relaxed)) {
                continue;
            }

            if (progressCallback && (i & 8191) == 0) {
                bool shouldCancel = false;
                #pragma omp critical(hash_grid_progress)
                {
                    shouldCancel = !cancelled.load(std::memory_order_relaxed) &&
                        progressCallback();
                }
                if (shouldCancel) {
                    cancelled.store(true, std::memory_order_relaxed);
                    continue;
                }
            }

            const size_t index = static_cast<size_t>(i);
            const VCMPathVertex &v = lightPhotons[index];
            if (!isMergeable(v, causticOnly)) {
                continue;
            }
            Int3 cell = cellOf(v.pos);
            local[cell].push_back(index);
        }
    }

    if (cancelled.load(std::memory_order_relaxed)) {
        return false;
    }

    size_t localBucketCount = 0;
    for (const BucketMap &local : localBuckets) {
        localBucketCount += local.size();
    }
    buckets.reserve(localBucketCount);

    for (BucketMap &local : localBuckets) {
        for (auto &entry : local) {
            std::vector<size_t> &bucket = buckets[entry.first];
            bucket.insert(bucket.end(), entry.second.begin(), entry.second.end());
        }
    }

    if (progressCallback && progressCallback()) {
        return false;
    }
    return true;
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
