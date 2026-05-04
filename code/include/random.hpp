#pragma once

#include <cstdint>
#include <functional>
#include <thread>

class Random {
public:
    static float get_float() {
        static thread_local uint64_t state = seedState();
        uint32_t x = nextUInt(state);
        return (x >> 8) * 0x1.0p-24f;
    }

private:
    static uint64_t seedState() {
        uint64_t seed = static_cast<uint64_t>(
            std::hash<std::thread::id>{}(std::this_thread::get_id())
        );
        seed ^= 0x9e3779b97f4a7c15ULL;
        seed = (seed ^ (seed >> 30)) * 0xbf58476d1ce4e5b9ULL;
        seed = (seed ^ (seed >> 27)) * 0x94d049bb133111ebULL;
        seed ^= (seed >> 31);
        return seed == 0 ? 0x853c49e6748fea9bULL : seed;
    }

    static uint32_t nextUInt(uint64_t &state) {
        uint64_t oldState = state;
        state = oldState * 6364136223846793005ULL + 1442695040888963407ULL;
        uint32_t xorshifted = static_cast<uint32_t>(((oldState >> 18u) ^ oldState) >> 27u);
        uint32_t rot = static_cast<uint32_t>(oldState >> 59u);
        return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
    }
};
