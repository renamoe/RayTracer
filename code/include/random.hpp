#pragma once

#include <random>
#include <thread>

class Random {
public:
    static float get_float() {
        static thread_local std::mt19937 engine(std::hash<std::thread::id>{}(std::this_thread::get_id()));
        static std::uniform_real_distribution<float> dist(0.0f, 1.0f);
        return dist(engine);
    }
};