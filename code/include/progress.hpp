#pragma once

#include <atomic>
#include <chrono>
#include <mutex>

class ProgressBar {
public:
    explicit ProgressBar(long long total);

    void start();
    void advance(long long amount);
    void finish();

private:
    void print(long long completed, bool finished);

    long long total;
    std::atomic<long long> completed;
    std::atomic<long long> nextReportMs;
    std::chrono::steady_clock::time_point startTime;
    std::mutex outputMutex;
    bool interactive;
    long long reportIntervalMs;
};
