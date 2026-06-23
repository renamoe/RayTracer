#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>

class ProgressBar {
public:
    explicit ProgressBar(long long total, std::string unit = "px");

    void start();
    void advance(long long amount);
    void setProgress(long long value);
    void finish();

private:
    void print(long long completed, bool finished);
    void printIfDue(long long current);

    long long total;
    std::string unit;
    std::atomic<long long> completed;
    std::atomic<long long> nextReportMs;
    std::chrono::steady_clock::time_point startTime;
    std::mutex outputMutex;
    bool interactive;
    long long reportIntervalMs;
};
