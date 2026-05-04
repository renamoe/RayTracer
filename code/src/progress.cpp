#include "progress.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace {

std::string formatDuration(double seconds) {
    if (!std::isfinite(seconds) || seconds < 0.0) {
        return "--:--";
    }

    long long totalSeconds = static_cast<long long>(seconds + 0.5);
    long long hours = totalSeconds / 3600;
    long long minutes = (totalSeconds % 3600) / 60;
    long long secs = totalSeconds % 60;

    std::ostringstream out;
    if (hours > 0) {
        out << hours << ":" << std::setw(2) << std::setfill('0') << minutes
            << ":" << std::setw(2) << std::setfill('0') << secs;
    } else {
        out << minutes << ":" << std::setw(2) << std::setfill('0') << secs;
    }
    return out.str();
}

} // namespace

ProgressBar::ProgressBar(long long total)
    : total(total), completed(0), nextReportMs(0) {}

void ProgressBar::start() {
    completed.store(0);
    nextReportMs.store(0);
    startTime = std::chrono::steady_clock::now();
    print(0, false);
}

void ProgressBar::advance(long long amount) {
    long long current = completed.fetch_add(amount) + amount;
    long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - startTime
    ).count();

    long long nextReport = nextReportMs.load();
    if (elapsedMs >= nextReport &&
        nextReportMs.compare_exchange_strong(nextReport, elapsedMs + 250)) {
        print(current, false);
    }
}

void ProgressBar::finish() {
    print(total, true);
}

void ProgressBar::print(long long completedPixels, bool finished) {
    constexpr int BAR_WIDTH = 36;
    std::lock_guard<std::mutex> lock(outputMutex);

    double fraction = total > 0
        ? static_cast<double>(completedPixels) / static_cast<double>(total)
        : 1.0;
    fraction = std::max(0.0, std::min(1.0, fraction));

    const auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - startTime).count();
    double eta = fraction > 1e-6 ? elapsed * (1.0 - fraction) / fraction : -1.0;
    if (finished) {
        eta = 0.0;
    }

    int filled = static_cast<int>(std::round(fraction * BAR_WIDTH));
    filled = std::max(0, std::min(BAR_WIDTH, filled));

    std::ostringstream out;
    out << "\r[render] [";
    for (int i = 0; i < BAR_WIDTH; ++i) {
        out << (i < filled ? '#' : '-');
    }
    out << "] " << std::fixed << std::setprecision(1) << (fraction * 100.0) << "% "
        << completedPixels << "/" << total << " px"
        << " elapsed " << formatDuration(elapsed)
        << " eta " << formatDuration(eta) << "   ";

    std::cout << out.str() << std::flush;
    if (finished) {
        std::cout << "\n";
    }
}
