#include "progress.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>

#ifndef _WIN32
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace {

bool outputIsInteractive() {
#ifdef _WIN32
    return true;
#else
    return isatty(STDOUT_FILENO);
#endif
}

int terminalColumns() {
#ifdef _WIN32
    return 80;
#else
    winsize size{};
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 && size.ws_col > 0) {
        return size.ws_col;
    }
    return 80;
#endif
}

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

ProgressBar::ProgressBar(long long total, std::string unit)
    : total(total),
      unit(std::move(unit)),
      completed(0),
      nextReportMs(0),
      interactive(outputIsInteractive()),
      reportIntervalMs(interactive ? 250 : 5000) {}

void ProgressBar::start() {
    completed.store(0);
    nextReportMs.store(0);
    startTime = std::chrono::steady_clock::now();
    print(0, false);
}

void ProgressBar::advance(long long amount) {
    long long current = completed.fetch_add(amount) + amount;
    printIfDue(current);
}

void ProgressBar::setProgress(long long value) {
    completed.store(value);
    printIfDue(value);
}

void ProgressBar::printIfDue(long long current) {
    long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - startTime
    ).count();

    long long nextReport = nextReportMs.load();
    if (elapsedMs >= nextReport &&
        nextReportMs.compare_exchange_strong(nextReport, elapsedMs + reportIntervalMs)) {
        print(current, false);
    }
}

void ProgressBar::finish() {
    print(total, true);
}

void ProgressBar::print(long long completedPixels, bool finished) {
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

    int barWidth = interactive ? std::min(24, std::max(8, terminalColumns() - 56)) : 20;
    int filled = static_cast<int>(std::round(fraction * barWidth));
    filled = std::max(0, std::min(barWidth, filled));

    std::ostringstream out;
    if (interactive) {
        out << "\r";
    }
    out << "[render] [";
    for (int i = 0; i < barWidth; ++i) {
        out << (i < filled ? '#' : '-');
    }
    out << "] " << std::fixed << std::setprecision(1) << (fraction * 100.0) << "% "
        << completedPixels << "/" << total << " " << unit
        << " elapsed " << formatDuration(elapsed)
        << " eta " << formatDuration(eta);

    if (interactive) {
        out << "   ";
    } else {
        out << "\n";
    }

    std::cout << out.str() << std::flush;
    if (finished && interactive) {
        std::cout << "\n";
    }
}
