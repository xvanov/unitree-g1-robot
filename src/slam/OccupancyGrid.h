#pragma once

#include <vector>
#include <cmath>
#include <cstdint>
#include "util/Types.h"

// Log-odds conversion functions
inline float logOddsToProb(float l) {
    return 1.0f / (1.0f + std::exp(-l));
}

inline float probToLogOdds(float p) {
    return std::log(p / (1.0f - p));
}

// Log-odds to uint8_t for PNG output
// IMPORTANT: 0=obstacle (black), 255=free (white) - matches NavSim convention
inline uint8_t logOddsToPng(float l) {
    float p = logOddsToProb(l);
    // Invert: high probability (occupied) -> low pixel value (dark)
    return static_cast<uint8_t>((1.0f - p) * 255.0f);
}

struct OccupancyGrid {
    std::vector<float> log_odds;  // Log-odds values (float for precision)
    int width = 0;                // Grid dimensions in cells
    int height = 0;
    float resolution = 0.05f;     // Meters per cell (0.05f to match NavSim)
    Point2D origin;               // World position of grid origin (0,0 for MVP)

    // Convert entire log-odds map to probability map for PNG export
    // 0=obstacle (black), 255=free (white)
    std::vector<uint8_t> toProbabilityMap() const {
        std::vector<uint8_t> result(log_odds.size());
        for (size_t i = 0; i < log_odds.size(); ++i) {
            result[i] = logOddsToPng(log_odds[i]);
        }
        return result;
    }
};
