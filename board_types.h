#pragma once

#include <cstdint>

enum class ccfcModes_e : uint8_t {
    On   = 0,
    Auto = 1,
    Off  = 2,
};

enum class autotuneLearningRate_e : uint8_t {
    VerySlow = 0,
    Slow     = 1,
    Normal   = 2,
    Fast     = 3,
    VeryFast = 4,
};
