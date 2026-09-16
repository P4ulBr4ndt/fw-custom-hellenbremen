#pragma once

#include <cstdint>

enum class autotuneLearningRate_e : uint8_t {
    VerySlow = 0,
    Slow     = 1,
    Normal   = 2,
    Fast     = 3,
    VeryFast = 4,
};
