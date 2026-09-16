#pragma once

#include <cstdint>

enum class ccfcModes_e : uint8_t {
    On   = 0,
    Auto = 1,
    Off  = 2,
};

enum class autotuneCellChangeResistance_e : uint8_t {
    VeryHigh = 0,
    High     = 1,
    Normal   = 2,
    Low      = 3,
    VeryLow  = 4,
};
