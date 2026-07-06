#pragma once

#include <cstdint>

enum class ccfcModes_e : uint8_t {
    On   = 0,
    Auto = 1,
    Off  = 2,
};

enum class cfcUserForceModes_e : uint8_t {
	Off         = 0,
	Queueing    = 1,
	On          = 2,
	BlockChange = 3,
};
