#pragma once

#include <cstddef>

#include "can.h"
#include "rusefi/rusefi_time_types.h"

void handleCalibrationUdsCanRx(size_t busIndex, const CANRxFrame& frame, efitick_t nowNt);
