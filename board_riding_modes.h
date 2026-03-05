#pragma once

#include <cstdint>

#include "can.h"

void boardRidingModesProcessRx134(const CANRxFrame& frame);
void boardRidingModesPublishLive();
void boardRidingModesComposeTx148(uint8_t& b0, uint8_t& b1);
