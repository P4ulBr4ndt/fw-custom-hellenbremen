#pragma once

#include <cstddef>

#include "can.h"
#include "can_msg_tx.h"
#include "rusefi/rusefi_time_types.h"

void boardHandleCan(CanCycle cycle);
void boardProcessCanRx(size_t busIndex, const CANRxFrame& frame, efitick_t nowNt);
void setCfcForce(bool state);
void setCcfcForce(bool state);
void setPrgselForce(bool state);
void setCpcForce(bool state);
