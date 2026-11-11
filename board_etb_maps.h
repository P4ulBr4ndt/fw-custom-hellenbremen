#pragma once

#include "table_helper.h"

typedef Map3D<PEDAL_TO_TPS_RPM_SIZE, PEDAL_TO_TPS_SIZE, uint16_t, uint8_t, uint8_t> etb_pedal2tps_t;

void boardEtbMapsApplyDefaults();
