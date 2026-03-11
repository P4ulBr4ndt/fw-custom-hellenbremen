#include "pch.h"

#include "board_overrides.h"

#include "board_can.h"
#include "board_config.h"
#include "board_instant_accel_shot.h"

void setup_custom_board_overrides() {
	custom_board_DefaultConfiguration = boardDefaultConfiguration;
	custom_board_ConfigOverrides = boardConfigOverrides;
	custom_board_can_rx = boardProcessCanRx;
	custom_board_update_dash = boardHandleCan;
	custom_board_periodicSlowCallback = boardPeriodicSlowCallback;
	custom_board_periodicFastCallback = boardInstantAccelFastCallback;
}
