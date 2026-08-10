#include "pch.h"

#include "board_overrides.h"

#include "board_can.h"
#include "board_config.h"
#include "board_instant_accel_shot.h"

#include "pwm_generator_logic.h"
#include "tunerstudio.h"

// Required by Purge Solenoid PWM 
SimplePwm    prgselPwm("PRGSEL");
OutputPin    prgselPin;

// Feeds TunerStudio's detectedGearGauge from board_can.cpp's AuxLinear1-based gear detection.
// Requires totalGearsCount == 0 (see boardConfigOverrides) so rusEFI's built-in ratio-based
// GearDetector never registers SensorType::DetectedGear itself.
StoredValueSensor harleyDetectedGearSensor(SensorType::DetectedGear, MS2NT(250));

#if EFI_TUNER_STUDIO
bool isTouchingVe(uint16_t offset, uint16_t count) {
	return isTouchingArea(offset, count, offsetof(persistent_config_s, veTable), sizeof(config->veTable)) ||
		isTouchingArea(offset, count, offsetof(persistent_config_s, veFrontTable), sizeof(config->veFrontTable));
}
#endif // EFI_TUNER_STUDIO

void setup_custom_board_overrides() {
	custom_board_DefaultConfiguration = boardDefaultConfiguration;
	custom_board_ConfigOverrides = boardConfigOverrides;
	custom_board_can_rx = boardProcessCanRx;
	custom_board_update_dash = boardHandleCan;
	custom_board_periodicSlowCallback = boardPeriodicSlow;
	custom_board_periodicFastCallback = boardInstantAccelFastCallback;
	custom_board_InitHardware = boardCustomInitHardware;
	custom_board_ts_command = boardHandleTsCommand;
	custom_board_OnConfigurationChange = boardCustomOnConfigurationChange;
}
