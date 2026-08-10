#pragma once

#include "efi_output.h"

// Purge Valve Solenoid 
extern OutputPin    prgselPin;
extern SimplePwm    prgselPwm;

// TunerStudio's detectedGearGauge, fed from board_can.cpp
extern StoredValueSensor harleyDetectedGearSensor;

void boardDefaultConfiguration();
void boardConfigOverrides();
void boardCustomInitHardware();
void boardHandleTsCommand(uint16_t subsystem, uint16_t index);
void boardCustomOnConfigurationChange(engine_configuration_s* previousConfiguration);
