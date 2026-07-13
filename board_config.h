#pragma once

#include "efi_output.h"

// Purge Valve Solenoid 
extern OutputPin    prgselPin;
extern SimplePwm    prgselPwm;

void boardDefaultConfiguration();
void boardConfigOverrides();
void boardCustomInitHardware();
void boardHandleTsCommand(uint16_t subsystem, uint16_t index);
void boardCustomOnConfigurationChange(engine_configuration_s* previousConfiguration);
