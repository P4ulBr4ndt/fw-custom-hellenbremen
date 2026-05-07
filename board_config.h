#pragma once

#include "efi_output.h"

extern OutputPin    prgselPin;
extern SimplePwm    prgselPwm;
extern scheduling_s prgselSchedule;
extern bool         prgselWarmupTimeFinished;

void boardDefaultConfiguration();
void boardConfigOverrides();
void boardCustomInitHardware();
