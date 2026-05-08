#pragma once

#include "efi_output.h"

extern OutputPin    prgselPin;
extern SimplePwm    prgselPwm;
extern bool         prgselWarmupTimeFinished;

void boardDefaultConfiguration();
void boardConfigOverrides();
void boardCustomInitHardware();
