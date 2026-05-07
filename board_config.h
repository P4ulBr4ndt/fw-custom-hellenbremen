#pragma once

#include "efi_output.h"

extern OutputPin prgselPin;
extern SimplePwm prgselPwm;

void boardDefaultConfiguration();
void boardConfigOverrides();
