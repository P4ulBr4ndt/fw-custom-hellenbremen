#include "pch.h"

#include "board_can.h"
#include "board_config.h"

#include <cstring>

#include "board_etb_maps.h"
#include "board_instant_accel_shot.h"
#include "board_riding_modes.h"
#include "defaults.h"
#include "hellen_meta.h"

#define HARLEY_V_TWIN 45.0

void boardDefaultConfiguration() {
	// Trigger
	engineConfiguration->overrideTriggerGaps = true;
	engineConfiguration->gapTrackingLengthOverride = 3;
	engineConfiguration->triggerGapOverrideFrom[0] = 0.75;
	engineConfiguration->triggerGapOverrideTo[0] = 1.25;
	engineConfiguration->triggerGapOverrideFrom[1] = 0.05; // this one is custom
	engineConfiguration->triggerGapOverrideTo[1] = 0.5;
	engineConfiguration->triggerGapOverrideFrom[2] = 1.850; // this one is custom
	engineConfiguration->triggerGapOverrideTo[2] = 6;

	// Cam
	engineConfiguration->vvtMode[0] = VVT_MAP_V_TWIN;
	engineConfiguration->mapCamDetectionAnglePosition = 50;

	// Aux Outputs
	engineConfiguration->fanPin = Gpio::C7;
	engineConfiguration->fanOnTemperature = 0.f;
	engineConfiguration->fanOffTemperature = 0.f;
	engineConfiguration->fan2Pin = Gpio::Unassigned;
	engineConfiguration->fan2OnTemperature = 0.f;
	engineConfiguration->fan2OffTemperature = 0.f;
	engineConfiguration->mainRelayPin = Gpio::Unassigned;

	// AFR TODO
	engineConfiguration->afr.hwChannel = EFI_ADC_NONE;
	engineConfiguration->enableAemXSeries = false;

	// MAP
	setCustomMap(/*lowValue*/ 20, /*mapLowValueVoltage*/ 0.79, /*highValue*/ 101.3, /*mapHighValueVoltage*/ 4);

	// TPS
	engineConfiguration->tpsMin = 0.50;
	engineConfiguration->tpsMax = 4.54;
	engineConfiguration->tps1SecondaryMin = 4.53;
	engineConfiguration->tps1SecondaryMax = 0.47;

	engineConfiguration->throttlePedalUpVoltage = 1.36;
	engineConfiguration->throttlePedalWOTVoltage = 4.46;
	engineConfiguration->throttlePedalSecondaryUpVoltage = 1.37;
	engineConfiguration->throttlePedalSecondaryWOTVoltage = 4.48;

	// Fuel Level Calibration Road Glide 2025
	// Values are rising in percentage per index
	config->fuelLevelBins[0] = 0.335f; // fuelLevelBins in float of V
	config->fuelLevelBins[1] = 0.446f;
	config->fuelLevelBins[2] = 0.556f;
	config->fuelLevelBins[3] = 0.667f;
	config->fuelLevelBins[4] = 0.778f;
	config->fuelLevelBins[5] = 0.889f;
	config->fuelLevelBins[6] = 0.999f;
	config->fuelLevelBins[7] = 1.110f;

	config->fuelLevelValues[0] = 100; // fuelLevelValues in int of %
	config->fuelLevelValues[1] = 86;
	config->fuelLevelValues[2] = 71;
	config->fuelLevelValues[3] = 57;
	config->fuelLevelValues[4] = 43;
	config->fuelLevelValues[5] = 29;
	config->fuelLevelValues[6] = 14;
	config->fuelLevelValues[7] = 0; 
	
	engineConfiguration->fuelLevelAveragingAlpha = 0.003f; //TODO Too responsive, decrease alpha for more smoothing
	engineConfiguration->fuelLevelLowThresholdVoltage = 0.3f;
	engineConfiguration->fuelLevelHighThresholdVoltage = 1.3f;

	boardEtbMapsApplyDefaults();
	boardInstantAccelApplyDefaults();
	boardRidingModesApplyDefaults();

	setRpmTableBin(config->estimatedEngineTorqueRpmBins);
	setLinearCurve(config->estimatedEngineTorqueTpsBins, 0.0f, 100.0f, 1.0f);
	setTable(config->estimatedEngineTorqueTable, 0);
}

void boardConfigOverrides() {
	setHellenAnalogDividers();
	setHellenCan();
	hellenMegaSdWithAccelerometer();

	// VBATT is on PA0
	engineConfiguration->vbattAdcChannel = EFI_ADC_0;
	// set vbatt_divider 4,0303
	// 10k / 3.3k
	engineConfiguration->vbattDividerCoeff = 8.06f;

	// PWR EN
	setHellenEnPin(Gpio::E0, true);

	// Generic
	engineConfiguration->cylindersCount = 2;
	engineConfiguration->firingOrder = FO_1_2;
	strcpy(engineConfiguration->engineMake, "Harley");

	// Knock
	// engineConfiguration->enableSoftwareKnock = true;
	// engineConfiguration->knockFrequency = 0.0f; // TODO

	// Sensors
	engineConfiguration->map.sensor.hwChannel = EFI_ADC_13; // PC3
	engineConfiguration->fuelLevelSensor = EFI_ADC_2; // PA2

	// Switch Inputs
	engineConfiguration->jssPin = Gpio::G11;
	engineConfiguration->opsPin = Gpio::G12;
	engineConfiguration->jssPinMode = PI_PULLDOWN;
	engineConfiguration->opsPinMode = PI_PULLDOWN;

	// ETB
	engineConfiguration->etb_use_two_wires = false;
	engineConfiguration->etbIo[0].directionPin1 = Gpio::D13;
	engineConfiguration->etbIo[0].controlPin = Gpio::D12;
	engineConfiguration->etbIo[0].disablePin = Gpio::C12;
	engineConfiguration->tps1_1AdcChannel = EFI_ADC_4;
	engineConfiguration->tps1_2AdcChannel = EFI_ADC_14;
	engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_10;
	engineConfiguration->throttlePedalPositionSecondAdcChannel = EFI_ADC_12;

	// Temp Sensor Pullups
	// As CLT we use ETS because thats the important figure for us
	engineConfiguration->clt.config.bias_resistor = 1000; // ETS
	engineConfiguration->auxTempSensor1.config.bias_resistor = 1000; // ETS
	engineConfiguration->auxTempSensor2.config.bias_resistor = 1000; // CLT
	engineConfiguration->ambientTempSensor.config.bias_resistor = 10000; // AAT
	engineConfiguration->iat.config.bias_resistor = 1000; // IAT

	// Temp Sensors
	engineConfiguration->clt.adcChannel = EFI_ADC_17; // ETS PA1 MUX = 1
	engineConfiguration->auxTempSensor1.adcChannel = EFI_ADC_17; // ETS
	engineConfiguration->auxTempSensor2.adcChannel = EFI_ADC_5; // CLT
	engineConfiguration->ambientTempSensor.adcChannel = EFI_ADC_1; // PA1 // AAT for CANBUS
	engineConfiguration->iat.adcChannel = EFI_ADC_18; // PA2 MUX = 1

	// Injection
	engineConfiguration->injectionMode = IM_SEQUENTIAL;
	engineConfiguration->injectionPins[0] = Gpio::G8;
	engineConfiguration->injectionPins[1] = Gpio::G7;

	// Ignition
	engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
	engineConfiguration->ignitionPins[0] = Gpio::E5;
	engineConfiguration->ignitionPins[1] = Gpio::C13;

	// Trigger
	engineConfiguration->triggerInputPins[0] = Gpio::B0;
	engineConfiguration->trigger.type = trigger_type_e::TT_TOOTHED_WHEEL_32_2;

	engineConfiguration->engineType = engine_type_e::HARLEY;

	// total 45 degree odd fire, split across two cylinders mostly for fun
	engineConfiguration->timing_offset_cylinder[0] = -HARLEY_V_TWIN / 2;
	engineConfiguration->timing_offset_cylinder[1] = +HARLEY_V_TWIN / 2;

	// work-around for https://github.com/rusefi/rusefi/issues/5894 todo: fix it!
	engineConfiguration->maximumIgnitionTiming = 90;
	engineConfiguration->minimumIgnitionTiming = -90;
}

void boardCustomInitHardware() {
	// Purge Solenoid PWM init
	startSimplePwmExt(&prgselPwm, 
				   "PRGSEL", 
				   &engine->scheduler, 
				   Gpio::D10, 
				   &prgselPin, 
				   NAN,              // Frequency
				   0.3f              // Duty cycle
	);

	// Cooling Fan Control Pin init
	cfcPin.initPin("CFC", config->cfcOutputPin);
}

void boardHandleTsCommand(uint16_t subsystem, uint16_t index) {
	switch(index) {
		case 0:
			setCfcForceState(false);
			break;
		case 1:
			setCfcForceState(true);
			break;
	}
}
