#include "pch.h"
#include "hellen_meta.h"
#include "defaults.h"

#define HARLEY_V_TWIN 45.0

// board-specific configuration setup
void setBoardDefaultConfiguration() {
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
    engineConfiguration->fanOnTemperature = 80.f;
    engineConfiguration->fanOffTemperature = 70.f;
    engineConfiguration->mainRelayPin = Gpio::Unassigned;

    // ACR TODO: Is this still used?
    strcpy(engineConfiguration->scriptSettingName[0], "compReleaseRpm");
    engineConfiguration->scriptSetting[0] = 300;
    strcpy(engineConfiguration->scriptSettingName[1], "compReleaseDur");
    engineConfiguration->scriptSetting[1] = 5000;

    // AFR TODO
    engineConfiguration->afr.hwChannel = EFI_ADC_NONE;
    engineConfiguration->enableAemXSeries = false;

    // MAP
    setCustomMap(/*lowValue*/ 20, /*mapLowValueVoltage*/ 0.79, /*highValue*/ 101.3, /*mapHighValueVoltage*/ 4);

    // TPS
    engineConfiguration->tpsMin = 100;
    engineConfiguration->tpsMax = 900;
    engineConfiguration->throttlePedalUpVoltage = 1.36;
    engineConfiguration->throttlePedalWOTVoltage = 4.46;

    // 5.6k high side/10k low side = 1.56 ratio divider
    // engineConfiguration->analogInputDividerCoefficient = 1.56f;

    // 6.34k high side/ 1k low side
    // engineConfiguration->vbattDividerCoeff = (6.34 + 1) / 1;

    // engineConfiguration->adcVcc = 3.3f;

    // Injection
    engineConfiguration->injectionPins[0] = Gpio::G7;
    engineConfiguration->injectionPins[1] = Gpio::G8;

    // Ignition
    engineConfiguration->ignitionPins[0] = Gpio::E5;
    engineConfiguration->ignitionPins[1] = Gpio::C13;
}

void setBoardConfigOverrides() {
    setHellenAnalogDividers();
    setHellenCan();
    setDefaultHellenAtPullUps();

	// TODO: Remove this temp fix, we are currently missing voltage sensing
	engineConfiguration->vbattAdcChannel = Gpio::Unassigned;

    // Generic
    engineConfiguration->cylindersCount = 2;
    engineConfiguration->firingOrder = FO_1_2;
    strcpy(engineConfiguration->engineMake, "Harley");

    // Knock
    // engineConfiguration->enableSoftwareKnock = true;
    // engineConfiguration->knockFrequency = 0.0f; // TODO

    // Aux Outputs
    engineConfiguration->fanPin = Gpio::C7;
    //engineConfiguration->fanOnTemperature = 80.f;
    //engineConfiguration->fanOffTemperature = 70.f;

    // MAP
    engineConfiguration->map.sensor.hwChannel = EFI_ADC_13;

    // ETB
    engineConfiguration->etb_use_two_wires = false;
    engineConfiguration->etbIo[0].directionPin1 = Gpio::D13;
    engineConfiguration->etbIo[0].controlPin = Gpio::D12;
    engineConfiguration->etbIo[0].disablePin = Gpio::C12;
    engineConfiguration->tps1_1AdcChannel = EFI_ADC_4;
    engineConfiguration->tps1_2AdcChannel = EFI_ADC_14;
    engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_10;
    engineConfiguration->throttlePedalPositionSecondAdcChannel = EFI_ADC_12;

    // Temp Sensors
    engineConfiguration->clt.adcChannel = EFI_ADC_5;
    engineConfiguration->iat.adcChannel = EFI_ADC_2;

    // Injection
    engineConfiguration->injectionMode = IM_SEQUENTIAL;

    // Ignition
    engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;

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