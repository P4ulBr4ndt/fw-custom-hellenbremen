#include "pch.h"
#include "hellen_meta.h"
#include "defaults.h"

#define HARLEY_V_TWIN 45.0

// board-specific configuration setup
void setBoardDefaultConfiguration() {
    setHellenVbatt();
    setHellenCan();
    setDefaultHellenAtPullUps();
    // setHellenMegaEnPin(true);
    setHellenEnPin(GPIO::E10, true);
    hellenEnableEn("JustToBeSure");

    engineConfiguration->cylindersCount = 2;
    engineConfiguration->firingOrder = FO_1_2;
    strcpy(engineConfiguration->engineMake, "Harley");

    engineConfiguration->injectionPins[0] = Gpio::G7;
    engineConfiguration->injectionPins[1] = Gpio::G8;

    engineConfiguration->ignitionPins[0] = Gpio::C13;
    engineConfiguration->ignitionPins[1] = Gpio::E5;

    engineConfiguration->triggerInputPins[0] = Gpio::B1;

    engineConfiguration->engineType = engine_type_e::HARLEY;

    engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
    engineConfiguration->injectionMode = IM_SEQUENTIAL;


    strcpy(engineConfiguration->scriptSettingName[0], "compReleaseRpm");
    engineConfiguration->scriptSetting[0] = 300;
    strcpy(engineConfiguration->scriptSettingName[1], "compReleaseDur");
    engineConfiguration->scriptSetting[1] = 5000;
    engineConfiguration->afr.hwChannel = EFI_ADC_NONE;
    engineConfiguration->enableAemXSeries = true;

    // total 45 degree odd fire, split across two cylinders mostly for fun
    engineConfiguration->timing_offset_cylinder[0] = -HARLEY_V_TWIN / 2;
    engineConfiguration->timing_offset_cylinder[1] = +HARLEY_V_TWIN / 2;

    // work-around for https://github.com/rusefi/rusefi/issues/5894 todo: fix it!
    engineConfiguration->maximumIgnitionTiming = 90;
    engineConfiguration->minimumIgnitionTiming = -90;

    engineConfiguration->trigger.type = trigger_type_e::TT_TOOTHED_WHEEL_32_2;
    engineConfiguration->overrideTriggerGaps = true;
    engineConfiguration->gapTrackingLengthOverride = 3;
    engineConfiguration->triggerGapOverrideFrom[0] = 0.75;
    engineConfiguration->triggerGapOverrideTo[0] = 1.25;
    engineConfiguration->triggerGapOverrideFrom[1] = 0.05; // this one is custom
    engineConfiguration->triggerGapOverrideTo[1] = 0.5;
    engineConfiguration->triggerGapOverrideFrom[2] = 1.850; // this one is custom
    engineConfiguration->triggerGapOverrideTo[2] = 6;

    engineConfiguration->vvtMode[0] = VVT_MAP_V_TWIN;

    engineConfiguration->mainRelayPin = Gpio::Unassigned;
    engineConfiguration->mapCamDetectionAnglePosition = 50;

    setCustomMap(/*lowValue*/ 20, /*mapLowValueVoltage*/ 0.79, /*highValue*/ 101.3, /*mapHighValueVoltage*/ 4);



    // engineConfiguration->map.sensor.hwChannel = EFI_ADC_3;

    // engineConfiguration->clt.adcChannel = EFI_ADC_1;

    // engineConfiguration->iat.adcChannel = EFI_ADC_2;

    // 5.6k high side/10k low side = 1.56 ratio divider
    // engineConfiguration->analogInputDividerCoefficient = 1.56f;

    // 6.34k high side/ 1k low side
    // engineConfiguration->vbattDividerCoeff = (6.34 + 1) / 1;

    // engineConfiguration->adcVcc = 3.3f;
}