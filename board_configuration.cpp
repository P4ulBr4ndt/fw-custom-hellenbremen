#include "pch.h"
#include "hellen_meta.h"
#include "defaults.h"

// board-specific configuration setup
void setBoardDefaultConfiguration() {
    setHellenVbatt();
    setHellenCan();
    setDefaultHellenAtPullUps();
    setHellenEnPin(Gpio::G4, true);

    // engineConfiguration->injectionPins[0] = Gpio::F13;
    // engineConfiguration->ignitionPins[0] = Gpio::E15;

//   engineConfiguration->triggerInputPins[0] = Gpio::B1;
//	engineConfiguration->triggerInputPins[1] = Gpio::Unassigned;

//	engineConfiguration->map.sensor.hwChannel = EFI_ADC_3;

//	engineConfiguration->clt.adcChannel = EFI_ADC_1;

//	engineConfiguration->iat.adcChannel = EFI_ADC_2;


    	// 5.6k high side/10k low side = 1.56 ratio divider
  //  	engineConfiguration->analogInputDividerCoefficient = 1.56f;

    	// 6.34k high side/ 1k low side
//    	engineConfiguration->vbattDividerCoeff = (6.34 + 1) / 1;

//	engineConfiguration->adcVcc = 3.3f;
}