#include "pch.h"

#include "board_overrides.h"

#include "board_can.h"
#include "board_config.h"
#include "board_instant_accel_shot.h"

#include "pwm_generator_logic.h"

// Required by Purge Solenoid PWM 
SimplePwm    prgselPwm("PRGSEL");
OutputPin    prgselPin;

// CFC
OutputPin    cfcPin;

void boardPeriodicSlow() {
	bool jssDown = engine->engineState.jssState != 0;
	uint8_t currentGear = calculateHarleyGearIndex();
	bool isNeutral = currentGear == 0;
	bool isEngineActive = engine->rpmCalculator.isRunning() || engine->rpmCalculator.isCranking();

	bool shouldRequestStop = jssDown && !isNeutral && isEngineActive;
	if (shouldRequestStop && !jssStopRequestActive) {
		doScheduleStopEngine(StopRequestedReason::Board1);
	}

	jssStopRequestActive = shouldRequestStop;

	// Purge Valve Solenoid routines
	if((Sensor::getOrZero(SensorType::Rpm) >= 2000.0f) &&
	   (Sensor::getOrZero(SensorType::VehicleSpeed) >= 10.0f) &&
	   (Sensor::getOrZero(SensorType::AcceleratorPedal) >= 5.0f) &&
	   (Sensor::getOrZero(SensorType::AcceleratorPedal) <= 75.0f) &&
	   (Sensor::getOrZero(SensorType::Clt) >= 90.0f) && 
		engine->fuelComputer.running.timeSinceCrankingInSecs >= 180.0f) {
		prgselPwm.setFrequency(32.0f);
	} else {
		prgselPwm.setFrequency(NAN); // setFrequecy(NAN) deactivates the PWM schedule
	}

	// Cooling Fan Controller
	//TODO Idle Adder is not implemented yet
	bool  cfcRunning          = cfcPin.getLogicValue();
	float cfcCurrentTemp      = Sensor::getOrZero(SensorType::AuxTemp2);
	bool  cfcDisableSpeedCond = (config->cfcDisableAboveSpeed <= Sensor::getOrZero(SensorType::VehicleSpeed)) &&
								(config->cfcDisableAboveSpeed > 0);
	bool  cfcDisableEngCond   = (!config->cfcDisableWhenEngineStopped || isEngineActive);
	if ((cfcCurrentTemp > config->cfcOnTemperature) && !cfcRunning && cfcDisableEngCond && !cfcDisableSpeedCond)
		cfcPin.setValue(true);
	else if ((cfcCurrentTemp < config->cfcOffTemperature || cfcDisableSpeedCond) && cfcRunning) 
		cfcPin.setValue(false);
}

void setup_custom_board_overrides() {
	custom_board_DefaultConfiguration = boardDefaultConfiguration;
	custom_board_ConfigOverrides = boardConfigOverrides;
	custom_board_can_rx = boardProcessCanRx;
	custom_board_update_dash = boardHandleCan;
	custom_board_periodicSlowCallback = boardPeriodicSlow;
	custom_board_periodicFastCallback = boardInstantAccelFastCallback;
	custom_board_InitHardware = boardCustomInitHardware;
}
