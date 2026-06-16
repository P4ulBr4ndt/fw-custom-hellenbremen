#include "board_instant_accel_shot.h"

void boardPeriodicSlow() {
	bool jssDown        = engine->engineState.jssState != 0;
	uint8_t currentGear = calculateHarleyGearIndex();
	bool isNeutral      = currentGear == 0;
	bool isEngineActive = engine->rpmCalculator.isRunning() || engine->rpmCalculator.isCranking();
	
	float currRPM     = Sensor::getOrZero(SensorType::Rpm);
	float currSpeed   = Sensor::getOrZero(SensorType::VehicleSpeed);
	float currEngTemp = Sensor::getOrZero(SensorType::AuxTemp1);
	float currAmbTemp = Sensor::getOrZero(SensorType::AmbientTemperature);
	float currCltTemp = Sensor::getOrZero(SensorType::AuxTemp2);
	float currTGS     = Sensor::getOrZero(SensorType::AcceleratorPedal);
	float currCltTmp2 = Sensor::getOrZero(SensorType::Clt);
	float currRuntime = engine->fuelComputer.running.timeSinceCrankingInSecs;

	// Similar to engine->fuelComputer.running.timeSinceCrankingInSecs
	// but for engine not running time. See engine2.cpp:170
	if(isEngineActive) {
		engNotRunningTimer.reset();
	}

	bool shouldRequestStop = jssDown && !isNeutral && isEngineActive;
	if (shouldRequestStop && !jssStopRequestActive) {
		doScheduleStopEngine(StopRequestedReason::Board1);
	}

	jssStopRequestActive = shouldRequestStop;

	// Purge Valve Solenoid routines
	bool prgselRunning = config->prgselActive && 
						 (prgselForce || ((currRPM     >= config->prgselRPM) &&
		 				                  (currSpeed   >= config->prgselSpeed) &&
		 				                  (currTGS     >= config->prgselLowerTGS) &&
		 				                  (currTGS     <= config->prgselUpperTGS) &&
		 				                  (currCltTmp2 >= config->prgselCltTemp) &&
		 				                  (currRuntime >= config->prgselActAfterTime)));
	if (prgselRunning) {
		prgselPwm.setFrequency(config->prgselPWMFreq);
	} else {
		prgselPwm.setFrequency(NAN); // setFrequency(NAN) deactivates the PWM schedule
	}
	
	// Cooling Fan Controller
	//TODO Idle Adder is not implemented yet
	bool cfcRunning = cfcPin.getLogicValue();

	// Speed hysteresis cases
	if(currSpeed >= config->cfcHighSpeedThreshold) {
		cfcHighSpeedMode = true;
	} else if(currSpeed < config->cfcLowSpeedThreshold) {
		cfcHighSpeedMode = false;
	}

	bool cfcIsShutdownRunning  = cfcRunning 
	                             && !isEngineActive 
								 && (engNotRunningTimer.getElapsedSeconds() <= config->cfcMaxRuntimeAfterEngShutdown 
								     && currCltTemp > config->cfcEngShutdownOffTemp 
									 && config->cfcEngShutdownOffTemp <= config->cfcLowSpeedOffTemp 
									 && config->cfcEngShutdownOffTemp <= config->cfcHighSpeedOffTemp);
	bool cfcIsShutdownComplete = (engNotRunningTimer.getElapsedSeconds() > config->cfcMaxRuntimeAfterEngShutdown || currCltTemp <= config->cfcEngShutdownOffTemp) && cfcRunning && !isEngineActive;
	bool cfcDisabledEngCond    = (!config->cfcDisableWhenEngineStopped || isEngineActive || cfcIsShutdownRunning);
	bool cfcOnTempCond         = ((currCltTemp >  config->cfcLowSpeedOnTemp   && !cfcHighSpeedMode) ||
	     				          (currCltTemp >  config->cfcHighSpeedOnTemp  && cfcHighSpeedMode)) && !cfcRunning;
	bool cfcOffTempCond        = ((currCltTemp <= config->cfcLowSpeedOffTemp  && !cfcHighSpeedMode) || 
						          (currCltTemp <= config->cfcHighSpeedOffTemp && cfcHighSpeedMode)) && cfcRunning;

	if ((cfcOnTempCond && cfcDisabledEngCond) || cfcForce) {
		cfcPin.setValue(true);
	} else if ((cfcOffTempCond || cfcIsShutdownComplete) && !cfcForce) {
		cfcPin.setValue(false);
	}

	// Chassis Cooling Fan Controller
	//TODO Idle Adder is not implemented yet
	// Bikes without Amb. Temp. run... (!)
	//		i) for currSpeed <= 64 km/h && currCltTemp >= 95° C
	//      ii) not for currSpeed > 72 km/h || currCltTemp < 82°C
	bool ccfcRunning = ccfcPin.getLogicValue();

	// Ambient temparature hysteresis cases
	if(currAmbTemp >= config->ccfcHighAmbTempThreshold) {
		ccfcHighAmbMode = true;
	} else if(currAmbTemp <  config->ccfcLowAmbTempThreshold) {
		ccfcHighAmbMode = false;
	}

	bool ccfcOnSpeedCond  = (currSpeed < config->ccfcEnableBelowSpeed) && !ccfcRunning;
	bool ccfcOffSpeedCond = (currSpeed >= config->ccfcDisableAboveSpeed) && ccfcRunning;
	bool ccfcOnTempCond   = ((currEngTemp >  config->ccfcLowAmbEnableAboveEngTemp   && !ccfcHighAmbMode) ||
	     				     (currEngTemp >  config->ccfcHighAmbEnableAboveEngTemp  && ccfcHighAmbMode)) && !ccfcRunning;
	bool ccfcOffTempCond  = ((currEngTemp <= config->ccfcLowAmbDisableBelowEngTemp  && !ccfcHighAmbMode) || 
						     (currEngTemp <= config->ccfcHighAmbDisableBelowEngTemp && ccfcHighAmbMode)) && ccfcRunning;

	if(ccfcMode == ccfcModes_e::Off) {						
		if(ccfcForce && ccfcActivated) {
			ccfcPin.setValue(true);	
		} else {
			ccfcPin.setValue(false);
		}
	} else {
		if(ccfcMode == ccfcModes_e::On) {
			if((ccfcOnSpeedCond && isEngineActive) || ccfcForce) {
				ccfcPin.setValue(true);
			} else if((ccfcOffSpeedCond || (!isEngineActive && ccfcRunning)) && !ccfcForce) {
				ccfcPin.setValue(false);
			}
		} else if(ccfcMode == ccfcModes_e::Auto) {
			if((ccfcOnSpeedCond && ccfcOnTempCond) || ccfcForce) {
				ccfcPin.setValue(true);
			} else if(((ccfcOffSpeedCond || ccfcOffTempCond) || (!isEngineActive && ccfcRunning)) && !ccfcForce) {
				ccfcPin.setValue(false);
			}
		}
	}	

	// Coolant Pump Control
	bool cpcRunning         = cpcPin.getLogicValue();
	bool cpcOnTempCond      = (currCltTemp >  config->cpcOnTemp)  && isEngineActive && !cpcRunning;
	bool cpcOffTempCond     = (currCltTemp <= config->cpcOffTemp) && cpcRunning;
	bool cpcDisabledEngCond = isEngineActive || !config->cpcDisableWhenEngineStopped;
	bool cpcCurrentCfc      = cfcPin.getLogicValue();

	if (((cpcCurrentCfc || cpcOnTempCond) && cpcDisabledEngCond) || cpcForce) {
		cpcPin.setValue(true);
	} else if (!cpcForce && !cpcCurrentCfc && (!isEngineActive || cpcOffTempCond)) {
		cpcPin.setValue(false);
	}

	// luaGauges[6]: CCFC state    — 0=Disabled, 1=Off, 2=Auto, 3=On
	// luaGauges[7]: running flags — cpcRunning*8 + ccfcRunning*4 + cfcRunning*2 + prgselRunning  (0-15)
	luaGauges[6].setValidValue(ccfcActivated ? (3.0f - static_cast<float>(ccfcMode)) : 0.0f, getTimeNowNt());
	luaGauges[7].setValidValue((cpcPin.getLogicValue()    ? 8.0f : 0.0f)
	                           + (ccfcPin.getLogicValue() ? 4.0f : 0.0f)
	                           + (cfcPin.getLogicValue()  ? 2.0f : 0.0f)
							   + (prgselRunning           ? 1.0f : 0.0f), getTimeNowNt());

	if(harleyIgnitionOffRequested && !harleyIgnitionOnRequested) {
		cfcForce    = false;
		ccfcForce   = false;
		prgselForce = false;
		cpcForce    = false;
	}
}

void boardPeriodicFast() {
	updateInstantAccelShot();
}