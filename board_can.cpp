#include "pch.h"

#include "engine_state.h"
#include "board_types.h"

#include "board_can.h"

#include <cmath>
#include <cstdint>

#include "board_riding_modes.h"
#include "board_uds.h"
#include "board_config.h"
#include "cruise_control.h"
#include "electronic_throttle.h"
#include "shutdown_controller.h"

static uint8_t frameCounter142 = 0x0;
static uint8_t frameCounter144 = 0x0;
static uint8_t frameCounter146_342 = 0x0;
static uint8_t frameCounter148 = 0x40;

static bool harleyKeepAlive = true;
static bool harleyIgnitionOffRequested = false;
static bool harleyIgnitionOffRequestedPrev = false;
static bool harleyIgnitionOnRequested = false;
static bool harleyIgnitionOnRequestedPrev = false;
static bool cruiseEnablePressedPrev = false;
static bool cruiseDecPressedPrev = false;
static bool cruiseIncPressedPrev = false;
static efitick_t cruiseDecPressStartNt = 0;
static efitick_t cruiseIncPressStartNt = 0;
static efitick_t cruiseDecLastRepeatNt = 0;
static efitick_t cruiseIncLastRepeatNt = 0;
static bool jssStopRequestActive = false;
static uint32_t lastReceivedOdometer = 0;

// CFC
static bool  cfcForce         = false;
static bool  cfcHighSpeedMode = false;
static Timer engNotRunningTimer;

// CCFC
static bool ccfcForce       = false;
static bool ccfcHighAmbMode = false;

// Are set by CAN Bus frame 0x3C4
static ccfcModes_e ccfcMode      = ccfcModes_e::Off; 
static bool        ccfcActivated = false;

// Purge Valve Solenoid
static bool prgselForce = false;

// CPC
static bool cpcForce = false;

// Required for indicators
extern StoredValueSensor luaGauges[LUA_GAUGE_COUNT];

void setCfcForce(bool state) {
	cfcForce = state;
}

void setCcfcForce(bool state) {
	ccfcForce = state;
}

void setPrgselForce(bool state) {
	if(state && !config->prgselActive)
		prgselPwm.setFrequency(config->prgselPWMFreq);
	else if(!state && !config->prgselActive)
		prgselPwm.setFrequency(NAN);

	prgselForce = state;
}

void setCpcForce(bool state) {
	cpcForce = state;
}

struct CruiseGearLimits {
	bool allowCruise;
	float minSpeedKph;
	float maxSpeedKph;
};

namespace {
/*
TODO CLUTCH looks like 0xD0
N: 0.872V => 17.44% => 0xA0
1: 0.484V => 09.86% => 0x10
2: 1.262V => 25,24% => 0x20
3: 2.098V => 41,96% => 0x30
4: 2.874V => 57,48% => 0x40
5: 3.643V => 72,96% => 0x50
6: 4.439V => 88,78% => 0x60
*/
float harleyGearValues[] = { 17.44f, 9.86f, 25.24f, 41.96f, 57.48f, 72.96f, 88.78f };

uint32_t getFourBytesMsb(const CANRxFrame& frame, size_t offset) {
	return (static_cast<uint32_t>(frame.data8[offset]) << 24) |
		(static_cast<uint32_t>(frame.data8[offset + 1]) << 16) |
		(static_cast<uint32_t>(frame.data8[offset + 2]) << 8) |
		static_cast<uint32_t>(frame.data8[offset + 3]);
}

void setFourBytesMsb(CanTxMessage& msg, uint32_t value, size_t offset) {
	msg[offset] = (value >> 24) & 0xFF;
	msg[offset + 1] = (value >> 16) & 0xFF;
	msg[offset + 2] = (value >> 8) & 0xFF;
	msg[offset + 3] = value & 0xFF;
}

uint8_t calculateHarleyGearIndex() {
	float sensorValue = Sensor::getOrZero(SensorType::AuxLinear1);
	float bestMatch = 0.0f;
	uint8_t bestOffs = 0;

	for (uint8_t i = 0; i < sizeof(harleyGearValues) / sizeof(harleyGearValues[0]); i++) {
		float i_delta = std::abs(harleyGearValues[i] - sensorValue);
		float x_delta = std::abs(bestMatch - sensorValue);
		if (i_delta < x_delta) {
			bestMatch = harleyGearValues[i];
			bestOffs = i;
		}
	}

	return bestOffs;
}

uint8_t calculateHarleyGearValue() {
	uint8_t bestOffs = calculateHarleyGearIndex();

	switch (bestOffs) {
		case 0:
			return 0xA0; // N
		case 1:
			return 0x10; // 1
		case 2:
			return 0x20; // 2
		case 3:
			return 0x30; // 3
		case 4:
			return 0x40; // 4
		case 5:
			return 0x50; // 5
		case 6:
			return 0x60; // 6
		default:
			return 0x0;
	}
}

CruiseGearLimits getCruiseGearLimitsForCurrentGear() {
	switch (calculateHarleyGearIndex()) {
		case 2: // 2nd
			return { true, 42.0f, 98.0f };
		case 3: // 3rd
			return { true, 42.0f, 132.0f };
		case 4: // 4th
			return { true, 42.0f, 148.0f };
		case 5: // 5th
			return { true, 50.0f, 148.0f };
		case 6: // 6th
			return { true, 60.0f, 148.0f };
		case 0: // N
		case 1: // 1st
		default:
			return { false, 0.0f, 0.0f };
	}
}

float clampDesiredCcSpeedForCurrentGear(float requestedKph) {
	auto limits = getCruiseGearLimitsForCurrentGear();
	if (!limits.allowCruise) {
		return 0.0f;
	}

	return clampF(limits.minSpeedKph, requestedKph, limits.maxSpeedKph);
}

uint8_t getHarleyTractionControlStatus() {
	if (engine->engineState.tractionControlSparkSkip != 0) {
		return 0x40;
	}

	if (engine->ignitionState.tractionAdvanceDrop != 0) {
		return 0x10;
	}

	return 0x00;
}

float getEstimatedTorqueFromTable(float rpm, float tps) {
	const float estimatedTorque = interpolate3d(
		config->estimatedEngineTorqueTable,
		config->estimatedEngineTorqueTpsBins,
		tps,
		config->estimatedEngineTorqueRpmBins,
		rpm
	);

	return clampF(-400.0f, estimatedTorque, 400.0f);
}

uint16_t scaleTorqueForCan(float torque) {
	return static_cast<uint16_t>(((torque + 0.1f) * 5) + 5000);
}

bool isCurrentSpeedAllowedForCurrentGear() {
	auto speed = Sensor::get(SensorType::VehicleSpeed);
	if (!speed.Valid) {
		return false;
	}

	auto limits = getCruiseGearLimitsForCurrentGear();
	if (!limits.allowCruise) {
		return false;
	}

	return speed.Value >= limits.minSpeedKph && speed.Value <= limits.maxSpeedKph;
}

void engageCCAtCurrentSpeedForCurrentGear() {
	auto speed = Sensor::get(SensorType::VehicleSpeed);
	if (!speed.Valid) {
		return;
	}

	if (!isCurrentSpeedAllowedForCurrentGear()) {
		return;
	}

	setDesiredCCSpeed(clampDesiredCcSpeedForCurrentGear(speed.Value));
	setCCStatus(CruiseControlStatus::Enabled);
}

void resumeCCForCurrentGear() {
	auto limits = getCruiseGearLimitsForCurrentGear();
	if (!limits.allowCruise) {
		return;
	}

	float desiredSpeed = getDesiredCCSpeed();
	if (desiredSpeed <= 0) {
		return;
	}

	float clampedDesired = clampF(limits.minSpeedKph, desiredSpeed, limits.maxSpeedKph);
	setDesiredCCSpeed(clampedDesired);

	if (isCurrentSpeedAllowedForCurrentGear()) {
		resumeCC();
	}
}

void increaseDesiredCCSpeedForCurrentGear() {
	if (!getCruiseGearLimitsForCurrentGear().allowCruise) {
		setCCStatus(CruiseControlStatus::Standby);
		return;
	}

	setDesiredCCSpeed(clampDesiredCcSpeedForCurrentGear(getDesiredCCSpeed() + 1.0f));
}

void decreaseDesiredCCSpeedForCurrentGear() {
	if (!getCruiseGearLimitsForCurrentGear().allowCruise) {
		setCCStatus(CruiseControlStatus::Standby);
		return;
	}

	setDesiredCCSpeed(clampDesiredCcSpeedForCurrentGear(getDesiredCCSpeed() - 1.0f));
}
} // namespace

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

	bool ccfcOnSpeedCond  = (currSpeed <= config->ccfcEnableBelowSpeed) && !ccfcRunning;
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
			if(ccfcOnSpeedCond || (ccfcForce || isEngineActive)) {
				ccfcPin.setValue(true);
			} else if(ccfcOffSpeedCond && !ccfcForce) {
				ccfcPin.setValue(false);
			}
		} else if(ccfcMode == ccfcModes_e::Auto) {
			if((ccfcOnSpeedCond && ccfcOnTempCond) || (ccfcForce || isEngineActive)) {
				ccfcPin.setValue(true);
			} else if((ccfcOffSpeedCond && ccfcOffTempCond) && !ccfcForce) {
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
}

void boardHandleCan(CanCycle cycle) {
	boardRidingModesPublishLive();

	uint32_t tripDistanceMeters = engine->module<TripOdometer>()->getDistanceMeters();

	if (cycle.isInterval(CI::_10ms)) {
		CanTxMessage msg(CanCategory::NBC, 0x142);
		msg.setShortValueMsb(Sensor::getOrZero(SensorType::Rpm), 0x0);
		msg.setShortValueMsb(Sensor::getOrZero(SensorType::VehicleSpeed) * 10.f, 0x2);
		msg[0x4] = calculateHarleyGearValue();
		msg[6] = frameCounter142;
		msg[7] = crc8(msg.getFrame()->data8, 7);
		frameCounter142 = (frameCounter142 + 1) % 64;
	}

	if (cycle.isInterval(CI::_20ms)) {
		CanTxMessage msg(CanCategory::NBC, 0x144);

		const float rpm = Sensor::getOrZero(SensorType::Rpm);
		const float currentTps = Sensor::getOrZero(SensorType::Tps1);
		float targetTps = 0.0f;
		if (auto controller = engine->etbControllers[0]) {
			targetTps = controller->getCurrentTarget();
		}

		const float targetTorque = getEstimatedTorqueFromTable(rpm, targetTps);
		const float estimatedTorque = getEstimatedTorqueFromTable(rpm, currentTps);

		msg.setShortValueMsb(scaleTorqueForCan(targetTorque), 0x0); // TARGET ESTIMATED TORQUE
		msg.setShortValueMsb(scaleTorqueForCan(estimatedTorque), 0x2); // ACTUAL ESTIMATED TORQUE
		msg[4] = Sensor::getOrZero(SensorType::AcceleratorPedal) / 0.5;
		msg[5] = getHarleyTractionControlStatus();
		msg[6] = frameCounter144;
		msg[7] = crc8(msg.getFrame()->data8, 7);
		frameCounter144 = (frameCounter144 + 1) % 64;
	}

	if (cycle.isInterval(CI::_50ms)) {
		if (getCCStatus() == CruiseControlStatus::Enabled && !isCurrentSpeedAllowedForCurrentGear()) {
			setCCStatus(CruiseControlStatus::Standby);
		}

		bool running = engine->rpmCalculator.isRunning();

		{
			CanTxMessage msg(CanCategory::NBC, 0x146);
			msg[0] = (getCCStatus() == CruiseControlStatus::Enabled) ? 0x21 : 0x11;

			float desiredSpeed = getDesiredCCSpeed();
			if (desiredSpeed < 0) {
				desiredSpeed = 0;
			}
			uint16_t desiredSpeed10 = static_cast<uint16_t>(minF(desiredSpeed * 10.0f, 65535.0f));
			msg[1] = (desiredSpeed10 >> 8) & 0xFF;
			msg[2] = desiredSpeed10 & 0xFF;
			msg[3] = running ? 0x44 : 0x04;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = frameCounter146_342;
			msg[7] = crc8(msg.getFrame()->data8, 7);
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x342);

			// Remaining Range in Km = Tank volume [L] * (FuelLevel [0% - 100%] / 100) / Consumption [km / L]
			//                       = Tank volume [L] * (FuelLevel [0% - 100%] / 100) * (100 / Consumption [L / (100 km)])
			//                       = Tank volume [L] * (FuelLevel [0% - 100%]) / Consumption [L / (100 km)]
			uint16_t remainingRangeKM = static_cast<uint16_t>(minF(22.7f * (Sensor::getOrZero(SensorType::FuelLevel) / 5.5f), 65535.0f));

			switch (getCCStatus()) {
				case CruiseControlStatus::Enabled:
					msg[0] = 0xA4;
					break;
				case CruiseControlStatus::Standby:
					msg[0] = 0x64;
					break;
				case CruiseControlStatus::Disabled:
				default:
					msg[0] = 0x54; //0x56? 0b0101 0100 vs. 0b0101 0110 (0x54)
					break;
			}

			// msg[1]:  bit 7  6  5  4  3  2  1  0
			//                    │  |  │  │  │  └── range <= 16  km
			//                    │  |  │  │  └───── range <= 130 km (?)
			//                    │  |  │  └──────── !opsSwitchedState
			// 				      |  |  └─────────── opsSwitchedState
			//                    |  └────────────── Ignition Switch off
			//                    └───────────────── running (successfull start and running )
			if (running) {
				msg[1] |= 0x20;
			}

			if(harleyIgnitionOffRequested && !harleyIgnitionOnRequested) {
				msg[1] |= 0x10;
			}

			if (!getOPSState()) {
				msg[1] |= 0x8;
			} else {
				msg[1] |= 0x4;
			}

			if (remainingRangeKM < 30.f) {
				msg[1] |= 0x3;
			} else if (remainingRangeKM < 60.f) {
				msg[1] |= 0x2;
			}

			msg[2] = 0x54; // ?

			// msg[3]: Upper byte, lower nibble of 16-bit uint, capped lower 4-bit
			// msg[4]: Lower byte of 16-bit uint
			msg[3] = 0x10;
			msg[3] |= ((remainingRangeKM >> 8) & 0x0F);
			msg[4] = remainingRangeKM & 0xFF;

			msg[5] = Sensor::getOrZero(SensorType::FuelLevel);
			msg[6] = frameCounter146_342;
			msg[7] = crc8(msg.getFrame()->data8, 7);
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x344);
			msg[0] = 0x00;
			msg[1] = Sensor::getOrZero(SensorType::Clt) + 40;
			msg[2] = Sensor::getOrZero(SensorType::AuxTemp2) + 40;
			msg[3] = 0xFF;
			msg[4] = 0xCC;
			msg[5] = 0x21;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		frameCounter146_342 = (frameCounter146_342 + 1) % 64;
	}

	if (cycle.isInterval(CI::_200ms)) {
		CanTxMessage msg(CanCategory::NBC, 0x540);
		setFourBytesMsb(msg, tripDistanceMeters, 0);
		msg[4] = 0x00;
		msg[5] = 0x00;
		msg[6] = 0x00;
		msg[7] = 0x00;
	}

	if (cycle.isInterval(CI::_1000ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, 0x34D);
			msg[0] = engineConfiguration->vinNumber[0];
			msg[1] = engineConfiguration->vinNumber[1];
			msg[2] = engineConfiguration->vinNumber[2];
			msg[3] = engineConfiguration->vinNumber[3];
			msg[4] = engineConfiguration->vinNumber[4];
			msg[5] = engineConfiguration->vinNumber[5];
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x34E);
			msg[0] = engineConfiguration->vinNumber[6];
			msg[1] = engineConfiguration->vinNumber[7];
			msg[2] = engineConfiguration->vinNumber[8];
			msg[3] = engineConfiguration->vinNumber[9];
			msg[4] = engineConfiguration->vinNumber[10];
			msg[5] = engineConfiguration->vinNumber[11];
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x34F);
			msg[0] = engineConfiguration->vinNumber[12];
			msg[1] = engineConfiguration->vinNumber[13];
			msg[2] = engineConfiguration->vinNumber[14];
			msg[3] = engineConfiguration->vinNumber[15];
			msg[4] = engineConfiguration->vinNumber[16];
			msg[5] = engineConfiguration->vinNumber[17];
		}

		{
			uint8_t modeB0 = 0;
			uint8_t modeB1 = 0;
			boardRidingModesComposeTx148(modeB0, modeB1);

			CanTxMessage msg(CanCategory::NBC, 0x148);
			msg[0] = modeB0;
			msg[1] = modeB1;
			msg[2] = 0x00;
			msg[3] = 0x00;
			msg[4] = 0x00;
			msg[5] = 0x00;
			msg[6] = frameCounter148;
			msg[7] = crc8(msg.getFrame()->data8, 7);
		}
		frameCounter148++;
		if (frameCounter148 > 0x7F) {
			frameCounter148 = 0x40;
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x346);
			setFourBytesMsb(msg, tripDistanceMeters, 0);
			msg[4] = 0x00;
			msg[5] = Sensor::getOrZero(SensorType::AmbientTemperature) * 2 + 80;
			msg[6] = 0x80;
			msg[7] = 0x00;
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x348);
			msg[0] = 0x00;
			msg[1] = 0x00;
			msg[2] = 0x00;
			msg[3] = 0x0D;
			msg[4] = 0xAC;
			msg[5] = 0x00;
			msg[6] = 0x00;
			msg[7] = 0x00;
		}

		{
			CanTxMessage msg(CanCategory::NBC, 0x502, 0x1/* DLC */);
			msg[0] = harleyKeepAlive;
		}
	}
}

void boardProcessCanRx(size_t busIndex, const CANRxFrame& frame, efitick_t nowNt) {
	handleUdsCanRx(busIndex, frame, nowNt);

	if (CAN_SID(frame) == 0x134) {
		boardRidingModesProcessRx134(frame);
		boardRidingModesPublishLive();
	}

	if (CAN_SID(frame) == 0x500) {
			bool cfcRunning = cfcPin.getLogicValue();
			if(!cfcRunning || config->cfcDisableWhenEngineStopped)
				harleyKeepAlive = frame.data8[0];
	}

	if (CAN_SID(frame) == 0x15A) {
		harleyIgnitionOffRequested = (frame.data8[1] & 0x01) == 1 && (frame.data8[2] & 0x01) == 0;
		harleyIgnitionOnRequested = (frame.data8[1] & 0x01) == 0 && (frame.data8[2] & 0x01) == 1;
		if (harleyIgnitionOffRequested && !harleyIgnitionOffRequestedPrev) {
			doScheduleStopEngine(StopRequestedReason::StartButton);
			setCCStatus(CruiseControlStatus::Disabled);
			setDesiredCCSpeed(0.0f);
		}
		if (harleyIgnitionOnRequested && !harleyIgnitionOnRequestedPrev) {
			doCancelStopEngine();
		}
		harleyIgnitionOffRequestedPrev = harleyIgnitionOffRequested;
		harleyIgnitionOnRequestedPrev = harleyIgnitionOnRequested;
		bool rightHandBrake = frame.data8[5] & 0x40;
		if (getCCStatus() == CruiseControlStatus::Enabled && rightHandBrake) {
			setCCStatus(CruiseControlStatus::Standby);
		}
	}

	if (CAN_SID(frame) == 0x154) {
		bool cruiseEnablePressed = (frame.data8[2] & 0x10) != 0;
		bool cruiseDecPressed = (frame.data8[1] & 0x01) != 0;
		bool cruiseIncPressed = (frame.data8[1] & 0x10) != 0;
		const efitick_t cruiseHoldDelayNt = MS2NT(500);
		const efitick_t cruiseRepeatDelayNt = MS2NT(200);

		if (cruiseEnablePressed && !cruiseEnablePressedPrev) {
			if (getCCStatus() == CruiseControlStatus::Disabled) {
				setCCStatus(CruiseControlStatus::Standby);
			} else {
				setDesiredCCSpeed(0.0f);
				setCCStatus(CruiseControlStatus::Disabled);
			}
		}

		if (cruiseDecPressed && !cruiseDecPressedPrev) {
			cruiseDecPressStartNt = nowNt;
			cruiseDecLastRepeatNt = nowNt;
			if (getDesiredCCSpeed() > 0 && getCCStatus() == CruiseControlStatus::Enabled) {
				decreaseDesiredCCSpeedForCurrentGear();
			} else if (getCCStatus() == CruiseControlStatus::Standby) {
				engageCCAtCurrentSpeedForCurrentGear();
			}
		}
		if (!cruiseDecPressed && cruiseDecPressedPrev) {
			cruiseDecPressStartNt = 0;
			cruiseDecLastRepeatNt = 0;
		}
		if (cruiseDecPressed && cruiseDecPressStartNt != 0) {
			if ((nowNt - cruiseDecPressStartNt) >= cruiseHoldDelayNt &&
				(nowNt - cruiseDecLastRepeatNt) >= cruiseRepeatDelayNt) {
				if (getDesiredCCSpeed() > 0 && getCCStatus() == CruiseControlStatus::Enabled) {
					decreaseDesiredCCSpeedForCurrentGear();
				}
				cruiseDecLastRepeatNt = nowNt;
			}
		}

		if (cruiseIncPressed && !cruiseIncPressedPrev) {
			cruiseIncPressStartNt = nowNt;
			cruiseIncLastRepeatNt = nowNt;
			if (getCCStatus() == CruiseControlStatus::Standby) {
				resumeCCForCurrentGear();
			} else if (getCCStatus() == CruiseControlStatus::Enabled) {
				increaseDesiredCCSpeedForCurrentGear();
			}
		}
		if (!cruiseIncPressed && cruiseIncPressedPrev) {
			cruiseIncPressStartNt = 0;
			cruiseIncLastRepeatNt = 0;
		}
		if (cruiseIncPressed && cruiseIncPressStartNt != 0) {
			if ((nowNt - cruiseIncPressStartNt) >= cruiseHoldDelayNt &&
				(nowNt - cruiseIncLastRepeatNt) >= cruiseRepeatDelayNt) {
				if (getCCStatus() == CruiseControlStatus::Enabled) {
					increaseDesiredCCSpeedForCurrentGear();
				}
				cruiseIncLastRepeatNt = nowNt;
			}
		}

		cruiseEnablePressedPrev = cruiseEnablePressed;
		cruiseDecPressedPrev = cruiseDecPressed;
		cruiseIncPressedPrev = cruiseIncPressed;
	}

	if (CAN_SID(frame) == 0x152) {
		uint8_t clutchState = frame.data8[3] & 0x30;
		bool clutchEngagedLight = clutchState == 0x20;
		bool clutchEngagedStrong = clutchState == 0x30;
		if (getCCStatus() == CruiseControlStatus::Enabled && (clutchEngagedLight || clutchEngagedStrong)) {
			setCCStatus(CruiseControlStatus::Standby);
		}
	}

	if (CAN_SID(frame) == 0x133) {
		bool footBrakeEngaged = (frame.data8[3] & 0x10) != 0;
		if (getCCStatus() == CruiseControlStatus::Enabled && footBrakeEngaged) {
			setCCStatus(CruiseControlStatus::Standby);
		}
	}

	if(CAN_SID(frame) == 0x3C4) {
		// frame.data8[4]:  bit 7  6  5  4  3  2  1  0
		//                      │  │     │           └── Unknown sub-state
		//                      │  │     └────────────── Front fog lights
		//                      └──┴──────────────────── mode: 0b00=inactive, 0b01=Off, 0b10=On, 0b11=Auto
		uint8_t ifcuVehicleFunctionSetup = frame.data8[4];
		if((ifcuVehicleFunctionSetup & 0xC0) == 0x00) {  
			ccfcActivated = false;
			ccfcMode      = ccfcModes_e::Off;
		} else {
			ccfcActivated = true;
			if((ifcuVehicleFunctionSetup & 0xC0) == 0x40) 
				ccfcMode = ccfcModes_e::Off;
			else if((ifcuVehicleFunctionSetup & 0xC0) == 0xC0)  	                     
				ccfcMode = ccfcModes_e::Auto;
			else if((ifcuVehicleFunctionSetup & 0xC0) == 0x80)                         
				ccfcMode = ccfcModes_e::On;
		}
	}

	if (CAN_SID(frame) == 0x3C6) {
		uint32_t newOdometer = getFourBytesMsb(frame, 0);
		if (newOdometer != lastReceivedOdometer) {
			// Speedometer has a new odometer for us, set this as base value and count from that upwards
			lastReceivedOdometer = newOdometer;
			engine->module<TripOdometer>()->setDistanceMeters(newOdometer);
		}
	}
}
