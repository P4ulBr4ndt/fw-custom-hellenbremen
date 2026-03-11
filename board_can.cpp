#include "pch.h"

#include "board_can.h"

#include <cmath>
#include <cstdint>

#include "board_riding_modes.h"
#include "board_uds.h"
#include "cruise_control.h"
#include "shutdown_controller.h"

#define CAN_HD_VSS_ID 0x142
#define CAN_HD_THROTTLE_ID 0x144

#define CAN_HD_RPM_OFFSET 0x0
#define CAN_HD_VSS_OFFSET 0x2
#define CAN_HD_GEAR_OFFSET 0x4

#define CAN_HD_BCM_PING_ID 0x503 // Here we receive a "Ping" kind of thing
#define CAN_HD_ECM_PING_ID 0x502 // Here we send our own

#define CAN_HD_VIN_ID_1 0x34D
#define CAN_HD_VIN_ID_2 0x34E
#define CAN_HD_VIN_ID_3 0x34F
#define CAN_HD_RIDING_MODE_ID 0x134
#define CAN_HD_RIDING_MODE_CONFIRM_ID 0x148

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
static bool tripResetPressedPrev = false;
static bool tripResetHoldHandled = false;
static efitick_t tripResetPressStartNt = 0;
static bool jssStopRequestActive = false;

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

void boardPeriodicSlowCallback() {
	bool jssDown = engine->engineState.jssState != 0;
	uint8_t currentGear = calculateHarleyGearIndex();
	bool isNeutral = currentGear == 0;
	bool isEngineActive = engine->rpmCalculator.isRunning() || engine->rpmCalculator.isCranking();

	bool shouldRequestStop = jssDown && !isNeutral && isEngineActive;
	if (shouldRequestStop && !jssStopRequestActive) {
		doScheduleStopEngine(StopRequestedReason::Board1);
	}

	jssStopRequestActive = shouldRequestStop;
}

void boardHandleCan(CanCycle cycle) {
	boardRidingModesPublishLive();

	uint32_t tripDistanceMeters = engine->module<TripOdometer>()->getDistanceMeters();

	if (cycle.isInterval(CI::_10ms)) {
		CanTxMessage msg(CanCategory::NBC, CAN_HD_VSS_ID);
		msg.setShortValueMsb(Sensor::getOrZero(SensorType::Rpm), CAN_HD_RPM_OFFSET);
		msg.setShortValueMsb(Sensor::getOrZero(SensorType::VehicleSpeed) * 10.f, CAN_HD_VSS_OFFSET);
		msg[CAN_HD_GEAR_OFFSET] = calculateHarleyGearValue();
		msg[6] = frameCounter142;
		msg[7] = crc8(msg.getFrame()->data8, 7);
		frameCounter142 = (frameCounter142 + 1) % 64;
	}

	if (cycle.isInterval(CI::_20ms)) {
		CanTxMessage msg(CanCategory::NBC, CAN_HD_THROTTLE_ID);
		msg.setShortValueMsb(Sensor::getOrZero(SensorType::Tps1Primary), 0); // TARGET TPS?
		msg.setShortValueMsb(Sensor::getOrZero(SensorType::Tps1Secondary), 2); // ACTUAL TPS?
		msg[4] = Sensor::getOrZero(SensorType::AcceleratorPedal) / 0.4545; // As OEM does
		msg[5] = 0x00;
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
			uint16_t remainingRangeKM = static_cast<uint16_t>(minF(22.7f * (Sensor::getOrZero(SensorType::FuelLevel) / 100) * (100.0f / 5.5f), 65535.0f));
			msg[0] = 0x54;
			switch (getCCStatus()) {
				case CruiseControlStatus::Enabled:
					msg[0] = 0xA4;
					break;
				case CruiseControlStatus::Standby:
					msg[0] = 0x64;
					break;
				case CruiseControlStatus::Disabled:
				default:
					msg[0] = 0x54;
					break;
			}

			msg[1] = running ? 0x24 : 0x00;
			if (engine->opsSwitchedState) {
				msg[1] |= 0x8;
			}
			if (remainingRangeKM < 60.f) {
				msg[1] |= 0x2;
			}
			if (remainingRangeKM < 30.f) {
				msg[1] |= 0x3;
			}
			msg[2] = 0x54;
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
		msg[0] = (tripDistanceMeters >> 24) & 0xFF;
		msg[1] = (tripDistanceMeters >> 16) & 0xFF;
		msg[2] = (tripDistanceMeters >> 8) & 0xFF;
		msg[3] = tripDistanceMeters & 0xFF;
		msg[4] = 0x00;
		msg[5] = 0x00;
		msg[6] = 0x00;
		msg[7] = 0x00;
	}

	if (cycle.isInterval(CI::_1000ms)) {
		{
			CanTxMessage msg(CanCategory::NBC, CAN_HD_VIN_ID_1);
			msg[0] = engineConfiguration->vinNumber[0];
			msg[1] = engineConfiguration->vinNumber[1];
			msg[2] = engineConfiguration->vinNumber[2];
			msg[3] = engineConfiguration->vinNumber[3];
			msg[4] = engineConfiguration->vinNumber[4];
			msg[5] = engineConfiguration->vinNumber[5];
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_HD_VIN_ID_2);
			msg[0] = engineConfiguration->vinNumber[6];
			msg[1] = engineConfiguration->vinNumber[7];
			msg[2] = engineConfiguration->vinNumber[8];
			msg[3] = engineConfiguration->vinNumber[9];
			msg[4] = engineConfiguration->vinNumber[10];
			msg[5] = engineConfiguration->vinNumber[11];
		}

		{
			CanTxMessage msg(CanCategory::NBC, CAN_HD_VIN_ID_3);
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

			CanTxMessage msg(CanCategory::NBC, CAN_HD_RIDING_MODE_CONFIRM_ID);
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
			msg[0] = (tripDistanceMeters >> 24) & 0xFF;
			msg[1] = (tripDistanceMeters >> 16) & 0xFF;
			msg[2] = (tripDistanceMeters >> 8) & 0xFF;
			msg[3] = tripDistanceMeters & 0xFF;
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
			CanTxMessage msg(CanCategory::NBC, CAN_HD_ECM_PING_ID, 0x1/* DLC */);
			msg[0] = harleyKeepAlive;
		}
	}
}

void boardProcessCanRx(size_t busIndex, const CANRxFrame& frame, efitick_t nowNt) {
	handleUdsCanRx(busIndex, frame, nowNt);

	if (CAN_SID(frame) == CAN_HD_RIDING_MODE_ID) {
		boardRidingModesProcessRx134(frame);
		boardRidingModesPublishLive();
	}

	if (CAN_SID(frame) == 0x500) {
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

	if (CAN_SID(frame) == 0x354) {
		bool resetTripOdometer = (frame.data8[0] & 0x01) != 0;
		const efitick_t tripResetHoldDelayNt = MS2NT(2000);

		if (resetTripOdometer && !tripResetPressedPrev) {
			tripResetPressStartNt = nowNt;
			tripResetHoldHandled = false;
		}
		if (!resetTripOdometer && tripResetPressedPrev) {
			tripResetPressStartNt = 0;
			tripResetHoldHandled = false;
		}
		if (resetTripOdometer && tripResetPressStartNt != 0 && !tripResetHoldHandled) {
			if ((nowNt - tripResetPressStartNt) >= tripResetHoldDelayNt) {
				engine->module<TripOdometer>()->reset();
				tripResetHoldHandled = true;
			}
		}

		tripResetPressedPrev = resetTripOdometer;
	}
}
