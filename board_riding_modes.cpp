#include "pch.h"

#include "board_riding_modes.h"

static constexpr uint8_t HD_MODE_ROAD = 0x1;
static constexpr uint8_t HD_MODE_SPORT = 0x3;
static constexpr uint8_t HD_MODE_TRACK = 0x4;
static constexpr uint8_t HD_MODE_TRACK_PLUS = 0x7;
static constexpr uint8_t HD_MODE_USER_A = 0x8;
static constexpr uint8_t HD_MODE_USER_B = 0x9;
static constexpr uint8_t HD_MODE_RAIN = 0xC;

extern StoredValueSensor luaGauges[LUA_GAUGE_COUNT];

namespace {
static constexpr float ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_1 = 1.7f;
static constexpr float ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_2 = 1.2f;
static constexpr float ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_3 = 0.8f;
static constexpr float ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_4 = 0.5f;
static constexpr float ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_5 = 0.0f;
static constexpr float ENGINE_BRAKING_DEFAULT_RPM_ENGAGE = 1300.0f;
static constexpr float ENGINE_BRAKING_DEFAULT_RPM_FULL = 4500.0f;
static constexpr float ENGINE_BRAKING_DEFAULT_MIN_VSS = 3.0f;
static constexpr float ENGINE_BRAKING_DEFAULT_MAX_BASE_ETB_TARGET = 10.0f;
static constexpr float ENGINE_BRAKING_DEFAULT_THROTTLE_REOPEN_TAPER = 0.3f;

struct HarleyRideModeState {
	uint8_t activeMode = HD_MODE_SPORT;
	uint8_t requestedMode = HD_MODE_SPORT;
	uint8_t engineMap = 0x3;
	uint8_t engineBrake = 0x1;
	uint8_t throttleResponse = 0x1;
	float engineBrakeEtbOffset = 0.0f;
	float lastAdjustedEtbTarget = 0.0f;
	float throttleReopenStartTarget = 0.0f;
	efitimeus_t throttleReopenStartTimeUs = 0;
	bool hasLastAdjustedEtbTarget = false;
	bool wasInIdleOrCoasting = false;
	bool throttleReopenTaperActive = false;
};

HarleyRideModeState harleyRideModeState;

uint8_t highNibble(uint8_t value) {
	return (value >> 4) & 0x0F;
}

uint8_t lowNibble(uint8_t value) {
	return value & 0x0F;
}

bool isValidHarleyRideMode(uint8_t mode) {
	switch (mode) {
		case HD_MODE_ROAD:
		case HD_MODE_SPORT:
		case HD_MODE_TRACK:
		case HD_MODE_TRACK_PLUS:
		case HD_MODE_USER_A:
		case HD_MODE_USER_B:
		case HD_MODE_RAIN:
			return true;
		default:
			return false;
	}
}

bool isValidHarleyEngineMap(uint8_t map) {
	switch (map) {
		case 0x1:
		case 0x2:
		case 0x3:
		case 0x5:
			return true;
		default:
			return false;
	}
}

bool isValidHarleyEngineBrake(uint8_t engineBrake) {
	return engineBrake >= 0x1 && engineBrake <= 0x5;
}

bool isValidHarleyThrottleResponse(uint8_t throttleResponse) {
	return throttleResponse >= 0x1 && throttleResponse <= 0x5;
}

float getDecelEtbOffsetByEngineBrakeMode(uint8_t engineBrakeMode) {
	// Higher engineBrakeMode means stronger engine braking (more throttle closing).
	switch (engineBrakeMode) {
		case 0x1:
			return config->engineBrakingEtbOffsetMode1;
		case 0x2:
			return config->engineBrakingEtbOffsetMode2;
		case 0x3:
			return config->engineBrakingEtbOffsetMode3;
		case 0x4:
			return config->engineBrakingEtbOffsetMode4;
		case 0x5:
			return config->engineBrakingEtbOffsetMode5;
		default:
			return config->engineBrakingEtbOffsetMode5;
	}
}
} // namespace

void boardRidingModesApplyDefaults() {
	config->engineBrakingEtbOffsetMode1 = ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_1;
	config->engineBrakingEtbOffsetMode2 = ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_2;
	config->engineBrakingEtbOffsetMode3 = ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_3;
	config->engineBrakingEtbOffsetMode4 = ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_4;
	config->engineBrakingEtbOffsetMode5 = ENGINE_BRAKING_DEFAULT_ETB_OFFSET_MODE_5;
	config->engineBrakingRpmEngage = ENGINE_BRAKING_DEFAULT_RPM_ENGAGE;
	config->engineBrakingRpmFull = ENGINE_BRAKING_DEFAULT_RPM_FULL;
	config->engineBrakingMinVss = ENGINE_BRAKING_DEFAULT_MIN_VSS;
	config->engineBrakingMaxBaseEtbTarget = ENGINE_BRAKING_DEFAULT_MAX_BASE_ETB_TARGET;
	config->engineBrakingThrottleReopenTaper = ENGINE_BRAKING_DEFAULT_THROTTLE_REOPEN_TAPER;
}

void boardRidingModesPublishLive() {
	const efitick_t nowNt = getTimeNowNt();
	luaGauges[0].setValidValue(harleyRideModeState.activeMode, nowNt);
	luaGauges[1].setValidValue(harleyRideModeState.requestedMode, nowNt);
	luaGauges[2].setValidValue(harleyRideModeState.engineMap, nowNt);
	luaGauges[3].setValidValue(harleyRideModeState.engineBrake, nowNt);
	luaGauges[4].setValidValue(harleyRideModeState.throttleResponse, nowNt);
	luaGauges[5].setValidValue(harleyRideModeState.engineBrakeEtbOffset, nowNt);
}

void boardRidingModesProcessRx134(const CANRxFrame& frame) {
	const uint8_t engineMap = highNibble(frame.data8[0]);
	const uint8_t engineBrake = lowNibble(frame.data8[0]);
	const uint8_t throttleResponse = highNibble(frame.data8[1]);
	const uint8_t activeConfirmedMode = lowNibble(frame.data8[2]);
	const uint8_t requestedMode = highNibble(frame.data8[3]);

	if (isValidHarleyEngineMap(engineMap)) {
		harleyRideModeState.engineMap = engineMap;
	}

	if (isValidHarleyEngineBrake(engineBrake)) {
		harleyRideModeState.engineBrake = engineBrake;
	}

	if (isValidHarleyThrottleResponse(throttleResponse)) {
		harleyRideModeState.throttleResponse = throttleResponse;
	}

	if (isValidHarleyRideMode(requestedMode)) {
		harleyRideModeState.requestedMode = requestedMode;
		harleyRideModeState.activeMode = requestedMode;
	} else if (isValidHarleyRideMode(activeConfirmedMode)) {
		harleyRideModeState.activeMode = activeConfirmedMode;
	}
}

void boardRidingModesComposeTx148(uint8_t& b0, uint8_t& b1) {
	b0 = (harleyRideModeState.engineMap << 4) | harleyRideModeState.engineBrake;
	b1 = (harleyRideModeState.throttleResponse << 4);
}

uint8_t boardGetHarleyEngineMap() {
	return harleyRideModeState.engineMap;
}

float boardAdjustEtbTarget(float currentEtbTarget) {
	auto& idleController = engine->module<IdleController>().unmock();

	if (!engine->rpmCalculator.isRunning()) {
		harleyRideModeState.engineBrakeEtbOffset = 0.0f;
		harleyRideModeState.throttleReopenTaperActive = false;
		harleyRideModeState.hasLastAdjustedEtbTarget = false;
		harleyRideModeState.wasInIdleOrCoasting = false;
		return currentEtbTarget;
	}

	percent_t adjustedEtbTarget = currentEtbTarget;
	auto app = Sensor::get(SensorType::AcceleratorPedal);
	bool isPedalClosed = app.Valid && app.Value <= 3.0f;
	bool isCurrentlyCoasting = idleController.isIdleCoasting;

	if (isPedalClosed && isCurrentlyCoasting) {
		float rpm = Sensor::getOrZero(SensorType::Rpm);
		float vss = Sensor::getOrZero(SensorType::VehicleSpeed);

		float minRpmEngage = config->engineBrakingRpmEngage;
		if (minRpmEngage < 0.0f) {
			minRpmEngage = 0.0f;
		}

		float rpmFullEffect = config->engineBrakingRpmFull;
		if (rpmFullEffect <= minRpmEngage) {
			rpmFullEffect = minRpmEngage + 1.0f;
		}

		float minVss = config->engineBrakingMinVss;
		if (minVss < 0.0f) {
			minVss = 0.0f;
		}

		float maxBaseEtbTarget = config->engineBrakingMaxBaseEtbTarget;
		if (maxBaseEtbTarget < 0.0f) {
			maxBaseEtbTarget = 0.0f;
		}

		// Only influence closed-throttle decel, not idle or pedal-driven operation.
		if (rpm >= minRpmEngage && vss >= minVss && currentEtbTarget <= maxBaseEtbTarget) {
			float modeOffset = getDecelEtbOffsetByEngineBrakeMode(harleyRideModeState.engineBrake);
			float rpmFactor = interpolateClamped(minRpmEngage, 0.0f, rpmFullEffect, 1.0f, rpm);
			harleyRideModeState.engineBrakeEtbOffset = modeOffset * rpmFactor;
			adjustedEtbTarget += harleyRideModeState.engineBrakeEtbOffset;
		} else {
			harleyRideModeState.engineBrakeEtbOffset = 0.0f;
		}
	} else {
		harleyRideModeState.engineBrakeEtbOffset = 0.0f;
	}

	bool isInIdleOrCoasting = idleController.isIdling || idleController.isIdleCoasting;
	float reopenTaperSec = config->engineBrakingThrottleReopenTaper;
	if (reopenTaperSec < 0.0f) {
		reopenTaperSec = 0.0f;
	}

	if (!harleyRideModeState.hasLastAdjustedEtbTarget) {
		harleyRideModeState.lastAdjustedEtbTarget = adjustedEtbTarget;
		harleyRideModeState.hasLastAdjustedEtbTarget = true;
	}

	bool isLeavingIdleOrCoasting = harleyRideModeState.wasInIdleOrCoasting && !isInIdleOrCoasting;
	if (isLeavingIdleOrCoasting
		&& reopenTaperSec > 0.0f
		&& adjustedEtbTarget > harleyRideModeState.lastAdjustedEtbTarget) {
		harleyRideModeState.throttleReopenTaperActive = true;
		harleyRideModeState.throttleReopenStartTarget = harleyRideModeState.lastAdjustedEtbTarget;
		harleyRideModeState.throttleReopenStartTimeUs = getTimeNowUs();
	}

	if (harleyRideModeState.throttleReopenTaperActive) {
		bool shouldCancelRamp = isInIdleOrCoasting || adjustedEtbTarget <= harleyRideModeState.lastAdjustedEtbTarget;
		if (shouldCancelRamp) {
			harleyRideModeState.throttleReopenTaperActive = false;
		} else {
			float elapsedSec = (getTimeNowUs() - harleyRideModeState.throttleReopenStartTimeUs) / US_PER_SECOND_F;
			if (elapsedSec >= reopenTaperSec) {
				harleyRideModeState.throttleReopenTaperActive = false;
			} else {
				adjustedEtbTarget = interpolateClamped(
					0.0f, harleyRideModeState.throttleReopenStartTarget,
					reopenTaperSec, adjustedEtbTarget,
					elapsedSec
				);
			}
		}
	}

	harleyRideModeState.wasInIdleOrCoasting = isInIdleOrCoasting;
	harleyRideModeState.lastAdjustedEtbTarget = adjustedEtbTarget;
	return adjustedEtbTarget;
}
