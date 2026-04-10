#include "pch.h"

#include "board_riding_modes.h"

#include <algorithm>

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
static constexpr float ETB_TARGET_SLEW_DEFAULT_MAX_DOWN_RATE = 600.0f;

static constexpr float etbTargetSlewDefaultOpeningBins[ETB_TARGET_SLEW_BINS_COUNT] = {
	0.0f, 5.0f, 10.0f, 15.0f, 20.0f, 25.0f, 30.0f, 40.0f, 50.0f, 60.0f, 80.0f, 100.0f
};

static constexpr float etbTargetSlewDefaultMaxUpRate[ETB_TARGET_SLEW_BINS_COUNT] = {
	20.0f, 25.0f, 50.0f, 62.5f, 100.0f, 166.7f, 200.0f, 230.0f, 260.0f, 300.0f, 350.0f, 400.0f
};

struct HarleyRideModeState {
	uint8_t activeMode = HD_MODE_SPORT;
	uint8_t requestedMode = HD_MODE_SPORT;
	uint8_t engineMap = 0x3;
	uint8_t engineBrake = 0x1;
	uint8_t throttleResponse = 0x1;
	float engineBrakeEtbOffset = 0.0f;
};

HarleyRideModeState harleyRideModeState;

struct EtbTargetSlewState {
	Timer timer;
	float limitedTarget = 0.0f;
	bool initialized = false;
};

EtbTargetSlewState etbTargetSlewState;

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

float applyEngineBrakingOffset(float currentEtbTarget) {
	if (!engine->rpmCalculator.isRunning()) {
		harleyRideModeState.engineBrakeEtbOffset = 0.0f;
		return currentEtbTarget;
	}

	auto app = Sensor::get(SensorType::AcceleratorPedal);
	if (!app || app.Value > 1.0f) {
		harleyRideModeState.engineBrakeEtbOffset = 0.0f;
		return currentEtbTarget;
	}

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

	bool isCurrentlyCoasting = engine->module<IdleController>().unmock().isIdleCoasting;
	// Only influence closed-throttle decel, not idle or pedal-driven operation.
	if (rpm < minRpmEngage || vss < minVss || currentEtbTarget > maxBaseEtbTarget || !isCurrentlyCoasting) {
		harleyRideModeState.engineBrakeEtbOffset = 0.0f;
		return currentEtbTarget;
	}

	float modeOffset = getDecelEtbOffsetByEngineBrakeMode(harleyRideModeState.engineBrake);
	float rpmFactor = interpolateClamped(minRpmEngage, 0.0f, rpmFullEffect, 1.0f, rpm);
	harleyRideModeState.engineBrakeEtbOffset = modeOffset * rpmFactor;

	return currentEtbTarget + harleyRideModeState.engineBrakeEtbOffset;
}

float applyEtbTargetSlewLimit(float requestedEtbTarget) {
	auto& state = etbTargetSlewState;

	if (!engine->rpmCalculator.isRunning()) {
		state.initialized = false;
		return requestedEtbTarget;
	}

	// Invalid pedal should immediately pass through the safe (usually closed) command.
	auto app = Sensor::get(SensorType::AcceleratorPedal);
	if (!app) {
		state.initialized = false;
		return requestedEtbTarget;
	}

	efitick_t nowNt = getTimeNowNt();
	if (!state.initialized) {
		state.timer.reset(nowNt);
		state.limitedTarget = requestedEtbTarget;
		state.initialized = true;
		return requestedEtbTarget;
	}

	float dt = state.timer.getElapsedSecondsAndReset(nowNt);
	constexpr float ETB_TARGET_SLEW_MAX_DT = 0.25f;
	if (dt <= 0.0f || dt > ETB_TARGET_SLEW_MAX_DT) {
		// After long pauses, re-sync to avoid one giant stale-dt slew step.
		state.limitedTarget = requestedEtbTarget;
		return requestedEtbTarget;
	}

	float upRate = interpolate2d(state.limitedTarget, config->etbTargetSlewOpeningBins, config->etbTargetSlewMaxUpRate);
	if (upRate < 0.0f) {
		upRate = 0.0f;
	}

	float downRate = config->etbTargetSlewMaxDownRate;
	if (downRate <= 0.0f) {
		downRate = 10000.0f;
	}

	float limitedTarget = state.limitedTarget;
	if (requestedEtbTarget > limitedTarget) {
		limitedTarget += std::min((requestedEtbTarget - limitedTarget), upRate * dt);
	} else if (requestedEtbTarget < limitedTarget) {
		limitedTarget -= std::min((limitedTarget - requestedEtbTarget), downRate * dt);
	}

	state.limitedTarget = clampPercentValue(limitedTarget);
	return state.limitedTarget;
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
	config->etbTargetSlewMaxDownRate = ETB_TARGET_SLEW_DEFAULT_MAX_DOWN_RATE;

	copyArray(config->etbTargetSlewOpeningBins, etbTargetSlewDefaultOpeningBins);
	copyArray(config->etbTargetSlewMaxUpRate, etbTargetSlewDefaultMaxUpRate);
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
	float targetWithEngineBraking = applyEngineBrakingOffset(currentEtbTarget);
	return applyEtbTargetSlewLimit(targetWithEngineBraking);
}
