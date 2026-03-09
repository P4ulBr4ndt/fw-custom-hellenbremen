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
struct HarleyRideModeState {
	uint8_t activeMode = HD_MODE_SPORT;
	uint8_t requestedMode = HD_MODE_SPORT;
	uint8_t engineMap = 0x3;
	uint8_t engineBrake = 0x1;
	uint8_t throttleResponse = 0x1;
	float engineBrakeEtbOffset = 0.0f;
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
			return 1.7f;
		case 0x2:
			return 1.2f;
		case 0x3:
			return 0.8f;
		case 0x4:
			return 0.5f;
		case 0x5:
			return 0.0f;
		default:
			return 0.0f;
	}
}
} // namespace

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

	bool isCurrentlyCoasting = engine->module<IdleController>().unmock().isIdleCoasting;
	// Only influence closed-throttle decel, not idle or pedal-driven operation.
	if (rpm < 1300.0f || vss < 3.0f || currentEtbTarget > 10.0f || !isCurrentlyCoasting) {
		harleyRideModeState.engineBrakeEtbOffset = 0.0f;
		return currentEtbTarget;
	}

	float modeOffset = getDecelEtbOffsetByEngineBrakeMode(harleyRideModeState.engineBrake);
	float rpmFactor = interpolateClamped(1300.0f, 0.0f, 4500.0f, 1.0f, rpm);
	harleyRideModeState.engineBrakeEtbOffset = modeOffset * rpmFactor;

	return currentEtbTarget + harleyRideModeState.engineBrakeEtbOffset;
}
