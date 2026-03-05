#include "pch.h"

#include "board_instant_accel_shot.h"

#include <algorithm>
#include <cstdint>

#include "injection_gpio.h"

#define INSTANT_ACCEL_SHOT_WINDOW_MS 80
#define INSTANT_ACCEL_SHOT_WINDOW_MAX_MS 200
#define INSTANT_ACCEL_SHOT_TOPUP_MIN_SPACING_MS 5.0f
#define INSTANT_ACCEL_SHOT_TOPUP_MIN_INCREMENT_MS 2.0f
#define INSTANT_ACCEL_SHOT_MIN_DELTA_TPS 3.0f

static constexpr float instantAccelShotDefaultTpsBins[] = {0.0f, 20.0f, 40.0f, 60.0f, 80.0f, 100.0f};
static constexpr int16_t instantAccelShotDefaultCltCorrBins[] = {-40, -20, 0, 20, 40, 60, 80, 100, 125, 150};
static constexpr float instantAccelShotDefaultCltCorrMult[] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};

void boardInstantAccelApplyDefaults() {
	config->instantAccelShotWindowMs = INSTANT_ACCEL_SHOT_WINDOW_MS;
	config->instantAccelShotTopupMinSpacingMs = INSTANT_ACCEL_SHOT_TOPUP_MIN_SPACING_MS;
	config->instantAccelShotTopupMinIncrementMs = INSTANT_ACCEL_SHOT_TOPUP_MIN_INCREMENT_MS;
	config->instantAccelShotMinDeltaTps = INSTANT_ACCEL_SHOT_MIN_DELTA_TPS;

	for (auto& row : config->instantAccelShotTpsTable) {
		for (auto& cell : row) {
			cell = 0;
		}
	}

	for (size_t i = 0; i < efi::size(config->instantAccelShotTpsFromBins); i++) {
		config->instantAccelShotTpsFromBins[i] = 0;
	}

	for (size_t i = 0; i < efi::size(config->instantAccelShotTpsToBins); i++) {
		config->instantAccelShotTpsToBins[i] = 0;
	}

	for (size_t i = 0; i < efi::size(config->instantAccelShotCltCorrBins); i++) {
		config->instantAccelShotCltCorrBins[i] = 0;
		config->instantAccelShotCltCorrMult[i] = 1.0f;
	}

	size_t defaultCount = efi::size(config->instantAccelShotTpsFromBins);
	size_t defaultValues = efi::size(instantAccelShotDefaultTpsBins);
	if (defaultValues < defaultCount) {
		defaultCount = defaultValues;
	}
	for (size_t i = 0; i < defaultCount; i++) {
		config->instantAccelShotTpsFromBins[i] = instantAccelShotDefaultTpsBins[i];
		config->instantAccelShotTpsToBins[i] = instantAccelShotDefaultTpsBins[i];
	}

	size_t cltDefaultCount = efi::size(config->instantAccelShotCltCorrBins);
	size_t cltDefaultValues = efi::size(instantAccelShotDefaultCltCorrBins);
	if (cltDefaultValues < cltDefaultCount) {
		cltDefaultCount = cltDefaultValues;
	}
	for (size_t i = 0; i < cltDefaultCount; i++) {
		config->instantAccelShotCltCorrBins[i] = instantAccelShotDefaultCltCorrBins[i];
		config->instantAccelShotCltCorrMult[i] = instantAccelShotDefaultCltCorrMult[i];
	}
}

namespace {
struct InstantAccelShotState {
	static constexpr int kRawSamples = static_cast<int>(INSTANT_ACCEL_SHOT_WINDOW_MAX_MS / FAST_CALLBACK_PERIOD_MS) + 2;
	static constexpr int kSamples = (kRawSamples < 2) ? 2 : kRawSamples;

	float tps[kSamples];
	efitick_t timeNt[kSamples];
	int head;
	int count;
	bool latched;
	float deliveredPulseMs;
	efitick_t lastShotTimeNt;
};

InstantAccelShotState instantAccelShotState;

void resetInstantAccelShot() {
	instantAccelShotState.head = 0;
	instantAccelShotState.count = 0;
	instantAccelShotState.latched = false;
	instantAccelShotState.deliveredPulseMs = 0;
	instantAccelShotState.lastShotTimeNt = 0;
}

float getInstantAccelShotPulse(float tpsFrom, float tpsTo) {
	float basePulseMs = interpolate3d(config->instantAccelShotTpsTable,
		config->instantAccelShotTpsToBins, tpsTo,
		config->instantAccelShotTpsFromBins, tpsFrom);
	if (basePulseMs < 0) {
		basePulseMs = 0;
	}

	float clt = Sensor::getOrZero(SensorType::Clt);
	float cltMult = interpolate2d(clt, config->instantAccelShotCltCorrBins,
		config->instantAccelShotCltCorrMult);
	if (cltMult < 0) {
		cltMult = 0;
	}

	return basePulseMs * cltMult;
}

void updateInstantAccelShot() {
	if (!engine->rpmCalculator.isRunning()) {
		resetInstantAccelShot();
		return;
	}

	auto tps = Sensor::get(SensorType::Tps1);
	if (!tps) {
		resetInstantAccelShot();
		return;
	}

	float windowMs = config->instantAccelShotWindowMs;
	if (windowMs <= 0) {
		instantAccelShotState.latched = false;
		return;
	}

	if (windowMs > INSTANT_ACCEL_SHOT_WINDOW_MAX_MS) {
		windowMs = INSTANT_ACCEL_SHOT_WINDOW_MAX_MS;
	}

	auto nowNt = getTimeNowNt();

	instantAccelShotState.tps[instantAccelShotState.head] = tps.Value;
	instantAccelShotState.timeNt[instantAccelShotState.head] = nowNt;
	instantAccelShotState.head = (instantAccelShotState.head + 1) % InstantAccelShotState::kSamples;
	if (instantAccelShotState.count < InstantAccelShotState::kSamples) {
		instantAccelShotState.count++;
	}

	efitick_t earliestNt = nowNt - MSF2NT(windowMs);
	float minTps = tps.Value;
	int validSamples = 0;

	for (int i = 0; i < instantAccelShotState.count; i++) {
		int idx = instantAccelShotState.head - 1 - i;
		if (idx < 0) {
			idx += InstantAccelShotState::kSamples;
		}
		if (instantAccelShotState.timeNt[idx] < earliestNt) {
			continue;
		}
		float sampleTps = instantAccelShotState.tps[idx];
		if (sampleTps < minTps) {
			minTps = sampleTps;
		}
		validSamples++;
	}

	if (validSamples < 2) {
		instantAccelShotState.latched = false;
		return;
	}

	float tpsFrom = minTps;
	float tpsTo = tps.Value;
	float deltaTps = tpsTo - tpsFrom;
	float pulseMs = 0;
	bool aboveThreshold = false;
	float minDeltaTps = std::max(0.0f, config->instantAccelShotMinDeltaTps);
	if (deltaTps >= minDeltaTps) {
		pulseMs = getInstantAccelShotPulse(tpsFrom, tpsTo);
		aboveThreshold = pulseMs > 2.0f; // This could also be user editable, for now no relevance
	}
	float spacingMs = std::max(0.0f, config->instantAccelShotTopupMinSpacingMs);
	float minIncrementMs = std::max(0.0f, config->instantAccelShotTopupMinIncrementMs);

	if (aboveThreshold) {
		if (!instantAccelShotState.latched) {
			if (pulseMs > 0) {
				startSimultaneousInjection();
				auto endTime = sumTickAndFloat(nowNt, MSF2NT(pulseMs));
				getScheduler()->schedule("instantAccelShot", nullptr, endTime,
					action_s::make<endSimultaneousInjectionOnlyTogglePins>());
				instantAccelShotState.deliveredPulseMs = pulseMs;
				instantAccelShotState.lastShotTimeNt = nowNt;
			}
			instantAccelShotState.latched = true;
		} else {
			float extraPulseMs = pulseMs - instantAccelShotState.deliveredPulseMs;
			bool spacingOk = (nowNt - instantAccelShotState.lastShotTimeNt) >= MSF2NT(spacingMs);
			if (extraPulseMs >= minIncrementMs && spacingOk) {
				startSimultaneousInjection();
				auto endTime = sumTickAndFloat(nowNt, MSF2NT(extraPulseMs));
				getScheduler()->schedule("instantAccelShotTopup", nullptr, endTime,
					action_s::make<endSimultaneousInjectionOnlyTogglePins>());
				instantAccelShotState.deliveredPulseMs += extraPulseMs;
				instantAccelShotState.lastShotTimeNt = nowNt;
			}
		}
	} else {
		instantAccelShotState.latched = false;
		instantAccelShotState.deliveredPulseMs = 0;
	}
}
} // namespace

void boardInstantAccelFastCallback() {
	updateInstantAccelShot();
}
