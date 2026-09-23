#pragma once

// Minimal host environment for compiling the real board_autotune.cpp in CI.
// Firmware generation/builds separately validate the real configuration schema.
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <rusefi/scaled_channel.h>
#include "board_types.h"

#define VE_LOAD_COUNT 2
#define VE_RPM_COUNT 2
#define IGN_LOAD_COUNT 2
#define IGN_RPM_COUNT 2
#define LAMBDA_DELAY_LOAD_COUNT 3
#define LAMBDA_DELAY_RPM_COUNT 3
#define FAST_CALLBACK_PERIOD_MS 5

struct TestConfig {
	bool autotuneRunning = false;
	bool autotuneFetchDataDone = false;
	bool autotuneApplyToRamInd = false;
	bool autotuneAutoApply = false;
	bool autotuneAutoBurn = false;
	float autotuneApplyPeriod = 15;
	float autotuneMinRPM = 700;
	float autotuneMinETS = 70;
	float autotuneMaxETS = 140;
	float autotuneMinAFR = 8;
	float autotuneMaxAFR = 20;
	float autotuneMaxAbsoluteChange = 50;
	float autotuneMaxPercentageChange = 50;
	autotuneCellChangeResistance_e autotuneCellChangeResistance = autotuneCellChangeResistance_e::Normal;

#define TEST_WEIGHTS(level) \
	float autotune##level##InitialWeight = 5; \
	float autotune##level##WeightThreshold = 0; \
	float autotune##level##Deadband = 0; \
	float autotune##level##MaxWeight = 300;
	TEST_WEIGHTS(Active)
	TEST_WEIGHTS(VeryHigh)
	TEST_WEIGHTS(High)
	TEST_WEIGHTS(Normal)
	TEST_WEIGHTS(Low)
	TEST_WEIGHTS(VeryLow)
#undef TEST_WEIGHTS

	scaled_channel<uint16_t, 10, 1> veTable[2][2], veFrontTable[2][2];
	scaled_channel<uint16_t, 10, 1> veRearTableTmp[2][2], veFrontTableTmp[2][2];
	scaled_channel<int16_t, 10, 1> veRearTableDelta[2][2], veFrontTableDelta[2][2];
	uint16_t veRearTableHits[2][2]{}, veFrontTableHits[2][2]{};
	uint16_t veRpmBins[2]{800, 6500}, veLoadBins[2]{0, 100};
	uint16_t lambdaDelayTable[3][3]{};
	uint16_t lambdaDelayRpmBins[3]{800, 2190, 6500}, lambdaDelayLoadBins[3]{2, 35, 100};
	scaled_channel<uint8_t, 147, 1> lambdaTable[2][2], lambdaTableBefore[2][2];
	float ignitionTable[2][2]{}, ignitionFrontTable[2][2]{};
};

struct TestEngineConfiguration {
	bool enableAemXSeries = true;
	bool fuelClosedLoopCorrectionEnabled = true;
	int veOverrideMode = 0;
};

struct TpsAccelEnrichment { bool isAboveAccelThreshold = false; };
struct TestEngine {
	struct { float targetLambda = 1; } fuelComputer;
	struct { float stftCorrection[2]{1, 1}; } engineState;
	TpsAccelEnrichment enrichment;
	template <typename T> T* module() { return &enrichment; }
};

enum class SensorType { Clt, Lambda1, Lambda2, Rpm };
struct Sensor {
	static float getOrZero(SensorType type) {
		switch (type) {
		case SensorType::Clt: return 90;
		case SensorType::Rpm: return 2000;
		default: return 1.1f;
		}
	}
};

extern TestConfig* config;
extern TestEngineConfiguration* engineConfiguration;
extern TestEngine* engine;
inline float clampF(float low, float value, float high) { return std::clamp(value, low, high); }
