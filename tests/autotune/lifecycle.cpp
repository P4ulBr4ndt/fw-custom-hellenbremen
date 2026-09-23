#include "board_autotune.h"
#include "table_helper.h"
#include <cassert>
#include <iostream>

TestConfig configuration;
TestEngineConfiguration engineSettings;
TestEngine testEngine;
TestConfig* config = &configuration;
TestEngineConfiguration* engineConfiguration = &engineSettings;
TestEngine* engine = &testEngine;
float testTimeSeconds = 0;
int burns = 0;

void requestBurn() {
	// Temporary tuning settings must be restored before requesting persistence.
	assert(!config->autotuneRunning);
	assert(engineConfiguration->fuelClosedLoopCorrectionEnabled);
	assert(config->ignitionTable[0][0] == 20);
	assert(config->lambdaTable[0][0] < 0.95f);
	++burns;
}

void resetEnvironment() {
	configuration = TestConfig{};
	engineSettings = TestEngineConfiguration{};
	testEngine = TestEngine{};
	testTimeSeconds = 0;
	burns = 0;
	setTable(config->veTable, 100.0f);
	setTable(config->veFrontTable, 100.0f);
	setTable(config->lambdaTable, 0.9f);
	setTable(config->ignitionTable, 20.0f);
	setTable(config->ignitionFrontTable, 20.0f);
	setTable(config->lambdaDelayTable, uint16_t{350});
}

void learn(AutotuneState& state) {
	// Run enough fast-callback ticks to evaluate actual delayed samples.
	for (int tick = 0; tick < 80; ++tick) {
		state.recordProcessing();
		state.checkHistory();
	}
	assert(state.tuneRan);
}

void testRestartDropsOldSamples() {
	resetEnvironment();
	AutotuneState state;
	state.initializeStates();
	state.toggleRunning();
	state.recordProcessing();
	state.toggleRunning();
	state.toggleRunning();
	for (int tick = 0; tick < 80; ++tick) state.checkHistory();
	assert(!state.tuneRan);
	state.prepareFetchData();
	assert(config->veRearTableHits[0][0] == 0);
	assert(config->veRearTableTmp[0][0] == 100);
	learn(state);
	state.toggleRunning();
	state.toggleRunning();
	assert(!state.tuneRan);
	state.toggleRunning();
	state.burningROM();
	assert(burns == 0);
}

void testAppliedChangesCanStillBurn(bool automaticApply, bool restart) {
	resetEnvironment();
	AutotuneState state;
	state.initializeStates();
	state.toggleRunning();
	learn(state);
	if (automaticApply) {
		config->autotuneAutoApply = true;
		testTimeSeconds = 16;
		state.checkHistory();
	} else {
		state.applyingToRAM();
	}
	assert(config->autotuneApplyToRamInd);
	assert(config->veTable[0][0] > 100);
	assert(!state.tuneRan);
	state.burningROM();
	assert(burns == 0); // No flash write while tuning is active.
	state.toggleRunning();
	if (restart) {
		state.toggleRunning();
		state.toggleRunning();
	}
	state.burningROM();
	assert(burns == 1);
	state.burningROM();
	assert(burns == 1);
}

void testShutdown(bool automaticBurn, bool applied) {
	resetEnvironment();
	engineConfiguration->enableAemXSeries = false;
	config->autotuneAutoBurn = automaticBurn;
	const float originalLambda = config->lambdaTable[0][0];
	AutotuneState state;
	state.initializeStates();
	state.toggleRunning();
	assert(config->ignitionTable[0][0] == 16);
	assert(!engineConfiguration->fuelClosedLoopCorrectionEnabled);
	learn(state);
	if (applied) state.applyingToRAM();
	state.onShutdown();
	assert(!state.running && !config->autotuneRunning);
	assert(engineConfiguration->fuelClosedLoopCorrectionEnabled);
	assert(config->lambdaTable[0][0] == originalLambda);
	assert(config->ignitionTable[0][0] == 20);
	assert(config->ignitionFrontTable[0][0] == 20);
	assert(burns == (automaticBurn ? 1 : 0));
	assert((config->veTable[0][0] > 100) == (automaticBurn || applied));
	state.onShutdown(); // Repeated CAN shutdown frames must be idempotent.
	assert(burns == (automaticBurn ? 1 : 0));
	assert(config->ignitionTable[0][0] == 20);
}

int main() {
	testRestartDropsOldSamples();
	for (bool automatic : {false, true}) {
		for (bool restart : {false, true}) testAppliedChangesCanStillBurn(automatic, restart);
		for (bool applied : {false, true}) testShutdown(automatic, applied);
	}
	std::cout << "Autotune lifecycle regressions passed\n";
}
