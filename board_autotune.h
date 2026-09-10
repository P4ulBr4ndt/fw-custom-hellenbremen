#pragma once

#include "pch.h"
#include "cyclic_buffer.h"
#include <rusefi/timer.h>

struct bilinear_cell_selection_s {
	size_t loadIdx0; // Is size_t necessary? I don't need such big types here
	size_t loadIdx1; // Is size_t necessary? I don't need such big types here

	size_t rpmIdx0; // Is size_t necessary? I don't need such big types here
	size_t rpmIdx1; // Is size_t necessary? I don't need such big types here

	float cell00Weight; // Bottom-Left
	float cell01Weight; // Bottom-Right
	float cell10Weight; // Top-Left
	float cell11Weight; // Top-Right

	float  cell00; // Bottom-Left
	float  cell01; // Bottom-Right
	float  cell10; // Top-Left
	float  cell11; // Top-Right
};

struct live_data_autotune_s {
	float veTable[VE_LOAD_COUNT][VE_RPM_COUNT];
	float preTuneVeTable[VE_LOAD_COUNT][VE_RPM_COUNT];
	float VeTableDelta[VE_LOAD_COUNT][VE_RPM_COUNT];

	float    accumulatedWeight[VE_LOAD_COUNT][VE_RPM_COUNT];
	uint16_t hitCount[VE_LOAD_COUNT][VE_RPM_COUNT];
};

// One tick's worth of what a bank's closed-loop correction needs, buffered so it can be
// looked back up once the lambda sensor's transport delay for that (rpm, load) has passed.
struct autotune_sample_s {
	float   rpm;
	float   fuelLoad;
	
	float   correctionFront;
	float   correctionRear;

	bilinear_cell_selection_s frontCellSelection;
	bilinear_cell_selection_s rearCellSelection;

	// Ticks remaining until this sample fires; capped below the buffer's size so it
	// always fires before the buffer could cycle back around and evict it. See comment
	// in board_autotune.cpp:AutotuneState::recordProcessing() for more information.
	size_t delayIndex; // Is size_t necessary? I don't need such big types here

	bool processed = false;
};

class AutotuneState {
public:
	AutotuneState();
	~AutotuneState();

	void initializeLiveDataStructs();
	void checkCyclicBufferSize();
	
	void checkHistory();
	
	void toggleRunning();
	void applyingToRAM();
	void burningROM();

	void recordProcessing();
	autotune_sample_s getProposedVECellValue(float frontMeasuredAFR, float rearMeasuredAFR, autotune_sample_s& sample);
	void evaluateNewVECellValue(size_t idx); // Is size_t necessary? I don't need such big types here
	void averageWeighting(live_data_autotune_s& cylinder, const bilinear_cell_selection_s& proposed);

	bilinear_cell_selection_s bilinearCellSelection(float rpm, float fuelLoad, float veTable[VE_LOAD_COUNT][VE_RPM_COUNT]);

	bool autotuneRunning = false;
	bool autotuneTuneRan = false;

private:
	// TODO: Make this configurable
	const float initialWeight = 5.0f; 
	const float weightThreshold = 0.0f;
	const float deadband = 0.0f;
	const float maxWeight = 100.0f;
	const float maxAbsoluteChange = 50.0f;
	const float maxPercentageChange = 50.0f;

	const uint16_t minRPM = 700.0;
	const float    minCLT = 70.0f;
	const float    maxCLT = 140.0f;
	const float    minAFR = 8.0f;
	const float    maxAFR = 20.0f;

	Timer autoApplyTimer;
	const float autoApplyTimerPeriod = 15.0f;
	bool autoApplyEnabled = true;

	live_data_autotune_s front;
	live_data_autotune_s rear;

	cyclic_buffer<autotune_sample_s> autotuneHistory;

} autotuneState;