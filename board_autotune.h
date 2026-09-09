#pragma once

#include "pch.h"
#include "cyclic_buffer.h"

struct bilinear_cell_selection_s {
	size_t loadIdx0;
	size_t loadIdx1;
	float  loadFrac;

	size_t rpmIdx0;
	size_t rpmIdx1;
	float  rpmFrac;

	float  cell00; // Bottom-Left
	float  cell01; // Bottom-Right
	float  cell10; // Top-Left
	float  cell11; // Top-Right
};

struct live_data_autotune_s {
	float    veTable[VE_LOAD_COUNT][VE_RPM_COUNT];
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

	uint8_t delayIndex;

};

class AutotuneState {
public:
	AutotuneState();
	~AutotuneState();

	void initializeLiveDataStructs();
	void checkCyclicBufferSize();
	
	void checkHistory();
	
	void toggleRunning();
	void restoreStft();
	void burningROM();

	void recordProcessing();
	bilinear_cell_selection_s proposedVECellValue(float measuredAFR, autotune_sample_s& sample);
	void averageWeighting();
	void veTables();

	bilinear_cell_selection_s bilinearCellSelection(float rpm, float fuelLoad, float veTable[VE_LOAD_COUNT][VE_RPM_COUNT]);

private:
	bool autotuneArmed = false;

	bool autotuneRunning = false;
	bool autotuneTuneRan = false;

	float VeRearDelta[VE_LOAD_COUNT][VE_RPM_COUNT];
	float VeRearOld[VE_LOAD_COUNT][VE_RPM_COUNT];
	float VeFrontDelta[VE_LOAD_COUNT][VE_RPM_COUNT];
	float VeFrontOld[VE_LOAD_COUNT][VE_RPM_COUNT];
	float VeTableWeight[VE_LOAD_COUNT][VE_RPM_COUNT];

	stft_s oldStft;

	bilinear_cell_selection_s frontCellSelection;
	bilinear_cell_selection_s rearCellSelection;

	live_data_autotune_s front;
	live_data_autotune_s rear;

	cyclic_buffer<autotune_sample_s> autotuneHistory;

} autotuneState;