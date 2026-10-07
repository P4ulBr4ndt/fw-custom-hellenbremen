#pragma once

#include "pch.h"
#include <rusefi/timer.h>

class AtIdleBase {
public:
	struct bilinear_cellselection_atIdleBase_s {
		size_t cltIdx0;
		size_t cltIdx1;

		float  cell0Weight;
		float  cell1Weight;

		float  cell0; // left idle position
		float  cell1; // right idle position
	};

	struct live_data_atIdleBase_s {
		float idleBaseTable[CLT_IDLE_TABLE_CLT_SIZE];
		float preTuneIdleBaseTable[CLT_IDLE_TABLE_CLT_SIZE];
		float idleBaseTableDelta[CLT_IDLE_TABLE_CLT_SIZE];

		float    accumulatedWeight[CLT_IDLE_TABLE_CLT_SIZE];
		uint16_t hitCount[CLT_IDLE_TABLE_CLT_SIZE];
	};

	struct atIdleBaseSample_s {
		SensorResult clt = unexpected; // expected<float> has no default constructor
		float rpm;
		float idlePos;

		bilinear_cellselection_atIdleBase_s cellSelection;
	};

	AtIdleBase();
	~AtIdleBase();

	void initializeStates();
	void initializeLiveDataStructs();
	void onShutdown();

	void toggleRunning();
	void toggleAutoApply();
	void applyingToRAM();
	void burningROM();
	void applyCellChangeResistancePreset();

	void prepareFetchData();
	void resetApplyToRAMIndicator();

	bool areIdleBaseConditionsMet();
	bool areStartConditionsMet();

	void periodicSlowCallback();

	void                                recordProcessing();
	void                                evaluateNewCellSelection();
	bilinear_cellselection_atIdleBase_s getIdleBaseTblCellSelection(float clt);
	bilinear_cellselection_atIdleBase_s getProposedCellSelection(float clt, float idlePos, bilinear_cellselection_atIdleBase_s& sample);
	void                                averageWeighting(live_data_atIdleBase_s& liveTable, const bilinear_cellselection_atIdleBase_s& sample);

private:
	bool m_running       = false;
	bool m_fetchDataDone = false;
	bool m_pendingBurn   = false;
	bool m_tuneRan       = false;

	Timer m_autoApplyTimer;

	atIdleBaseSample_s     m_recordedSample;
	live_data_atIdleBase_s m_liveData;
};

extern AtIdleBase atIdleBaseState;
