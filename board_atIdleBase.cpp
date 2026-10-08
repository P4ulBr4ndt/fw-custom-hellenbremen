#include "pch.h"
#include "storage.h"
#include "tunerstudio.h"
#include "table_helper.h"
#include "idle_thread.h"
#include "board_types.h"
#include "board_atIdleBase.h"

AtIdleBase atIdleBaseState;

void AtIdleBase::initializeStates() {
	config->atIdleBaseRunning       = m_running;
	config->atIdleBaseFetchDataDone = m_fetchDataDone;
	config->atIdleBaseApplyToRamInd = false;

	initializeLiveDataStructs();

	prepareFetchData();

	return;
}

AtIdleBase::AtIdleBase() {
	return;
}

AtIdleBase::~AtIdleBase() {
	return;
}

void AtIdleBase::initializeLiveDataStructs() {
	for (size_t m = 0; m < CLT_IDLE_TABLE_CLT_SIZE; m++) {
		m_liveData.idleBaseTable[m]        = config->cltIdleCorrTable[0][m];
		m_liveData.preTuneIdleBaseTable[m] = config->cltIdleCorrTable[0][m];
	}

	setArrayValues(m_liveData.accumulatedWeight, config->atIdleBaseActiveInitialWeight);
	setArrayValues(m_liveData.idleBaseTableDelta, 0.0f);
	setArrayValues(m_liveData.hitCount, (uint16_t)0);

	return;
}

void AtIdleBase::toggleRunning() {
	if (m_running) {
		m_running = false;
	} else {
		if(areStartConditionsMet()) {
			applyCellChangeResistancePreset();

			m_tuneRan = false;
			initializeLiveDataStructs();

			m_running = true;
			config->atIdleBaseFetchDataDone = false;
			m_autoApplyTimer.reset();
		}
	}

	config->atIdleBaseRunning = m_running;

	return;
}

void AtIdleBase::applyingToRAM() {
	if (!m_tuneRan) {
		return;
	}

	for (size_t m = 0; m < CLT_IDLE_TABLE_CLT_SIZE; m++) {
		config->cltIdleCorrTable[0][m] = m_liveData.idleBaseTable[m];
		config->cltIdleCorrTable[1][m] = m_liveData.idleBaseTable[m];
	}

	config->atIdleBaseApplyToRamInd = true;
	m_pendingBurn                   = true; 
	m_tuneRan                       = false;

	return;
}

void AtIdleBase::burningROM() {
	if (!m_running && (m_tuneRan || m_pendingBurn)) {
		applyingToRAM(); // Otherwise the tune doesn't land in config->cltIdleCorrTable

		// Respecting the free choice of bins, find the index of config->cltIdleCorrBins
		// that is closest to 100° C
		size_t resultIdx = 0;
		for (size_t m = 1; m < CLT_IDLE_TABLE_CLT_SIZE; m++) {
			if (std::abs(config->cltIdleCorrBins[m] - 100.0f) < std::abs(config->cltIdleCorrBins[resultIdx] - 100.0f)) {
				resultIdx = m;
			}
		}

		// Every table cell above the resultIdx entry should have the same value
		// of the resultIdx entry.
		for (size_t m = resultIdx + 1; m < CLT_IDLE_TABLE_CLT_SIZE; m++) {
			config->cltIdleCorrTable[0][m] = config->cltIdleCorrTable[0][resultIdx];
			config->cltIdleCorrTable[1][m] = config->cltIdleCorrTable[1][resultIdx];
		}

		// Add a half percent to make the open-loop idle position more robust
		for (size_t m = 0; m < CLT_IDLE_TABLE_CLT_SIZE; m++) {
			config->cltIdleCorrTable[0][m] = config->cltIdleCorrTable[0][resultIdx] + 0.5f;
			config->cltIdleCorrTable[1][m] = config->cltIdleCorrTable[1][resultIdx] + 0.5f;
		}

		writeIACCoastingTable();

		// A write already pending (e.g. requested by the VE autotune) saves the whole config, including this tune
		if (!getNeedToWriteConfiguration()) {
			requestBurn();
		}

		m_pendingBurn = false;
	}

	return;
}

void AtIdleBase::onShutdown() {
	// Restore temporary tuning settings and publish the stopped state first.
	if (m_running) {
		toggleRunning();
	}

	if (config->atIdleBaseAutoBurn) {
		burningROM();
	}

	return;
}

void AtIdleBase::prepareFetchData() {
	copyArray(config->cltIdleBaseTableTmp,   m_liveData.idleBaseTable);
	copyArray(config->cltIdleBaseTableDelta, m_liveData.idleBaseTableDelta);
	copyArray(config->cltIdleBaseTableHits,  m_liveData.hitCount);

	return;
}

void AtIdleBase::resetApplyToRAMIndicator() {
	config->atIdleBaseApplyToRamInd = false;

	return;
}

bool AtIdleBase::areIdleBaseConditionsMet() {
	const auto& idle = engine->module<IdleController>().unmock();
	const auto  tgs  = Sensor::get(SensorType::AcceleratorPedal);

	if(m_recordedSample.clt.Valid &&
	   m_recordedSample.rpm > config->atIdleBaseMinRpm &&
	   idle.getCurrentPhase() == IIdleController::Phase::Idling &&
	   tgs.Value < config->atIdleBaseMaxTgs) {
		return true;
	} else {
		return false;
	}
}

bool AtIdleBase::areStartConditionsMet() {
	const auto& idle = engine->module<IdleController>().unmock();
	const auto  clt  = Sensor::get(SensorType::Clt);

	// Dashpot taper still active after leaving the running phase would distort the first samples
	if(clt.Valid && 
	   clt.Value < config->atIdleBaseMinClt &&
	   idle.iacByTpsTaper == 0.0f) {
		return true;
	} else {
		return false;
	}
	
}

// TODO: Is the integrator of the closed-loop idle
// likely interferring with the autotune when learned values are saved?
// Elaboration: After the save, the integrator of the PID still has old
// information stored, which might creep up the idle position again. 
void AtIdleBase::evaluateNewCellSelection() {
	if (!areIdleBaseConditionsMet()) {
		toggleRunning();
		return;
	}
	
	bilinear_cellselection_atIdleBase_s proposedCellSelection = getProposedCellSelection(m_recordedSample.clt.Value, m_recordedSample.idlePos, m_recordedSample.cellSelection);
	averageWeighting(m_liveData, proposedCellSelection);
}

AtIdleBase::bilinear_cellselection_atIdleBase_s AtIdleBase::getProposedCellSelection(float clt, float idlePos, bilinear_cellselection_atIdleBase_s& sample) {
	bilinear_cellselection_atIdleBase_s proposedSelection = sample;

	const float targetIdlePos = interpolate2d(clt, config->cltIdleCorrBins, config->cltIdleCorrTable[0]);

	// No meaningful reference to scale against: zero weights make averageWeighting() skip both cells
	if (targetIdlePos < 0.1f) {
		proposedSelection.cell0Weight = 0.0f;
		proposedSelection.cell1Weight = 0.0f;
		return proposedSelection;
	}

	const float idlePosCorrectionFactor = idlePos / targetIdlePos;

	const float cellInterpolated = (proposedSelection.cell0 * proposedSelection.cell0Weight) + 
	                               (proposedSelection.cell1 * proposedSelection.cell1Weight);

	proposedSelection.cell0 = idlePosCorrectionFactor * (sample.cell0 + cellInterpolated) / 2.0f;
	proposedSelection.cell1 = idlePosCorrectionFactor * (sample.cell1 + cellInterpolated) / 2.0f;

	return proposedSelection;
}

void AtIdleBase::averageWeighting(live_data_atIdleBase_s& liveTable, const bilinear_cellselection_atIdleBase_s& sample) {
	struct WeightedVote {
		size_t cltIdx;
		float  proposedValue;
		float  weight;
	};

	const WeightedVote votes[2] = {
		{sample.cltIdx0, sample.cell0, sample.cell0Weight},
		{sample.cltIdx1, sample.cell1, sample.cell1Weight},
	};

	for (const WeightedVote& vote : votes) {
		if (vote.weight <= config->atIdleBaseActiveWeightThreshold) {
			continue;
		}

		float&      runningAverage    = liveTable.idleBaseTable[vote.cltIdx];
		float&      tableDelta        = liveTable.idleBaseTableDelta[vote.cltIdx];
		const float originalValue     = liveTable.preTuneIdleBaseTable[vote.cltIdx];

		float&      accumulatedWeight = liveTable.accumulatedWeight[vote.cltIdx];
		uint16_t&   hitCount          = liveTable.hitCount[vote.cltIdx];
		
		// Robinson-Monro stochastic approximation
		const float candidateAverage = (runningAverage * (accumulatedWeight + config->atIdleBaseActiveInitialWeight) + vote.proposedValue * vote.weight)
		                                / (accumulatedWeight + vote.weight + config->atIdleBaseActiveInitialWeight);

		// Guard rails compare against the frozen original value, not the running average,
		// so a cell can never drift further than these bounds from where the session started.
		if (std::abs(candidateAverage - originalValue) > config->atIdleBaseMaxAbsoluteChange) {
			continue;
		}
		if (originalValue != 0.0f
			&& std::abs(candidateAverage - originalValue) / originalValue * 100.0f > config->atIdleBaseMaxPercentageChange) {
			continue;
		}

		if (config->atIdleBaseActiveDeadband > std::abs(candidateAverage - originalValue)) {
			runningAverage = originalValue;
			continue;
		}

		runningAverage    = candidateAverage;
		tableDelta        = runningAverage - originalValue;
		accumulatedWeight = clampF(0.0f, accumulatedWeight + vote.weight, config->atIdleBaseActiveMaxWeight);
		hitCount++;

		m_tuneRan = true;
	}

	return;
}

void AtIdleBase::applyCellChangeResistancePreset() {
	switch (config->atIdleBaseCellChangeResistance) {
	case autotuneCellChangeResistance_e::VeryHigh:
		config->atIdleBaseActiveInitialWeight   = config->atIdleBaseVeryHighInitialWeight;
		config->atIdleBaseActiveWeightThreshold = config->atIdleBaseVeryHighWeightThreshold;
		config->atIdleBaseActiveDeadband        = config->atIdleBaseVeryHighDeadband;
		config->atIdleBaseActiveMaxWeight       = config->atIdleBaseVeryHighMaxWeight;
		break;
	case autotuneCellChangeResistance_e::High:
		config->atIdleBaseActiveInitialWeight   = config->atIdleBaseHighInitialWeight;
		config->atIdleBaseActiveWeightThreshold = config->atIdleBaseHighWeightThreshold;
		config->atIdleBaseActiveDeadband        = config->atIdleBaseHighDeadband;
		config->atIdleBaseActiveMaxWeight       = config->atIdleBaseHighMaxWeight;
		break;
	case autotuneCellChangeResistance_e::Normal:
		config->atIdleBaseActiveInitialWeight   = config->atIdleBaseNormalInitialWeight;
		config->atIdleBaseActiveWeightThreshold = config->atIdleBaseNormalWeightThreshold;
		config->atIdleBaseActiveDeadband        = config->atIdleBaseNormalDeadband;
		config->atIdleBaseActiveMaxWeight       = config->atIdleBaseNormalMaxWeight;
		break;
	case autotuneCellChangeResistance_e::Low:
		config->atIdleBaseActiveInitialWeight   = config->atIdleBaseLowInitialWeight;
		config->atIdleBaseActiveWeightThreshold = config->atIdleBaseLowWeightThreshold;
		config->atIdleBaseActiveDeadband        = config->atIdleBaseLowDeadband;
		config->atIdleBaseActiveMaxWeight       = config->atIdleBaseLowMaxWeight;
		break;
	case autotuneCellChangeResistance_e::VeryLow:
		config->atIdleBaseActiveInitialWeight   = config->atIdleBaseVeryLowInitialWeight;
		config->atIdleBaseActiveWeightThreshold = config->atIdleBaseVeryLowWeightThreshold;
		config->atIdleBaseActiveDeadband        = config->atIdleBaseVeryLowDeadband;
		config->atIdleBaseActiveMaxWeight       = config->atIdleBaseVeryLowMaxWeight;
		break;
	default:
		break;
	}

	return;
}

AtIdleBase::bilinear_cellselection_atIdleBase_s AtIdleBase::getIdleBaseTblCellSelection(float clt) {
	bilinear_cellselection_atIdleBase_s selection;

	const auto cltBin = priv::getBin(clt, config->cltIdleCorrBins);

	selection.cltIdx0 = cltBin.Idx;
	selection.cltIdx1 = cltBin.Idx + 1;
	const float cltFrac = cltBin.Frac;

	selection.cell0Weight = 1.0f - cltFrac;
	selection.cell1Weight = cltFrac;

	// Read from the active table (as the VE autotune does), not the learned values,
	// otherwise the same correction gets applied again every sample.
	selection.cell0 = config->cltIdleCorrTable[0][selection.cltIdx0];
	selection.cell1 = config->cltIdleCorrTable[0][selection.cltIdx1];

	return selection;
}

void AtIdleBase::periodicSlowCallback() {
	if (m_running) {
		recordProcessing();
		evaluateNewCellSelection();

		if(m_autoApplyTimer.hasElapsedSec(config->atIdleBaseApplyPeriod)) {
			if(config->atIdleBaseAutoApply && m_tuneRan) {
				applyingToRAM();
			}
			m_autoApplyTimer.reset();
		}
	}

	return;
}

void AtIdleBase::recordProcessing() {
	m_recordedSample.clt           = Sensor::get(SensorType::Clt);
	m_recordedSample.rpm           = Sensor::getOrZero(SensorType::Rpm);
	m_recordedSample.idlePos       = engine->module<IdleController>().unmock().currentIdlePosition; // "Idle position" gauge
	m_recordedSample.cellSelection = getIdleBaseTblCellSelection(m_recordedSample.clt.Value);

	return;
}

void AtIdleBase::toggleAutoApply() {
	config->atIdleBaseAutoApply = !config->atIdleBaseAutoApply;

	return;
}

void AtIdleBase::writeIACCoastingTable() {
	for (size_t m = 0; m < CLT_IDLE_TABLE_CLT_SIZE; m++) {
		for (size_t n = 0; n < IAC_COASTING_RPM_SIZE; n++) {
			if (n < 2) {
				config->iacCoasting[n][m] = config->cltIdleCorrTable[0][m];
			} else if (n == 2) {
				config->iacCoasting[n][m] = (config->cltIdleCorrTable[0][m] + 5.0f) / 2.0f;
			} else {
				config->iacCoasting[n][m] = 5.0f;
			}
		}
	}

	return;
}