#include "pch.h"
#include "tunerstudio.h"
#include "table_helper.h"
#include "airmass.h"
#include "board_types.h"
#include "board_autotune.h"

AutotuneState autotuneState;

AutotuneState::AutotuneState() {
	return;
}

AutotuneState::~AutotuneState() {
	m_autotuneHistory.clear();

	return;
}

void AutotuneState::initializeStates() {
	config->autotuneRunning = running;
	config->autotuneFetchDataDone = fetchDataDone;
	config->autotuneApplyToRamInd = false;

	narrowbandTuning  = !engineConfiguration->enableAemXSeries;
	stftBefore        = engineConfiguration->fuelClosedLoopCorrectionEnabled;

	checkCyclicBufferSize();
	initializeLiveDataStructs();

	prepareFetchData();
}

void AutotuneState::initializeLiveDataStructs() {
	copyTable(m_rear.veTable,  config->veTable);
	copyTable(m_front.veTable, config->veFrontTable);

	copyTable(m_rear.preTuneVeTable,  config->veTable);
	copyTable(m_front.preTuneVeTable, config->veFrontTable);

	setTable(m_rear.accumulatedWeight,  config->autotuneActiveInitialWeight);
	setTable(m_front.accumulatedWeight, config->autotuneActiveInitialWeight);

	setTable(m_rear.veTableDelta,  0.0f);
	setTable(m_front.veTableDelta, 0.0f);

	setTable(m_rear.hitCount,  (uint16_t)0);
	setTable(m_front.hitCount, (uint16_t)0);

	return;
}

void AutotuneState::evaluateNewVECellValue(size_t idx) {
	autotune_sample_s autotuneSample   = m_autotuneHistory.get(idx);

	// Also historic data?
	// Consider this: Some records could contain a Clt, which would then
	// dismiss these records since they do not fulfill the following guards.
	const float clt       = Sensor::getOrZero(SensorType::Clt);
	const float rearAFR   = Sensor::getOrZero(SensorType::Lambda1) * 14.7f;
	const float frontAFR  = Sensor::getOrZero(SensorType::Lambda2) * 14.7f;

	const float rpm = autotuneSample.rpm;

	// TODO: Also add Min TPS, Min VBatt (Engine running?), dTPS (?)
	if (    rpm      < config->autotuneMinRPM
		|| (clt      < config->autotuneMinETS || clt      > config->autotuneMaxETS)
		|| (frontAFR < config->autotuneMinAFR || frontAFR > config->autotuneMaxAFR)
		|| (rearAFR  < config->autotuneMinAFR || rearAFR  > config->autotuneMaxAFR)
	    || autotuneSample.targetAFR == 0.0f   || std::isnan(autotuneSample.targetAFR)
	    || engine->module<TpsAccelEnrichment>()->isAboveAccelThreshold) {
		return;
	}

	autotune_sample_s proposedVEValues = getProposedVECellValue(frontAFR, rearAFR, autotuneSample);

	averageWeighting(m_front, proposedVEValues.frontCellSelection);
	averageWeighting(m_rear,  proposedVEValues.rearCellSelection);

	return;
}

void AutotuneState::averageWeighting(live_data_autotune_s& cylinder, const bilinear_cell_selection_s& proposed) {
	struct WeightedVote {
		size_t loadIdx;
		size_t rpmIdx;
		float  proposedValue;
		float  weight;
	};

	const WeightedVote votes[4] = {
		{ proposed.loadIdx0, proposed.rpmIdx0, proposed.cell00, proposed.cell00Weight },
		{ proposed.loadIdx0, proposed.rpmIdx1, proposed.cell01, proposed.cell01Weight },
		{ proposed.loadIdx1, proposed.rpmIdx0, proposed.cell10, proposed.cell10Weight },
		{ proposed.loadIdx1, proposed.rpmIdx1, proposed.cell11, proposed.cell11Weight },
	};

	for (const auto& vote : votes) {
		if (vote.weight <= config->autotuneActiveWeightThreshold) {
			continue;
		}

		float&    runningAverage    = cylinder.veTable[vote.loadIdx][vote.rpmIdx];
		float&    accumulatedWeight = cylinder.accumulatedWeight[vote.loadIdx][vote.rpmIdx];
		uint16_t& hitCount          = cylinder.hitCount[vote.loadIdx][vote.rpmIdx];
		float&    tableDelta        = cylinder.veTableDelta[vote.loadIdx][vote.rpmIdx];
		const float originalValue   = cylinder.preTuneVeTable[vote.loadIdx][vote.rpmIdx];

		// Robinson-Monro stochastic approximation
		const float candidateAverage = (runningAverage * (accumulatedWeight + config->autotuneActiveInitialWeight) + vote.proposedValue * vote.weight)
		                                / (accumulatedWeight + vote.weight + config->autotuneActiveInitialWeight);

		// Guard rails compare against the frozen original value, not the running average,
		// so a cell can never drift further than these bounds from where the session started.
		if (std::abs(candidateAverage - originalValue) > config->autotuneMaxAbsoluteChange) {
			continue;
		}
		if (originalValue != 0.0f
			&& std::abs(candidateAverage - originalValue) / originalValue * 100.0f > config->autotuneMaxPercentageChange) {
			continue;
		}

		if (config->autotuneActiveDeadband > std::abs(candidateAverage - originalValue)) {
			runningAverage = originalValue;
			continue;
		}

		runningAverage    = candidateAverage;
		tableDelta        = runningAverage - originalValue;
		accumulatedWeight = clampF(0.0f, accumulatedWeight + vote.weight, config->autotuneActiveMaxWeight);
		hitCount++;

		tuneRan = true;
	}

	return;
}

void AutotuneState::toggleRunning() {
	if (running) {
		running = false;
		if(narrowbandTuning) {
			endNarrowBandTuning();
		}

		engineConfiguration->fuelClosedLoopCorrectionEnabled = stftBefore;
	} else {
		narrowbandTuning = !engineConfiguration->enableAemXSeries;

		if(narrowbandTuning) {
			prepareNarrowBandTuning();
			if(config->autotuneAutomaticCellChangeResistance) {
				config->autotuneCellChangeResistance = autotuneCellChangeResistance_e::High;
			}
		} else {
			if(config->autotuneAutomaticCellChangeResistance) {
				config->autotuneCellChangeResistance = autotuneCellChangeResistance_e::Normal;
			}
		}

		applyCellChangeResistancePreset();

		config->autotuneFetchDataDone = false;
		checkCyclicBufferSize();
		// A new session must not evaluate delayed samples from the previous one,
		// even if the configured buffer size has not changed.
		m_autotuneHistory.clear();
		tuneRan = false;
		initializeLiveDataStructs();
		m_autoApplyTimer.reset();

		stftBefore = engineConfiguration->fuelClosedLoopCorrectionEnabled;
		engineConfiguration->fuelClosedLoopCorrectionEnabled = false;
		running = true;
	}

	config->autotuneRunning = running;

	return;
}

void AutotuneState::recordProcessing() {
	autotune_sample_s sample;

	sample.rpm       = Sensor::getOrZero(SensorType::Rpm);
	sample.fuelLoad  = getVeLoadAxis(engineConfiguration->veOverrideMode, getFuelingLoad());
	sample.targetAFR = engine->fuelComputer.targetLambda * 14.7f;

	sample.correctionRear  = 100.0f * engine->engineState.stftCorrection[0];
	sample.correctionFront = 100.0f * engine->engineState.stftCorrection[1];

	sample.frontCellSelection = bilinearCellSelection(sample.rpm, sample.fuelLoad, config->veFrontTable);
	sample.rearCellSelection  = bilinearCellSelection(sample.rpm, sample.fuelLoad, config->veTable);

	// x - load, y - rpm, matching the same row/column convention as veTable itself
	float delayMs = interpolate3d(
		config->lambdaDelayTable,
		config->lambdaDelayLoadBins, sample.fuelLoad,
		config->lambdaDelayRpmBins, sample.rpm
	);

	int rawIndex = (int)(delayMs / FAST_CALLBACK_PERIOD_MS + 0.5f);

	// Clamp strictly below getSize(): guarantees this entry fully counts down and fires
	// before the buffer could cycle back around and evict it. Note: the buffer size
	// is determined in AutotuneState::checkCyclicBuffer size, which looks for the maximum
	// of delay in the Lambda Delay Table. Thus, in the worst case of maximum Lambda Delay
	// this specific entry will never be overwritten before it was once evaluated in
	// AutotuneState::evaluateNewVECellValue.
	// Example: max table delay 350ms -> getSize() = round(350/5) = 70 -> max delayIndex = 69.
	int maxIndex = m_autotuneHistory.getSize() > 0 ? m_autotuneHistory.getSize() - 1 : 0;
	if (rawIndex > maxIndex) {
		rawIndex = maxIndex;
	}

	sample.delayIndex = (uint8_t)rawIndex;

	m_autotuneHistory.add(sample);

	return;
}

bilinear_cell_selection_s AutotuneState::bilinearCellSelection(float rpm, float fuelLoad, scaled_channel<uint16_t, 10, 1> (&veTable)[VE_LOAD_COUNT][VE_RPM_COUNT]) {
	bilinear_cell_selection_s selection;

	const auto rpmBin      = priv::getBin(rpm,      config->veRpmBins);
	const auto fuelLoadBin = priv::getBin(fuelLoad, config->veLoadBins);

	selection.loadIdx0   = fuelLoadBin.Idx;
	selection.loadIdx1   = fuelLoadBin.Idx + 1;
	const float loadFrac = fuelLoadBin.Frac;

	selection.rpmIdx0   = rpmBin.Idx;
	selection.rpmIdx1   = rpmBin.Idx + 1;
	const float rpmFrac = rpmBin.Frac;

	selection.cell00Weight = (1.0f - loadFrac) * (1.0f - rpmFrac);
	selection.cell01Weight = (1.0f - loadFrac) * (rpmFrac);
	selection.cell10Weight = (loadFrac) * (1.0f - rpmFrac);
	selection.cell11Weight = (loadFrac) * (rpmFrac);

	selection.cell00   = veTable[selection.loadIdx0][selection.rpmIdx0];
	selection.cell01   = veTable[selection.loadIdx0][selection.rpmIdx1];
	selection.cell10   = veTable[selection.loadIdx1][selection.rpmIdx0];
	selection.cell11   = veTable[selection.loadIdx1][selection.rpmIdx1];

	return selection;
}

autotune_sample_s AutotuneState::getProposedVECellValue(float frontMeasuredAFR, float rearMeasuredAFR, autotune_sample_s& sample) {
	// Initialize from sample, so every other value is set
	autotune_sample_s proposedVESelection = sample;

	const float frontCorrectionFactor = (sample.correctionFront / 100.0f) * (frontMeasuredAFR / sample.targetAFR);
	const float rearCorrectionFactor  = (sample.correctionRear  / 100.0f) * (rearMeasuredAFR / sample.targetAFR);

	const float frontCellInterpolated =
		sample.frontCellSelection.cell00 * sample.frontCellSelection.cell00Weight +
		sample.frontCellSelection.cell01 * sample.frontCellSelection.cell01Weight +
		sample.frontCellSelection.cell10 * sample.frontCellSelection.cell10Weight +
		sample.frontCellSelection.cell11 * sample.frontCellSelection.cell11Weight;

	const float rearCellInterpolated =
		sample.rearCellSelection.cell00 * sample.rearCellSelection.cell00Weight +
		sample.rearCellSelection.cell01 * sample.rearCellSelection.cell01Weight +
		sample.rearCellSelection.cell10 * sample.rearCellSelection.cell10Weight +
		sample.rearCellSelection.cell11 * sample.rearCellSelection.cell11Weight;

	proposedVESelection.frontCellSelection.cell00 = frontCorrectionFactor * (sample.frontCellSelection.cell00 + frontCellInterpolated) / 2.0f;
	proposedVESelection.frontCellSelection.cell01 = frontCorrectionFactor * (sample.frontCellSelection.cell01 + frontCellInterpolated) / 2.0f;
	proposedVESelection.frontCellSelection.cell10 = frontCorrectionFactor * (sample.frontCellSelection.cell10 + frontCellInterpolated) / 2.0f;
	proposedVESelection.frontCellSelection.cell11 = frontCorrectionFactor * (sample.frontCellSelection.cell11 + frontCellInterpolated) / 2.0f;

	proposedVESelection.rearCellSelection.cell00  = rearCorrectionFactor * (sample.rearCellSelection.cell00 + rearCellInterpolated) / 2.0f;
	proposedVESelection.rearCellSelection.cell01  = rearCorrectionFactor * (sample.rearCellSelection.cell01 + rearCellInterpolated) / 2.0f;
	proposedVESelection.rearCellSelection.cell10  = rearCorrectionFactor * (sample.rearCellSelection.cell10 + rearCellInterpolated) / 2.0f;
	proposedVESelection.rearCellSelection.cell11  = rearCorrectionFactor * (sample.rearCellSelection.cell11 + rearCellInterpolated) / 2.0f;

	return proposedVESelection;
}

void AutotuneState::checkHistory() {
	size_t validCount = (size_t)m_autotuneHistory.getCount();
	if (validCount > (size_t)m_autotuneHistory.getSize()) {
		validCount = (size_t)m_autotuneHistory.getSize();
	}

	for (size_t i = 0; i < validCount; i++) {
		autotune_sample_s& sample = m_autotuneHistory.elements[i];

		if (sample.processed) {
			continue;
		}

		// Negative elements cannot appear by design. If-Case obsolete?
		if (sample.delayIndex > 0) {
			sample.delayIndex--;
		}

		if (sample.delayIndex == 0) {
			evaluateNewVECellValue(i);
			m_autotuneHistory.elements[i].processed = true;
		}
	}

	if(m_autoApplyTimer.hasElapsedSec(config->autotuneApplyPeriod)) {
		if(config->autotuneAutoApply && tuneRan) {
			applyingToRAM();
		}
		m_autoApplyTimer.reset();
	}

	return;
}

void AutotuneState::checkCyclicBufferSize() {
	if(running) {
		return;
	}

	uint16_t maxDelayMs = 0;

	for (size_t loadIdx = 0; loadIdx < LAMBDA_DELAY_LOAD_COUNT; loadIdx++) {
		for (size_t rpmIdx = 0; rpmIdx < LAMBDA_DELAY_RPM_COUNT; rpmIdx++) {
			uint16_t cellMs = config->lambdaDelayTable[loadIdx][rpmIdx];
			if (cellMs > maxDelayMs) {
				maxDelayMs = cellMs;
			}
		}
	}

	size_t desiredSize = (size_t)((float)maxDelayMs / FAST_CALLBACK_PERIOD_MS + 0.5f);
	if (desiredSize < 1) {
		desiredSize = 1;
	}

	// setSize() clears the buffer, so only touch it when the size actually needs to change -
	// otherwise every unrelated config change would wipe out valid buffered history.
	if ((size_t)m_autotuneHistory.getSize() != desiredSize) {
		m_autotuneHistory.setSize(desiredSize);
	}

	return;
}

void AutotuneState::applyingToRAM() {
	if (!tuneRan) {
		return;
	}

	copyTable(config->veTable,      m_rear.veTable);
	copyTable(config->veFrontTable, m_front.veTable);

	config->autotuneApplyToRamInd = true;
	m_pendingBurn = true;
	tuneRan = false;

	return;
}

void AutotuneState::burningROM() {
	if (!running && (tuneRan || m_pendingBurn)) {
		applyingToRAM(); // Otherwise the tune doesn't land in config->veTable/veFrontTable
		requestBurn();
		m_pendingBurn = false;
	}

	return;
}

void AutotuneState::onShutdown() {
	// Restore temporary tuning settings and publish the stopped state first.
	if (running) {
		toggleRunning();
	}

	if (config->autotuneAutoBurn) {
		burningROM();
	}
}

void AutotuneState::prepareFetchData() {
	copyTable(config->veFrontTableTmp,   m_front.veTable);
	copyTable(config->veFrontTableDelta, m_front.veTableDelta);
	copyTable(config->veFrontTableHits,  m_front.hitCount);

	copyTable(config->veRearTableTmp,   m_rear.veTable);
	copyTable(config->veRearTableDelta, m_rear.veTableDelta);
	copyTable(config->veRearTableHits,  m_rear.hitCount);

	return;
}

void AutotuneState::toggleAutoApply() {
	config->autotuneAutoApply = !config->autotuneAutoApply;
}

void AutotuneState::applyCellChangeResistancePreset() {
	switch (config->autotuneCellChangeResistance) {
	case autotuneCellChangeResistance_e::VeryHigh:
		config->autotuneActiveInitialWeight   = config->autotuneVeryHighInitialWeight;
		config->autotuneActiveWeightThreshold = config->autotuneVeryHighWeightThreshold;
		config->autotuneActiveDeadband        = config->autotuneVeryHighDeadband;
		config->autotuneActiveMaxWeight       = config->autotuneVeryHighMaxWeight;
		break;
	case autotuneCellChangeResistance_e::High:
		config->autotuneActiveInitialWeight   = config->autotuneHighInitialWeight;
		config->autotuneActiveWeightThreshold = config->autotuneHighWeightThreshold;
		config->autotuneActiveDeadband        = config->autotuneHighDeadband;
		config->autotuneActiveMaxWeight       = config->autotuneHighMaxWeight;
		break;
	case autotuneCellChangeResistance_e::Normal:
		config->autotuneActiveInitialWeight   = config->autotuneNormalInitialWeight;
		config->autotuneActiveWeightThreshold = config->autotuneNormalWeightThreshold;
		config->autotuneActiveDeadband        = config->autotuneNormalDeadband;
		config->autotuneActiveMaxWeight       = config->autotuneNormalMaxWeight;
		break;
	case autotuneCellChangeResistance_e::Low:
		config->autotuneActiveInitialWeight   = config->autotuneLowInitialWeight;
		config->autotuneActiveWeightThreshold = config->autotuneLowWeightThreshold;
		config->autotuneActiveDeadband        = config->autotuneLowDeadband;
		config->autotuneActiveMaxWeight       = config->autotuneLowMaxWeight;
		break;
	case autotuneCellChangeResistance_e::VeryLow:
		config->autotuneActiveInitialWeight   = config->autotuneVeryLowInitialWeight;
		config->autotuneActiveWeightThreshold = config->autotuneVeryLowWeightThreshold;
		config->autotuneActiveDeadband        = config->autotuneVeryLowDeadband;
		config->autotuneActiveMaxWeight       = config->autotuneVeryLowMaxWeight;
		break;
	default:
		break;
	}
}

void AutotuneState::resetApplyToRAMIndicator() {
	config->autotuneApplyToRamInd = false;
}

void AutotuneState::prepareNarrowBandTuning() {
	copyTable(config->lambdaTableBefore, config->lambdaTable);
	setTable(config->lambdaTable, 14.6f / 14.7f);


	for (size_t n = 0; n < IGN_LOAD_COUNT; n++) {
		for (size_t m = 0; m < IGN_RPM_COUNT; m++) {
			config->ignitionFrontTable[n][m] = config->ignitionFrontTable[n][m] - 4;
			config->ignitionTable[n][m]      = config->ignitionTable[n][m] - 4;
		}
	}

	// TODO: VE Table modification. For Narrowband, a leaner VE table might be more favourable
}

void AutotuneState::endNarrowBandTuning() {
	copyTable(config->lambdaTable, config->lambdaTableBefore);

	for (size_t n = 0; n < IGN_LOAD_COUNT; n++) {
		for (size_t m = 0; m < IGN_RPM_COUNT; m++) {
			config->ignitionFrontTable[n][m] = config->ignitionFrontTable[n][m] + 4;
			config->ignitionTable[n][m]      = config->ignitionTable[n][m] + 4;
		}
	}
}
