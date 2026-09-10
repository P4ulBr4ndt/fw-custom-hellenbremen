#include "pch.h"
#include "tunerstudio.h"
#include "table_helper.h"
#include "board_autotune.h"

AutotuneState autotuneState;

AutotuneState::AutotuneState() {
	checkCyclicBufferSize();
	initializeLiveDataStructs();

	return;
}

AutotuneState::~AutotuneState() {
	autotuneHistory.clear();

	return;
}

void AutotuneState::initializeLiveDataStructs() {
	copyTable(rear.veTable,  config->veTable);
	copyTable(front.veTable, config->veFrontTable);
	
	copyTable(rear.preTuneVeTable,  config->veTable);
	copyTable(front.preTuneVeTable, config->veFrontTable);

	setTable(rear.accumulatedWeight,  initialWeight);
	setTable(front.accumulatedWeight, initialWeight);

	setTable(rear.hitCount,  (uint16_t)0);
	setTable(front.hitCount, (uint16_t)0);

	return;
}

void AutotuneState::evaluateNewVECellValue(size_t idx) {
	autotune_sample_s autotuneSample   = autotuneHistory.get(idx);
	
	const float clt       = Sensor::getOrZero(SensorType::Clt); // Also historic data?
	const float frontAFR  = Sensor::getOrZero(SensorType::Lambda1) * 14.7f;
	const float rearAFR   = Sensor::getOrZero(SensorType::Lambda2) * 14.7f;
	const float targetLam = engine->fuelComputer.targetLambda;

	float rpm = autotuneSample.rpm;

	// Also add Min TPS, Min VBatt (Engine running?), dTPS (?)
	if (    rpm      < minRPM
		|| (clt      < minCLT || clt      > maxCLT)
		|| (frontAFR < minAFR || frontAFR > maxAFR)
		|| (rearAFR  < minAFR || rearAFR  > maxAFR)
	    || targetLam == 0.0f  || std::isnan(targetLam)) {
		return;
	}

		autotune_sample_s proposedVEValues = getProposedVECellValue(frontAFR, rearAFR, autotuneSample);
		
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
		if (vote.weight <= weightThreshold) {
			continue;
		}

		float&    runningAverage    = cylinder.veTable[vote.loadIdx][vote.rpmIdx];
		float&    accumulatedWeight = cylinder.accumulatedWeight[vote.loadIdx][vote.rpmIdx];
		uint16_t& hitCount          = cylinder.hitCount[vote.loadIdx][vote.rpmIdx];
		const float originalValue   = cylinder.preTuneVeTable[vote.loadIdx][vote.rpmIdx];

		const float candidateAverage = (runningAverage * (accumulatedWeight + initialWeight) + vote.proposedValue * vote.weight)
		                                / (accumulatedWeight + vote.weight + initialWeight);

		// Guard rails compare against the frozen original value, not the running average,
		// so a cell can never drift further than these bounds from where the session started.
		if (std::abs(candidateAverage - originalValue) > maxAbsoluteChange) {
			continue;
		}
		if (originalValue != 0.0f
			&& std::abs(candidateAverage - originalValue) / originalValue * 100.0f > maxPercentageChange) {
			continue;
		}

		if (deadband > std::abs(candidateAverage - originalValue)) {
			runningAverage = originalValue;
			continue;
		}

		runningAverage    = candidateAverage;
		accumulatedWeight = clampF(0.0f, accumulatedWeight + vote.weight, maxWeight);
		hitCount++;
		
		autotuneTuneRan = true;
	}

	return;
}

void AutotuneState::toggleRunning() {
	if (autotuneRunning) {
		autotuneRunning = false;
	} else {
		checkCyclicBufferSize();
		initializeLiveDataStructs();
		autoApplyTimer.reset();

		autotuneRunning = true;
	}

	return;
}

void AutotuneState::recordProcessing() {
	autotune_sample_s sample;

	sample.rpm       = engine->engineState.Rpm;
	sample.fuelLoad  = engine->engineState.Load;

	sample.correctionRear  = 100.0f * engine->engineState.stftCorrection[0];
	sample.correctionFront = 100.0f * engine->engineState.stftCorrection[1];

	sample.frontCellSelection = bilinearCellSelection(sample.rpm, sample.fuelLoad, front.veTable);
	sample.rearCellSelection  = bilinearCellSelection(sample.rpm, sample.fuelLoad, rear.veTable);

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
	int maxIndex = autotuneHistory.getSize() > 0 ? autotuneHistory.getSize() - 1 : 0;
	if (rawIndex > maxIndex) {
		rawIndex = maxIndex;
	}

	sample.delayIndex = (uint8_t)rawIndex;

	autotuneHistory.add(sample);

	return;
}

// Pointer for veTable?
bilinear_cell_selection_s AutotuneState::bilinearCellSelection(float rpm, float fuelLoad, float veTable[VE_LOAD_COUNT][VE_RPM_COUNT]) {
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
	autotune_sample_s proposedVESelection = sample;

	const float targetAFR = engine->fuelComputer.targetLambda * 14.7f;

	const float frontCorrectionFactor = (sample.correctionFront / 100.0f) * (frontMeasuredAFR / targetAFR);
	const float rearCorrectionFactor  = (sample.correctionRear  / 100.0f) * (rearMeasuredAFR / targetAFR);

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
	size_t validCount = (size_t)autotuneHistory.getCount();
	if (validCount > (size_t)autotuneHistory.getSize()) {
		validCount = (size_t)autotuneHistory.getSize();
	}

	for (size_t i = 0; i < validCount; i++) {
		autotune_sample_s& sample = autotuneHistory.elements[i];

		if (sample.processed) {
			continue;
		}

		// Negative elements cannot appear by design
		if (sample.delayIndex > 0) {
			sample.delayIndex--;
		}

		if (sample.delayIndex == 0) {
			evaluateNewVECellValue(i);
			autotuneHistory.elements[i].processed = true;
		}
	}

	if(autoApplyTimer.hasElapsedSec(autoApplyTimerPeriod)) {
		if(autoApplyEnabled) {
			applyingToRAM();
			autotuneTuneRan = false;
		}
		autoApplyTimer.reset();
	}

	return;
}

void AutotuneState::checkCyclicBufferSize() {
	if(autotuneRunning) {
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

	// TODO: cyclic_buffer clamped to 128?
	// setSize() clears the buffer, so only touch it when the size actually needs to change -
	// otherwise every unrelated config change would wipe out valid buffered history.
	if ((size_t)autotuneHistory.getSize() != desiredSize) {
		autotuneHistory.setSize(desiredSize);
	}

	return;
}

void AutotuneState::applyingToRAM() {
	copyTable(config->veTable,      rear.veTable);
	copyTable(config->veFrontTable, front.veTable);
}

void AutotuneState::burningROM() {
	if(autotuneTuneRan && !autotuneRunning) {
		requestBurn();
		autotuneTuneRan = false;
	}

	return;
}
