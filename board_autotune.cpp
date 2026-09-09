#include "pch.h"
#include "tunerstudio.h"
#include "table_helper.h"
#include "board_autotune.h"

AutotuneState::AutotuneState() {
	checkCyclicBufferSize();
	initializeLiveDataStructs();

	return;
}

AutotuneState::~AutotuneState() {
	return;
}

void AutotuneState::initializeLiveDataStructs() {
	copyTable(front.veTable, config->veFrontTable);
	copyTable(rear.veTable,  config->veTable);

	setTable(front.accumulatedWeight, 0.0f);
	setTable(rear.accumulatedWeight,  0.0f);

	setTable(front.hitCount, (uint16_t)0);
	setTable(rear.hitCount,  (uint16_t)0);

	return;
}

void AutotuneState::averageWeighting() {

	recordProcessing();
	
}

void AutotuneState::toggleRunning() {
	if (autotuneRunning) {
		autotuneRunning = false;
		restoreStft();
	} else {
		checkCyclicBufferSize();
		initializeLiveDataStructs();

		autotuneRunning = true;
		oldStft         = engineConfiguration->stft;
		
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionIdle].maxAdd       = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionIdle].maxRemove    = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionIdle].timeConstant = 1.00;

		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionOverrun].maxAdd       = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionOverrun].maxRemove    = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionOverrun].timeConstant = 1.00;

		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionPower].maxAdd       = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionPower].maxRemove    = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionPower].timeConstant = 1.00;

		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionCruise].maxAdd       = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionCruise].maxRemove    = 0.25;
		engineConfiguration->stft.cellCfgs[ft_region_e::ftRegionCruise].timeConstant = 1.00;
	}

	return;
}

void AutotuneState::restoreStft() {
	engineConfiguration->stft = oldStft;

	return;
}

void AutotuneState::recordProcessing() {
	autotune_sample_s sample;

	sample.rpm         = engine->engineState.Rpm;
	sample.fuelLoad    = engine->engineState.Load;

	sample.correctionFront = 100.0f * engine->engineState.stftCorrection[0];
	sample.correctionRear  = 100.0f * engine->engineState.stftCorrection[1];

	sample.frontCellSelection = bilinearCellSelection(sample.rpm, sample.fuelLoad, config->veFrontTable);
	sample.rearCellSelection  = bilinearCellSelection(sample.rpm, sample.fuelLoad, config->veTable);

	// x - load, y - rpm, matching the same row/column convention as veTable itself
	float delayMs = interpolate3d(
		config->lambdaDelayTable,
		config->lambdaDelayLoadBins, sample.fuelLoad,
		config->lambdaDelayRpmBins, sample.rpm
	);

	int rawIndex = (int)(delayMs / FAST_CALLBACK_PERIOD_MS + 0.5f);

	int maxIndex = autotuneHistory.getSize() > 0 ? autotuneHistory.getSize() - 1 : 0;
	if (rawIndex > maxIndex) {
		rawIndex = maxIndex;
	}

	sample.delayIndex = (uint8_t)rawIndex;

	autotuneHistory.add(sample);

	return;
}

bilinear_cell_selection_s AutotuneState::bilinearCellSelection(float rpm, float fuelLoad, float veTable[VE_LOAD_COUNT][VE_RPM_COUNT]) {
	bilinear_cell_selection_s selection;

	const auto rpmBin      = priv::getBin(rpm,      config->veRpmBins);
	const auto fuelLoadBin = priv::getBin(fuelLoad, config->veLoadBins);

	selection.loadIdx0 = fuelLoadBin.Idx;
	selection.loadIdx1 = fuelLoadBin.Idx + 1;
	selection.loadFrac = fuelLoadBin.Frac;

	selection.rpmIdx0  = rpmBin.Idx;
	selection.rpmIdx1  = rpmBin.Idx + 1;
	selection.rpmFrac  = rpmBin.Frac;

	selection.cell00   = veTable[selection.loadIdx0][selection.rpmIdx0];
	selection.cell01   = veTable[selection.loadIdx0][selection.rpmIdx1];
	selection.cell10   = veTable[selection.loadIdx1][selection.rpmIdx0];
	selection.cell11   = veTable[selection.loadIdx1][selection.rpmIdx1];

	return selection;
}

bilinear_cell_selection_s AutotuneState::proposedVECellValue(float measuredAFR, autotune_sample_s& sample) {
	bilinear_cell_selection_s frontProposedVESelection = sample.frontCellSelection;
	bilinear_cell_selection_s rearProposedVESelection  = sample.rearCellSelection;

	float targetAFR = engine->fuelComputer.targetLambda * 14.7f;

	float correctionFactorFront = (sample.correctionFront / 100.0f) * (measuredAFR / targetAFR);
	float correctionFactorRear  = (sample.correctionRear  / 100.0f) * (measuredAFR / targetAFR);
	
	
}

void AutotuneState::checkHistory() {
	size_t validCount = (size_t)autotuneHistory.getCount();
	if (validCount > (size_t)autotuneHistory.getSize()) {
		validCount = (size_t)autotuneHistory.getSize();
	}

	for (size_t i = 0; i < validCount; i++) {
		autotune_sample_s& sample = autotuneHistory.elements[i];

		// Negative elements cannot appear by design
		if (sample.delayIndex > 0) {
			sample.delayIndex--;
		}

		if (sample.delayIndex == 0) {
			averageWeighting();
		}
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

	// setSize() clears the buffer, so only touch it when the size actually needs to change -
	// otherwise every unrelated config change would wipe out valid buffered history.
	if ((size_t)autotuneHistory.getSize() != desiredSize) {
		autotuneHistory.setSize(desiredSize);
	}

	return;
}

void AutotuneState::burningROM() {
	if(autotuneTuneRan && !autotuneRunning) {
		requestBurn();
		autotuneTuneRan = false;
	}

	return;
}

void AutotuneState::veTables() {
	if(!autotuneRunning) {
		return;
	}

	if (!autotuneTuneRan) {
		autotuneTuneRan = true;

		copyTable(autotuneVeFrontTableOld, config->veFrontTable);
		copyTable(autotuneVeRearTableOld, config->veTable);
	}

	const float rpm      = engine->engineState.Rpm;
	const float fuelLoad = engine->engineState.Load;

	const auto rpmBin      = priv::getBin(rpm,      config->veRpmBins);
	const auto fuelLoadBin = priv::getBin(fuelLoad, config->veLoadBins);

	const float clt  = Sensor::getOrZero(SensorType::Clt);
	const float afr1 = Sensor::getOrZero(SensorType::Lambda1) * 14.7f;
	const float afr2 = Sensor::getOrZero(SensorType::Lambda2) * 14.7f;

	// TODO: Improve this code by making it 
	// Also add Min TPS, Min VBatt (Engine running?), dTPS (?)
	if (rpm < 700 
		|| (clt < 70.0 || clt > 140.0) 
		|| (afr1 < 8.0 || afr1 > 20.0) 
		|| (afr2 < 8.0 || afr2 > 20.0)) {
		return;
	}

	const auto clResult = engine->engineState.stftCorrection;

	// Adapt the 2x2 grid around the (rpm, fuelLoad) point based on the STFT results
	// This includes updating the veTable and veFrontTable cells, increasing the cell 
	// weight and the cell differences.

	// TODO: Add Max. Cell value change, Max. Cell percentage change guards

	for (int dLoad = 0; dLoad < 2; dLoad++) {
		size_t loadIdx = fuelLoadBin.Idx + dLoad;
		float loadFrac = dLoad ? fuelLoadBin.Frac : (1 - fuelLoadBin.Frac);

		for (int dRpm = 0; dRpm < 2; dRpm++) {
			size_t rpmIdx = rpmBin.Idx + dRpm;
			float rpmFrac = dRpm ? rpmBin.Frac : (1 - rpmBin.Frac);
			float weight = loadFrac * rpmFrac;

			config->veTable[loadIdx][rpmIdx]      *= (clResult[0] - 1) * weight + 1;
			config->veFrontTable[loadIdx][rpmIdx] *= (clResult[1] - 1) * weight + 1;

			autotuneVeTableWeight[loadIdx][rpmIdx] =
				clampF(0, autotuneVeTableWeight[loadIdx][rpmIdx] + 1, 65535);

			// move to client side?
			autotuneVeFrontTableDelta[loadIdx][rpmIdx] =
				autotuneVeFrontTableOld[loadIdx][rpmIdx] - config->veFrontTable[loadIdx][rpmIdx];
			autotuneVeRearTableDelta[loadIdx][rpmIdx] =
				autotuneVeRearTableOld[loadIdx][rpmIdx] - config->veTable[loadIdx][rpmIdx];
		}
	}

	return;
}