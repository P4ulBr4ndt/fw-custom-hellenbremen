#include "pch.h"
#include "tunerstudio.h"
#include "table_helper.h"

void autotuneToggleRunning() {
	if (config->autotuneRunning) {
		config->autotuneRunning = false;
		engineConfiguration->stft = config->oldStft;
	} else {
		config->autotuneRunning = true;
		config->oldStft = engineConfiguration->stft;
		
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

void autotuneBurnToROM() {
	if(config->autotuneTuneRan && !config->autotuneRunning) {
		requestBurn();
		config->autotuneTuneRan = false;
	}

	return;
}

void autotuneVETables() {
	if(!config->autotuneRunning) {
		return;
	}

	if (!config->autotuneTuneRan) {
		config->autotuneTuneRan = true;

		copyTable(config->autotuneVeFrontTableOld, config->veFrontTable);
		copyTable(config->autotuneVeRearTableOld, config->veTable);
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
		|| (clt < 70.0  && clt > 140.0) 
		|| (afr1 < 8.0 && afr1 > 20.0) 
		|| (afr2 < 8.0 && afr2 > 20.0)) {
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

			config->autotuneVeTableWeight[loadIdx][rpmIdx] =
				clampF(0, config->autotuneVeTableWeight[loadIdx][rpmIdx] + 1, 65535);

			// move to client side?
			config->autotuneVeFrontTableDelta[loadIdx][rpmIdx] =
				config->autotuneVeFrontTableOld[loadIdx][rpmIdx] - config->veFrontTable[loadIdx][rpmIdx];
			config->autotuneVeRearTableDelta[loadIdx][rpmIdx] =
				config->autotuneVeRearTableOld[loadIdx][rpmIdx] - config->veTable[loadIdx][rpmIdx];
		}
	}

	return;
}