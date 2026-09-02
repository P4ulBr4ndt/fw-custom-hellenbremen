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

	float rpm      = Sensor::getOrZero(SensorType::Rpm);
	float fuelLoad = getFuelingLoad();

	auto rpmBin      = priv::getBin(rpm,      config->veRpmBins);
	auto fuelLoadBin = priv::getBin(fuelLoad, config->veLoadBins);

	float low_rpmFuelLoad_frac  = (1 - rpmBin.Frac) * (1 - fuelLoadBin.Frac);
	float high_rpmFrac          = (rpmBin.Frac) * (1 - fuelLoadBin.Frac);
	float high_fuelLoadFrac     = (fuelLoadBin.Frac) * (1 - rpmBin.Frac);
	float diag_rpmFuelLoad_frac = rpmBin.Frac * fuelLoadBin.Frac;

	auto clResult = engine->module<ShortTermFuelTrim>()->getCorrection(rpm, fuelLoad);

	config->veTable[fuelLoadBin.Idx][rpmBin.Idx]         *= (clResult.banks[0] - 1) * low_rpmFuelLoad_frac + 1;
	config->veTable[fuelLoadBin.Idx][rpmBin.Idx + 1]     *= (clResult.banks[0] - 1) * high_rpmFrac + 1;
	config->veTable[fuelLoadBin.Idx + 1][rpmBin.Idx]     *= (clResult.banks[0] - 1) * high_fuelLoadFrac + 1;
	config->veTable[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] *= (clResult.banks[0] - 1) * diag_rpmFuelLoad_frac + 1;

	config->veFrontTable[fuelLoadBin.Idx][rpmBin.Idx]         *= (clResult.banks[1] - 1) * low_rpmFuelLoad_frac + 1;
	config->veFrontTable[fuelLoadBin.Idx][rpmBin.Idx + 1]     *= (clResult.banks[1] - 1) * high_rpmFrac + 1;
	config->veFrontTable[fuelLoadBin.Idx + 1][rpmBin.Idx]     *= (clResult.banks[1] - 1) * high_fuelLoadFrac + 1;
	config->veFrontTable[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] *= (clResult.banks[1] - 1) * diag_rpmFuelLoad_frac + 1;

	config->autotuneVeTableWeight[fuelLoadBin.Idx][rpmBin.Idx]         = clampF(0, config->autotuneVeTableWeight[fuelLoadBin.Idx][rpmBin.Idx] + 1, 65535);
	config->autotuneVeTableWeight[fuelLoadBin.Idx][rpmBin.Idx + 1]     = clampF(0, config->autotuneVeTableWeight[fuelLoadBin.Idx][rpmBin.Idx + 1] + 1, 65535);
	config->autotuneVeTableWeight[fuelLoadBin.Idx + 1][rpmBin.Idx]     = clampF(0, config->autotuneVeTableWeight[fuelLoadBin.Idx + 1][rpmBin.Idx] + 1, 65535);
	config->autotuneVeTableWeight[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] = clampF(0, config->autotuneVeTableWeight[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] + 1, 65535);

	// move to client side?
	config->autotuneVeFrontTableDelta[fuelLoadBin.Idx][rpmBin.Idx]         = config->autotuneVeFrontTableOld[fuelLoadBin.Idx][rpmBin.Idx] - config->veFrontTable[fuelLoadBin.Idx][rpmBin.Idx];
	config->autotuneVeFrontTableDelta[fuelLoadBin.Idx][rpmBin.Idx + 1]     = config->autotuneVeFrontTableOld[fuelLoadBin.Idx][rpmBin.Idx + 1] - config->veFrontTable[fuelLoadBin.Idx][rpmBin.Idx + 1];
	config->autotuneVeFrontTableDelta[fuelLoadBin.Idx + 1][rpmBin.Idx]     = config->autotuneVeFrontTableOld[fuelLoadBin.Idx + 1][rpmBin.Idx] - config->veFrontTable[fuelLoadBin.Idx + 1][rpmBin.Idx];
	config->autotuneVeFrontTableDelta[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] = config->autotuneVeFrontTableOld[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] - config->veFrontTable[fuelLoadBin.Idx + 1][rpmBin.Idx + 1];

	config->autotuneVeRearTableDelta[fuelLoadBin.Idx][rpmBin.Idx]         = config->autotuneVeRearTableOld[fuelLoadBin.Idx][rpmBin.Idx] - config->veTable[fuelLoadBin.Idx][rpmBin.Idx];
	config->autotuneVeRearTableDelta[fuelLoadBin.Idx][rpmBin.Idx + 1]     = config->autotuneVeRearTableOld[fuelLoadBin.Idx][rpmBin.Idx + 1] - config->veTable[fuelLoadBin.Idx][rpmBin.Idx + 1];
	config->autotuneVeRearTableDelta[fuelLoadBin.Idx + 1][rpmBin.Idx]     = config->autotuneVeRearTableOld[fuelLoadBin.Idx + 1][rpmBin.Idx] - config->veTable[fuelLoadBin.Idx + 1][rpmBin.Idx];
	config->autotuneVeRearTableDelta[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] = config->autotuneVeRearTableOld[fuelLoadBin.Idx + 1][rpmBin.Idx + 1] - config->veTable[fuelLoadBin.Idx + 1][rpmBin.Idx + 1];

	return;
}