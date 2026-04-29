#include "pch.h"

#include "board_torque_maps.h"

#include "table_helper.h"

namespace {
Map3D<PEDAL_TO_TPS_RPM_SIZE, PEDAL_TO_TPS_SIZE, int16_t, uint8_t, uint8_t> torqueMap{"torqueMap"};

bool torqueMapsInitialized = false;

void ensureTorqueMapsInitialized() {
	if (torqueMapsInitialized) {
		return;
	}

	torqueMap.initTable(config->torqueTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);

	torqueMapsInitialized = true;
}

class HarleyTorqueMapProvider : public ValueProvider3D {
public:
	float getValue(float xColumn, float yRow) const override {
		ensureTorqueMapsInitialized();

		return torqueMap.getValue(xColumn, yRow);
		}
	}
};

HarleyTorqueMapProvider harleyTorqueMapProvider;
} // namespace

ValueProvider3D* pedal2TpsProvider() {
	return &harleyTorqueMapProvider;
}

void boardTorqueMapsApplyDefaults() {
	copyTable(config->torqueTable, config->pedalToTpsTable);
}
