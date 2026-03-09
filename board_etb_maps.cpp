#include "pch.h"

#include "board_etb_maps.h"
#include "board_riding_modes.h"

#include "table_helper.h"

namespace {
static constexpr uint8_t HD_ENGINE_MAP_ROAD = 0x1;
static constexpr uint8_t HD_ENGINE_MAP_RAIN = 0x2;
static constexpr uint8_t HD_ENGINE_MAP_SPORT = 0x3;
static constexpr uint8_t HD_ENGINE_MAP_TRACK = 0x5;

pedal2tps_t etbRoadMap{"etbRoad"};
pedal2tps_t etbRainMap{"etbRain"};
pedal2tps_t etbSportMap{"etbSport"};
pedal2tps_t etbTrackMap{"etbTrack"};
pedal2tps_t etbBaseMap{"etbBase"};

bool etbMapsInitialized = false;

void ensureEtbMapsInitialized() {
	if (etbMapsInitialized) {
		return;
	}

	etbRoadMap.initTable(config->etbRoadTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);
	etbRainMap.initTable(config->etbRainTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);
	etbSportMap.initTable(config->etbSportTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);
	etbTrackMap.initTable(config->etbTrackTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);
	etbBaseMap.initTable(config->pedalToTpsTable, config->pedalToTpsRpmBins, config->pedalToTpsPedalBins);

	etbMapsInitialized = true;
}

class HarleyEtbMapProvider : public ValueProvider3D {
public:
	float getValue(float xColumn, float yRow) const override {
		ensureEtbMapsInitialized();

		switch (boardGetHarleyEngineMap()) {
			case HD_ENGINE_MAP_ROAD:
				return etbRoadMap.getValue(xColumn, yRow);
			case HD_ENGINE_MAP_RAIN:
				return etbRainMap.getValue(xColumn, yRow);
			case HD_ENGINE_MAP_SPORT:
				return etbSportMap.getValue(xColumn, yRow);
			case HD_ENGINE_MAP_TRACK:
				return etbTrackMap.getValue(xColumn, yRow);
			default:
				// Fall back to base ETB pedal table for unsupported map values.
				return etbBaseMap.getValue(xColumn, yRow);
		}
	}
};

HarleyEtbMapProvider harleyEtbMapProvider;
} // namespace

ValueProvider3D* pedal2TpsProvider() {
	return &harleyEtbMapProvider;
}

void boardEtbMapsApplyDefaults() {
	copyTable(config->etbRoadTable, config->pedalToTpsTable);
	copyTable(config->etbRainTable, config->pedalToTpsTable);
	copyTable(config->etbSportTable, config->pedalToTpsTable);
	copyTable(config->etbTrackTable, config->pedalToTpsTable);
}
