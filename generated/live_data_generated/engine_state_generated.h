// this section was generated automatically by rusEFI tool config_definition_base-all.jar based on (unknown script) controllers/algo/engine_state.txt
// by class com.rusefi.output.CHeaderConsumer
// begin
#pragma once
#include "rusefi_types.h"
// start of LuaAdjustments
struct LuaAdjustments {
	/**
	 * Lua: Fuel add
	 * units: g
	 * offset 0
	 */
	float fuelAdd = (float)0;
	/**
	 * Lua: Fuel mult
	 * offset 4
	 */
	float fuelMult = (float)0;
	/**
	offset 8 bit 0 */
	bool clutchUpState : 1 {};
	/**
	offset 8 bit 1 */
	bool brakePedalState : 1 {};
	/**
	offset 8 bit 2 */
	bool acRequestState : 1 {};
	/**
	offset 8 bit 3 */
	bool luaDisableEtb : 1 {};
	/**
	offset 8 bit 4 */
	bool luaIgnCut : 1 {};
	/**
	offset 8 bit 5 */
	bool luaFuelCut : 1 {};
	/**
	offset 8 bit 6 */
	bool clutchDownState : 1 {};
	/**
	offset 8 bit 7 */
	bool disableDecelerationFuelCutOff : 1 {};
	/**
	offset 8 bit 8 */
	bool torqueReductionState : 1 {};
	/**
	offset 8 bit 9 */
	bool jssState : 1 {};
	/**
	offset 8 bit 10 */
	bool opsState : 1 {};
	/**
	offset 8 bit 11 */
	bool unusedBit_13_11 : 1 {};
	/**
	offset 8 bit 12 */
	bool unusedBit_13_12 : 1 {};
	/**
	offset 8 bit 13 */
	bool unusedBit_13_13 : 1 {};
	/**
	offset 8 bit 14 */
	bool unusedBit_13_14 : 1 {};
	/**
	offset 8 bit 15 */
	bool unusedBit_13_15 : 1 {};
	/**
	offset 8 bit 16 */
	bool unusedBit_13_16 : 1 {};
	/**
	offset 8 bit 17 */
	bool unusedBit_13_17 : 1 {};
	/**
	offset 8 bit 18 */
	bool unusedBit_13_18 : 1 {};
	/**
	offset 8 bit 19 */
	bool unusedBit_13_19 : 1 {};
	/**
	offset 8 bit 20 */
	bool unusedBit_13_20 : 1 {};
	/**
	offset 8 bit 21 */
	bool unusedBit_13_21 : 1 {};
	/**
	offset 8 bit 22 */
	bool unusedBit_13_22 : 1 {};
	/**
	offset 8 bit 23 */
	bool unusedBit_13_23 : 1 {};
	/**
	offset 8 bit 24 */
	bool unusedBit_13_24 : 1 {};
	/**
	offset 8 bit 25 */
	bool unusedBit_13_25 : 1 {};
	/**
	offset 8 bit 26 */
	bool unusedBit_13_26 : 1 {};
	/**
	offset 8 bit 27 */
	bool unusedBit_13_27 : 1 {};
	/**
	offset 8 bit 28 */
	bool unusedBit_13_28 : 1 {};
	/**
	offset 8 bit 29 */
	bool unusedBit_13_29 : 1 {};
	/**
	offset 8 bit 30 */
	bool unusedBit_13_30 : 1 {};
	/**
	offset 8 bit 31 */
	bool unusedBit_13_31 : 1 {};
};
static_assert(sizeof(LuaAdjustments) == 12);

// start of speed_density_s
struct speed_density_s {
	/**
	 * Air: Charge temperature estimate
	 * units: deg C
	 * offset 0
	 */
	scaled_channel<int16_t, 100, 1> tCharge = (int16_t)0;
	/**
	 * need 4 byte alignment
	 * units: units
	 * offset 2
	 */
	uint8_t alignmentFill_at_2[2] = {};
	/**
	 * Air: Charge temperature estimate K
	 * offset 4
	 */
	float tChargeK = (float)0;
};
static_assert(sizeof(speed_density_s) == 8);

// start of cranking_fuel_s
struct cranking_fuel_s {
	/**
	 * Fuel: cranking CLT mult
	 * offset 0
	 */
	float coolantTemperatureCoefficient = (float)0;
	/**
	 * Fuel: cranking TPS mult
	 * offset 4
	 */
	float tpsCoefficient = (float)0;
	/**
	 * Fuel: Cranking cycle base mass
	 * units: mg
	 * offset 8
	 */
	scaled_channel<uint16_t, 100, 1> baseFuel = (uint16_t)0;
	/**
	 * Fuel: Cranking cycle mass
	 * units: mg
	 * offset 10
	 */
	scaled_channel<uint16_t, 100, 1> fuel = (uint16_t)0;
};
static_assert(sizeof(cranking_fuel_s) == 12);

// start of engine_state_s
struct engine_state_s {
	/**
	 * offset 0
	 */
	float injectionMass[MAX_CYLINDER_COUNT] = {};
	/**
	 * offset 48
	 */
	LuaAdjustments lua;
	/**
	 * offset 60
	 */
	speed_density_s sd;
	/**
	 * offset 68
	 */
	cranking_fuel_s crankingFuel;
	/**
	 * @@GAUGE_NAME_FUEL_BARO_CORR@@
	 * offset 80
	 */
	float baroCorrection = (float)0;
	/**
	 * Detected Board ID
	 * units: id
	 * offset 84
	 */
	int16_t hellenBoardId = (int16_t)0;
	/**
	 * @@INDICATOR_NAME_CLUTCH_UP@@
	 * offset 86
	 */
	int8_t clutchUpState = (int8_t)0;
	/**
	 * @@INDICATOR_NAME_BRAKE_DOWN@@
	 * offset 87
	 */
	int8_t brakePedalState = (int8_t)0;
	/**
	 * JSS State
	 * offset 88
	 */
	int8_t jssState = (int8_t)0;
	/**
	 * OPS State
	 * offset 89
	 */
	int8_t opsState = (int8_t)0;
	/**
	 * offset 90
	 */
	int8_t startStopState = (int8_t)0;
	/**
	 * offset 91
	 */
	int8_t smartChipState = (int8_t)0;
	/**
	 * offset 92
	 */
	int8_t smartChipRestartCounter = (int8_t)0;
	/**
	 * offset 93
	 */
	int8_t smartChipAliveCounter = (int8_t)0;
	/**
	 * need 4 byte alignment
	 * units: units
	 * offset 94
	 */
	uint8_t alignmentFill_at_94[2] = {};
	/**
	offset 96 bit 0 */
	bool startStopPhysicalState : 1 {};
	/**
	 * Harley ACR Active
	offset 96 bit 1 */
	bool acrActive : 1 {};
	/**
	offset 96 bit 2 */
	bool acrEngineMovedRecently : 1 {};
	/**
	offset 96 bit 3 */
	bool heaterControlEnabled : 1 {};
	/**
	offset 96 bit 4 */
	bool luaDigitalState0 : 1 {};
	/**
	offset 96 bit 5 */
	bool luaDigitalState1 : 1 {};
	/**
	offset 96 bit 6 */
	bool luaDigitalState2 : 1 {};
	/**
	offset 96 bit 7 */
	bool luaDigitalState3 : 1 {};
	/**
	 * @@INDICATOR_NAME_CLUTCH_DOWN@@
	offset 96 bit 8 */
	bool clutchDownState : 1 {};
	/**
	offset 96 bit 9 */
	bool unusedBit_24_9 : 1 {};
	/**
	offset 96 bit 10 */
	bool unusedBit_24_10 : 1 {};
	/**
	offset 96 bit 11 */
	bool unusedBit_24_11 : 1 {};
	/**
	offset 96 bit 12 */
	bool unusedBit_24_12 : 1 {};
	/**
	offset 96 bit 13 */
	bool unusedBit_24_13 : 1 {};
	/**
	offset 96 bit 14 */
	bool unusedBit_24_14 : 1 {};
	/**
	offset 96 bit 15 */
	bool unusedBit_24_15 : 1 {};
	/**
	offset 96 bit 16 */
	bool unusedBit_24_16 : 1 {};
	/**
	offset 96 bit 17 */
	bool unusedBit_24_17 : 1 {};
	/**
	offset 96 bit 18 */
	bool unusedBit_24_18 : 1 {};
	/**
	offset 96 bit 19 */
	bool unusedBit_24_19 : 1 {};
	/**
	offset 96 bit 20 */
	bool unusedBit_24_20 : 1 {};
	/**
	offset 96 bit 21 */
	bool unusedBit_24_21 : 1 {};
	/**
	offset 96 bit 22 */
	bool unusedBit_24_22 : 1 {};
	/**
	offset 96 bit 23 */
	bool unusedBit_24_23 : 1 {};
	/**
	offset 96 bit 24 */
	bool unusedBit_24_24 : 1 {};
	/**
	offset 96 bit 25 */
	bool unusedBit_24_25 : 1 {};
	/**
	offset 96 bit 26 */
	bool unusedBit_24_26 : 1 {};
	/**
	offset 96 bit 27 */
	bool unusedBit_24_27 : 1 {};
	/**
	offset 96 bit 28 */
	bool unusedBit_24_28 : 1 {};
	/**
	offset 96 bit 29 */
	bool unusedBit_24_29 : 1 {};
	/**
	offset 96 bit 30 */
	bool unusedBit_24_30 : 1 {};
	/**
	offset 96 bit 31 */
	bool unusedBit_24_31 : 1 {};
	/**
	 * offset 100
	 */
	uint32_t startStopStateToggleCounter = (uint32_t)0;
	/**
	 * offset 104
	 */
	float currentVe = (float)0;
	/**
	 * offset 108
	 */
	float luaSoftSparkSkip = (float)0;
	/**
	 * offset 112
	 */
	float luaHardSparkSkip = (float)0;
	/**
	 * offset 116
	 */
	float tractionControlSparkSkip = (float)0;
	/**
	 * Fuel: Injection counter
	 * offset 120
	 */
	uint32_t fuelInjectionCounter = (uint32_t)0;
	/**
	 * Ign: Spark counter
	 * offset 124
	 */
	uint32_t globalSparkCounter = (uint32_t)0;
	/**
	 * @@GAUGE_NAME_FUEL_LOAD@@
	 * offset 128
	 */
	float fuelingLoad = (float)0;
	/**
	 * @@GAUGE_NAME_IGNITION_LOAD@@
	 * offset 132
	 */
	float ignitionLoad = (float)0;
	/**
	 * units: %
	 * offset 136
	 */
	scaled_channel<uint16_t, 100, 1> veTableYAxis = (uint16_t)0;
	/**
	 * offset 138
	 */
	uint8_t overDwellCounter = (uint8_t)0;
	/**
	 * offset 139
	 */
	uint8_t overDwellNotScheduledCounter = (uint8_t)0;
	/**
	 * offset 140
	 */
	uint8_t sparkOutOfOrderCounter = (uint8_t)0;
	/**
	 * need 4 byte alignment
	 * units: units
	 * offset 141
	 */
	uint8_t alignmentFill_at_141[3] = {};
};
static_assert(sizeof(engine_state_s) == 144);

// end
// this section was generated automatically by rusEFI tool config_definition_base-all.jar based on (unknown script) controllers/algo/engine_state.txt
