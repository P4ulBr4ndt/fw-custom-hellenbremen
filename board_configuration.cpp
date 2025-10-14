#include "pch.h"
#include "hellen_meta.h"
#include "defaults.h"
#include "board_overrides.h"
#include "unused.h"

#define HARLEY_V_TWIN 45.0

// board-specific configuration setup
static void boardDefaultConfiguration() {
    // Trigger
    engineConfiguration->overrideTriggerGaps = true;
    engineConfiguration->gapTrackingLengthOverride = 3;
    engineConfiguration->triggerGapOverrideFrom[0] = 0.75;
    engineConfiguration->triggerGapOverrideTo[0] = 1.25;
    engineConfiguration->triggerGapOverrideFrom[1] = 0.05; // this one is custom
    engineConfiguration->triggerGapOverrideTo[1] = 0.5;
    engineConfiguration->triggerGapOverrideFrom[2] = 1.850; // this one is custom
    engineConfiguration->triggerGapOverrideTo[2] = 6;

    // Cam
    engineConfiguration->vvtMode[0] = VVT_MAP_V_TWIN;
    engineConfiguration->mapCamDetectionAnglePosition = 50;

    // Aux Outputs
    engineConfiguration->fanPin = Gpio::C7;
    engineConfiguration->fanOnTemperature = 0.f;
    engineConfiguration->fanOffTemperature = 0.f;
	engineConfiguration->fan2Pin = Gpio::C8;
    engineConfiguration->fan2OnTemperature = 80.f;
    engineConfiguration->fan2OffTemperature = 70.f;
    engineConfiguration->mainRelayPin = Gpio::Unassigned;

    // AFR TODO
    engineConfiguration->afr.hwChannel = EFI_ADC_NONE;
    engineConfiguration->enableAemXSeries = false;

    // MAP
    setCustomMap(/*lowValue*/ 20, /*mapLowValueVoltage*/ 0.79, /*highValue*/ 101.3, /*mapHighValueVoltage*/ 4);

    // TPS
    engineConfiguration->tpsMin = 0.50;
    engineConfiguration->tpsMax = 4.54;
	engineConfiguration->tps1SecondaryMin = 4.53;
	engineConfiguration->tps1SecondaryMax = 0.47;

    engineConfiguration->throttlePedalUpVoltage = 1.36;
    engineConfiguration->throttlePedalWOTVoltage = 4.46;
	engineConfiguration->throttlePedalSecondaryUpVoltage = 1.37;
	engineConfiguration->throttlePedalSecondaryWOTVoltage = 4.48;
}

static void boardConfigOverrides() {
    setHellenAnalogDividers();
    setHellenCan();
	hellenMegaSdWithAccelerometer();

	// VBATT is on PA0
	engineConfiguration->vbattAdcChannel = EFI_ADC_0;
	// set vbatt_divider 4,0303
	// 10k / 3.3k
	engineConfiguration->vbattDividerCoeff = 8.06f;

	// PWR EN
	setHellenEnPin(Gpio::E0, true);

    // Generic
    engineConfiguration->cylindersCount = 2;
    engineConfiguration->firingOrder = FO_1_2;
    strcpy(engineConfiguration->engineMake, "Harley");

    // Knock
    // engineConfiguration->enableSoftwareKnock = true;
    // engineConfiguration->knockFrequency = 0.0f; // TODO

    // Sensors
    engineConfiguration->map.sensor.hwChannel = EFI_ADC_13; // PC3
	engineConfiguration->fuelLevelSensor = EFI_ADC_2; // PA2

	// Switch Inputs
	engineConfiguration->jssPin = Gpio::G11;
	engineConfiguration->opsPin = Gpio::G12;

    // ETB
    engineConfiguration->etb_use_two_wires = false;
    engineConfiguration->etbIo[0].directionPin1 = Gpio::D13;
    engineConfiguration->etbIo[0].controlPin = Gpio::D12;
    engineConfiguration->etbIo[0].disablePin = Gpio::C12;
    engineConfiguration->tps1_1AdcChannel = EFI_ADC_4;
    engineConfiguration->tps1_2AdcChannel = EFI_ADC_14;
    engineConfiguration->throttlePedalPositionAdcChannel = EFI_ADC_10;
    engineConfiguration->throttlePedalPositionSecondAdcChannel = EFI_ADC_12;

	// Temp Sensor Pullups
	// As CLT we use ETS because thats the important figure for us
	// We have 820 against GND and 4700 against +5VA = 698 OHM for CLT
	engineConfiguration->clt.config.bias_resistor = 698; // ETS
	engineConfiguration->auxTempSensor1.config.bias_resistor = 698; // ETS
	engineConfiguration->auxTempSensor2.config.bias_resistor = 2350; // CLT
	engineConfiguration->ambientTempSensor.config.bias_resistor = 2350; // AAT
	engineConfiguration->iat.config.bias_resistor = 2350; // IAT

    // Temp Sensors
	engineConfiguration->clt.adcChannel = EFI_ADC_17; // ETS PA1 MUX = 1
	engineConfiguration->auxTempSensor1.adcChannel = EFI_ADC_17; // ETS
	engineConfiguration->auxTempSensor2.adcChannel = EFI_ADC_5; // CLT
	engineConfiguration->ambientTempSensor.adcChannel = EFI_ADC_1; // PA1 // AAT for CANBUS
    engineConfiguration->iat.adcChannel = EFI_ADC_18; // PA2 MUX = 1

    // Injection
    engineConfiguration->injectionMode = IM_SEQUENTIAL;
    engineConfiguration->injectionPins[0] = Gpio::G8;
    engineConfiguration->injectionPins[1] = Gpio::G7;

    // Ignition
    engineConfiguration->ignitionMode = IM_INDIVIDUAL_COILS;
    engineConfiguration->ignitionPins[0] = Gpio::E5;
    engineConfiguration->ignitionPins[1] = Gpio::C13;

    // Trigger
    engineConfiguration->triggerInputPins[0] = Gpio::B0;
    engineConfiguration->trigger.type = trigger_type_e::TT_TOOTHED_WHEEL_32_2;

    engineConfiguration->engineType = engine_type_e::HARLEY;

    // total 45 degree odd fire, split across two cylinders mostly for fun
    engineConfiguration->timing_offset_cylinder[0] = -HARLEY_V_TWIN / 2;
    engineConfiguration->timing_offset_cylinder[1] = +HARLEY_V_TWIN / 2;

    // work-around for https://github.com/rusefi/rusefi/issues/5894 todo: fix it!
    engineConfiguration->maximumIgnitionTiming = 90;
    engineConfiguration->minimumIgnitionTiming = -90;
}

#define CAN_HD_VSS_ID 0x142
#define CAN_HD_THROTTLE_ID 0x144

#define CAN_HD_RPM_OFFSET 0x0
#define CAN_HD_VSS_OFFSET 0x2
#define CAN_HD_GEAR_OFFSET 0x4

#define CAN_HD_BCM_PING_ID 0x503 // Here we receive a "Ping" kind of thing
#define CAN_HD_ECM_PING_ID 0x502 // Here we send our own

#define CAN_HD_VIN_ID_1 0x34D
#define CAN_HD_VIN_ID_2 0x34E
#define CAN_HD_VIN_ID_3 0x34F

static uint8_t frameCounter142 = 0x0;
static uint8_t frameCounter144 = 0x0;
static uint8_t frameCounter146_342 = 0x0;
static uint8_t frameCounter148 = 0x40;

static bool harleyKeepAlive = true;

/*
TODO CLUTCH looks like 0xD0
N: 0.872V => 17.44% => 0xA0
1: 0.484V => 09.86% => 0x10
2: 1.262V => 25,24% => 0x20
3: 2.098V => 41,96% => 0x30
4: 2.874V => 57,48% => 0x40
5: 3.643V => 72,96% => 0x50
6: 4.439V => 88,78% => 0x60
*/
static float harleyGearValues[] = { 17.44f, 9.86f, 25.24f, 41.96f, 57.48f, 72.96f, 88.78f };
static uint8_t calculateHarleyGearValue() {
  float sensorValue = Sensor::getOrZero(SensorType::AuxLinear1);
  float bestMatch = 0.0f;
  uint8_t bestOffs = 0;
  for (uint8_t i = 0; i < sizeof(harleyGearValues)/sizeof(harleyGearValues[0]); i++) {
    float i_delta = abs(harleyGearValues[i] - sensorValue);
    float x_delta = abs(bestMatch - sensorValue);
    if (i_delta < x_delta) {
      bestMatch = harleyGearValues[i];
      bestOffs = i;
    }
  }

  switch (bestOffs)
  {
    case 0:
      return 0xA0; // N
      break;
    case 1:
      return 0x10; // 1
      break;
    case 2:
      return 0x20; // 2
      break;
    case 3:
      return 0x30; // 3
      break;
    case 4:
      return 0x40; // 4
      break;
    case 5:
      return 0x50; // 5
      break;
    case 6:
      return 0x60; // 6
      break;
    
    default:
    return 0x0;
      break;
  }
}

static void handleHarleyCAN(CanCycle cycle) {
  if (cycle.isInterval(CI::_10ms)) {
    {
      CanTxMessage msg(CanCategory::NBC, CAN_HD_VSS_ID);
      msg.setShortValueMsb(Sensor::getOrZero(SensorType::Rpm), CAN_HD_RPM_OFFSET);
      msg.setShortValueMsb(Sensor::getOrZero(SensorType::VehicleSpeed) * 10.f, CAN_HD_VSS_OFFSET);
      msg[CAN_HD_GEAR_OFFSET] = calculateHarleyGearValue();
      msg[6] = frameCounter142;
      msg[7] = crc8(msg.getFrame()->data8, 7);
      frameCounter142 = (frameCounter142 + 1) % 64;
    }
  }
  
  /**
  uint floatToShortForTPS(float tps)
  {
    float buf;
    
    buf = FLOOR((float)(int)((tps + 0.1) * 5.0));
    return ((int)buf + 5000) * (uint)(-5001 < SUB42(buf,0)) & 0xffff;
  }
  */
  if (cycle.isInterval(CI::_20ms)) {
    {
      CanTxMessage msg(CanCategory::NBC, CAN_HD_THROTTLE_ID);
      msg.setShortValueMsb(Sensor::getOrZero(SensorType::Tps1Primary), 0); // TARGET TPS?
      msg.setShortValueMsb(Sensor::getOrZero(SensorType::Tps1Secondary), 2); // ACTUAL TPS?
      msg[4] = Sensor::getOrZero(SensorType::AcceleratorPedal) / 0.4545; // As OEM does
      msg[5] = 0x00; // TODO: What is this? It rarely moves up from 0 to 0x20 or 0x40 && 0x78 = TRACTION CONTROL FAILED. IS it slip?
      msg[6] = frameCounter144;
      msg[7] = crc8(msg.getFrame()->data8, 7);
      frameCounter144 = (frameCounter144 + 1) % 64;
    }
  }

  if (cycle.isInterval(CI::_50ms)) {
    bool running = engine->rpmCalculator.isRunning();
    {
      CanTxMessage msg(CanCategory::NBC, 0x146);
      msg[0] = 0x11; // JIFFY STAND somehow in here 0x11 = UP, 0x12 = DOWN
      msg[1] = 0x00; // CC MSB
      msg[2] = 0x00; // CC LSB
      msg[3] = running ? 0x44 : 0x04; // ENGINE LIGHT: 0x80 vs 0x40
      msg[4] = 0x00;
      msg[5] = 0x00;
      msg[6] = frameCounter146_342;
      msg[7] = crc8(msg.getFrame()->data8, 7);
    }
    
    {
      CanTxMessage msg(CanCategory::NBC, 0x342);
      msg[0] = 0x54; // 0x54 tempo aus, 0x64 tempo gelb, 0x84 tempo grün. Bit4: Jiffy Warning, Bit7: Turn on HAZARDLIGHT, Bit0: Tempo Grün, Bit 1+2: Tempo Gelb
      msg[1] = running ? 0x2A : 0x04; // MILES VS KM & DISPLAY RANGE POPUP,  16 = KM 17= MI, 0x18 = OIL LAMP
      msg[2] = 0x54;
      msg[3] = 0x00; // BATTERY RED LED, REMAINING RANGE MSB
      msg[4] = 21.0f * (Sensor::getOrZero(SensorType::FuelLevel) / 100) * (100.0f / 5.5f); // REMAINING RANGE LSB in KM. 6.0f = l/100km 21.0f = tank volume TODO: Make tank volume and fuel usage somewhat dynamic
      msg[5] = Sensor::getOrZero(SensorType::FuelLevel);
      msg[6] = frameCounter146_342;
      msg[7] = crc8(msg.getFrame()->data8, 7);
    }

    {
      CanTxMessage msg(CanCategory::NBC, 0x344);
      msg[0] = 0x00;
      msg[1] = Sensor::getOrZero(SensorType::OilTemperature) + 40; // ENGINE TEMPERATURE
      msg[2] = Sensor::getOrZero(SensorType::Clt) + 40; // CLT WHEN AVAILABLE
      msg[3] = 0xFF;
      msg[4] = 0xCC; // Mostly 0xCC, in the end of log 0xCB, Ambient Pressure * 2 ?? 204 / 2 = 102 = 1020hPa?
      msg[5] = 0x21;
      msg[6] = 0x00;
      msg[7] = 0x00;
    }
    frameCounter146_342 = (frameCounter146_342 + 1) % 64;
  }

  if (cycle.isInterval(CI::_200ms)) {
    {
      CanTxMessage msg(CanCategory::NBC, 0x540); // THIS IS ODOMETER AS WELL
      msg[0] = 0x00; // ODOMETER 0x000A8658 = 689752 Displayed as 689.8KM on PAM AMERICA ST
      msg[1] = 0x0A; // ODOMETER
      msg[2] = 0x86; // ODOMETER
      msg[3] = 0x58; // ODOMETER
      msg[4] = 0x00;
      msg[5] = 0x00;
      msg[6] = 0x00;
      msg[7] = 0x00;
    }
  }

  if (cycle.isInterval(CI::_1000ms)) {
    {
      CanTxMessage msg(CanCategory::NBC, CAN_HD_VIN_ID_1);
      msg[0] = engineConfiguration->vinNumber[0];
      msg[1] = engineConfiguration->vinNumber[1];
      msg[2] = engineConfiguration->vinNumber[2];
      msg[3] = engineConfiguration->vinNumber[3];
      msg[4] = engineConfiguration->vinNumber[4];
      msg[5] = engineConfiguration->vinNumber[5];
    }

    {
      CanTxMessage msg(CanCategory::NBC, CAN_HD_VIN_ID_2);
      msg[0] = engineConfiguration->vinNumber[6];
      msg[1] = engineConfiguration->vinNumber[7];
      msg[2] = engineConfiguration->vinNumber[8];
      msg[3] = engineConfiguration->vinNumber[9];
      msg[4] = engineConfiguration->vinNumber[10];
      msg[5] = engineConfiguration->vinNumber[11];
    } 

    {
      CanTxMessage msg(CanCategory::NBC, CAN_HD_VIN_ID_3);
      msg[0] = engineConfiguration->vinNumber[12];
      msg[1] = engineConfiguration->vinNumber[13];
      msg[2] = engineConfiguration->vinNumber[14];
      msg[3] = engineConfiguration->vinNumber[15];
      msg[4] = engineConfiguration->vinNumber[16];
      msg[5] = engineConfiguration->vinNumber[17];
    }

    {
      CanTxMessage msg(CanCategory::NBC, 0x148);
      msg[0] = 0x31; // Sometimes swichtes between 0x31 and 0x11
      msg[1] = 0x13;
      msg[2] = 0x00;
      msg[3] = 0x00;
      msg[4] = 0x00;
      msg[5] = 0x00;
      msg[6] = frameCounter148;
      msg[7] = crc8(msg.getFrame()->data8, 7);
    }
    frameCounter148++;
    if (frameCounter148 > 0x7F) {
      frameCounter148 = 0x40;
    }

    {
      CanTxMessage msg(CanCategory::NBC, 0x346);
      msg[0] = 0x00; // ODOMETER 0x000A8658 = 689752 Displayed as 689.8KM on PAM AMERICA ST
      msg[1] = 0x0A; // ODOMETER
      msg[2] = 0x86; // ODOMETER
      msg[3] = 0x58; // ODOMETER
      msg[4] = 0x00;
      msg[5] = Sensor::getOrZero(SensorType::AmbientTemperature) * 2 + 80;
      msg[6] = 0x80;
      msg[7] = 0x00;
    }

    {
      CanTxMessage msg(CanCategory::NBC, 0x348);
      msg[0] = 0x00;
      msg[1] = 0x00; // ALWAYS COUNTING UP
      msg[2] = 0x00; // ALWAYS COUNTING UP
      msg[3] = 0x0D; // STANDING STILL: 0x0D, MOVING: 0x17
      msg[4] = 0xAC; // IDLE: 0xAC, DRIVING: Moving down a lot from base value 0xD4
      msg[5] = 0x00;
      msg[6] = 0x00;
      msg[7] = 0x00;
    }

    {
      CanTxMessage msg(CanCategory::NBC, CAN_HD_ECM_PING_ID, 0x1/* DLC */);
      msg[0] = harleyKeepAlive;
    }
  }
}

void boardProcessCanRx(const size_t busIndex, const CANRxFrame &frame, efitick_t nowNt) {
  UNUSED(busIndex);
  UNUSED(nowNt);
  if (CAN_SID(frame) == 0x500) {
    harleyKeepAlive = frame.data8[0];
  }
}


void setup_custom_board_overrides() {
	custom_board_DefaultConfiguration = boardDefaultConfiguration;
	custom_board_ConfigOverrides = boardConfigOverrides;
	custom_board_can_rx = boardProcessCanRx;
	custom_board_update_dash = handleHarleyCAN;
}