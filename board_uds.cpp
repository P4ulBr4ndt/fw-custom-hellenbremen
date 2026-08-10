#include "pch.h"

#include "board_uds.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "efitime.h"
#include "can_msg_tx.h"
#include "flash_main.h"

namespace {
constexpr uint32_t kUdsReqId = 0x7E0;
constexpr uint32_t kUdsBroadcastId = 0x7DF;
constexpr uint32_t kUdsRespId = 0x7E8;

constexpr size_t kMaxUdsPayload = 263;

constexpr efitick_t kIsoTpTimeout = MS2NT(1000);
constexpr float kMinBatteryVoltage = 11.0f;

constexpr uint8_t kNrcGeneralReject = 0x10;
constexpr uint8_t kNrcIncorrectLength = 0x13;
constexpr uint8_t kNrcConditionsNotCorrect = 0x22;
constexpr uint8_t kNrcRequestSequenceError = 0x24;
constexpr uint8_t kNrcRequestOutOfRange = 0x31;
constexpr uint8_t kNrcGeneralProgrammingFailure = 0x72;

constexpr uint16_t kDidBasicEngineData = 0x0200;
constexpr uint16_t kDidVin = 0xF190;
constexpr uint16_t kDidProgrammingDate = 0xF199;
constexpr uint16_t kDidCalibrationId = 0xF1ED;
constexpr size_t kDidBasicEngineDataSize = 10;
constexpr size_t kDidStringMaxLen = 11;
constexpr size_t kDidProgrammingDateLen = 8;
constexpr size_t kVinLength = sizeof(engineConfiguration->vinNumber);
constexpr size_t kMaxUdsTxPayload = 32;

struct IsoTpRxState {
	bool active = false;
	size_t totalLen = 0;
	size_t receivedLen = 0;
	uint8_t nextSeq = 1;
	efitick_t lastRxNt = 0;
	std::array<uint8_t, kMaxUdsPayload> buffer{};
};

struct IsoTpTxState {
	bool active = false;
	size_t totalLen = 0;
	size_t sentLen = 0;
	uint8_t nextSeq = 1;
	uint8_t blockSize = 0;
	uint8_t stMin = 0;
	std::array<uint8_t, kMaxUdsTxPayload> buffer{};
};

IsoTpRxState isoTpRx;
IsoTpTxState isoTpTx;
bool udsProgrammingSession = false;

static uint16_t readU16be(const uint8_t* data) {
	return static_cast<uint16_t>((data[0] << 8) | data[1]);
}

static void writeU16be(uint8_t* dest, uint16_t value) {
	dest[0] = static_cast<uint8_t>((value >> 8) & 0xFF);
	dest[1] = static_cast<uint8_t>(value & 0xFF);
}

static uint32_t decodeIsoTpSeparationTimeUs(uint8_t stMin) {
	if (stMin <= 0x7F) {
		return static_cast<uint32_t>(stMin) * 1000;
	}

	if (stMin >= 0xF1 && stMin <= 0xF9) {
		return static_cast<uint32_t>(stMin - 0xF0) * 100;
	}

	return 0;
}

static void sleepIsoTpSeparationTime(uint8_t stMin) {
	uint32_t separationTimeUs = decodeIsoTpSeparationTimeUs(stMin);
	if (separationTimeUs == 0) {
		return;
	}

	chThdSleepMicroseconds(separationTimeUs);
}

static bool validateVin(const uint8_t* vin, size_t len, std::array<uint8_t, kVinLength>& normalized) {
	if (len != kVinLength) {
		return false;
	}

	for (size_t i = 0; i < len; i++) {
		char value = static_cast<char>(vin[i]);
		if (value >= 'a' && value <= 'z') {
			value = static_cast<char>(value - 'a' + 'A');
		}

		bool isDigit = value >= '0' && value <= '9';
		bool isUpper = value >= 'A' && value <= 'Z';
		if (!isDigit && !isUpper) {
			return false;
		}

		normalized[i] = static_cast<uint8_t>(value);
	}

	return true;
}

static bool storeVin(const std::array<uint8_t, kVinLength>& vin) {
	if (std::memcmp(engineConfiguration->vinNumber, vin.data(), kVinLength) == 0) {
		return true;
	}

	std::memcpy(engineConfiguration->vinNumber, vin.data(), kVinLength);
	setNeedToWriteConfiguration();
	return true;
}

static size_t copyDidStringTail(uint8_t* dest, size_t maxLen, const char* src) {
	if (!src || maxLen == 0) {
		return 0;
	}

	size_t len = std::strlen(src);
	if (len <= maxLen) {
		std::memcpy(dest, src, len);
		return len;
	}

	const char* start = src + (len - maxLen);
	std::memcpy(dest, start, maxLen);
	return maxLen;
}

static size_t writeProgrammingDate(uint8_t* dest, size_t maxLen) {
	if (maxLen == 0) {
		return 0;
	}

	int value = engineConfiguration->calibrationBirthday;
	char tmp[kDidProgrammingDateLen];
	for (int i = static_cast<int>(kDidProgrammingDateLen) - 1; i >= 0; i--) {
		tmp[i] = static_cast<char>('0' + (value % 10));
		value /= 10;
	}

	size_t len = std::min(maxLen, kDidProgrammingDateLen);
	std::memcpy(dest, tmp, len);
	return len;
}

static uint16_t clampU16(int32_t value) {
	if (value < 0) {
		return 0;
	}
	if (value > 0xFFFF) {
		return 0xFFFF;
	}
	return static_cast<uint16_t>(value);
}

static int16_t clampI16(int32_t value) {
	if (value < INT16_MIN) {
		return INT16_MIN;
	}
	if (value > INT16_MAX) {
		return INT16_MAX;
	}
	return static_cast<int16_t>(value);
}

static void sendIsoTpSingleFrame(size_t busIndex, const uint8_t* payload, size_t len) {
	CanTxMessage msg(CanCategory::OBD, kUdsRespId, 8, busIndex, false);
	msg[0] = static_cast<uint8_t>(len & 0x0F);
	for (size_t i = 0; i < len && i < 7; i++) {
		msg[i + 1] = payload[i];
	}
	for (size_t i = len + 1; i < 8; i++) {
		msg[i] = 0;
	}
}

static void resetIsoTpTx() {
	isoTpTx.active = false;
	isoTpTx.totalLen = 0;
	isoTpTx.sentLen = 0;
	isoTpTx.nextSeq = 1;
	isoTpTx.blockSize = 0;
	isoTpTx.stMin = 0;
}

static void sendIsoTpFirstFrame(size_t busIndex, const uint8_t* payload, size_t len) {
	CanTxMessage msg(CanCategory::OBD, kUdsRespId, 8, busIndex, false);
	msg[0] = static_cast<uint8_t>(0x10 | ((len >> 8) & 0x0F));
	msg[1] = static_cast<uint8_t>(len & 0xFF);
	for (size_t i = 0; i < 6; i++) {
		msg[i + 2] = (i < len) ? payload[i] : 0;
	}
}

static void sendIsoTpConsecutiveFrame(size_t busIndex, uint8_t seq, const uint8_t* payload, size_t len) {
	CanTxMessage msg(CanCategory::OBD, kUdsRespId, 8, busIndex, false);
	msg[0] = static_cast<uint8_t>(0x20 | (seq & 0x0F));
	for (size_t i = 0; i < 7; i++) {
		msg[i + 1] = (i < len) ? payload[i] : 0;
	}
}

static void sendIsoTpResponse(size_t busIndex, const uint8_t* payload, size_t len) {
	if (len <= 7) {
		sendIsoTpSingleFrame(busIndex, payload, len);
		return;
	}

	if (len > isoTpTx.buffer.size()) {
		efiPrintf("UDS response too large: %u", static_cast<unsigned>(len));
		return;
	}

	resetIsoTpTx();
	std::memcpy(isoTpTx.buffer.data(), payload, len);
	isoTpTx.totalLen = len;
	isoTpTx.sentLen = std::min<size_t>(len, 6);
	isoTpTx.nextSeq = 1;
	isoTpTx.active = true;

	sendIsoTpFirstFrame(busIndex, isoTpTx.buffer.data(), len);
}

static void sendIsoTpFlowControl(size_t busIndex, uint8_t fs, uint8_t blockSize, uint8_t stMin) {
	CanTxMessage msg(CanCategory::OBD, kUdsRespId, 8, busIndex, false);
	msg[0] = static_cast<uint8_t>(0x30 | (fs & 0x0F));
	msg[1] = blockSize;
	msg[2] = stMin;
	for (size_t i = 3; i < 8; i++) {
		msg[i] = 0;
	}
}

static void sendIsoTpConsecutiveFrames(size_t busIndex) {
	if (!isoTpTx.active) {
		return;
	}

	size_t blocksLeft = isoTpTx.blockSize == 0 ? SIZE_MAX : isoTpTx.blockSize;
	while (isoTpTx.sentLen < isoTpTx.totalLen && blocksLeft > 0) {
		size_t remaining = isoTpTx.totalLen - isoTpTx.sentLen;
		size_t chunk = std::min<size_t>(remaining, 7);
		sendIsoTpConsecutiveFrame(busIndex, isoTpTx.nextSeq,
			&isoTpTx.buffer[isoTpTx.sentLen], chunk);
		isoTpTx.sentLen += chunk;
		isoTpTx.nextSeq = (isoTpTx.nextSeq + 1) & 0x0F;
		if (isoTpTx.nextSeq == 0) {
			isoTpTx.nextSeq = 1;
		}
		blocksLeft--;

		if (isoTpTx.sentLen < isoTpTx.totalLen && blocksLeft > 0) {
			sleepIsoTpSeparationTime(isoTpTx.stMin);
		}
	}

	if (isoTpTx.sentLen >= isoTpTx.totalLen) {
		resetIsoTpTx();
	}
}

static const char* nrcToString(uint8_t nrc) {
	switch (nrc) {
		case kNrcGeneralReject:
			return "GeneralReject";
		case kNrcIncorrectLength:
			return "IncorrectLength";
		case kNrcConditionsNotCorrect:
			return "ConditionsNotCorrect";
		case kNrcRequestSequenceError:
			return "RequestSequenceError";
		case kNrcRequestOutOfRange:
			return "RequestOutOfRange";
		case kNrcGeneralProgrammingFailure:
			return "GeneralProgrammingFailure";
		default:
			return "UnknownNrc";
	}
}

static const char* serviceToString(uint8_t service) {
	switch (service) {
		case 0x10:
			return "DiagnosticSessionControl";
		case 0x11:
			return "EcuReset";
		case 0x22:
			return "ReadDataByIdentifier";
		case 0x2E:
			return "WriteDataByIdentifier";
		case 0x3E:
			return "TesterPresent";
		default:
			return "UnknownService";
	}
}

static void sendUdsNegativeResponse(size_t busIndex, uint8_t service, uint8_t nrc, const char* reason) {
	const char* reasonText = reason ? reason : "unspecified";
	efiPrintf("UDS NRC: service=0x%02X (%s) nrc=0x%02X (%s) reason=%s",
		service, serviceToString(service), nrc, nrcToString(nrc), reasonText);
	const uint8_t payload[3] = {0x7F, service, nrc};
	sendIsoTpResponse(busIndex, payload, sizeof(payload));
}

static bool handleTesterPresent(size_t busIndex, const uint8_t* data, size_t len) {
	if (len != 3 || data[0] != 0x3E || data[1] != 0x00 || data[2] != 0x01) {
		sendUdsNegativeResponse(busIndex, 0x3E, kNrcIncorrectLength, "tester present format invalid");
		return true;
	}

	const uint8_t response[] = {0x7E, 0x00, 0x01};
	sendIsoTpResponse(busIndex, response, sizeof(response));
	return true;
}

static bool handleEcuReset(size_t busIndex, const uint8_t* data, size_t len) {
	if (len < 2) {
		sendUdsNegativeResponse(busIndex, 0x11, kNrcIncorrectLength,
			"ecu reset length invalid");
		return true;
	}

	uint8_t subFunction = data[1];
	bool suppressResponse = (subFunction & 0x80) != 0;
	subFunction &= 0x7F;

	auto sendPositive = [&]() {
		if (suppressResponse) {
			return;
		}
		const uint8_t response[] = {0x51, subFunction};
		sendIsoTpResponse(busIndex, response, sizeof(response));
	};

	switch (subFunction) {
		case 0x01: // Hard reset
		case 0x03: // Soft reset
			sendPositive();
			scheduleReboot();
			return true;
		case 0x02: // Key off/on reset -> use DFU bootloader when available
#if EFI_DFU_JUMP
			sendPositive();
			jump_to_bootloader();
			return true;
#else
			sendUdsNegativeResponse(busIndex, 0x11, kNrcRequestOutOfRange,
				"dfu bootloader jump not enabled");
			return true;
#endif
		case 0x04: // Rapid power shutdown -> use OpenBLT when available
#if EFI_USE_OPENBLT
			sendPositive();
			jump_to_openblt();
			return true;
#else
			sendUdsNegativeResponse(busIndex, 0x11, kNrcRequestOutOfRange,
				"openblt jump not enabled");
			return true;
#endif
		default:
			sendUdsNegativeResponse(busIndex, 0x11, kNrcRequestOutOfRange,
				"ecu reset subfunction not supported");
			return true;
	}
}

static bool handleReadDataByIdentifier(size_t busIndex, const uint8_t* data, size_t len) {
	if (len != 3) {
		sendUdsNegativeResponse(busIndex, 0x22, kNrcIncorrectLength,
			"read data by identifier length invalid");
		return true;
	}

	uint16_t did = readU16be(&data[1]);

	switch (did) {
		case kDidBasicEngineData: {
			std::array<uint8_t, 3 + kDidBasicEngineDataSize> response{};
			response[0] = 0x62;
			response[1] = static_cast<uint8_t>((did >> 8) & 0xFF);
			response[2] = static_cast<uint8_t>(did & 0xFF);

			// DID 0x0200 payload: RPM (1 rpm), vehicle speed (0.1 kph), CLT (0.1 C), AFR1/AFR2 (0.01 AFR)
			uint16_t rpm = clampU16(static_cast<int32_t>(Sensor::getOrZero(SensorType::Rpm) + 0.5f));
			uint16_t vss = clampU16(static_cast<int32_t>(Sensor::getOrZero(SensorType::VehicleSpeed) * 10.0f + 0.5f));
			int16_t clt = clampI16(static_cast<int32_t>(Sensor::getOrZero(SensorType::Clt) * 10.0f));
			float stoich = engine->fuelComputer.getStoichiometricRatio();
			uint16_t afr1 = clampU16(static_cast<int32_t>(Sensor::getOrZero(SensorType::Lambda1) * stoich * 100.0f + 0.5f));
			uint16_t afr2 = clampU16(static_cast<int32_t>(Sensor::getOrZero(SensorType::Lambda2) * stoich * 100.0f + 0.5f));

			writeU16be(&response[3], rpm);
			writeU16be(&response[5], vss);
			writeU16be(&response[7], static_cast<uint16_t>(clt));
			writeU16be(&response[9], afr1);
			writeU16be(&response[11], afr2);

			sendIsoTpResponse(busIndex, response.data(), response.size());
			return true;
		}
		case kDidVin: {
			std::array<uint8_t, 3 + kVinLength> response{};
			response[0] = 0x62;
			response[1] = static_cast<uint8_t>((did >> 8) & 0xFF);
			response[2] = static_cast<uint8_t>(did & 0xFF);
			std::memcpy(&response[3], engineConfiguration->vinNumber, kVinLength);
			sendIsoTpResponse(busIndex, response.data(), response.size());
			return true;
		}
		case kDidProgrammingDate: {
			std::array<uint8_t, 3 + kDidProgrammingDateLen> response{};
			response[0] = 0x62;
			response[1] = static_cast<uint8_t>((did >> 8) & 0xFF);
			response[2] = static_cast<uint8_t>(did & 0xFF);
			size_t dateLen = writeProgrammingDate(&response[3], kDidProgrammingDateLen);
			sendIsoTpResponse(busIndex, response.data(), 3 + dateLen);
			return true;
		}
		case kDidCalibrationId: {
			std::array<uint8_t, 3 + kDidStringMaxLen> response{};
			response[0] = 0x62;
			response[1] = static_cast<uint8_t>((did >> 8) & 0xFF);
			response[2] = static_cast<uint8_t>(did & 0xFF);
			size_t strLen = copyDidStringTail(&response[3], kDidStringMaxLen, getTsSignature());
			sendIsoTpResponse(busIndex, response.data(), 3 + strLen);
			return true;
		}
		default:
			sendUdsNegativeResponse(busIndex, 0x22, kNrcRequestOutOfRange,
				"identifier not supported");
			return true;
	}
}

static bool isProgrammingAllowed() {
	if (engine->rpmCalculator.isRunning()) {
		return false;
	}

	if (!isIgnVoltage()) {
		return false;
	}

	float vbatt = Sensor::getOrZero(SensorType::BatteryVoltage);
	return vbatt >= kMinBatteryVoltage;
}

static bool handleWriteDataByIdentifier(size_t busIndex, const uint8_t* data, size_t len) {
	if (len < 3) {
		sendUdsNegativeResponse(busIndex, 0x2E, kNrcIncorrectLength,
			"write data by identifier length invalid");
		return true;
	}

	uint16_t did = readU16be(&data[1]);
	if (did != kDidVin) {
		sendUdsNegativeResponse(busIndex, 0x2E, kNrcRequestOutOfRange,
			"identifier not supported");
		return true;
	}

	if (len != (3 + kVinLength)) {
		sendUdsNegativeResponse(busIndex, 0x2E, kNrcIncorrectLength,
			"vin length invalid");
		return true;
	}

	if (!isProgrammingAllowed()) {
		sendUdsNegativeResponse(busIndex, 0x2E, kNrcConditionsNotCorrect,
			"programming conditions not met");
		return true;
	}

	std::array<uint8_t, kVinLength> normalized{};
	if (!validateVin(&data[3], kVinLength, normalized)) {
		sendUdsNegativeResponse(busIndex, 0x2E, kNrcRequestOutOfRange,
			"vin invalid");
		return true;
	}

	if (!storeVin(normalized)) {
		sendUdsNegativeResponse(busIndex, 0x2E, kNrcGeneralProgrammingFailure,
			"vin write failed");
		return true;
	}

	const uint8_t response[] = {
		0x6E,
		static_cast<uint8_t>((did >> 8) & 0xFF),
		static_cast<uint8_t>(did & 0xFF),
	};
	sendIsoTpResponse(busIndex, response, sizeof(response));
	return true;
}

static void resetIsoTpState() {
	isoTpRx.active = false;
	isoTpRx.totalLen = 0;
	isoTpRx.receivedLen = 0;
	isoTpRx.nextSeq = 1;
	isoTpRx.lastRxNt = 0;
}

static void startProgrammingSession() {
	udsProgrammingSession = true;
}

static void stopProgrammingSession() {
	udsProgrammingSession = false;
}

static bool handleDiagnosticSessionControl(size_t busIndex, const uint8_t* data, size_t len) {
	if (len != 2) {
		sendUdsNegativeResponse(busIndex, 0x10, kNrcIncorrectLength,
			"session control length invalid");
		return true;
	}

	uint8_t sessionType = data[1];
	if (sessionType == 0x01) {
		stopProgrammingSession();
		const uint8_t response[] = {0x50, 0x01};
		sendIsoTpSingleFrame(busIndex, response, sizeof(response));
		return true;
	}

	if (sessionType == 0x02) {
		if (!isProgrammingAllowed()) {
			sendUdsNegativeResponse(busIndex, 0x10, kNrcConditionsNotCorrect,
				"programming conditions not met");
			return true;
		}
		startProgrammingSession();
		const uint8_t response[] = {0x50, 0x02};
		sendIsoTpSingleFrame(busIndex, response, sizeof(response));
		return true;
	}

	sendUdsNegativeResponse(busIndex, 0x10, kNrcRequestOutOfRange,
		"session type not supported");
	return true;
}

static bool handleUdsRequest(size_t busIndex, const uint8_t* data, size_t len) {
	if (len == 0) {
		return false;
	}

	switch (data[0]) {
		case 0x10:
			return handleDiagnosticSessionControl(busIndex, data, len);
		case 0x11:
			return handleEcuReset(busIndex, data, len);
		case 0x22:
			return handleReadDataByIdentifier(busIndex, data, len);
		case 0x2E:
			return handleWriteDataByIdentifier(busIndex, data, len);
		case 0x3E:
			return handleTesterPresent(busIndex, data, len);
		default:
			sendUdsNegativeResponse(busIndex, data[0], kNrcRequestOutOfRange,
				"service not supported");
			return true;
	}
}

static void handleIsoTpFrame(size_t busIndex, const CANRxFrame& frame, efitick_t nowNt) {
	if (isoTpRx.active && (nowNt - isoTpRx.lastRxNt > kIsoTpTimeout)) {
		resetIsoTpState();
	}

	uint8_t pci = frame.data8[0];
	uint8_t frameType = (pci >> 4) & 0x0F;

	if (frameType == 0x0) {
		uint8_t len = pci & 0x0F;
		if (len == 0 || len > 7 || len > frame.DLC - 1) {
			return;
		}
		handleUdsRequest(busIndex, &frame.data8[1], len);
		return;
	}

	if (frameType == 0x3) {
		if (!isoTpTx.active) {
			return;
		}

		uint8_t fs = pci & 0x0F;
		if (fs == 0x0) {
			isoTpTx.blockSize = frame.data8[1];
			isoTpTx.stMin = frame.data8[2];
			sendIsoTpConsecutiveFrames(busIndex);
		} else if (fs == 0x2) {
			resetIsoTpTx();
		}
		return;
	}

	if (frameType == 0x1) {
		size_t totalLen = static_cast<size_t>((pci & 0x0F) << 8) | frame.data8[1];
		if (totalLen == 0 || totalLen > isoTpRx.buffer.size()) {
			resetIsoTpState();
			return;
		}

		isoTpRx.active = true;
		isoTpRx.totalLen = totalLen;
		isoTpRx.receivedLen = 0;
		isoTpRx.nextSeq = 1;
		isoTpRx.lastRxNt = nowNt;

		size_t chunk = std::min<size_t>(totalLen, 6);
		std::memcpy(isoTpRx.buffer.data(), &frame.data8[2], chunk);
		isoTpRx.receivedLen = chunk;

		sendIsoTpFlowControl(busIndex, 0x0, 0x0, 0x0);
		return;
	}

	if (frameType == 0x2) {
		if (!isoTpRx.active) {
			return;
		}

		uint8_t seq = pci & 0x0F;
		if (seq != isoTpRx.nextSeq) {
			resetIsoTpState();
			return;
		}

		size_t remaining = isoTpRx.totalLen - isoTpRx.receivedLen;
		size_t chunk = std::min<size_t>(remaining, 7);
		std::memcpy(&isoTpRx.buffer[isoTpRx.receivedLen], &frame.data8[1], chunk);
		isoTpRx.receivedLen += chunk;
		isoTpRx.nextSeq = (isoTpRx.nextSeq + 1) & 0x0F;
		isoTpRx.lastRxNt = nowNt;

		if (isoTpRx.receivedLen >= isoTpRx.totalLen) {
			handleUdsRequest(busIndex, isoTpRx.buffer.data(), isoTpRx.totalLen);
			resetIsoTpState();
		}
		return;
	}
}
} // namespace

void handleUdsCanRx(size_t busIndex, const CANRxFrame& frame, efitick_t nowNt) {
	if (CAN_ISX(frame)) {
		return;
	}

	uint32_t sid = CAN_SID(frame);
	if (sid == kUdsReqId || sid == kUdsBroadcastId) {
		handleIsoTpFrame(busIndex, frame, nowNt);
		return;
	}
}
