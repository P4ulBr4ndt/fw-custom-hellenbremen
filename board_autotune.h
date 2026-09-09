#pragma once

#include "cyclic_buffer.h"

void autotuneToggleRunning();
void autotuneBurnToROM();
void autotuneVETables();

struct live_data_autotune_s {
	float veTable[VE_LOAD_COUNT][VE_RPM_COUNT];
	float accumulatedWeight[VE_LOAD_COUNT][VE_RPM_COUNT];
	uint16_t hitCount[VE_LOAD_COUNT][VE_RPM_COUNT];
};

// One tick's worth of what a bank's closed-loop correction needs, buffered so it can be
// looked back up once the lambda sensor's transport delay for that (rpm, load) has passed.
struct autotune_sample_s {
	float rpm;
	float fuelLoad;
	float correction;
};

class AutotuneState {
public:
	AutotuneState() = default;
	~AutotuneState() = default;

private:
	live_data_autotune_s front;
	live_data_autotune_s rear;

	cyclic_buffer<autotune_sample_s> frontHistory;
	cyclic_buffer<autotune_sample_s> rearHistory;

	// History of engine->outputChannels.Gego, rusEFI's existing EGO correction gauge.
	cyclic_buffer<float> gegoHistory;

} autotuneState;