#pragma once
extern float testTimeSeconds;
class Timer {
public:
	void reset() { started = testTimeSeconds; }
	bool hasElapsedSec(float seconds) const { return testTimeSeconds - started >= seconds; }
private:
	float started = 0;
};
