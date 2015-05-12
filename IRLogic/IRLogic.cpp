#include "Arduino.h"
#include "IRLogic.h"

IRLogic::IRLogic() {
	far = 1024;
	near = 0;
	bias = 1;
}

IRLogic::~IRLogic() {}

double IRLogic::getState() {
  return (far + near) / 2;
}

unsigned int IRLogic::getFrequency() {
	long a = max(bias, 1);
	long b = max(-bias, 1);
	long x = (near * a + far * b) / (a + b);
	return x * (farFrequency - nearFrequency) / 1024 + nearFrequency;
}

void IRLogic::mark(long _frequency, bool _detect) {
	long x = (_frequency - nearFrequency) * 1024 / (farFrequency - nearFrequency);
	long mu = getState();
	if (_detect && x < far) {
		if (x < mu) {
			near = max(near - mu + x, 0);
			far = mu;
		} else far = x;
		bias = max(1, bias + 1);
	}
	else if (!_detect && x > near) {
		if (x > mu) {
			far = min(far - mu + x, 1024);
			near = mu;
		} else near = x;
		bias = min(-1, bias - 1);
	}
}

void IRLogic::reset() {
	far = 1024;
	near = 0;
	bias = 1;
}
