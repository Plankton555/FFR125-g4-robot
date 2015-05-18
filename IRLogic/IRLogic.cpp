#include "Arduino.h"
#include "IRLogic.h"

IRLogic::IRLogic() {
	frequency = (farFrequency + nearFrequency) / 2;
	decay = 19.0 / 20.0;
	delta = 512;
}

IRLogic::~IRLogic() {}

void IRLogic::mark(unsigned int _frequency, bool _detect) {
	
	float x;
	
	if (_detect) {
	  if (_frequency >= frequency) {
		  frequency += delta;
		  if (lastDetect) {
		  	delta <<= 1;
		  	delta &= 510;
	  	}
	  	else
	  		delta >>= 1;
		  delta |= 1;
		  lastDetect = _detect;
	  }
	  else
	  	frequency++;
	 }
	 else {
	  if (_frequency <= frequency) {
	    frequency -= delta;
	    if (!lastDetect) {
		    delta <<= 1;
		    delta &= 510;
	    }
	    else
  	  	delta >>= 1;
		  delta |= 1;
  	  lastDetect = _detect;
	  }
	  else
		  frequency--;
	}
	frequency = constrain(frequency, farFrequency, nearFrequency);
	x = float(nearFrequency - frequency) / 25000;
	state = state * decay + x * (1 - decay);
}
