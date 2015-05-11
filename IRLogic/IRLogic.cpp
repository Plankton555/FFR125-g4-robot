#include "Arduino.h"
#include "IRLogic.h"

IRLogic::IRLogic() {
	mu = 0.5;
	sigma = 0.1;
	inertia = 10.0;
}

IRLogic::~IRLogic() {}

void IRLogic::setInertia(double _inertia) {
  inertia = constrain(_inertia, 1, 1e3);
}

void IRLogic::setSigma(double _sigma) {
  sigma = constrain(_sigma, 1e-3, 0.5);
}

double IRLogic::getState() {
  return mu;
}

unsigned int IRLogic::getFrequency() {
	return nearFrequency + (unsigned int) (mu * (farFrequency - nearFrequency));
}

void IRLogic::mark(uint16_t _frequency, bool _detect) {
	
  double k, x, z;
  
  // Scale frequency to unit range
  x = ((double)_frequency - nearFrequency) / (farFrequency - nearFrequency);
  
  // Rescale to z-score
  z = (x - mu) / sigma;
  if (_detect)
    z *= -1;

  // Calculate information gained wrt current distribution
  k = log_erfc(z);
  
  // Update mean
  mu = (inertia * mu + k * !_detect) / (inertia + k);
}

// 5th-order polynomial approximation of log(cdf(z))
double log_erfc(double z) {  
  double k =  9.728651322615837e-04;
  k = k * z - 1.544435817384158e-03;
  k = k * z - 8.843786613101896e-02;
  k = k * z + 5.638690930288579e-01;
  k = k * z - 1.174907902964256e+00;
  k = k * z + 7.518621306864579e-01;
  return k;
}
