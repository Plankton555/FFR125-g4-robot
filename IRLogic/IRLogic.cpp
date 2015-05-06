#include "Arduino.h"
#include "IRLogic.h"

IRLogic::IRLogic() {
	mu = 0.5;
	sigma2 = 0.25;
	inertia = 10.0;
}

IRLogic::~IRLogic() {}

void IRLogic::setInertia(double _inertia) {
  inertia = _inertia;
}

double IRLogic::getState() {
  return mu;
}

unsigned int IRLogic::getFrequency() {
	return nearFrequency + (unsigned int) (mu * (farFrequency - nearFrequency));
}

void IRLogic::mark(uint16_t _frequency, bool _detect) {
	
  double a, k, p, sigma, x, z;
  
  // Scale frequency to unit range
  x = ((double)_frequency - nearFrequency) / (farFrequency - nearFrequency);
  
  // Rescale to z-score
  sigma = sqrt(sigma2);
  z = (x - mu) / sigma;
  if (_detect)
    z *= -1;

  // Calculate probability of measurement and
  // information gained based on current dist
  p = erfc(z);
  k = -log(p);
  
  // Boundary z-score
  a = (_detect - mu) / sigma;
  
  // Expected discrete observation
  x = exp(-z * z / 2) - exp(-a * a / 2);
  if (_detect)
	  x = -x;
  x *= sigma * 0.398942280401433;
  
  // Update variance
  sigma2 = (inertia * sigma2 + k * x * x) / (inertia + k);
  
  // Un-center
  x += mu;
  // Update mean
//  sigma2 = (inertia * sigma2 + k * z * z * sigma2) / (inertia + k);
  mu = (inertia * mu + k * x) / (inertia + k);
}

double erfc(double z) {
  // approximation from (Bell 2015)
  return 0.5 * (1 + (z>0?1:-1) * sqrt(1 - exp(-0.636619772367581 * z * z)));
}
