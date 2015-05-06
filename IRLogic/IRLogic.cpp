#include "Arduino.h"
#include "IRLogic.h"

IRLogic::IRLogic(double _inertia)
  : inertia(_inertia) {}

IRLogic::~IRLogic() {}

double IRLogic::getState() {
  return mu;
}

void IRLogic::mark(double _x, bool _detect) {
  double a, dmu, k, p, sigma, z;
  sigma = sqrt(sigma2);
  z = (_x - mu) / sigma;
  if (_detect)
    z *= -1;
  p = erfc(z);
  k = -log(p);
  a = (_detect - mu) / sigma;
  dmu = exp(-z * z / 2) - exp(-a * a / 2);
  if (_detect)
	  dmu = -dmu;
  dmu *= sigma * 0.398942280401433;
  dmu += mu;
  sigma2 = (inertia * sigma2 + k * z * z * sigma2) / (inertia + k);
  mu = (inertia * mu + k * dmu) / (inertia + k);
}

double erfc(double z) {
  double b, k, p;
  double zabs = abs(z);
  if (zabs > 37)
    p = 0;
  else {
    k = exp(-0.5 * zabs * zabs);
    if (zabs < 7.07106781186547) {
      b = 3.52624965998911e-2 * zabs + 0.700383064443688;
      b = b * zabs + 6.37396220353165;
      b = b * zabs + 33.912866078383;
      b = b * zabs + 112.079291497871;
      b = b * zabs + 221.213596169931;
      b = b * zabs + 220.206867912376;
      p = k * b;
      b = 8.83883476483184e-2 * zabs + 1.75566716318264;
      b = b * zabs + 16.064177579207;
      b = b * zabs + 86.7807322029461;
      b = b * zabs + 296.564248779674;
      b = b * zabs + 637.333633378831;
      b = b * zabs + 793.826512519948;
      b = b * zabs + 440.413735824752;
      p = p / b;
    } else {
      b = zabs + 0.65;
      b = zabs + 4 / b;
      b = zabs + 3 / b;
      b = zabs + 2 / b;
      b = zabs + 1 / b;
      p = k / b / 2.506628274631;
    }
  }
  if (z > 0)
    p = 1 - p;
  return p;
}
