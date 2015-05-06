#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  private:
    double inertia;
    double mu;
    double sigma2;
  public:
    IRBrain(double _inertia);
    ~IRBrain();
    double getState();
    void mark(float _x, bool _detect);
};

float erfc();

#endif
