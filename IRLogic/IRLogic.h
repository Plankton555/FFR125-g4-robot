#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  private:
    double inertia;
    double mu;
    double sigma2;
  public:
    IRLogic(double _inertia);
    ~IRLogic();
    double getState();
    void mark(double _x, bool _detect);
};

double erfc(double z);

#endif
