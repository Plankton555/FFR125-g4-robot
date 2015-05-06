#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  private:
    double inertia;
    double mu;
    double sigma2;
  public:
    IRLogic();
    ~IRLogic();
    void setInertia(double _inertia);
    double getState();
    unsigned int getFrequency();
    void mark(unsigned int _frequency, bool _detect);
    static const unsigned int farFrequency = 37000;
    static const unsigned int nearFrequency = 42000;
};

double erfc(double z);

#endif
