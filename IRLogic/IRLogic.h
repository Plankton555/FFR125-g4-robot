#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  private:
    double inertia;
    double mu;
    double sigma;
  public:
    IRLogic();
    ~IRLogic();
    void setInertia(double _inertia);
    void setSigma(double _sigma);
    double getState();
    unsigned int getFrequency();
    void mark(unsigned int _frequency, bool _detect);
    static const unsigned int farFrequency = 37000;
    static const unsigned int nearFrequency = 42000;
};

double log_erfc(double z);

#endif
