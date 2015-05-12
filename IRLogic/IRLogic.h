#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  private:
    long near;
    long far;
    int bias;
  public:
    IRLogic();
    ~IRLogic();
    long getState();
    long getFrequency();
    void mark(long _frequency, bool _detect);
    void reset();
    static const long farFrequency = 37900;
    static const long nearFrequency = 43000;
};

#endif
