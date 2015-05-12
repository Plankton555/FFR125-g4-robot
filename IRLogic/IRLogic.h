#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  private:
    long near;
    long far;
    byte bias;
  public:
    IRLogic();
    ~IRLogic();
    long getState();
    unsigned int getFrequency();
    void mark(long _frequency, bool _detect);
    void reset();
    static const unsigned int farFrequency = 37900;
    static const unsigned int nearFrequency = 43000;
};

#endif
