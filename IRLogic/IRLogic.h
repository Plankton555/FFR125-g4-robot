#ifndef IRLOGIC_H
#define IRLOGIC_H

class IRLogic {
  public:
    IRLogic();
    ~IRLogic();
    void mark(unsigned int _frequency, bool _detect);
		unsigned int frequency;
		unsigned int delta;
		float decay;
		float state;
		bool lastDetect;
    static const unsigned int farFrequency = 38000;
    static const unsigned int nearFrequency = 63000;
};

#endif
