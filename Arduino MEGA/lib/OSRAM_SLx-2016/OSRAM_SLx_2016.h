
#ifndef OSRAM_SLx_2016_H_
#define OSRAM_SLx_2016_H_

#include "Arduino.h"

class OSRAM_SLx{

public:
  void writeDigit(int digit, char dchar);
  void clearDisplay();
  void print_slx2016(char mesg_string[]);

};


//Do not add code below this line
#endif /* OSRAM_SLx-2016_H_ */
