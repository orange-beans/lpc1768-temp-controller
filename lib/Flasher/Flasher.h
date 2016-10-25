#ifndef MBED_FLASHER_H
#define MBED_FLASHER_H

#include "mbed.h"

// Create a Flasher class
class Flasher {
public:
  Flasher(PinName pin);
  Flasher(PinName pin, int num);
  void flash();
  void flash(int n);

private:
  DigitalOut _pin;
  int def_time;
};

#endif
