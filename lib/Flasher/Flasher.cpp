#include "Flasher.h"
#include "mbed.h"

// For class init, normal assignment not working
Flasher::Flasher(PinName pin, int num) : _pin(pin) {
  //_pin = pin;
  def_time = num;
}

Flasher::Flasher(PinName pin) : _pin(pin) {
  //_pin = pin;
  def_time = 0;
}

void Flasher::flash(int n) {
  for(int i=0; i< n*2; i++) {
    _pin = !_pin;
    wait(0.2);
  }
}
// overloading with defualt time
void Flasher::flash() {
  for(int i=0; i< def_time*2; i++) {
    _pin = !_pin;
    wait(0.2);
  }
}
