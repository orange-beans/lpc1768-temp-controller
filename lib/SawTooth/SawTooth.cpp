#include "SawTooth.h"
#include "mbed.h"

// For class init, normal assignment not working
SawTooth::SawTooth(PinName pin, double factor) : _pin(pin) {
  length = 1000;
  updateTable(factor);
}

// TODO: create a private method to generate waveform table
SawTooth::SawTooth(PinName pin) : _pin(pin) {
  length = 1000;
  updateTable();
}

void SawTooth::waveOut(uint16_t n) {
  // Run for n times
  for ( int j = 0; j < n; j++) {
    // This is a single-shot wave
    for(int i = 0; i < length; i++) {
      _pin.write_u16(table[i]);
    }
  }
}

void SawTooth::setWave(double factor) {
  updateTable(factor);
}

// calculate the table
// longer length, higher resolution, longer period
// maximum 1023

void SawTooth::updateTable() {
  uint16_t amplitude = 65535;
  // Low to high waveform
  // for (int i = 0; i < length; i++) {
  //   // scale to amplitude range
  //   table[i] = i * amplitude / length;
  // }
  // High to low waveform
  for (int i = 0; i < length; i++) {
    // scale to amplitude range
    table[i] = (length - i) * amplitude / length;
  }
}

void SawTooth::updateTable(double factor) {
  if (factor > 1 || factor <= 0) factor = 1;
  uint16_t amplitude = (uint16_t)(factor * 65535);
  // Low to high waveform
  // for (int i = 0; i < length; i++) {
  //   // scale to amplitude range
  //   table[i] = i * amplitude / length;
  // }
  // High to low waveform
  for (int i = 0; i < length; i++) {
    // scale to amplitude range
    table[i] = (length - i) * amplitude / length;
  }
}
