#include "mbed.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <SawTooth.h>
Serial pc(USBTX, USBRX);

// DAC pin18
//AnalogOut aout(p18);

Ticker flipper;
Ticker flipper2;
PwmOut led1(LED1);
//DigitalOut led1(LED1);
DigitalOut led2(LED2);
//DigitalOut led3(LED3);
//DigitalOut led4(LED4);

SawTooth sawTooth(p18, 0.5);
Flasher led3(LED3);
Flasher led4(LED4, 2);

void flip2() {
  led1 = !led1;
}

void flip() {
  led2 = !led2;
}

void readPC() {
  // Note: you need to actually read from the serial to clear the RX interrupt
  //char _buffer[128];
  string holder;
  cJSON *json;
  // parameters list
  // factor: scale of 3V
  // ccles: number of periods to run
  int period;
  int cycles;
  double factor;

  char temp;
  while(temp != '\n') {
    temp = pc.getc();
    holder += temp;
  }
  json = cJSON_Parse(holder.c_str());
  if (!json) {
    printf("Error before: [%s]\n", cJSON_GetErrorPtr());
  } else {
    period = cJSON_GetObjectItem(json, "period")->valueint;
    cycles = cJSON_GetObjectItem(json, "cycles")->valueint;
    factor = cJSON_GetObjectItem(json, "factor")->valuedouble;
    cJSON_Delete(json);
  }

  printf("%s\n", holder.c_str());
  printf("period is %d ms\n", period);
  led1.period_ms(period);
  led1.write(0.5f);
  sawTooth.setWave(factor);
  sawTooth.waveOut(cycles);
  //led1 = !led1;
}



int main() {
  led2 = 1;
  pc.attach(&readPC);
  flipper.attach(&flip, 1); // the address of the function to be attached (flip) and the interval (2 seconds)
  //flipper2.attach(&flip2, 1);
  // spin in a main loop. flipper will interrupt it to call flip
  //sawTooth.waveOut(1);
  while(1) {
    //led3.flash(1);
    //led4.flash(3);
    //pc.printf("testing\n");
    //wait(1.0f);
  }
}
