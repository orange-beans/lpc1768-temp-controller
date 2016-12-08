#include "mbed.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <SawTooth.h>
#include <PID.h>

Serial pc(USBTX, USBRX);

//****************************************************************************/
// Defines PID parameters
//****************************************************************************/
#define RATE  0.5
#define Kc    1.0
#define Ti    0.001
#define Td    0.0

PID controllerA(Kc, Ti, Td, RATE);
PID controllerB(Kc, Ti, Td, RATE);
//ADC pins
AnalogIn tempReadA(p15);
AnalogIn tempReadB(p16);
// DAC pin18
//AnalogOut aout(p18);

//Ticker flipper;
//Ticker flipper2;
//PwmOut led1(LED1);
//DigitalOut led1(LED1);
//DigitalOut led2(LED2);
//DigitalOut led3(LED3);
//DigitalOut led4(LED4);

SawTooth sawTooth(p18, 0.5);
Flasher led3(LED3);
Flasher led4(LED4, 2);

void readPC() {
  // Note: you need to actually read from the serial to clear the RX interrupt
  string holder;
  cJSON *json;
  // parameters list
  double setPointA, setPointB, kc, ti, td;

  char temp;
  while(temp != '\n') {
    temp = pc.getc();
    holder += temp;
  }
  if (holder.length() < 10) return;

  json = cJSON_Parse(holder.c_str());
  if (!json) {
    printf("Error before: [%s]\n", cJSON_GetErrorPtr());
  } else {
    setPointA = cJSON_GetObjectItem(json, "setPointA")->valuedouble;
    setPointB = cJSON_GetObjectItem(json, "setPointB")->valuedouble;
    kc = cJSON_GetObjectItem(json, "kc")->valuedouble;
    ti = cJSON_GetObjectItem(json, "ti")->valuedouble;
    td = cJSON_GetObjectItem(json, "td")->valuedouble;
    cJSON_Delete(json);
  }

  controllerA.setSetPoint(setPointA);
  controllerA.setTunings(kc, ti, td);
  controllerB.setSetPoint(setPointB);
  controllerB.setTunings(kc, ti, td);
  printf("setPoints: %3.1f'C %f3.1'C\n", setPointA, setPointB);
  printf("%s\n", holder.c_str());
}

// Function to convert ADC reading to actual temp reading
double theta[3] = {1050.7, -4826, 5481.5};
double readRTD(double x) {
  return theta[0] + x*theta[1] + x*x*theta[2];
}

int main() {
  double tempA, tempB;
  //double read_bufferA[10] = {0,0,0,0,0,0,0,0,0,0};
  //double read_bufferB[10] = {0,0,0,0,0,0,0,0,0,0};
  double sumA = 0, sumB = 0;
  pc.attach(&readPC);

  // Init PIC controllers
  controllerA.setInputLimits(0.0, 350.0);
  controllerA.setOutputLimits(0.0, 1.0);
  controllerA.setSetPoint(20);
  //controllerA.setBias(0.0);
  controllerA.setMode(1);

  while(1) {
    // print the temperatures
    // Read 10 times then average
    sumA = 0;
    sumB = 0;
    for (int i=0; i<10; i++) {
      sumA += tempReadA.read();
      sumB += tempReadB.read();
    }
    tempA = readRTD(sumA/10);
    tempB = readRTD(sumB/10);
    //printf("Tube Sealer Temperature A: %3.4f'C\n", temp*3.3);
    //printf("normalized: 0x%04X \n", tempReadA.read_u16());
    controllerA.setProcessValue(tempA);
    printf("Tube Sealer Temperature A: %3.1f'C\n", tempA);
    printf("Compute PWM %3.3f\n", controllerA.compute());
    //printf("Tube Sealer Temperature B: %3.1f'C\n", readRTD(tempB));
    wait(RATE);
  }
}
