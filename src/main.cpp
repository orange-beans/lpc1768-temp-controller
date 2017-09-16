#include "mbed.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <SawTooth.h>
#include <PID.h>
#include <WS2812.h>
#include <PixelArray.h>
#include <Adafruit_ADS1015.h>

#define WS2812_BUF 6
#define NUM_COLORS 6
#define NUM_LEDS_PER_COLOR 10

PixelArray px(WS2812_BUF);

// NOTE: timer is critical for different platforms:
// K64F, KL46Z: 0, 5, 5, 0
// LPC1768: 3, 11, 10, 11
// NUCLEO_F401RE: 3, 12, 9, 12
// NUCELO_F746ZG: 32, 105, 70, 123

int slider_position = 0;
double setPointA = 110, setPointB = 110, kc_A, ti_A, td_A, kc_B, ti_B, td_B;
double temperatureA, temperatureB, counterA = 0, counterB = 0;
bool C_FLEX_READY = false, A_FLEX_READY = false, C_FLEX_DONE = false, A_FLEX_DONE = false, ERROR_FLAG = false;

//I2C i2c(p28, p27);
Serial pc(USBTX, USBRX, 115200);
Serial dev(p28, p27, 115200);
//Adafruit_ADS1015 ads(&i2c);

DigitalIn  pos_1(p13);
DigitalIn  pos_2(p14);
DigitalOut red_led(p5);
DigitalOut blue_led(p6);
DigitalOut green_led(p7);

//****************************************************************************/
// Defines Preset conditions
//****************************************************************************/
#define C_FLEX 1
#define A_FLEX 2
#define C_FLEX_SETPOINT 105
#define C_FLEX_TIME 118
#define A_FLEX_SETPOINT 115
#define A_FLEX_TIME 236

#define M_FLEX 255

//****************************************************************************/
// Defines PID parameters
//****************************************************************************/
#define SAMPLES 5
#define RATE    0.1
#define Kc_A    0.65
#define Ti_A    0.001
#define Td_A    0.0
#define Kc_B    0.65
#define Ti_B    0.001
#define Td_B    0.0
// #define Kc_B    0.65
// #define Ti_B    0.001
// #define Td_B    0.0
#define HIGH_FACTOR 4
#define LOW_FACTOR 2

PID controllerA(Kc_A, Ti_A, Td_A, RATE);
PID controllerB(Kc_B, Ti_B, Td_B, RATE);
//ADC pins
AnalogIn tempReadA(p15);
AnalogIn tempReadB(p16);

//PWM pins
PwmOut heaterA(p21);
PwmOut heaterB(p22);

// Buzzer pin
PwmOut buzzer(p26);

// Interface pins
InterruptIn limitSwitch(p23);

// Ti_Amers
Ticker isr;

// DAC pin18
//AnalogOut aout(p18);

//Ti_Acker flipper;
//Ti_Acker flipper2;
//PwmOut led1(LED1);
//DigitalOut led1(LED1);
//DigitalOut led2(LED2);
//DigitalOut led3(LED3);
//DigitalOut led4(LED4);

SawTooth sawTooth(p18, 0.5);
Flasher led3(LED3);
Flasher led4(LED4, 2);

void offLight() {
  red_led = 0;
  blue_led = 0;
  green_led = 0;
}

void onRedLight() {
  offLight();
  red_led = 1;
}

void onBlueLight() {
  offLight();
  blue_led = 1;
}

void onGreenLight() {
  offLight();
  green_led = 1;
}

void onOrangeLight() {
  offLight();
  red_led = 1;
  green_led = 1;
}

void toggleGreenLight() {
  blue_led = 0;
  green_led = !green_led;
}

void toggleBlueLight() {
  green_led = 0;
  blue_led = !blue_led;
}

void onAlarm() {
  buzzer.period_ms(100);
  buzzer.write(0.5f);
}

void offAlarm() {
  buzzer.write(0);
}

void onHeaters(int type) {
  switch(type) {
    case C_FLEX:
      setPointA = C_FLEX_SETPOINT;
      setPointB = C_FLEX_SETPOINT;
      controllerA.setSetPoint(setPointA);
      controllerB.setSetPoint(setPointB);
      break;
    case A_FLEX:
      setPointA = A_FLEX_SETPOINT;
      setPointB = A_FLEX_SETPOINT;
      controllerA.setSetPoint(setPointA);
      controllerB.setSetPoint(setPointB);
      break;
    case M_FLEX:
      controllerA.setSetPoint(setPointA);
      controllerB.setSetPoint(setPointB);
      break;
    default:
      break;
  }
}

void offHeaters() {
  controllerA.setSetPoint(0);
  controllerB.setSetPoint(0);
}

bool areHeatersReady(int type) {
  if (type == C_FLEX) return abs(temperatureA - setPointA) <= 2.5 && abs(temperatureB - setPointA) <= 2.5;
  if (type == A_FLEX) return abs(temperatureA - setPointB) <= 2.5 && abs(temperatureB - setPointB) <= 2.5;
}

void startProcess() {
  printf("Limit Switch Triggered\n");
  dev.printf("Limit Switch Triggered\n");
  counterA = 0;
  counterB = 0;
}

void resetProcess() {
  printf("Reset Triggered\n");
  offAlarm();
  if ( (C_FLEX_READY == true && C_FLEX_DONE == false) || (A_FLEX_READY == true && A_FLEX_DONE == false)) ERROR_FLAG = true;

  C_FLEX_READY = false;
  C_FLEX_DONE = false;
  A_FLEX_READY = false;
  A_FLEX_DONE = false;
}

void isrProcess() {
  //printf("ISR Triggered\n");
  printf("Slider Pos: %d\n", pos_1 + (pos_2 << 1));

  slider_position = pos_1 + (pos_2 << 1);

  counterA += 1;
  counterB += 1;

  // TODO: check status, and update LED
  switch (slider_position) {
    // if slider switch off,
    // Standby mode, LED off, setPoint 20
    case 3:
      // TEMP REMOVE
      // offLight();
      // offHeaters();
      // ERROR_FLAG = false;
      // C_FLEX_READY = false;
      // C_FLEX_DONE = false;
      // A_FLEX_READY = false;
      // A_FLEX_DONE = false;
      onHeaters(M_FLEX);
      break;

    case 2:
      // if limitSwitch pos1 or pos2, and temperature !== setPoint, and Limit Switch none-trigger
      // heating mode, LED orange blink
      // if limitSwitch pos1 or pos2, and temperature === setPoint, and Limit Switch none-trigger
      // ready mode, LED green
      //
      if (limitSwitch == 1) {
        onHeaters(C_FLEX);
        if (areHeatersReady(C_FLEX) == false && C_FLEX_READY == false) {
          toggleGreenLight();
          C_FLEX_READY = false;
        }
        else {
          onGreenLight();
          C_FLEX_READY = true;
        }
      }

      else {
        offHeaters();
        if (C_FLEX_READY == false) {
          ERROR_FLAG = true;
        }
        else {
          toggleBlueLight();
          if (counterA >= C_FLEX_TIME) {
            onBlueLight();
            onAlarm();
            C_FLEX_DONE = true;
          }
        }
      }
      break;

    case 1:
      if (limitSwitch == 1) {
        onHeaters(A_FLEX);
        if (areHeatersReady(A_FLEX) == false && A_FLEX_READY == false) {
          toggleGreenLight();
          A_FLEX_READY = false;
        }
        else {
          onGreenLight();
          A_FLEX_READY = true;
        }
      }

      else {
        offHeaters();
        if (A_FLEX_READY == false) {
          ERROR_FLAG = true;
        }
        else {
          toggleBlueLight();
          if (counterA >= A_FLEX_TIME) {
            onBlueLight();
            onAlarm();
            A_FLEX_DONE = true;
          }
        }
      }
      break;

    default:
      offLight();
      break;
  }
}

void readPC() {
  // Note: you need to actually read from the serial to clear the RX interrupt
  // Example command:
  // {"setPointA":20, "setPointB":45, "kc":0.08, "ti":0.005, "td":0.0}
  string holder;
  cJSON *json;
  // parameters list

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
    kc_A = cJSON_GetObjectItem(json, "kc_A")->valuedouble;
    ti_A = cJSON_GetObjectItem(json, "ti_A")->valuedouble;
    td_A = cJSON_GetObjectItem(json, "td_A")->valuedouble;
    kc_B = cJSON_GetObjectItem(json, "kc_B")->valuedouble;
    ti_B = cJSON_GetObjectItem(json, "ti_B")->valuedouble;
    td_B = cJSON_GetObjectItem(json, "td_B")->valuedouble;
    cJSON_Delete(json);
  }

  controllerA.setSetPoint(setPointA);
  controllerA.setTunings(kc_A, ti_A, td_A);
  controllerB.setSetPoint(setPointB);
  controllerB.setTunings(kc_B, ti_B, td_B);
  printf("setPoints: %3.1f'C %f3.1'C\n", setPointA, setPointB);
  printf("%s\n", holder.c_str());
}

void readDev() {
  // Disable the ISR during handling
  dev.attach(0);
  dev.printf("Before\n");
  // Note: you need to actually read from the serial to clear the RX interrupt
  // Example command:
  // {"setPointA":20, "setPointB":45, "kc_A":0.08, "ti_A":0.005, "td_A":0.0, "kc_B":0.08, "ti_B":0.005, "td_B":0.0}
  string holder;
  cJSON *json;
  // parameters list

  char temp;
  while(temp != '\n') {
    temp = dev.getc();
    holder += temp;
  }
  if (holder.length() < 10) return;

  json = cJSON_Parse(holder.c_str());
  if (!json) {
    dev.printf("Error before: [%s]\n", cJSON_GetErrorPtr());
  } else {
    setPointA = cJSON_GetObjectItem(json, "setPointA")->valuedouble;
    setPointB = cJSON_GetObjectItem(json, "setPointB")->valuedouble;
    kc_A = cJSON_GetObjectItem(json, "kc_A")->valuedouble;
    ti_A = cJSON_GetObjectItem(json, "ti_A")->valuedouble;
    td_A = cJSON_GetObjectItem(json, "td_A")->valuedouble;
    kc_B = cJSON_GetObjectItem(json, "kc_B")->valuedouble;
    ti_B = cJSON_GetObjectItem(json, "ti_B")->valuedouble;
    td_B = cJSON_GetObjectItem(json, "td_B")->valuedouble;
    cJSON_Delete(json);
  }

  controllerA.setSetPoint(setPointA);
  controllerA.setTunings(kc_A, ti_A, td_A);
  controllerB.setSetPoint(setPointB);
  controllerB.setTunings(kc_B, ti_B, td_B);
  dev.printf("setPoints: %3.1f'C %f3.1'C\n", setPointA, setPointB);
  dev.printf("%s\n", holder.c_str());

  dev.attach(&readDev);
}

// Function to convert ADC reading to actual temp reading
double theta[3] = {1050.7, -4826, 5481.5};
double readRTD(double x) {
  return theta[0] + x*theta[1] + x*x*theta[2];
}

int main() {
  double tempA, tempB, outA, outB;
  double read_bufferA[SAMPLES] = {25};
  double read_bufferB[5] = {25};
  double sumA = 0, sumB = 0;
  long int reading = 0;

  // Attach ISR
  //ads.setGain(GAIN_TWO);
  pc.attach(&readPC);
  dev.attach(&readDev);
  isr.attach(&isrProcess, 0.25);
  limitSwitch.fall(&startProcess);
  limitSwitch.rise(&resetProcess);

  heaterA.period_ms(50);
  heaterB.period_ms(50);

  // Init PIC controllers
  controllerA.setInputLimits(0.0, 350.0);
  controllerA.setOutputLimits(0.0, 1.0);
  controllerA.setSetPoint(20);
  //controllerA.setBias(0.0);
  controllerA.setMode(1);

  controllerB.setInputLimits(0.0, 350.0);
  controllerB.setOutputLimits(0.0, 1.0);
  controllerB.setSetPoint(20);
  controllerA.setBias(0);
  controllerB.setMode(MANUAL_MODE);

  offLight();

  while(1) {

    // TODO: put the following in ISR
    // use while loop to check the interface status

    // Limit switch onChange, set setPoint
    //
    // Modify LED color inside ISR

    sumA = 0;
    sumB = 0;
    // Moving average
    for (int i=SAMPLES - 1; i>0; i--) {
      read_bufferA[i] = read_bufferA[i-1];
      read_bufferB[i] = read_bufferB[i-1];
    }

    read_bufferA[0] = readRTD(tempReadA.read());
    read_bufferB[0] = readRTD(tempReadB.read());

    for (int i=0; i<SAMPLES; i++) {
      sumA += read_bufferA[i];
      sumB += read_bufferB[i];
    }

    tempA = sumA/SAMPLES;
    tempB = sumB/SAMPLES;

    // print the temperatures
    // Read 10 times then average
    for (int i=0; i<10; i++) {
      //sumA += tempReadA.read();
      //sumB += tempReadB.read();

    }
    //tempA = readRTD(sumA/10);
    //tempB = readRTD(sumB/10);

    temperatureA = tempA;
    temperatureB = tempB;

    //printf("Tube Sealer Temperature A: %3.4f'C\n", temp*3.3);
    //printf("normalized: 0x%04X \n", tempReadA.read_u16());
    controllerA.setProcessValue(tempA);
    controllerB.setProcessValue(tempB);
    outA = controllerA.compute();
    outB = controllerB.compute();

    if(tempA >= setPointA) {
      outA = outA/HIGH_FACTOR;
    }

    if((setPointA - tempA) <= 0.6) {
      outA = outA/LOW_FACTOR;
    }

    if(tempB >= setPointB) {
      outB = outB/HIGH_FACTOR;
    }

    if((setPointB - tempB) <= 0.6) {
      outB = outB/LOW_FACTOR;
    }

    // Update Heaters PWM output
    heaterA.write(outA);
    heaterB.write(outB);

    // Try ADS1015

    //reading = ads.readADC_SingleEnded(0);
    //reading = ads.readADC_Differential_2_3();
    //printf("reading: %d\r\n", reading); // print reading
    printf("HeaterA: Temp: %3.1f 'C, PWM: %3.3f %%; HeaterB: Temp: %3.1f 'C, PWM: %3.3f %%;\n", tempA, outA*100, tempB, outB*100);
    dev.printf("HeaterA: Temp: %3.1f 'C, PWM: %3.3f %%; HeaterB: Temp: %3.1f 'C, PWM: %3.3f %%;\n", tempA, outA*100, tempB, outB*100);
    //printf("Compute PWM A: %3.3f; B: %3.3f\n", outA, outB);
    //printf("Tube Sealer Temperature B: %3.1f'C\n", readRTD(tempB));
    wait(RATE);

    if (ERROR_FLAG == true) {
      __disable_irq();
      onAlarm();
      offHeaters();
      onRedLight();
      while(slider_position != 3) {
        slider_position = pos_1 + (pos_2 << 1);
      };
      ERROR_FLAG = false;
      offAlarm();
      __enable_irq();
    }

    //onAlarm();
  }
}
