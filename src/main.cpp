#include "mbed.h"
#include "mbed_events.h"
#include "main.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <SawTooth.h>
#include <PID.h>
#include <WS2812.h>
#include <MAX31855.h>
//#include <PixelArray.h>
//#include <Adafruit_ADS1015.h>

//****** Define Function Pins ******//
DigitalOut led1(LED1);
DigitalOut led2(LED2);

//ADC pins
AnalogIn analogInA(p15);
AnalogIn analogInB(p16);

//PWM pins
PwmOut heaterA(p21);
PwmOut heaterB(p22);

// Setup Serial Ports
Serial pc(USBTX, USBRX, 115200);
Serial dev(p28, p27, 115200);

// Setup SPI
SPI thermoSPI(p5, p6, p7);
max31855 maxThermo(thermoSPI, p8);

//****** Define User Variables ******//
typedef struct {
  float setpoint;
  float kc;
  float ti;
  float td;
} heater_setting_t;

heater_setting_t heater_setting = { 25, 0.08, 0.01, 0.0 };

//****** Define Threads ******//
// Define threads
Thread realtimeThread(osPriorityRealtime, OS_STACK_SIZE, NULL, NULL);
//Thread operateThread(osPriorityAboveNormal, MEDIUM_STACK_SIZE, NULL, NULL);
Thread displayTread(osPriorityBelowNormal, MEDIUM_STACK_SIZE, NULL, NULL);
Thread commandThread(osPriorityNormal, OS_STACK_SIZE, NULL, NULL);

// Define threads functions
void realtimeHandle();
void commandHandle();
void displayHandle();

void realtimeTick();
//****** Define ISR ******//
void commandISR();

//****** Define Mails ******//
typedef struct {
  float   temperature;
  float   pwm;
} mail_t;

typedef struct {
  char *cmdStr;
} cmd_t;

Mail<mail_t, 4> mail_box;
Mail<cmd_t, 4> cmd_box;

//****** Define Events ******//
EventFlags event;

//****** Define Queues ******//
//EventQueue queue(32 * EVENTS_EVENT_SIZE);

//****** Main ******//
int main() {
  // MAX31855 init
  maxThermo.initialise();
  // ISR handlers
  pc.attach(&commandISR);

  // Create a queue with the default size
  EventQueue queue;
  //queue.call_in(2000, printf, "called in 2 seconds\n");
  queue.call_every(REALTIME_INTERVAL, realtimeTick);
  //queue.call_every(1000, blink, "called every 1 seconds\n\r");

  // Start Threads
  realtimeThread.start(realtimeHandle);
  //displayTread.start(callback(displayHandle, &led1));
  commandThread.start(commandHandle);
  displayTread.start(displayHandle);

  // events are executed by the dispatch method
  queue.dispatch();

  while(true) {}
}


//****** Threads Callbacks ******//
void realtimeHandle() {
  float temperature = 0;
  float pwm = 0;
  mail_t *sent_mail;

  while(true) {
    // 1.Wait until timer tick
    event.wait_all(REALTIME_TICK_S);

    // 2.Read temperature
    //temperature = analogInA.read();
    if (maxThermo.ready() ==1) {
      printf("MAX31855 Ready!\r\n");
      temperature = maxThermo.read_temp();
    }

    // 3.Calculate PWM
    pwm = 10.0;
    printf("executing Realtime Staffs!\r\n");

    // 4.Send mail
    sent_mail = mail_box.alloc();
    sent_mail->temperature = temperature;
    sent_mail->pwm = pwm;
    mail_box.put(sent_mail);
  }
}

void displayHandle() {
  osEvent evt;
  mail_t *received_mail;
  float temperature, pwm;
  while(true) {
    // Wait for mail to be avaliable;
    evt = mail_box.get();
    // Read mail
    if (evt.status == osEventMail) {
      received_mail = (mail_t*)evt.value.p;
      temperature = received_mail->temperature;
      pwm = received_mail->pwm;
      // Free memory
      // NOTE: need to process data before free, otherwise data may get corrupted
      mail_box.free(received_mail);
      printf("temperature read is: %3.2f\r\n", temperature);
      printf("pwm setting is: %3.1f\r\n", pwm);
      printf("heater setpoint is: %3.1f\r\n", heater_setting.setpoint);
    }

    led2 = !led2;
    //Thread::wait(1000);
  }
}

void commandHandle() {
  osEvent evt;
  cmd_t *received_cmd;
  cJSON *json;

  while(true) {
    //event.wait_all(COMMAND_S);

    evt = cmd_box.get();
    if (evt.status == osEventMail) {
      received_cmd = (cmd_t*)evt.value.p;
      cmd_box.free(received_cmd);
      // Process command
      json = cJSON_Parse(received_cmd->cmdStr);
      if (!json) printf("Error before: [%s]\n", cJSON_GetErrorPtr());
      else {
        printf("%s\n", received_cmd->cmdStr);
        cJSON *setpoint = cJSON_GetObjectItem(json, "setpoint");
        if (cJSON_IsNumber(setpoint)) {
          heater_setting.setpoint = setpoint->valuedouble;
          printf("setpoint change to %3.2f\r\n", heater_setting.setpoint);
        }

        // Must delete json object
        cJSON_Delete(json);
      }
    }
  }
}

void realtimeTick() {
  led1 = !led1;
  event.set(REALTIME_TICK_S);
}


//****** ISR handles ******//
void commandISR() {
  //event.set(COMMAND_S);
  string holder;
  char temp;

  while(temp != '\n') {
    temp = pc.getc();
    holder += temp;
  }

  //printf("%s\n", holder.c_str());

  cmd_t *sent_cmd = cmd_box.alloc();
  strcpy(sent_cmd->cmdStr, holder.c_str());
  cmd_box.put(sent_cmd);
}
