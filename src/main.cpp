#include "mbed.h"
#include "mbed_events.h"
#include "main.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <SawTooth.h>
#include <PID.h>
#include <WS2812.h>
#include <PixelArray.h>
#include <Adafruit_ADS1015.h>

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
  int cmdStr;
} cmd_t;

Mail<mail_t, 4> mail_box;
Mail<cmd_t, 4> cmd_box;

//****** Define Events ******//
EventFlags event;

//****** Define Queues ******//
//EventQueue queue(32 * EVENTS_EVENT_SIZE);

//****** Main ******//
int main() {
  // ISR handlers
  pc.attach(&commandISR);

  // Create a queue with the default size
  EventQueue queue;
  //queue.call_in(2000, printf, "called in 2 seconds\n");
  queue.call_every(INTERVAL_500MS, realtimeTick);
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
  double temperature = 0;
  double pwm = 0;
  mail_t *sent_mail;

  while(true) {
    // 1.Wait until timer tick
    event.wait_all(REALTIME_TICK_S);

    // 2.Read temperature
    temperature = analogInA.read();

    // 3.Calculate PWM
    pwm = 10;
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
  while(true) {
    // Wait for mail to be avaliable;
    evt = mail_box.get();
    // Read mail
    if (evt.status = osEventMail) {
      received_mail = (mail_t*)evt.value.p;
      printf("analog read is: %3.1f\r\n", received_mail->temperature);
      printf("pwm setting is: %3.1f\r\n", received_mail->pwm);
      // Free memory
      mail_box.free(received_mail);
    }

    led2 = !led2;
    //Thread::wait(1000);
  }
}

void commandHandle() {
  osEvent evt;
  cmd_t *received_cmd;
  //string cmdStr;
  int cmd;
  while(true) {
    //event.wait_all(COMMAND_S);

    evt = cmd_box.get();
    if (evt.status = osEventMail) {
      received_cmd = (cmd_t*)evt.value.p;
      cmd = received_cmd->cmdStr;
      cmd_box.free(received_cmd);

      printf("%d\n", cmd);
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

  printf("%s\n", holder.c_str());

  cmd_t *sent_cmd = cmd_box.alloc();
  sent_cmd->cmdStr = 666;
  cmd_box.put(sent_cmd);
}
