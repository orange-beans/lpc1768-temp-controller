#include "mbed.h"
#include "mbed_events.h"
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
AnalogIn tempReadA(p15);
AnalogIn tempReadB(p16);

//PWM pins
PwmOut heaterA(p21);
PwmOut heaterB(p22);

// Setup Serial Ports
Serial pc(USBTX, USBRX, 115200);
Serial dev(p28, p27, 115200);

//****** Define Threads ******//
// Define threads
Thread realtimeThread(osPriorityRealtime, OS_STACK_SIZE, NULL, NULL);
Thread operateThread(osPriorityHigh, OS_STACK_SIZE, NULL, NULL);
Thread commandThread(osPriorityNormal, OS_STACK_SIZE, NULL, NULL);

// Define threads functions
void RealtimeHandle();
void LEDThread(void *param);
void RealtimeTick();

//****** Define Events ******//
EventFlags event;


//****** Define Queues ******//
//EventQueue queue(32 * EVENTS_EVENT_SIZE);

//****** Main ******//
int main() {

  // Create a queue with the default size
  EventQueue queue;
  //EventQueue queue;
  queue.call_in(2000, printf, "called in 2 seconds\n");
  queue.call_every(1000, RealtimeTick);
  //queue.call_every(1000, blink, "called every 1 seconds\n\r");

  //thread.start(LEDThread);
  realtimeThread.start(RealtimeHandle);
  commandThread.start(callback(LEDThread, &led1));

  // events are executed by the dispatch method
  queue.dispatch();

  while(true) {}
}


//****** Threads Callbacks ******//
void RealtimeHandle() {
  while(true) {
    event.wait_all(0x01);

    printf("Start executing Realtime Staffs!\r\n");
  }
}

void LEDThread(void *param) {
  while (true) {
    led2 = !led2;
    //led1 = !led1;
    Thread::wait(1000);
  }
}

void RealtimeTick() {
  led1 = !led1;
  event.set(0x01);
}
