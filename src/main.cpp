#include "mbed.h"
#include "mbed_events.h"
#include "rtos.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <SawTooth.h>
#include <PID.h>
#include <WS2812.h>
#include <PixelArray.h>
#include <Adafruit_ADS1015.h>

// Define Function Pins
DigitalOut led1(LED1);
DigitalOut led2(LED2);

// Define threads
Thread thread(osPriorityNormal, OS_STACK_SIZE, NULL, NULL);

//EventQueue queue(32 * EVENTS_EVENT_SIZE);

void LEDThread() {
  while (true) {
    led2 = !led2;
    //led1 = !led1;
    Thread::wait(1000);
  }
}

void blink() {
  led1 = !led1;
}

int main() {

  // Create a queue with the default size
  EventQueue queue;
  //EventQueue queue;
  queue.call_every(1000, blink);
  //queue.call_every(1000, blink, "called every 1 seconds\n\r");

  thread.start(LEDThread);

  while(true) {}
}
