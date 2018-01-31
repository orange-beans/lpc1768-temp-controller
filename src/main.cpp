#include "mbed.h"
#include "mbed_events.h"
#include "main.h"
#include <string>
#include <cJSON.h>
#include <Flasher.h>
#include <PID.h>
// #include <MAX31855.h>
// #include <Adafruit_SSD1306.h>
//#include <WS2812.h>
//#include <SawTooth.h>
//#include <PixelArray.h>
//#include <Adafruit_ADS1015.h>

//****** Define User Variables ******//
typedef struct {
  float setpoint;
  float kc;
  float ti;
  float td;
} heater_setting_t;

heater_setting_t heater_setting = { 25, 0.08, 0.01, 0.0 };

unsigned char BACKBONE_ADDRESS = 0x0000;
unsigned char COUNT_LIMIT = 1000/REALTIME_INTERVAL;

//****** Define Function Pins ******//
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut statusLED(p26);

// ID reading
DigitalIn ID_0(p8);
DigitalIn ID_1(p7);
DigitalIn ID_2(p6);
DigitalIn ID_3(p5);

//ADC pins
AnalogIn analogInA(p15);
AnalogIn analogInB(p16);

//PWM pins
PwmOut heater(p21);
PwmOut cooler(p22);

// Setup Serial Ports
Serial pc(USBTX, USBRX, 115200);
//Serial dev(p28, p27, 115200);

// Setup SPI
// SPI thermoSPI(p5, p6, p7);
// max31855 maxThermo(thermoSPI, p8);

// Setup I2C & OLED
// I2CPreInit gI2C(p9, p10);
// Adafruit_SSD1306_I2c gOled(gI2C, p26);

// Setup PID Controller
PID controller(heater_setting.kc, heater_setting.ti, heater_setting.td, (float)REALTIME_INTERVAL/1000);

//****** Define Threads ******//
// Define threads
Thread realtimeThread(osPriorityRealtime, MEDIUM_STACK_SIZE, NULL, NULL);
//Thread operateThread(osPriorityAboveNormal, MEDIUM_STACK_SIZE, NULL, NULL);
Thread displayTread(osPriorityBelowNormal, MEDIUM_STACK_SIZE, NULL, NULL);
Thread commandThread(osPriorityNormal, MEDIUM_STACK_SIZE, NULL, NULL);

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
  float   output;
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

//****** Local Helpers ******//
// TODO: put into lib later
double theta[3] = {1050.7, -4826, 5481.5};
double readRTD(double x) {
  return theta[0] + x*theta[1] + x*x*theta[2];
}

//****** System Init ******//
void initSystem() {
  // Read Address
  BACKBONE_ADDRESS = ID_3.read() * 8 + ID_2.read() * 4 + ID_1.read() * 2 + ID_0.read();
  printf("Device Address is 0x%04x\r\n", BACKBONE_ADDRESS);
  // MAX31855 init
  //maxThermo.initialise();
  // Heater init
  heater.period_ms(50);
  cooler.period_ms(50);
  // PID init
  controller.setInputLimits(0.0, 120.0);
  controller.setOutputLimits(0.0, 1.0);
  controller.setSetPoint(20);
  //controllerA.setBias(0.0);
  controller.setMode(1);

  //gOled.clearDisplay();
  //gOled.printf("%ux%u OLED Display\r\n", gOled.width(), gOled.height());
  //gOled.display();
  //gOled.invertDisplay(true);
}

//****** Main ******//
int main() {
  // Init System
  initSystem();

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
  float output = 0, temperature = 0;
  unsigned char counter = 0;
  mail_t *sent_mail;

  while(true) {
    // 1.Wait until timer tick
    event.wait_all(REALTIME_TICK_S);

    // 2.Read temperature
    // TODO: add moving average for reading
    temperature = readRTD(analogInA.read());
    // if (maxThermo.ready() ==1) {
    //   printf("MAX31855 Ready!\r\n");
    //   temperature = maxThermo.read_temp();
    // }

    // 3.Calculate PWM
    controller.setProcessValue(temperature);
    output = controller.compute();

    // 4.Cooler control
    // TODO: write as a dedicated function
    if (temperature > heater_setting.setpoint) {
      heater.write(0);
      cooler.write(1.0);
    } else {
      heater.write(output);
      cooler.write(0);
    }

    // 5.Send mail after every COUNT_LIMIT counts
    counter += 1;
    if (counter >= COUNT_LIMIT) {
      counter = 0;
      sent_mail = mail_box.alloc();
      sent_mail->temperature = temperature;
      sent_mail->output = output;
      mail_box.put(sent_mail);
    }
  }
}

void displayHandle() {
  osEvent evt;
  mail_t *received_mail;
  float temperature, output;
  while(true) {
    // Wait for mail to be avaliable;
    evt = mail_box.get();
    // Read mail
    if (evt.status == osEventMail) {
      received_mail = (mail_t*)evt.value.p;
      temperature = received_mail->temperature;
      output = received_mail->output;
      // Free memory
      // NOTE: need to process data before free, otherwise data may get corrupted
      mail_box.free(received_mail);
      // printf("0x%04x/temperature read is: %3.1f\r\n", BACKBONE_ADDRESS, temperature);
      // printf("0x%04x/output setting is: %3.1f%%\r\n", BACKBONE_ADDRESS, output*100);
      // printf("0x%04x/heater setpoint is: %3.1f\r\n", BACKBONE_ADDRESS, heater_setting.setpoint);

      printf("{\"address\":0x%04x, \"setpoint\":%3.1f, \"temperature\":%3.1f, \"output\":%3.1f%%}\r\n", BACKBONE_ADDRESS, heater_setting.setpoint, temperature, output*100);

      // gOled.clearDisplay();
      // gOled.setTextCursor(0,0);
      // gOled.printf("setpoint is: %3.1f\r\n", heater_setting.setpoint);
      // gOled.printf("temperature: %3.2f'C\r\n", temperature);
      // gOled.printf("powerOutput: %3.1f%%\r\n", output*100);
      // gOled.display();
    }

    led2 = !led2;
  }
}

void commandHandle() {
  osEvent evt;
  cmd_t *received_cmd;
  cJSON *json;
  unsigned char cmd_address = 0xffff;

  while(true) {
    //event.wait_all(COMMAND_S);
    // reset cmd_address everytime
    cmd_address = 0xffff;

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
        cJSON *kc = cJSON_GetObjectItem(json, "kc");
        cJSON *ti = cJSON_GetObjectItem(json, "ti");
        cJSON *td = cJSON_GetObjectItem(json, "td");
        cJSON *address = cJSON_GetObjectItem(json, "address");

        if (cJSON_IsNumber(address)) {
          cmd_address = address->valueint;
        }

        if (cmd_address == BACKBONE_ADDRESS) {
          if (cJSON_IsNumber(setpoint)) {
            heater_setting.setpoint = setpoint->valuedouble;
          }

          if (cJSON_IsNumber(kc)) {
            heater_setting.kc = kc->valuedouble;
          }

          if (cJSON_IsNumber(ti)) {
            heater_setting.ti = ti->valuedouble;
          }

          if (cJSON_IsNumber(td)) {
            heater_setting.td = td->valuedouble;
          }
          // Only execute if cmd address match

          // Modify setpoint and tuning
          controller.setSetPoint(heater_setting.setpoint);
          controller.setTunings(heater_setting.kc, heater_setting.ti, heater_setting.td);
        }

        // Must delete json object
        cJSON_Delete(json);
      }
    }
  }
}

void realtimeTick() {
  led1 = !led1;
  statusLED = !statusLED;
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
