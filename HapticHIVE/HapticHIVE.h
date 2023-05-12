#ifndef HapticHIVE_h
#define HapticHIVE_h
#include "Arduino.h"
#include <Wire.h>
#include <xtensa/core-macros.h>
#include <WiFi.h>
#include <ESPAsyncWebSrv.h>
#include <esp32-hal-log.h>
#include <ArduinoJson.h>
#include <esp_wifi.h>
//#include <ESPAsyncWebServer.h>

#define SERIAL_BAUDRATE 115200
// THREAD
#define SEMAPHORE_TICKS_WAIT portMAX_DELAY

// DRV
#define STACK_SIZE 10000
#define DRV2605_ADDRESS 0x5A    //DRV2605 slave address
#define DRV2605_MODEREG 0x01

// I2C
#define I2C_FREQUENCY 400000
#define SCL0_PIN 23
#define SDA0_PIN 22
#define SCL1_PIN 33
#define SDA1_PIN 32

// PWM
#define PWM_FREQUENCY 11719
#define PWM_RESOLUTION_BITS 8
#define PWM0_PIN 13
#define PWM1_PIN 14
#define PWM2_PIN 18
#define PWM3_PIN 19

// PWM_CHANNEL
#define PWM0_CHANNEL 0
#define PWM1_CHANNEL 1
#define PWM2_CHANNEL 2
#define PWM3_CHANNEL 3

// MUX
#define MUX0_PIN 21
#define MUX1_PIN 21
#define MUX2_PIN 4
#define MUX3_PIN 4

// PWM_CHANNEL
#define MUX0_CHANNEL 0
#define MUX1_CHANNEL 1
#define MUX2_CHANNEL 0
#define MUX3_CHANNEL 1
#define MUX_NANOSECONDS_DELAY 200


struct HitAndPulseParameters{
   double intensity;
   double milliseconds;
};

struct VibrateParameters{
   double frequency;
   double intensity;
   double milliseconds;
   double duration;
   double dutycycle;
};

struct TacHammer{
   String name;
   int mux_pin;
   int mux_channel;
   int pwm_pin;
   int pwm_channel;
   SemaphoreHandle_t* twoWireSemaphore;
   TwoWire* twoWire;
   HitAndPulseParameters* hitAndPulseParams;
   VibrateParameters* vibrateParams;
   TaskHandle_t taskHandle;
   eTaskState taskStatus;
   bool active;
   bool stop;
};

extern TacHammer* M0;
extern TacHammer* M1;
extern TacHammer* M2;
extern TacHammer* M3;


// Functions
 void setupSmartWatch(
    void (* heartRateCallback)(unsigned int, double),
    void (* accelerometerCallback)(unsigned int, double, double, double),
    void (* gyroscopeCallback)(unsigned int, double, double, double),
    void (* lightCallback)(unsigned int, double),
    void (* stepCounterCallback)(unsigned int, double)
 );
 void requestSmartWatch(unsigned int delay);
 void setupTacHammers();
 void hit(TacHammer* tacHammer, double intensity, double milliseconds);
 void pulse(TacHammer* tacHammer, double intensity, double milliseconds);
 void singlePulse(TacHammer* tacHammer, double intensity, double milliseconds);
 void vibrate(TacHammer* tacHammer, double frequency, double intensity, double duration, int dutycycle);
 void pause(TacHammer* tacHammer, double milliseconds);
 bool isFree(TacHammer* tacHammer);
 bool isBusy(TacHammer* tacHammer);
 void waitUntilFree(TacHammer* tacHammer);
 void stop(TacHammer* tacHammer);

// Utilities
 String getMacAddress();
 void setupTacHammer(TacHammer* tacHammer);
 void selectMux(int mux_pin, int mux_channel);
 void standbyOnB(TacHammer* tacHammer) ;
 void standbyOffB(TacHammer* tacHammer);
 void usdelay(double time);
 void pauseTask(void *pvParameter);
 void pulseTask(void *pvParameter);
 void singlePulseTask(void *pvParameter);
 void hitTask(void *pvParameter);
 void vibratePulse(TacHammer* tacHammer, double intensity, int hitduration);
 void vibrateTask(void *pvParameter);

#endif