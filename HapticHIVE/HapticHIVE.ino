// #ifdef CORE_DEBUG_LEVEL
// #undef CORE_DEBUG_LEVEL
// #endif

// #define CORE_DEBUG_LEVEL 4
// #define LOG_LOCAL_LEVEL CORE_DEBUG_LEVEL

#include "HapticHIVE.h"
#define SERIAL_BAUDRATE 115200

extern TacHammer* M0;
extern TacHammer* M1;
extern TacHammer* M2;
extern TacHammer* M3;
TacHammer* A;
TacHammer* B;
TacHammer* C;
TacHammer* D;

unsigned long prev_pulse_time = 0;
unsigned long animation_sequence_start_time;
double i = 0;
double j = 0;
double k = 10;
int nextAnimationM0;
int nextAnimationM1;
int nextAnimationM2andM3;


void heartRateCallback(unsigned int time, double hr) {
  Serial.print("heart rate:");
  Serial.println(hr);
}

void accelerometerCallback(unsigned int time, double xAccel, double yAccel, double zAccel) {
  Serial.print("accelerometer: [");
  Serial.print(xAccel);
  Serial.print(", ");
  Serial.print(yAccel);
  Serial.print(", ");
  Serial.print(zAccel);
  Serial.println("]");
}

void gyroscopeCallback(unsigned int time, double xRot, double yRot, double zRot) {
  Serial.print("gyroscope: [");
  Serial.print(xRot);
  Serial.print(", ");
  Serial.print(yRot);
  Serial.print(", ");
  Serial.print(zRot);
  Serial.println("]");
}

void lightCallback(unsigned int time, double l) {
  Serial.print("light:");
  Serial.println(l);
}

void stepCounterCallback(unsigned int time, double s) {
  Serial.print("step counter:");
  Serial.println(s);
}


// ================================================================================
// code to run once goes here
void setup() {
  // SERIAL
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setDebugOutput(true);
  setupTacHammers(A, B, C, D);
  setupSmartWatch(&heartRateCallback, &accelerometerCallback,  &gyroscopeCallback,  &lightCallback, &stepCounterCallback);
  nextAnimationM0 = 0;
  nextAnimationM1 = 0;
  nextAnimationM2andM3 = 0;
}


void hitRampM0(){
  if(isFree(M0)){ // if M0 is free
    switch (nextAnimationM0){ // check the animation to run
      case 0: // animation n°1
        pulse(M0, i, 6); // ask M0 to generate a pulse of intensity i for 6ms
        nextAnimationM0 = 1; // set the next animation to run once M0 is free again
      break;
      case 1: // animation n°2
        hit(M0, i, 21);  // ask M0 to generate a hit of intensity i for 20ms
        nextAnimationM0 = 2; // set the next animation to run once M0 is free again
      break;
      case 2: // animation n°3
        pause(M0, 100); // ask M0 to pause itself for 100ms
        nextAnimationM0 = 3; // set the next animation to run once M0 is free again
      break;
      case 3: // animation n°4 
          i += 0.05; // increase the hit intensity
          if (i > 1.0) { // if max hit intensity is not exceeded
            i = 0; // reset the hit intensity
          }
        nextAnimationM0 = 0; // restart the animation sequence
      break;
      default:
      break;
    }
  }
}
void singlePulseRampM1(){
  if(isFree(M1)){ // if M3 is free
    switch (nextAnimationM1){ // check the next animation to run
      case 0: // animation n°1
        singlePulse(M1, j, 8); // ask M1 to generate a single pulse of intensity i for 8ms
        nextAnimationM1 = 1; // set the next animation to run once M1 is free again
      break;
      case 1: // animation n°2
        pause(M1, 84); // ask M1 to pause itself for 84ms
        nextAnimationM1 = 2; // set the next animation to run once M1 is free again
      break;
      case 2: // animation n°3 
          j += 0.05; // increase the single pulse intensity
          if (j > 1.0) { // if max single pulse intensity is not exceeded
           j = 0; // reset the single pulse intensity 
          }
        nextAnimationM1 = 0; // restart the animation sequence
      break;
      default:
      break;
    }
  }
}

void vibrateRampM2andM3(){
  if(isFree(M2) && isFree(M3)){ /// if M2 and M3 is free
    switch (nextAnimationM2andM3){ // check the next animation to run
      case 0: // ask both M2 and M3 to generate a vibration for 0.2s with an intensity of 0.7, a dutycyle of 70%, and a frequency of k Hz.
        //Serial.print("vibrate ");
        //Serial.println(k);
        vibrate(M2, k, 0.7, 0.2, 70);
        vibrate(M3, k, 0.7, 0.2, 70);
        nextAnimationM2andM3 = 1; // set the next animation to run once M2 and M3 are free again
      break;
      case 1: // increase the frequency of the vibration animation
          if (k < 210) {
            k += 10;
          } else {
            k = 10;
          }
        nextAnimationM2andM3 = 0;// restart the sequence
      break;
      default:
      break;
    }
  }
}

// ================================================================================
// active code goes here
void loop()
{ 
    cleanupSmartWatch();
    hitRampM0();
    singlePulseRampM1();
    vibrateRampM2andM3();
}
