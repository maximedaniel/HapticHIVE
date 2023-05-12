#include "HapticHIVE.h"
#define SERIAL_BAUDRATE 115200
#define SMARTWATCH_REQUEST_DELAY 500 // decrease this delay (ms) to increase number of physiological readings per second

// TacHammer objects
extern TacHammer* M0;
extern TacHammer* M1;
extern TacHammer* M2;
extern TacHammer* M3;

// User variables
double i = 0;
double j = 0;
double k = 10;
int nextAnimationM0 = 0;
int nextAnimationM1 = 0;
int nextAnimationM2andM3 = 0;
double currentHeartRate = 0;
double respiratoryRate = 0;

/**
 * Called whenever a new heart rate reading is received from the smartwatch. 
 * The function gives the time in ms at which the measurement was taken (*time*) and the heart rate in bpm (*bpm*).
 **/
void heartRateCallback(unsigned int time, double bpm) {
  Serial.print("[");
  Serial.print(time);
  Serial.print("] heart rate:");
  Serial.println(bpm);
  currentHeartRate = bpm; // save heartrate reading
  respiratoryRate = currentHeartRate/4; // rought estimation of the respiratory rate
}

/**
 * Called whenever new accelerometer reading is received from the smartwatch.
 * The function gives the time in ms at which the measurement was taken (*time*) and the acceleration force along the X-axis (*xAccel*), Y-axis (*yAccel*), and the Z-axis (*zAccel*).
 **/
void accelerometerCallback(unsigned int time, double xAccel, double yAccel, double zAccel) {
  // Serial.print("accelerometer: [");
  // Serial.print(xAccel);
  // Serial.print(", ");
  // Serial.print(yAccel);
  // Serial.print(", ");
  // Serial.print(zAccel);
  // Serial.println("]");
}

/**
 * Called whenever new gyroscope reading is received from the smartwatch. 
 * The function gives the time in ms at which the measurement was taken (*time*) and the rate of rotation in rad/s along the X-axis (*xRot*), Y-axis (*yRot*) and the Z-axis (*zRot*).
 **/
void gyroscopeCallback(unsigned int time, double xRot, double yRot, double zRot) {
  // Serial.print("gyroscope: [");
  // Serial.print(xRot);
  // Serial.print(", ");
  // Serial.print(yRot);
  // Serial.print(", ");
  // Serial.print(zRot);
  // Serial.println("]");
}

/**
 * Called whenever a new light reading is received from the smartwatch.
 * The function gives the time in ms at which the measurement was taken (*time*) and the light measurement in lux (*lux*).
 **/
void lightCallback(unsigned int time, double lux) {
  // Serial.print("light:");
  // Serial.println(lux);
}

/**
 * Called whenever a new step count reading is received from the smartwatch. 
 * The function gives the time in ms at which the measurement was taken (*time*) and the number of steps since the smartwatch is started (*steps*).
 **/
void stepCounterCallback(unsigned int time, double steps) {
  // Serial.print("step counter:");
  // Serial.println(steps);
}


// ================================================================================
// code to run once goes here
void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setDebugOutput(true);
  setupTacHammers();
  setupSmartWatch(&heartRateCallback, &accelerometerCallback, &gyroscopeCallback, &lightCallback, &stepCounterCallback);
}

// ================================================================================
// active code goes here
void loop()
{ 
    requestSmartWatch(SMARTWATCH_REQUEST_DELAY); // requests new physiological readings from the smartwatch every SMARTWATCH_REQUEST_DELAY milliseconds
    
    // Repeatedly generate an hit with ramping up intensity using M0
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

    // repeatedly generate a single pulse with intensity proportional to heart rate using M0
    if(isFree(M1)){ // if M3 is free
      if(currentHeartRate != 0){
        switch (nextAnimationM1){ // check the next animation to run
          case 0: // animation n°1
            singlePulse(M1, 0.5, 8); // ask M1 to generate a single pulse at 50% intensity for 8ms
            nextAnimationM1 = 1; // set the next animation to run once M1 is free again
          break;
          case 1: // animation n°2
            pause(M1, 60000/currentHeartRate); // ask M1 to pause itself for a time proportional to the heart rate
            nextAnimationM1 = 0; // set the next animation to run once M1 is free again
          break;
          default:
          break;
        }
      }
    }

    // Repeatedly generate an vibrate with ramping up frequency using both M2 and M3
    if(isFree(M2) && isFree(M3)){ /// if M2 and M3 is free
      switch (nextAnimationM2andM3){ // check the next animation to run
        case 0: // ask both M2 and M3 to generate a vibration for 0.2s with an intensity of 70%, a dutycyle of 70%, and a frequency of k Hz.
          vibrate(M2, k, 0.7, 0.2, 70);
          vibrate(M3, k, 0.7, 0.2, 70);
          nextAnimationM2andM3 = 1; // set the next animation to run once M2 and M3 are free again
        break;
        case 1: // increase the frequency of the vibration animation
            k += 10;
            if (k > 210) {
              k = 10;
            }
          nextAnimationM2andM3 = 0;// restart the sequence
        break;
        default:
        break;
      }
    }
}
