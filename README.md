# dfp-hackathon
Instructions to install and use the HapticHIVE hardware during the DFP hackathon on May 19th, 2023 at UBC's Okanagan Campus.

## Installation 
1. Download & Install Arduino IDE 2.1.0
2. Install ESP32 board manager using Arduino IDE
    - Go to ‘Preferences’ under ‘File’ menu
    - Add additional url: https://espressif.github.io/arduino-esp32/package_esp32_index.json
    - Open Boards Manager from Tools > Board menu and install esp32 platform
3. Install ESPAsyncWebSrv and ArduinoJson library using Arduino IDE
    - Tools > Manage Libraries…
    - Search for “ESPAsyncWebSrv” and install latest version. If prompted to install dependencies → "Install all".
    - Search for “ArduinoJson” and install latest version. If prompted to install dependencies → "Install all".

5. Configure ESP32 board manager:
    - Tools > Boards: … > esp32 > ESP32 Dev Module
    - Tools > Core Debug Level: … > Info	
6. Download and Unzip HapticHIVE archive
7. Open HapticHIVE source code using Arduino IDE:
    - File > Open
    - navigate to the HapticHIVE directory
    - Sketch > Verify/Compile


## Documentation - Haptic Actuators

```C
void pulse(TacHammer* tacHammer, double intensity, double milliseconds)
```
Drives the hammer towards the closed end of the TacHammer. When the hammer rebounds off of the repelling magnetic array, the inaudible pulse haptic sensation is created. pulse is intended to be sequenced with subsequent pulse and hit commands and if called on its own, the hammer may travel after the rebound and strike the open end.</br>
**Example:** `pulse(M0, 0.5, 8);` asks the tacHammer M0 to generate a pulse at 50% intensity for 8 ms.

</br>

```C
void singlePulse(TacHammer* tacHammer, double intensity, double milliseconds)
```
Includes a command to pulse the hammer followed by a command to brake the hammer. This function is intended to be called on its own and should be followed by a pause command of at least 50ms before the next command is called.</br>
**Example:** `singlePulse(M1, 0.75, 6);` aks the tacHammer M1 to generate a pulse at 75% intensity for 6 ms.

</br>

```C
void hit(TacHammer* tacHammer, double intensity, double milliseconds)
```
Drives the hammer towards the open end of the TacHammer. When the hammer strikes the device housing, the audible click haptic sensation is created.</br>
**Example:** `hit(M2, 0.25, 21);` asks the tacHammer M2 to generate a hit at 25% intensity for 21ms.

</br>

```C
void vibrate(TacHammer* tacHammer, double frequency, double intensity, double duration, int dutycycle)
```
Repeatedly calls the pulse command to drive the hammer into the closed end of the TacHammer.</br>
**Example:** `vibrate(M3, 210, 0.7, 0.25, 70);` asks the tacHammer M3 to generate a vibrate for 250ms at 210Hz with 70% intensity and 70% dutycyle.

</br>

```C
void pause(TacHammer* tacHammer, double milliseconds)
```
Prevent any animation of the tacHammer for a given amount of milliseconds. </br>
**Example:** `pause(M0, 100);` pause the tacHammer M0 for 100 ms.


</br>

```C
bool isFree(TacHammer* tacHammer)
```
Returns true if the tacHammer is not busy running an animation else returns false. isFree should always be called before starting a new animation to check that the tacHammer is free.</br>
**Example:** `if(isFree(M0)) pulse(M0, 0.5, 8);` check that tacHammer M0 is free before starting a pulse animation.

## Documentation - Physiological Sensors
```C
void heartRateCallback(unsigned int time, double bpm)
```
Called whenever a new heart rate reading is received from the smartwatch. The function gives the time in ms at which the measurement was taken (*time*) and the heart rate in bpm (*bpm*).

</br>

```C
void accelerometerCallback(unsigned int time, double xAccel, double yAccel, double zAccel)
```
Called whenever new accelerometer reading is received from the smartwatch. The function gives the time in ms at which the measurement was taken (*time*) and the acceleration force along the X-axis (*xAccel*), Y-axis (*yAccel*), and the Z-axis (*zAccel*).

</br>

```C
void void gyroscopeCallback(unsigned int time, double xRot, double yRot, double zRot)
```
Called whenever new gyroscope reading is received from the smartwatch. The function gives the time in ms at which the measurement was taken (*time*) and the rate of rotation in rad/s along the X-axis (*xRot*), Y-axis (*yRot*) and the Z-axis (*zRot*).

```C
void lightCallback(unsigned int time, double lux)
```
Called whenever a new light reading is received from the smartwatch.  The function gives the time in ms at which the measurement was taken (*time*) and the light measurement in lux (*lux*).

</br>

```C
void stepCounterCallback(unsigned int time, double steps)
```
Called whenever a new step count reading is received from the smartwatch.  The function gives the time in ms at which the measurement was taken (*time*) and the number of steps since the smartwatch is started (*steps*).