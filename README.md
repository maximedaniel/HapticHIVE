# dfp-hackathon

## Installation

- Install the latest Arduino IDE
- Install the latest arduino-espressif

## Walkthrough an example / first step

- Unplug battery
- Plug ESP32
- Upload Sample code
- plug battery
- turn on smartwatch

## Installation 
1. Download & Install Arduino IDE 2.1.0
2. Install ESP32 board manager using Arduino IDE
3. Install ESPAsyncWebSrv library using Arduino IDE:
    - Tools > Manage Libraries…
4. Search for “ESPAsyncWebSrv” and install latest version
4. Search for "ArduinoJson" and install latest version
5. Configure ESP32 board manager:
    - Tools > Boards: … > esp32 > ESP32 Dev Module
    - Tools > Core Debug Level: … > Info	
6. Download and Unzip HapticHIVE archive
7. Open HapticHIVE source code using Arduino IDE:
    - File > Open
    - navigate to the HapticHIVE directory
    - Sketch > Verify/Compile


- heart_rate (double)
- stationary_detect (int)
- motion_detect (int)
- step_counter (int)
- accelerometer [double, double, double]
- Gyroscope [double, double, double]
- Light (double)
- Presssure (double)
- Proximity (double)


## Documentation
```C
void pulse(TacHammer* tacHammer, double intensity, double milliseconds)
```
### **Description**
drives the hammer towards the closed end of the TacHammer. When the hammer rebounds off of the repelling magnetic array, the inaudible pulse haptic sensation is created. pulse is intended to be sequenced with subsequent pulse and hit commands and if called on its own, the hammer may travel after the rebound and strike the open end.
### **Example**
`pulse(M0, 0.5, 8)` asks the tacHammer M0 to generate a pulse at 50% intensity for 8 ms.

</br></br></br>

```C
void singlePulse(TacHammer* tacHammer, double intensity, double milliseconds)
```
### **Description**
includes a command to pulse the hammer followed by a command to brake the hammer. This function is intended to be called on its own and should be followed by a pause command of at least 50ms before the next command is called.
### **Example**
`singlePulse(M1, 0.75, 6); // aks the tacHammer M1 to generate a pulse at 75% intensity for 6 ms.`

</br></br></br>

```C
void hit(TacHammer* tacHammer, double intensity, double milliseconds)
```
### **Description**
drives the hammer towards the open end of the TacHammer. When the hammer strikes the device housing, the audible click haptic sensation is created.
### **Example**
`hit(M2, 0.25, 21); asks the tacHammer M2 to generate a hit at 25% intensity for 21ms.`

</br></br></br>

```C
void vibrate(TacHammer* tacHammer, double frequency, double intensity, double duration, int dutycycle)
```
### **Description** 
vibrate repeatedly calls the pulse command to drive the hammer into the closed end of the TacHammer.
### **Example**
`vibrate(M3, 210, 0.7, 0.25, 70); asks the tacHammer M3 to generate a vibrate for 250ms at 210Hz with 70% intensity and 70% dutycyle.`

</br></br></br>

```C
bool isFree(TacHammer* tacHammer)
```

### **Description**
return true if the tacHammer is not busy running an animation else return false. isFree should always be called before starting an haptic animation.
### **Example**
`if( isFree(M0) ) pulse(M0, 0.5, 8); check that tacHammer M0 is free before starting a pulse animation.`
