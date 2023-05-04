// #ifdef CORE_DEBUG_LEVEL
// #undef CORE_DEBUG_LEVEL
// #endif

// #define CORE_DEBUG_LEVEL 5
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
//                    NANOPORT TACHAMMER DEVELOPER KIT FIRMWARE
//
// This firmware is intended for use with the following hardware:
//    1. Arduino Micro
//    2. DRV2605
//
// Removal or replacement of one of the above components may affect the operation
// of the TacHammer actuator.
//
// ================================================================================
// ================================ QUICK LINKS ===================================
// ================================================================================
// TACHAMMER BASIC FUNCTION LIBRARY........................................[TACLIB]
// HAPTIC LIBRARY..........................................................[HAPLIB]
// TEST FUNCTIONS..........................................................[TSTFNC]
// DRV SETUP...............................................................[DRVSET]

// 1. install Arduino IDE
// 2. install board manager "arduino-esp32"
//  a) go to 'File > Preferences'
//  b) copy "https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json" and paste it in "Additional boards manager URLs"
//  c) go to 'Tools > Board:XXX > Boards Manager...' then search for 'esp32 By Espressif Systems' and click install.



#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include <xtensa/core-macros.h>

#define SERIAL_BAUDRATE 115200
// THREAD
#define SEMAPHORE_TICKS_WAIT 10

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

// Communication parameters
char ssid [100];
const char* password = "ubc";
// IPAddress ip(192, 168, 137, 171);
// IPAddress dns(192, 168, 137,  1);
// IPAddress gateway(192, 168, 137,  1);
// IPAddress subnet(255, 255, 255, 0);
// Web server running on port 80

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


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
};




// I2C OBJECTS AND SEMAPHORES
static TwoWire twoWire0(0);
static TwoWire twoWire1(1);
static SemaphoreHandle_t  twoWire0Semaphore; 
static SemaphoreHandle_t  twoWire1Semaphore; 

// TacHammers

static HitAndPulseParameters hitAndPulseParameters0 = {0, 0};
static VibrateParameters vibrateParameters0 = {0, 0, 0, 0, 0};
static TacHammer tacHammer0 = {MUX0_PIN, MUX0_CHANNEL, PWM0_PIN, PWM0_CHANNEL, &twoWire0Semaphore, &twoWire0, &hitAndPulseParameters0, &vibrateParameters0, NULL, eDeleted, false};

static HitAndPulseParameters hitAndPulseParameters1 = {0, 0};
static VibrateParameters vibrateParameters1 = {0, 0, 0, 0, 0};
static TacHammer tacHammer1 = {MUX1_PIN, MUX1_CHANNEL, PWM1_PIN, PWM1_CHANNEL, &twoWire0Semaphore, &twoWire0, &hitAndPulseParameters1, &vibrateParameters1, NULL, eDeleted, false};

static HitAndPulseParameters hitAndPulseParameters2 = {0, 0};
static VibrateParameters vibrateParameters2 = {0, 0, 0, 0, 0};
static TacHammer tacHammer2 = {MUX2_PIN, MUX2_CHANNEL, PWM2_PIN, PWM2_CHANNEL, &twoWire1Semaphore, &twoWire1, &hitAndPulseParameters2, &vibrateParameters2, NULL, eDeleted, false};

static HitAndPulseParameters hitAndPulseParameters3 = {0, 0};
static VibrateParameters vibrateParameters3 = {0, 0, 0, 0, 0};
static TacHammer tacHammer3 = {MUX3_PIN, MUX3_CHANNEL, PWM3_PIN, PWM3_CHANNEL, &twoWire1Semaphore, &twoWire1, &hitAndPulseParameters3, &vibrateParameters3, NULL, eDeleted, false};

// clock speed variables
uint32_t prevCycleCount = 0;
uint32_t currCycleCount = 0;
uint32_t  CPU_Frequency_Mhz;
uint32_t CPU_tick_ns;

// ================================================================================
// code to run once goes here
void setup() {
  // CLOCK SPEED
  CPU_Frequency_Mhz = getCpuFrequencyMhz();
  CPU_tick_ns = 1.0 / (CPU_Frequency_Mhz / 1000.0);

  // I2C
  twoWire0Semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(twoWire0Semaphore);
  twoWire0.begin(SDA0_PIN, SCL0_PIN, I2C_FREQUENCY);
  twoWire1Semaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(twoWire1Semaphore);
  twoWire1.begin(SDA1_PIN, SCL1_PIN, I2C_FREQUENCY);

  // MUX
  pinMode(MUX0_PIN, OUTPUT); 
  pinMode(MUX1_PIN, OUTPUT); 
  pinMode(MUX2_PIN, OUTPUT); 
  pinMode(MUX3_PIN, OUTPUT); 
  digitalWrite(MUX0_PIN, LOW);
  digitalWrite(MUX1_PIN, LOW);
  digitalWrite(MUX2_PIN, LOW); 
  digitalWrite(MUX3_PIN, LOW); 
  
  // SERIAL
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setDebugOutput(true);

  // TacHammer
  setupTacHammer(&tacHammer0);
  setupTacHammer(&tacHammer1);
  setupTacHammer(&tacHammer2);
  setupTacHammer(&tacHammer3);

  // WIFI
  String macAddress = getMacAddress();
  snprintf(ssid, 100, "HackathonKit-%s", macAddress);
  Serial.print("Setting AP(Access Point) @");
  Serial.print(macAddress);
  Serial.println("...");
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  //ws.onEvent(onEvent);
  //server.addHandler(&ws);
  //server.begin();
}


String getMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    char baseMacChr[18] = {0};
    sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    return String(baseMacChr);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    // deserializeJson(jsonDocument, ((char*)data));
    // String topic = jsonDocument["topic"];
    // if (topic == GET_STATE_CMD){
    //   jsonDocument.clear();
    //   jsonDocument["currentVelocity"] = current_velocity;
    //   jsonDocument["currRefectanceValue"] = currRefectanceValue;
    //   jsonDocument["targetVelocity"] = target_velocity;
    //   jsonDocument["currentAcceleration"] = current_acceleration;
    //   jsonDocument["outputPID"] = throttle_percent;
    //   jsonDocument["proportional"] = PIDregulator.GetProportional();
    //   jsonDocument["integral"] = PIDregulator.GetIntegral();
    //   jsonDocument["derivative"] = PIDregulator.GetDerivative();
    //   jsonDocument["currentThrottle"] = current_throttle;
    //   jsonDocument["currentThrottleUint"] = current_throttle_uint16;
    //   jsonDocument["Kp"] = consKp;
    //   jsonDocument["Ki"] = consKi;
    //   jsonDocument["Kd"] = consKd;
    //   serializeJson(jsonDocument, buffer);
    //   ws.textAll((char*)buffer);
    // }

    // }
    // if (topic == SET_GAIN_CMD){
    //   consKp=0.5, consKi=0.25, consKd=0.25; 
    //   consKp = jsonDocument["payload"]["Kp"];
    //   consKi = jsonDocument["payload"]["Ki"];
    //   consKd = jsonDocument["payload"]["Kd"];
    //   alpha = jsonDocument["payload"]["alpha"];
    //   beta = jsonDocument["payload"]["beta"];
    //   PIDregulator.SetTunings(consKp, consKi, consKd, alpha, beta);
    // }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
             void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
      break;
    case WS_EVT_DISCONNECT:
      Serial.printf("WebSocket client #%u disconnected\n", client->id());
      break;
    case WS_EVT_DATA:
      handleWebSocketMessage(arg, data, len);
      break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
      break;
  }
}

void setupTacHammer(TacHammer* tacHammer){
  // PWM SETUP
  uint32_t  created_pwm = ledcSetup(tacHammer -> pwm_channel, PWM_FREQUENCY, PWM_RESOLUTION_BITS);
  ledcAttachPin(tacHammer -> pwm_pin, tacHammer -> pwm_channel);
  ledcWrite(tacHammer -> pwm_channel, 0);
  // DRV INIT
  selectMux(tacHammer -> mux_pin, tacHammer -> pwm_channel);
  tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
  tacHammer -> twoWire -> write(DRV2605_MODEREG);          // sets register pointer to the mode register (0x01)
  tacHammer -> twoWire -> write(0x00);               // clear standby
  tacHammer -> twoWire -> endTransmission();

  tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
  tacHammer -> twoWire -> write(0x1D);              // sets register pointer to the Libarary Selection register (0x1D)
  tacHammer -> twoWire -> write(0xA8);              // set RTP unsigned
  tacHammer -> twoWire -> endTransmission();

  tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
  tacHammer -> twoWire -> write(0x03);
  tacHammer -> twoWire -> write(0x02);              // set to Library B, most aggresive
  tacHammer -> twoWire -> endTransmission();

  tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
  tacHammer -> twoWire -> write(0x17);               // sets full scale reference
  tacHammer -> twoWire -> write(0xff);               //
  tacHammer -> twoWire -> endTransmission();

  tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
  tacHammer -> twoWire -> write(DRV2605_MODEREG);               // sets register pointer to the mode register (0x01)
  tacHammer -> twoWire -> write(0x03);               // Sets Mode to pwm
  tacHammer -> twoWire -> endTransmission();
  delay(100);

}


void selectMux(int mux_pin, int mux_channel){
  int A = bitRead(mux_channel,0);
  digitalWrite(mux_pin, A);
  prevCycleCount = currCycleCount = XTHAL_GET_CCOUNT();
  while((currCycleCount - prevCycleCount) * CPU_tick_ns < MUX_NANOSECONDS_DELAY){
    currCycleCount = XTHAL_GET_CCOUNT();
  }
}


void standbyOnB(TacHammer* tacHammer) {
    selectMux(tacHammer -> mux_pin, tacHammer -> mux_channel);
    tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
    tacHammer -> twoWire -> write(DRV2605_MODEREG);               // sets register pointer to the mode register (0x01)
    tacHammer -> twoWire -> write(0x43);               // Puts the device pwm mode
    tacHammer -> twoWire -> endTransmission();
}

void standbyOffB(TacHammer* tacHammer) {
    selectMux(tacHammer -> mux_pin, tacHammer -> mux_channel);
    tacHammer -> twoWire -> beginTransmission(DRV2605_ADDRESS);
    tacHammer -> twoWire -> write(DRV2605_MODEREG);               // sets register pointer to the mode register (0x01)
    tacHammer -> twoWire -> write(0x03);               // Sets Waveform Mode to pwm
    tacHammer -> twoWire -> endTransmission();
}

void usdelay(double time) {
  double us = time - ((int)time);
  for (int i = 0; i <= time; i++) {
    delay(1);
  }
  delayMicroseconds(us * 1000);
}

void pause(TacHammer* tacHammer, double milliseconds) {
  double us = milliseconds - ((int)milliseconds);
  standbyOnB(tacHammer);
  for (int i = 0; i <= milliseconds; i++) {
    delay(1);
  }
  delayMicroseconds(us * 1000);
}

void pulse(TacHammer* tacHammer, double intensity, double milliseconds) {
  if (isDone(tacHammer)){
    tacHammer -> hitAndPulseParams -> intensity = intensity;
    tacHammer -> hitAndPulseParams -> milliseconds = milliseconds;
    BaseType_t ans = xTaskCreate(&pulseTask, "pulseTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      Serial.println("Could not create a hitTask for this tacHammer!");
    }
  } else {
    Serial.println("a pulseTask is already running for this tacHammer!");
  }
  
}

void pulseTask(void *pvParameter) {
  TacHammer* tacHammer = (TacHammer*)pvParameter; 
  tacHammer -> active = true;
  HitAndPulseParameters* hitAndPulseParams = tacHammer -> hitAndPulseParams;
  //Serial.println(tacHammer -> mux_pin);
  //Serial.println(tacHammer -> pwm_pin);
  SemaphoreHandle_t* twoWireSemaphore = tacHammer -> twoWireSemaphore;
  int minimumint = 140;
  int maximumint = 255;
  int pwmintensity = (hitAndPulseParams -> intensity * (maximumint - minimumint)) + minimumint;
  //Serial.println(hitAndPulseParams -> intensity);
  if(*twoWireSemaphore != NULL ){
        //Serial.println("Wait for semaphore...");
        if( xSemaphoreTake(*twoWireSemaphore, ( TickType_t ) SEMAPHORE_TICKS_WAIT ) == pdTRUE ){
          //Serial.println("got semaphore!");
          standbyOffB(tacHammer);
          ledcWrite(tacHammer -> pwm_channel, pwmintensity);
          usdelay(hitAndPulseParams -> milliseconds);
          standbyOnB(tacHammer);
          xSemaphoreGive(*twoWireSemaphore);
        }  else {
        //Serial.println("Semaphore is NOT free!");
        }
    }else {
      //Serial.println("Semaphore is null!");
    }
  tacHammer -> active = false;
  vTaskDelete(tacHammer -> taskHandle);
}

void hit(TacHammer* tacHammer, double intensity, double milliseconds) {
  if (isDone(tacHammer)){
    //HitAndPulseParameters hitAndPulseParams = {tacHammer, intensity, milliseconds};
    tacHammer -> hitAndPulseParams -> intensity = intensity;
    tacHammer -> hitAndPulseParams -> milliseconds = milliseconds;
    BaseType_t ans = xTaskCreate(&hitTask, "hitTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      Serial.println("Could not create a hitTask for this tacHammer!");
    }
  } else {
    Serial.println("a hitTask is already running for this tacHammer!");
  }
}

void hitTask(void *pvParameter) {
  TacHammer* tacHammer = (TacHammer*)pvParameter; 
  tacHammer -> active = true;
  HitAndPulseParameters* hitAndPulseParams = tacHammer -> hitAndPulseParams;
  SemaphoreHandle_t* twoWireSemaphore = tacHammer -> twoWireSemaphore;
  int minimumint = 0;
  int maximumint = 110;
  int pwmintensity = maximumint - (hitAndPulseParams -> intensity * (maximumint - minimumint));
  if(*twoWireSemaphore != NULL ){
        //Serial.println("Wait for semaphore...");
        if( xSemaphoreTake(*twoWireSemaphore, ( TickType_t ) SEMAPHORE_TICKS_WAIT ) == pdTRUE ){
            standbyOffB(tacHammer);
            ledcWrite(tacHammer -> pwm_channel, pwmintensity);
            usdelay(hitAndPulseParams -> milliseconds);
            standbyOnB(tacHammer);
            xSemaphoreGive(*twoWireSemaphore);
        }  else {
        //Serial.println("Semaphore is NOT free!");
        }
    }else {
      //Serial.println("Semaphore is null!");
    }
  tacHammer -> active = false;
  vTaskDelete(tacHammer -> taskHandle);
}

bool isDone(TacHammer* tacHammer){
  //Serial.print ("tacHammer -> taskHandle == NULL:");
  //Serial.println (tacHammer -> taskHandle == NULL);
  //Serial.print ("(tacHammer -> taskStatus = eTaskGetState(tacHammer -> taskHandle)) == eDeleted");
  //Serial.println ((tacHammer -> taskStatus = eTaskGetState(tacHammer -> taskHandle)) == eDeleted);
  //return tacHammer -> taskHandle == NULL || (tacHammer -> taskStatus = eTaskGetState(tacHammer -> taskHandle)) == eDeleted;
  return !(tacHammer -> active);
}

void vibrate(TacHammer* tacHammer, double frequency, double intensity, double duration, int dutycycle) {
  if (isDone(tacHammer)){
    tacHammer -> vibrateParams -> frequency = frequency;
    tacHammer -> vibrateParams -> intensity = intensity;
    tacHammer -> vibrateParams -> duration = duration;
    tacHammer -> vibrateParams -> dutycycle = dutycycle;
    BaseType_t ans =  xTaskCreate(&vibrateTask, "vibrateTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      Serial.println("Could not create a hitTask for this tacHammer!");
    }

  } else {
    Serial.println("a vibrateTask is already running for this tacHammer!");
  }
}

void vibratePulse(TacHammer* tacHammer, double intensity, int hitduration) {
  SemaphoreHandle_t* twoWireSemaphore = tacHammer -> twoWireSemaphore;
  int minimumint = 140;
  int maximumint = 255;
  int pwmintensity = (intensity * (maximumint - minimumint)) + minimumint;
  if(*twoWireSemaphore != NULL ){
        //Serial.println("Wait for semaphore...");
        if( xSemaphoreTake(*twoWireSemaphore, ( TickType_t ) SEMAPHORE_TICKS_WAIT ) == pdTRUE ){
          standbyOffB(tacHammer);
          ledcWrite(tacHammer -> pwm_channel, pwmintensity);
          usdelay(hitduration);
          //usdelay(hitAndPulseParams -> milliseconds);
          standbyOnB(tacHammer);
          xSemaphoreGive(*twoWireSemaphore);
        }  else {
        //Serial.println("Semaphore is NOT free!");
        }
    }else {
      //Serial.println("Semaphore is null!");
    }
}

void vibrateTask(void *pvParameter) {
  
  TacHammer* tacHammer = (TacHammer*)pvParameter; 
  tacHammer -> active = true;
  VibrateParameters* vibrateParams = tacHammer -> vibrateParams;
  SemaphoreHandle_t* twoWireSemaphore = tacHammer -> twoWireSemaphore;
  double frequency = vibrateParams -> frequency;
  double intensity = vibrateParams -> intensity;
  double milliseconds = vibrateParams -> milliseconds;
  double duration = vibrateParams -> duration;
  double dutycycle = vibrateParams -> dutycycle;

  int max_hit = 12;
  int min_hit = 1;
  int crossover = 60;
  int hitduration = 10 * dutycycle / frequency;

  bool hold = false;
  double delayy;
  delayy = (1 / frequency * 1000) - hitduration;
  double timedown;
  timedown = duration * 1000;

  if (duration == 0) {
    hold =  true;
  }
  
  //HitAndPulseParameters firstHitAndPulseParams = {tacHammer, intensity, hitduration};
  //HitAndPulseParameters secondHitAndPulseParams = {tacHammer, 0.002, delayy-3};

  while (hold) {
    vibratePulse(tacHammer, intensity, hitduration);
    pause(tacHammer, delayy);
  }

  while (timedown >= 0 && frequency < crossover) {
    vibratePulse(tacHammer, intensity, hitduration);
    pause(tacHammer, 3);
    vibratePulse(tacHammer, 0.002, delayy-3);
    timedown -= (delayy + hitduration);
  }

  while (timedown >= 0 && frequency >= crossover) {
    vibratePulse(tacHammer, intensity, hitduration);
    pause(tacHammer, delayy);
    timedown -= (delayy + hitduration);
  }
  tacHammer -> active = false;
  vTaskDelete(tacHammer -> taskHandle);
}


void laser() {
  Serial.print("Laser Charging");

  // LASER PISTOL CHARGING
  // The effect is created by linking a series of increasing frequency vibrations
  vibrate(&tacHammer0, 35, 0.007, 0.07, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 40, 0.008, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 45, 0.009, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 60, 0.01, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 75, 0.03, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 80, 0.08, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 85, 0.1, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 88, 0.1, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 90, 0.15, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 130, 0.15, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 140, 0.2, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 160, 0.2, 0.04, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }
  vibrate(&tacHammer0, 200, 0.3, 0.05, 50);
  while (!isDone(&tacHammer0)){
    delay(1);
  }

  for (int i=0;i<4;i++){
    vibrate(&tacHammer0, 200, 0.5, 0.01, 50);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    pulse(&tacHammer0, .1, 2);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    vibrate(&tacHammer0, 200, 0.5, 0.01, 50);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    pulse(&tacHammer0, .7, 2);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    vibrate(&tacHammer0, 200, 0.5, 0.01, 50);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    pulse(&tacHammer0, .95, 2);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    vibrate(&tacHammer0, 200, 0.5, 0.01, 50);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    pulse(&tacHammer0, .7, 2);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    vibrate(&tacHammer0, 200, 0.5, 0.01, 50);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
    pulse(&tacHammer0, .1, 2);
    while (!isDone(&tacHammer0)){
      delay(1);
    }
  }
}

// ================================================================================
// active code goes here
void loop()
{
    double i = 0;
    Serial.println("pulse all tacHammers... ");
    i = 0;
    for (int j=0;j<20;j++){
      pulse(&tacHammer0, i, 8);
      pulse(&tacHammer1, i, 8);
      pulse(&tacHammer2, i, 8);
      pulse(&tacHammer3, i, 8);
      while (!isDone(&tacHammer0) || !isDone(&tacHammer1)|| !isDone(&tacHammer2) || !isDone(&tacHammer3)){
        delay(1);
      }
      delay(84);
      if (i < 0.99) {
        i += 0.05;
      }
    }
    delay(1000);
    Serial.println("hit all tacHammers... ");
    i = 0;
    for (int j=0;j<20;j++){
      
      pulse(&tacHammer0, i, 6);
      pulse(&tacHammer1, i, 6);
      pulse(&tacHammer2, i, 6);
      pulse(&tacHammer3, i, 6);
      while (!isDone(&tacHammer0) || !isDone(&tacHammer1)|| !isDone(&tacHammer2) || !isDone(&tacHammer3)){
        delay(1);
      }
      hit(&tacHammer0, i, 21);
      hit(&tacHammer1, i, 21);
      hit(&tacHammer2, i, 21);
      hit(&tacHammer3, i, 21);
      while (!isDone(&tacHammer0) || !isDone(&tacHammer1)|| !isDone(&tacHammer2) || !isDone(&tacHammer3)){
        delay(1);
      }
      delay(100);

      if (i < 0.99) {
        i += 0.05;
      }
    }
    delay(1000);
  // laser();
  //   delay(1000);
    
  // vibrate(&tacHammer0, 30,0.7,0.33,30);
  // while (!isDone(&tacHammer0)){
  //   delay(1);
  // }
  // delay(1000);
  
  // vibrate(&tacHammer0, 250,1,0.01,70);
  // while (!isDone(&tacHammer0)){
  //   delay(1);
  // }
  // delay(1000);
  // ws.cleanupClients();
    // Serial.println("pulse tacHammer0... ");
    // i = 0;
    // for (int j=0;j<20;j++){
    // // Serial.print("Pulse Intensity: ");
    // // Serial.println(i)
    //   pulse(&tacHammer0, i, 8);
    //   while (!isDone(&tacHammer0)){
    //     delay(84);
    //   }
    //   if (i < 0.99) {
    //     i += 0.05;
    //   }
    // }
    // delay(1000);

    // Serial.println("pulse tacHammer1... ");
    // i = 0;
    // for (int j=0;j<20;j++){
    // // Serial.print("Pulse Intensity: ");
    // // Serial.println(i)
    //   pulse(&tacHammer1, i, 8);
    //   while (!isDone(&tacHammer1)){
    //     delay(84);
    //   }
    //   if (i < 0.99) {
    //     i += 0.05;
    //   }
    // }
    // delay(1000);
    // Serial.println("pulse tacHammer2... ");
    // i = 0;
    // for (int j=0;j<20;j++){
    // // Serial.print("Pulse Intensity: ");
    // // Serial.println(i)
    //   pulse(&tacHammer2, i, 8);
    //   while (!isDone(&tacHammer2)){
    //     delay(84);
    //   }
    //   if (i < 0.99) {
    //     i += 0.05;
    //   }
    // }
    // delay(1000);
    // Serial.println("pulse tacHammer3... ");
    // i = 0;
    // for (int j=0;j<20;j++){
    // // Serial.print("Pulse Intensity: ");
    // // Serial.println(i)
    //   pulse(&tacHammer3, i, 8);
    //   while (!isDone(&tacHammer3)){
    //     delay(84);
    //   }
    //   if (i < 0.99) {
    //     i += 0.05;
    //   }
    // }
    // delay(1000);
    // Serial.println("pulse tacHammer2... ");
    // pulse(&tacHammer2, 0.5, 30);
    // while (!isDone(&tacHammer2)){
    //   delay(84);
    // }
    // Serial.println("pulse tacHammer3... ");
    // pulse(&tacHammer3, 0.5, 30);
    // while (!isDone(&tacHammer3)){
    //   delay(84);
    // }

    delay(1000);
  
}
