
#include "TacHammers.h"
static const char* TAG = "MyModule";
// I2C OBJECTS AND SEMAPHORES
TwoWire twoWire0(0);
TwoWire twoWire1(1);
SemaphoreHandle_t  twoWire0Semaphore; 
SemaphoreHandle_t  twoWire1Semaphore; 

// TacHammers
HitAndPulseParameters hitAndPulseParameters0 = {0, 0};
VibrateParameters vibrateParameters0 = {0, 0, 0, 0, 0};
TacHammer tacHammer0 = {"M0", MUX0_PIN, MUX0_CHANNEL, PWM0_PIN, PWM0_CHANNEL, &twoWire0Semaphore, &twoWire0, &hitAndPulseParameters0, &vibrateParameters0, NULL, eDeleted, false, false};
TacHammer* M0 = &tacHammer0;

HitAndPulseParameters hitAndPulseParameters1 = {0, 0};
VibrateParameters vibrateParameters1 = {0, 0, 0, 0, 0};
TacHammer tacHammer1 = {"M1", MUX1_PIN, MUX1_CHANNEL, PWM1_PIN, PWM1_CHANNEL, &twoWire0Semaphore, &twoWire0, &hitAndPulseParameters1, &vibrateParameters1, NULL, eDeleted, false, false};
TacHammer* M1 = &tacHammer1;

HitAndPulseParameters hitAndPulseParameters2 = {0, 0};
VibrateParameters vibrateParameters2 = {0, 0, 0, 0, 0};
TacHammer tacHammer2 = {"M2", MUX2_PIN, MUX2_CHANNEL, PWM2_PIN, PWM2_CHANNEL, &twoWire1Semaphore, &twoWire1, &hitAndPulseParameters2, &vibrateParameters2, NULL, eDeleted, false, false};
TacHammer* M2 = &tacHammer2;

HitAndPulseParameters hitAndPulseParameters3 = {0, 0};
VibrateParameters vibrateParameters3 = {0, 0, 0, 0, 0};
TacHammer tacHammer3 = {"M3", MUX3_PIN, MUX3_CHANNEL, PWM3_PIN, PWM3_CHANNEL, &twoWire1Semaphore, &twoWire1, &hitAndPulseParameters3, &vibrateParameters3, NULL, eDeleted, false, false};
TacHammer* M3 = &tacHammer3;

// clock speed variables
uint32_t prevCycleCount = 0;
uint32_t currCycleCount = 0;
uint32_t  CPU_Frequency_Mhz;
uint32_t CPU_tick_ns;


// Replace with your network credentials
//char ssid [100];
const char* ssid     = "HapticHIVE";
const char* password = "ubc";

// Set web server port number to 80
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


/**
* pulse drives the hammer towards the closed end of the TacHammer.
* When the hammer rebounds off of the repelling magnetic array, the inaudible pulse haptic sensation is created.
* pulse is intended to be sequenced with subsequent pulse and hit commands and if called on its own, the hammer may travel after the rebound and strike the open end
* parameters:
*    - tacHammer: is either M0, M1, M2 or M3.
*    - intensity: defines the strength of the pulse. It ranges from [0-1] with 1 being strongest
*    - milliseconds: is the length of time the coil is activated in ms
**/
void pulse(TacHammer* tacHammer, double intensity, double milliseconds) {
  if (isFree(tacHammer)){
    tacHammer -> hitAndPulseParams -> intensity = intensity;
    tacHammer -> hitAndPulseParams -> milliseconds = milliseconds;
    BaseType_t ans = xTaskCreate(&pulseTask, "pulseTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      Serial.println("Could not create a pulseTask for this tacHammer!");
    }
  } else {
    Serial.println("a pulseTask is already running for this tacHammer!");
  }
  
}


/**
* singlePulse includes a command to pulse the hammer followed by a command to brake the hammer.
* This function is intended to be called on its own and should be followed by a pause command of
* at least 50ms before the next command is called.
* parameters:
*    - tacHammer: is either M0, M1, M2 or M3.
*    - intensity: defines the strength of the single pulse. It ranges from [0-1] with 1 being strongest.
*    - milliseconds: is the length of time the coil is activated in ms.
**/
void singlePulse(TacHammer* tacHammer, double intensity, double milliseconds) {
  if (isFree(tacHammer)){
    tacHammer -> hitAndPulseParams -> intensity = intensity;
    tacHammer -> hitAndPulseParams -> milliseconds = milliseconds;
    BaseType_t ans = xTaskCreate(&singlePulseTask, "singlePulseTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      Serial.println("Could not create a singlePulseTask for this tacHammer!");
    }
  } else {
    Serial.println("a singlePulseTask is already running for this tacHammer!");
  }
}


/**
*  hit drives the hammer towards the open end of the TacHammer.
* When the hammer strikes the device housing, the audible click haptic sensation is created.
* parameters:
*    - tacHammer: is either M0, M1, M2 or M3.
*    - intensity: defines the strength of the hit. It ranges from [0-1] with 1 being strongest.
*    - milliseconds: is the length of time the coil is activated in ms.
**/ 
void hit(TacHammer* tacHammer, double intensity, double milliseconds) {
  if (isFree(tacHammer)){
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




/**
*  Vibrate repeatedly calls the pulse command to drive the hammer into the closed end of the TacHammer.
* parameters:
*    - tacHammer: is either M0, M1, M2 or M3.
*    - frequency: defines the frequency of the vibration.. it ranges from 0-300.
*    - intensity: defines the strength of the vibrate. It ranges from [0-1] with 1 being strongest
*    - dutycycle: defines what percent of the period the TacHammer is active. It ranges from 0-100.
*    - duration: defines the length of vibration in s.
* suggested duty cycles for vibrate:
*  - frequency| 10| 30| 50| 70| 90|110|130|150|170|190|210|230|250|270|290
*  - dutycycle| 32| 28| 33| 36| 38| 37| 41| 47| 45| 47| 39| 40| 44| 43| 40
**/
void vibrate(TacHammer* tacHammer, double frequency, double intensity, double duration, int dutycycle) {
  if (frequency < 1 || frequency > 300){
    ESP_LOGE(tacHammer -> name, "frequency must be between 1Hz and 300Hz.");
    return;
  }
    if (intensity < 0 || intensity > 1){
    ESP_LOGE(tacHammer -> name, "intensity must be between 0 and 1.");
    return;
  }
  if (duration <= 0){
    ESP_LOGE(tacHammer -> name, "duration must be greater than 0s.");
    return;
  }
  if (dutycycle < 1 || dutycycle > 100){
    ESP_LOGE(tacHammer -> name, "dutycycle must be between 1%% and 100%%.");
    return;
  }
  // error frequency must be greater than zero
  if (isFree(tacHammer)){
    tacHammer -> vibrateParams -> frequency = frequency;
    tacHammer -> vibrateParams -> intensity = intensity;
    tacHammer -> vibrateParams -> duration = duration;
    tacHammer -> vibrateParams -> dutycycle = dutycycle;
    BaseType_t ans =  xTaskCreate(&vibrateTask, "vibrateTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      ESP_LOGE(tacHammer -> name, "failed to create a vibrateTask.");
    }

  } else {
      ESP_LOGE(tacHammer -> name, "a vibrateTask is already running. Before starting a new task, use isFree(%s) to check that no task is running OR Use stop(%s) to kill the running task.", tacHammer -> name, tacHammer -> name);
  }
}


// ================================================================================
// !!! DO NOT MODIFY THE FOLLOWING FUNCTIONS !!!
// ================================================================================
void onWsEvent(AsyncWebSocket * server, AsyncWebSocketClient * client, AwsEventType type, void * arg, uint8_t *data, size_t len){
  if(type == WS_EVT_CONNECT){
    ESP_LOGI("WebSocket", "(%s | %u) connect", server->url(), client->id());
  } else if(type == WS_EVT_DISCONNECT){
    ESP_LOGI("WebSocket", "(%s | %u) disconnect", server->url(), client->id());
  } else if(type == WS_EVT_ERROR){
    ESP_LOGI("WebSocket", "(%s | %u) error(%u): %s", server->url(), client->id(), *((uint16_t*)arg), (char*)data);

  } else if(type == WS_EVT_PONG){
    ESP_LOGI("WebSocket", "(%s | %u) pong[%u]: %s", server->url(), client->id(), len, (len)?(char*)data:"");
  } else if(type == WS_EVT_DATA){
    AwsFrameInfo * info = (AwsFrameInfo*)arg;
    String msg = "";
    if(info->final && info->index == 0 && info->len == len){
      //the whole message is in a single frame and we got all of it's data
      ESP_LOGI("WebSocket", "(%s | %u) %s-message[%llu]: ", server->url(), client->id(), (info->opcode == WS_TEXT)?"text":"binary", info->len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < info->len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < info->len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      Serial.printf("%s\n",msg.c_str());

      // if(info->opcode == WS_TEXT)
      //   client->text("I got your text message");
      // else
      //   client->binary("I got your binary message");
    } else {
      //message is comprised of multiple frames or the frame is split into multiple packets
      if(info->index == 0){
        if(info->num == 0)
        ESP_LOGI("WebSocket", "(%s | %u)  %s-message start", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        ESP_LOGI("WebSocket", "(%s | %u) frame[%u] start[%llu]", server->url(), client->id(), info->num, info->len);
      }

      ESP_LOGI("WebSocket", "(%s | %u) frame[%u] %s[%llu - %llu]: ", server->url(), client->id(), info->num, (info->message_opcode == WS_TEXT)?"text":"binary", info->index, info->index + len);

      if(info->opcode == WS_TEXT){
        for(size_t i=0; i < len; i++) {
          msg += (char) data[i];
        }
      } else {
        char buff[3];
        for(size_t i=0; i < len; i++) {
          sprintf(buff, "%02x ", (uint8_t) data[i]);
          msg += buff ;
        }
      }
      ESP_LOGI("WebSocket", "(%s | %u) %s", server->url(), client->id(), msg.c_str());

      if((info->index + len) == info->len){
        ESP_LOGI("WebSocket", "(%s | %u) frame[%u] end[%llu]", server->url(), client->id(), info->num, info->len);
        if(info->final){
          ESP_LOGI("WebSocket", "(%s | %u) %s-message end\n", server->url(), client->id(), (info->message_opcode == WS_TEXT)?"text":"binary");
        }
      }
    }
  }
}

void cleanupSmartWatch(){
    ws.cleanupClients();
}

void setupSmartWatch(){
  
  // WIFI
  // Connect to Wi-Fi network with SSID and password
  // Remove the password parameter, if you want the AP (Access Point) to be open
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid); //  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  ESP_LOGI("WiFi", "AP name: %s, IP address: %s", ssid, IP.toString());
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();

  // String macAddress = getMacAddress();
  // snprintf(ssid, 100, "HackathonKit-%s", macAddress);
  // Serial.print("Setting AP(Access Point) @");
  // Serial.print(macAddress);
  // Serial.println("...");
  // WiFi.softAP(ssid, password);
  // IPAddress IP = WiFi.softAPIP();
  // Serial.print("AP IP address: ");
  // Serial.println(IP);
  //ws.onEvent(onEvent);
  //server.addHandler(&ws);
  //server.begin();
  
}

void setupTacHammers(){
  // ALIAS
  M0 = &tacHammer0;
  M1 = &tacHammer1;
  M2 = &tacHammer2;
  M3 = &tacHammer3;


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

  // TacHammer
  setupTacHammer(&tacHammer0);
  setupTacHammer(&tacHammer1);
  setupTacHammer(&tacHammer2);
  setupTacHammer(&tacHammer3);

}




void selectMux(int mux_pin, int mux_channel){
  int A = bitRead(mux_channel,0);
  digitalWrite(mux_pin, A);
  prevCycleCount = currCycleCount = XTHAL_GET_CCOUNT();
  while((currCycleCount - prevCycleCount) * CPU_tick_ns < MUX_NANOSECONDS_DELAY){
    currCycleCount = XTHAL_GET_CCOUNT();
  }
}


String getMacAddress() {
    uint8_t baseMac[6];
    // Get MAC address for WiFi station
    esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
    char baseMacChr[18] = {0};
    sprintf(baseMacChr, "%02X:%02X:%02X:%02X:%02X:%02X", baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
    return String(baseMacChr);
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


bool isFree(TacHammer* tacHammer){
  return !(tacHammer -> active);
}
bool isBusy(TacHammer* tacHammer){
  return tacHammer -> active;
}

void waitUntilFree(TacHammer* tacHammer){
  while(!isFree(tacHammer)) delay(10);
}

void stop(TacHammer* tacHammer){
  tacHammer -> stop = true;
  waitUntilFree(tacHammer);
}

void usdelay(double time) {
  double us = time - ((int)time);
  for (int i = 0; i <= time; i++) {
    delay(1);
  }
  delayMicroseconds(us * 1000);
}


void pause(TacHammer* tacHammer, double milliseconds) {
  if (isFree(tacHammer)){
    tacHammer -> hitAndPulseParams -> intensity = 0;
    tacHammer -> hitAndPulseParams -> milliseconds = milliseconds;
    BaseType_t ans = xTaskCreate(&pauseTask, "pauseTask", 4096, tacHammer, 1, &(tacHammer -> taskHandle));
    if (ans == pdPASS){
      tacHammer -> active = true;
    } else {
      Serial.println("Could not create a pauseTask for this tacHammer!");
    }
  } else {
    Serial.println("a pauseTask is already running for this tacHammer!");
  }
}

void pauseTask(void *pvParameter) {
  TacHammer* tacHammer = (TacHammer*)pvParameter; 
  tacHammer -> active = true;
  double milliseconds = tacHammer -> hitAndPulseParams -> milliseconds;
  SemaphoreHandle_t* twoWireSemaphore = tacHammer -> twoWireSemaphore;
  double us = milliseconds - ((int)milliseconds);
  if(*twoWireSemaphore != NULL ){
        //Serial.println("Wait for semaphore...");
        if( xSemaphoreTake(*twoWireSemaphore, ( TickType_t ) SEMAPHORE_TICKS_WAIT ) == pdTRUE ){
          //Serial.println("got semaphore!");
          standbyOnB(tacHammer);
          xSemaphoreGive(*twoWireSemaphore);
        }  else {
        //Serial.println("Semaphore is NOT free!");
        }
  }else {
    //Serial.println("Semaphore is null!");
  }
  for (int i = 0; i <= milliseconds; i++) {
    delay(1);
  }
  delayMicroseconds(us * 1000);
  tacHammer -> active = tacHammer -> stop = false;
  vTaskDelete(tacHammer -> taskHandle);
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
  tacHammer -> active = tacHammer -> stop = false;
  vTaskDelete(tacHammer -> taskHandle);
}

void singlePulseTask(void *pvParameter) {
//   pulse(intensity, milliseconds);
//   pause(3);
//   pulse(intensity * 3 / 100, milliseconds * 2);
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

  pause(tacHammer, 3);

  pwmintensity = ((hitAndPulseParams -> intensity * 3 / 100) * (maximumint - minimumint)) + minimumint;
  if(*twoWireSemaphore != NULL ){
        //Serial.println("Wait for semaphore...");
        if( xSemaphoreTake(*twoWireSemaphore, ( TickType_t ) SEMAPHORE_TICKS_WAIT ) == pdTRUE ){
          //Serial.println("got semaphore!");
          standbyOffB(tacHammer);
          ledcWrite(tacHammer -> pwm_channel, pwmintensity);
          usdelay(hitAndPulseParams -> milliseconds * 2);
          standbyOnB(tacHammer);
          xSemaphoreGive(*twoWireSemaphore);
        }  else {
        //Serial.println("Semaphore is NOT free!");
        }
    }else {
      //Serial.println("Semaphore is null!");
    }
  tacHammer -> active = tacHammer -> stop = false;
  vTaskDelete(tacHammer -> taskHandle);
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
  tacHammer -> active = tacHammer -> stop = false;
  vTaskDelete(tacHammer -> taskHandle);
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
    if(tacHammer -> stop) break;
    vibratePulse(tacHammer, intensity, hitduration);
    usdelay(delayy);
    //vibratePause(tacHammer, delayy);
  }

  while (timedown >= 0 && frequency < crossover) {
    if(tacHammer -> stop) break;
    vibratePulse(tacHammer, intensity, hitduration);
    usdelay(3);
    //vibratePause(tacHammer, 3);
    vibratePulse(tacHammer, 0.002, delayy-3);
    timedown -= (delayy + hitduration);
  }

  while (timedown >= 0 && frequency >= crossover) {
    if(tacHammer -> stop) break;
    vibratePulse(tacHammer, intensity, hitduration);
    usdelay(delayy);
    //vibratePause(tacHammer, delayy);
    timedown -= (delayy + hitduration);
  }
  tacHammer -> active = tacHammer -> stop = false;
  vTaskDelete(tacHammer -> taskHandle);
}

