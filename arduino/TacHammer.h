#ifndef TacHammer_h
#define TacHammer_h


// struct HitAndPulseParameters{
//    double intensity;
//    double milliseconds;
// };

// struct VibrateParameters{
//    double frequency;
//    double intensity;
//    double milliseconds;
//    double duration;
//    double dutycycle;
// };

// struct TacHammer{
//    int mux_pin;
//    int mux_channel;
//    int pwm_pin;
//    int pwm_channel;
//    SemaphoreHandle_t* twoWireSemaphore;
//    TwoWire* twoWire;
//    HitAndPulseParameters* hitAndPulseParams;
//    VibrateParameters* vibrateParams;
//    TaskHandle_t taskHandle;
//    eTaskState taskStatus;
//    bool active;
// };

class TacHammers {
public:
	TacHammers();


private:
  // static TwoWire twoWire0(0);
  // static TwoWire twoWire1(1);
  // static SemaphoreHandle_t  twoWire0Semaphore; 
  // static SemaphoreHandle_t  twoWire1Semaphore; 
  // // TacHammers
  // static HitAndPulseParameters hitAndPulseParameters0 = {0, 0};
  // static VibrateParameters vibrateParameters0 = {0, 0, 0, 0, 0};
  // static TacHammer tacHammer0 = {MUX0_PIN, MUX0_CHANNEL, PWM0_PIN, PWM0_CHANNEL, &twoWire0Semaphore, &twoWire0, &hitAndPulseParameters0, &vibrateParameters0, NULL, eDeleted, false};

  // static HitAndPulseParameters hitAndPulseParameters1 = {0, 0};
  // static VibrateParameters vibrateParameters1 = {0, 0, 0, 0, 0};
  // static TacHammer tacHammer1 = {MUX1_PIN, MUX1_CHANNEL, PWM1_PIN, PWM1_CHANNEL, &twoWire0Semaphore, &twoWire0, &hitAndPulseParameters1, &vibrateParameters1, NULL, eDeleted, false};

  // static HitAndPulseParameters hitAndPulseParameters2 = {0, 0};
  // static VibrateParameters vibrateParameters2 = {0, 0, 0, 0, 0};
  // static TacHammer tacHammer2 = {MUX2_PIN, MUX2_CHANNEL, PWM2_PIN, PWM2_CHANNEL, &twoWire1Semaphore, &twoWire1, &hitAndPulseParameters2, &vibrateParameters2, NULL, eDeleted, false};

  // static HitAndPulseParameters hitAndPulseParameters3 = {0, 0};
  // static VibrateParameters vibrateParameters3 = {0, 0, 0, 0, 0};
  // static TacHammer tacHammer3 = {MUX3_PIN, MUX3_CHANNEL, PWM3_PIN, PWM3_CHANNEL, &twoWire1Semaphore, &twoWire1, &hitAndPulseParameters3, &vibrateParameters3, NULL, eDeleted, false};
};
#endif