/*
  ================================================================
   TT-02R RC Receiver with PCA9685 + nRF24 + EZRun ESC (Debug Build)
  ================================================================

  Hardware:
  - Microcontroller: Arduino Pro Mini (5V, 16MHz)
  - PCA9685 PWM Driver (I2C, 16-channel)
      * ESC connected to CH0
      * Steering servo connected to CH1
      * Head swivel servo connected to CH2
  - Radio: nRF24L01+ module, receiving throttle/steering/head commands
  - ESC: Hobbywing EzRun (brushed/brushless, TT-02R car)

  Why PCA9685?
  - Provides stable, hardware-timed PWM at 50 Hz (20 ms frame).
  - Offloads timing from Pro Mini (no Servo library jitter).
  - Needed for consistent ESC arming and servo response.

  EZRun Reverse Logic:
  - ESCs like this require a "double-tap" sequence:
      1. First pull below neutral -> Brake only (no reverse)
      2. Return to neutral
      3. Second pull below neutral -> Reverse engages
  - This sketch enforces that sequence with a small state machine.

  Software Features:
  - Receives 6-byte control packet from transmitter:
      * throttle (0–255), steering (0–255), head swivel (0–255)
      * button1, button2, speedMode (for throttle scaling)
  - Speed modes:
      * Slow, Medium, Fast (change forward max µs pulse)
  - Failsafe: If no radio packets for >1s, throttle is held and decays toward neutral.
  - Steering + head servos run directly off PCA9685 outputs.
  - Serial Monitor prints ESC pulse values + reverse state transitions
    for debugging the double-tap logic.

  ================================================================
*/

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// ===== CONFIGURATION =====
#define STEERING_REVERSED false   // flip true if steering is mirrored

// PCA9685 channel assignments
#define ESC_CH   0   // ESC on PCA9685 channel 0
#define STR_CH   1   // Steering servo on channel 1
#define HEAD_CH  2   // Head swivel servo on channel 2

// Radio
RF24 radio(9, 10);   // CE, CSN pins on Pro Mini
const byte address[6] = "00001"; // Pipe address

// Control packet format
struct Data_Package {
  byte throttle, steering, button1, button2, speedMode, headSwivel;
} data;

// ==== ESC CALIBRATION VALUES (TT-02R) ====
// These values were determined by bench testing your TT-02R ESC + PCA9685
const int NEUTRAL_US   = 1500;   // neutral pulse (ESC armed, no movement)
const int BRAKE_US     = 1400;   // brake pulse (applies brake, no reverse)
const int REV_FULL_US  = 1300;   // full reverse pulse

// Forward caps (by speed mode)
const int FWD_SLOW_MAX = 1550;   // max forward in slow mode
const int FWD_MED_MAX  = 1570;   // max forward in medium mode
const int FWD_FAST_MAX = 1600;   // max forward in fast mode

// Convert microseconds to PCA9685 ticks (at 50Hz frame)
int usToTicks(int us){ return map(us, 0, 20000, 0, 4096); }

// Reverse state machine for "double-tap" logic
enum RevState : uint8_t { RS_IDLE, RS_BRAKE, RS_WAITING_REV, RS_REV };
RevState revState = RS_IDLE;

// Forward declaration of state transition helper
void changeState(RevState newState);

// Radio failsafe tracking
unsigned long lastUpdate = 0;
byte failsafeThrottle = 127;
byte lastThrottle = 127;

// PCA9685 driver
Adafruit_PWMServoDriver pwm(0x40);

// --- State transition helper (prints to Serial for debugging) ---
void changeState(RevState newState){
  if (revState != newState){
    const char* labels[] = {"IDLE","BRAKE","WAITING_REV","REV"};
    Serial.print("Reverse state: ");
    Serial.print(labels[revState]);
    Serial.print(" -> ");
    Serial.println(labels[newState]);
    revState = newState;
  }
}

void setup(){
  Serial.begin(9600);

  // Initialize PCA9685
  pwm.begin();
  pwm.setPWMFreq(50); // 50 Hz = 20 ms frame (servo/ESC standard)

  // Initialize radio
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_HIGH);
  radio.openReadingPipe(0, address);
  radio.startListening();

  // Safe startup positions
  pwm.setPWM(ESC_CH,  0, usToTicks(NEUTRAL_US));
  pwm.setPWM(STR_CH,  0, usToTicks(1090)); // wheels straight ≈ 1090 µs
  pwm.setPWM(HEAD_CH, 0, usToTicks(1500)); // head centered

  Serial.println("TT-02R PCA9685 receiver ready (with debug logging)");
}

void loop(){
  unsigned long now = millis();

  // === RADIO RECEIVE ===
  if (radio.available()){
    radio.read(&data, sizeof(data));
    lastUpdate = now;
    lastThrottle = data.throttle;
    failsafeThrottle = data.throttle;
  }

  // Failsafe: if no packet in >1s, slowly decay throttle toward neutral
  bool failsafe = (now - lastUpdate > 1000);
  if (failsafe){
    if (failsafeThrottle > 127) failsafeThrottle--;
    else if (failsafeThrottle < 127) failsafeThrottle++;
  }

  byte t = failsafe ? failsafeThrottle : data.throttle;
  int centered = int(t) - 127;
  if (abs(centered) < 5) centered = 0; // deadband around neutral

  int escUs = NEUTRAL_US;

  // === FORWARD LOGIC ===
  if (centered > 0){
    // Forward range scaled by speedMode
    int fwdMax = (data.speedMode==0)?FWD_SLOW_MAX:
                 (data.speedMode==1)?FWD_MED_MAX:FWD_FAST_MAX;

    escUs = map(t, 127, 255, NEUTRAL_US, fwdMax);
    changeState(RS_IDLE);  // any forward resets reverse logic
  }
  // === REVERSE LOGIC (true double-tap) ===
  else if (centered < 0){
    if (revState == RS_IDLE){
      escUs = BRAKE_US;        // first pull = brake
      changeState(RS_BRAKE);
    }
    else if (revState == RS_BRAKE){
      escUs = BRAKE_US;        // hold brake, must return to neutral
    }
    else if (revState == RS_WAITING_REV){
      escUs = BRAKE_US;        // next pull starts with brake again
      changeState(RS_REV);     // then enters reverse state
    }
    else if (revState == RS_REV){
      escUs = map(t, 127, 0, NEUTRAL_US, REV_FULL_US); // reverse modulation
    }
  }
  else {
    // Neutral
    escUs = NEUTRAL_US;
    if (revState == RS_BRAKE){
      changeState(RS_WAITING_REV); // saw neutral after brake
    }
    else {
      changeState(RS_IDLE);        // reset to idle
    }
  }

  // === OUTPUT ESC SIGNAL ===
  pwm.setPWM(ESC_CH, 0, usToTicks(escUs));

  // === OUTPUT STEERING ===
  int steerCenter = 1090;  // measured straight-ahead pulse
  int steerUs;
  if (STEERING_REVERSED) {
    steerUs = map(data.steering, 0,255, steerCenter+400, steerCenter-400);
  } else {
    steerUs = map(data.steering, 0,255, steerCenter-400, steerCenter+400);
  }
  steerUs = constrain(steerUs, steerCenter-400, steerCenter+400);
  pwm.setPWM(STR_CH,  0, usToTicks(steerUs));

  // === OUTPUT HEAD SWIVEL ===
  int headUs = map(data.headSwivel, 0,255, 1100,1900);
  pwm.setPWM(HEAD_CH, 0, usToTicks(headUs));

  // === DEBUG SERIAL OUTPUT ===
  Serial.print("Throttle: "); Serial.print(t);
  Serial.print(" | ESC µs: "); Serial.print(escUs);
  Serial.print(" | State: "); Serial.print((int)revState);
  Serial.print(" | Steering µs: "); Serial.print(steerUs);
  Serial.print(" | Head µs: "); Serial.println(headUs);
}
