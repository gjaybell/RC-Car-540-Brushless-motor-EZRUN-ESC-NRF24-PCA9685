/*
  ===== TRANSMITTER: RC Remote with Trim + Head Swivel =====
  - Uses two analog sticks: A0 = throttle, A1 = steering
  - Two trim pots: A2 = steering trim, A3 = throttle trim
  - 3-position speed switch: A4
  - Head swivel control: A5 (potentiometer for servo)
  - Two buttons: D2 and D4
  - Transmits all data using nRF24L01 radio
*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// === nRF24L01 Setup ===
RF24 radio(7, 8);  // CE, CSN pins
const byte address[6] = "00001";

// === Analog Inputs ===
const int trimSteeringPin = A2;
const int trimThrottlePin = A3;
const int speedSwitchPin  = A4;   // 3-position switch for speed control
const int headSwivelPin   = A5;   // Potentiometer for swivel servo

// === Button Inputs ===
const int button1Pin = 2;
const int button2Pin = 4;

// === Data Structure Sent to Receiver (6 bytes total) ===
struct Data_Package {
  byte throttle;
  byte steering;
  byte button1;
  byte button2;
  byte speedMode;
  byte headSwivel;
};

Data_Package data;

void setup() {
  Serial.begin(9600);

  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  // Initialize default values
  data.throttle   = 127;
  data.steering   = 127;
  data.button1    = 1;
  data.button2    = 1;
  data.speedMode  = 2;    // Default to High speed
  data.headSwivel = 127;  // Centered

  // Initialize nRF24
  radio.begin();
  radio.setAutoAck(false);
  radio.setDataRate(RF24_250KBPS);  // More reliable at distance
  radio.setPALevel(RF24_PA_HIGH);
  radio.openWritingPipe(address);
  radio.stopListening();
}

void loop() {
  // === Read Analog Stick Inputs ===
  int rawThrottle = analogRead(A0);
  int rawSteering = analogRead(A1);
  int rawSwivel   = analogRead(headSwivelPin);

  // === Apply Trim Adjustments ===
  int trimThrottle = map(analogRead(trimThrottlePin), 0, 1023, -50, 50);
  int trimSteering = map(analogRead(trimSteeringPin), 0, 1023, -50, 50);

  data.throttle = constrain(map(rawThrottle, 0, 1023, 0, 255) + trimThrottle, 0, 255);
  data.steering = constrain(map(rawSteering, 0, 1023, 0, 255) + trimSteering, 0, 255);

  // === Head Swivel Mapping (0–255) ===
  data.headSwivel = map(rawSwivel, 0, 1023, 0, 255);

  // === Button States ===
  data.button1 = digitalRead(button1Pin);
  data.button2 = digitalRead(button2Pin);

  // === 3-Way Speed Switch (A4) ===
  int speedRaw = analogRead(speedSwitchPin);
  if (speedRaw > 800)      data.speedMode = 2;  // High
  else if (speedRaw > 400) data.speedMode = 1;  // Medium
  else                     data.speedMode = 0;  // Low

  // === Transmit ===
  bool sent = radio.write(&data, sizeof(data));

  // === Debug Output ===
  Serial.print("Throttle: ");    Serial.print(data.throttle);
  Serial.print(" | Steering: "); Serial.print(data.steering);
  Serial.print(" | Swivel: ");   Serial.print(data.headSwivel);
  Serial.print(" | Speed: ");    Serial.print(data.speedMode);
  Serial.print(" | Sent: ");     Serial.println(sent ? "OK" : "Failed");

  delay(50);  // Small delay between transmissions
}
