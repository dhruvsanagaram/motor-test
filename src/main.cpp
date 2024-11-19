// #include <Arduino.h>
// #define QUAD_ENCODER_A 12
// #define QUAD_ENCODER_B 14

// // put function declarations here:

// void setup()
// {
//   // put your setup code here, to run once:
//   Serial.begin(9600);
//   pinMode(QUAD_ENCODER_A, INPUT);
//   pinMode(QUAD_ENCODER_B, INPUT);
// }

// void loop()
// {
//   auto encoder_a = digitalRead(QUAD_ENCODER_A);
//   auto encoder_b = digitalRead(QUAD_ENCODER_B);
//   Serial.println("ENCODER A: ");
//   Serial.println(encoder_a);
//   Serial.println("ENCODER B: ");
//   Serial.println(encoder_b);
//   delay(300);
// }

#include <Arduino.h>

#define QUAD_ENCODER_A 12
#define QUAD_ENCODER_B 14

volatile int position = 0; // Encoder position
volatile int direction = 0; // 1 for clockwise, -1 for counterclockwise

void updateEncoder() {
  // Read the encoder signals
  int a = digitalRead(QUAD_ENCODER_A);
  int b = digitalRead(QUAD_ENCODER_B);

  // Determine direction based on state transitions
  if (a == HIGH && b == LOW) direction = 1; // Clockwise
  else if (a == LOW && b == HIGH) direction = -1; // Counterclockwise

  // Update position
  position += direction;
}

void setup() {
  Serial.begin(9600);
  pinMode(QUAD_ENCODER_A, INPUT_PULLUP);
  pinMode(QUAD_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_B), updateEncoder, CHANGE);
}

void loop() {
  Serial.print("Position: ");
  Serial.println(position);
  delay(500);
}


