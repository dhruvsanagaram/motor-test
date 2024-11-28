#include <Arduino.h>

#define QUAD_ENCODER_A 27
#define QUAD_ENCODER_B 26


// #define MOTOR_ENA 32
// #define MOTOR_IN1 16
// #define MOTOR_IN2 17

#define BR_MOTOR_IN1 32
#define BR_MOTOR_IN2 16

#define BL_MOTOR_IN3 17
#define BL_MOTOR_IN4 18

volatile int position = 0; // Encoder position
volatile int direction = 0; // 1 for clockwise, -1 for counterclockwise
volatile bool printFlag = false;
int speed = 0;
const int pwmChannel = 0;
const int pwmFrequency = 5000;
const int pwmResolution = 12;

const int pwmChannel_2 = 0;

void updateEncoder() {
  int a = digitalRead(QUAD_ENCODER_A);
  int b = digitalRead(QUAD_ENCODER_B);


  // if (a == HIGH && b == LOW) direction = 1; // Clockwise
  // else if (a == LOW && b == HIGH) direction = -1; // Counterclockwise

  if(a == HIGH && b == LOW){
    position += 1;
  }

}

void setMotor(int speed, int dir){
  ledcWrite(pwmChannel, abs(speed));

  digitalWrite(BR_MOTOR_IN2, HIGH);
  // digitalWrite(MOTOR_IN2, LOW);

  ledcWrite(pwmChannel_2, abs(speed));
  digitalWrite(BL_MOTOR_IN4, HIGH);
}

void setup() {
  Serial.begin(9600);
  pinMode(QUAD_ENCODER_A, INPUT_PULLUP);
  pinMode(QUAD_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_B), updateEncoder, CHANGE);

  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(BR_MOTOR_IN1, pwmChannel);
  pinMode(BR_MOTOR_IN2, OUTPUT);
  // pinMode(MOTOR_IN2, OUTPUT);

  ledcSetup(pwmChannel_2, pwmFrequency, pwmResolution);
  ledcAttachPin(BL_MOTOR_IN3, pwmChannel_2);
  pinMode(BL_MOTOR_IN4, OUTPUT);

  setMotor(0,0);
}

void loop() {
  int maxSpeed = 4095;        // Max PWM value for 8-bit resolution
  int steps = 30;            // Number of steps to vary speed
  int delayPerStep = 500;    // Delay per step (500ms = 0.5s per step)
  int increment = maxSpeed / steps; // Incremental change in speed

  for (int i = 0; i <= steps; i++) {
    speed = i * increment;
    setMotor(speed, 1); // Forward direction
    delay(delayPerStep);
  }

  for (int i = steps; i >= 0; i--) {
    speed = i * increment;
    setMotor(speed, 1);
    // Serial.print("Decreasing Speed: ");
    // Serial.println(speed);
    delay(delayPerStep);
  }

  // setMotor(2500, 1);

  Serial.print("Position: ");
  Serial.println(position);
  Serial.print("Direction: ");
  Serial.println(direction);
  delay(500);
}


