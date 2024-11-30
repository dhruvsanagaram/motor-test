#include <Arduino.h>
#include <BluetoothSerial.h>

#define QUAD_ENCODER_A 27
#define QUAD_ENCODER_B 26

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
int targetDistance = 1000;    // Target distance in encoder counts
// int maxSpeed = 4095;          // Maximum PWM value (12-bit resolution)
int maxSpeed = 3000;
int accelerationStep = 50;    // Incremental speed step for acceleration and deceleration
int delayPerStep = 10;        // Delay per step during acceleration and deceleration (ms)

BluetoothSerial SerialBT;

void IRAM_ATTR updateEncoder() {
  int a = digitalRead(QUAD_ENCODER_A);
  int b = digitalRead(QUAD_ENCODER_B);

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
  SerialBT.begin("ESP32-WROOM-Receiver", true);
  SerialBT.println("Waiting for Bluetooth data...");
  // Serial.begin(9600);
  // pinMode(QUAD_ENCODER_A, INPUT_PULLUP);
  // pinMode(QUAD_ENCODER_B, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_A), updateEncoder, CHANGE);
  // attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_B), updateEncoder, CHANGE);

  // ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  // ledcAttachPin(BR_MOTOR_IN1, pwmChannel);
  // pinMode(BR_MOTOR_IN2, OUTPUT);
  // // pinMode(MOTOR_IN2, OUTPUT);

  // ledcSetup(pwmChannel_2, pwmFrequency, pwmResolution);
  // ledcAttachPin(BL_MOTOR_IN3, pwmChannel_2);
  // pinMode(BL_MOTOR_IN4, OUTPUT);

  // setMotor(0,0);
}

void loop() {
  // // Move forward to the target distance
  // int speed = 0;
  // position = 0; // Reset encoder position to start tracking movement
  // Serial.println("Moving forward...");
  // while (position < targetDistance) { // Accelerate until reaching the target distance
  //   // Serial.println('aldkjfaljasdlkdjfasldkjffa');
  //   speed = map(position, 0, targetDistance, accelerationStep, maxSpeed); // Gradually increase speed
  //   // Serial.println(speed);
  //   setMotor(speed, 1); // Set motor speed and forward direction
  //   delay(delayPerStep); // Delay for smooth acceleration
  //   Serial.println(speed);
  //   Serial.println(position);
  // }

  // // Decelerate smoothly to a stop
  // Serial.println("Decelerating to stop...");
  // while (speed > 0) { // Gradually reduce speed to 0
  //   speed -= accelerationStep;
  //   if (speed < 0) speed = 0; // Ensure speed does not go below 0
  //   setMotor(speed, 1); // Update motor speed
  //   delay(delayPerStep);
  // }

  // // Stop the motor for a moment
  // setMotor(0, 1);
  // delay(1000); // Pause before restarting the cycle

  // // Accelerate back to full speed and repeat the movement
  // Serial.println("Starting again...");
  // speed = 0;
  // position = 0; // Reset encoder position again
  // while (position < targetDistance) { // Accelerate until reaching the target distance
  //   speed = map(position, 0, targetDistance, accelerationStep, maxSpeed); // Gradually increase speed
  //   setMotor(speed, 1);
  //   delay(delayPerStep);
  // }

  // // Decelerate smoothly to stop again
  // Serial.println("Decelerating to stop...");
  // while (speed > 0) {
  //   speed -= accelerationStep;
  //   if (speed < 0) speed = 0; // Ensure speed does not go below 0
  //   setMotor(speed, 1);
  //   delay(delayPerStep);
  // }

  // // Pause before restarting the loop
  // setMotor(0, 1);
  // delay(1000); // Delay before the next cycle

  if (!SerialBT.connected()) {
    Serial.println("Attempting to connect to Makerfabs-Anchor...");
    if (SerialBT.connect("Makerfabs-Anchor")) {
        Serial.println("Successfully connected to Makerfabs-Anchor!");
    } else {
        Serial.println("Failed to connect to Makerfabs-Anchor.");
    }
  }

  // Serial.println(SerialBT.available());

  if (SerialBT.available()) {
    String receivedData = SerialBT.readStringUntil('\n'); // Read incoming data

    // Parse the distance if needed and use it in your application logic
    if (receivedData.startsWith("Distance: ")) {
      float distance = receivedData.substring(10).toFloat();
      Serial.print("Parsed Distance: ");
      Serial.println(distance);

      SerialBT.println("ACK");
      // You can use this distance to adjust motor behavior
      // delay(500);
    }
  }
}


