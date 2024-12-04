#include <Arduino.h>
#include <BluetoothSerial.h>

#define QUAD_ENCODER_A 27
#define QUAD_ENCODER_B 26

#define BL_QUAD_ENCODER_A 34
#define BL_QUAD_ENCODER_B 35

#define BR_MOTOR_IN1 32
#define BR_MOTOR_IN2 16

#define BL_MOTOR_IN3 17
#define BL_MOTOR_IN4 18

#define ADC_PIN 13
#define ADC_PIN2 14

volatile int position = 0; // Encoder position
volatile int bl_position = 0; 
volatile int direction = 0; // 1 for clockwise, -1 for counterclockwise

const int pwmChannel = 0;
const int pwmFrequency = 5000;
const int pwmResolution = 12;
const int pwmChannel_2 = 1;
int targetDistance = 1000;    // Target distance in encoder counts
// int maxSpeed = 3000;
int maxSpeed = 100;
// int accelerationStep = 100;    // Incremental speed step for acceleration and deceleration
int accelerationStep = 10;
int delayPerStep = 10;        // Delay per step during acceleration and deceleration (ms)

int stops_done = 0;

int waypoints = 4;

float alen = 0.75;

uint8_t speed;
uint8_t bl_speed;

float increment;

volatile float coord1;
volatile float coord2;

BluetoothSerial SerialBT;

void IRAM_ATTR updateEncoder() {
  int a = digitalRead(QUAD_ENCODER_A);
  int b = digitalRead(QUAD_ENCODER_B);

  int c = digitalRead(BL_QUAD_ENCODER_A);
  int d = digitalRead(BL_QUAD_ENCODER_B);

  if(a == HIGH && b == LOW){
    position += 1;
  }

  if(c == HIGH && d == LOW){
    bl_position += 1;
  }
}


void setMotor(uint8_t speed, uint8_t bl_speed, uint8_t dir){
  Serial.println(speed);
  Serial.println(bl_speed);
  if(speed == 0){
    ledcWrite(pwmChannel, 0);
    digitalWrite(BR_MOTOR_IN1, LOW);
    digitalWrite(BR_MOTOR_IN2, LOW);
  }
  else{
    ledcWrite(pwmChannel, abs(speed));
    digitalWrite(BR_MOTOR_IN2, HIGH);
    // digitalWrite(BR_MOTOR_IN2, LOW);
  }

  if(bl_speed == 0){
    ledcWrite(pwmChannel_2, 0);
    digitalWrite(BL_MOTOR_IN3, LOW);
    digitalWrite(BL_MOTOR_IN4, LOW);
  }
  else{
    ledcWrite(pwmChannel_2, abs(bl_speed));
    digitalWrite(BL_MOTOR_IN4, HIGH);
    // digitalWrite(BL_MOTOR_IN4, LOW);
  }
}


void setup() {
  Serial.begin(9600);
  SerialBT.begin("ESP32-WROOM-Receiver", true);
  SerialBT.println("Waiting for Bluetooth data...");

  // Serial.begin(9600);
  pinMode(QUAD_ENCODER_A, INPUT_PULLUP);
  pinMode(QUAD_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(QUAD_ENCODER_B), updateEncoder, CHANGE);

  pinMode(BL_QUAD_ENCODER_A, INPUT_PULLUP);
  pinMode(BL_QUAD_ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BL_QUAD_ENCODER_A), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_QUAD_ENCODER_B), updateEncoder, CHANGE);


  ledcSetup(pwmChannel, pwmFrequency, pwmResolution);
  ledcAttachPin(BR_MOTOR_IN1, pwmChannel);
  pinMode(BR_MOTOR_IN2, OUTPUT);

  ledcSetup(pwmChannel_2, pwmFrequency, pwmResolution);
  ledcAttachPin(BL_MOTOR_IN3, pwmChannel_2);
  pinMode(BL_MOTOR_IN4, OUTPUT);

  setMotor(0,0,0);

  int analogValue = analogRead(ADC_PIN);
  int analogValue2 = analogRead(ADC_PIN2);

  int receivedValue = map(analogValue, 0, 4095, 0, 255);
  int receivedValue2 = map(analogValue2, 0, 4095, 0, 255);

  coord1 = receivedValue/255.0 * 5.0;
  coord2 = receivedValue2/255.0 * 5.0;

  float clen = (coord1+coord2)/2;
  float blen = sqrt(clen*clen-alen*alen);
  increment = blen/waypoints;
  // Serial.println(mid_clen);
}

void loop() {
  speed = 0;
  bl_speed = 0;
  // position = 0;
  // bl_position = 0;

  // speed = 100;
  // bl_speed = 0;
  // setMotor(speed, bl_speed, 1);
  // delay(5000);

  // speed = 0;
  // bl_speed = speed;
  // setMotor(speed, bl_speed, 1);
  // delay(5000);

  speed = 0;
  bl_speed = speed;
  setMotor(speed, bl_speed, 1);
  delay(1000);
  int remaining = waypoints - stops_done;
  int height_left = increment*remaining;
  if ((coord1+coord2)/2 < sqrt(height_left*height_left+alen*alen) + 0.2) {
    delay(10000);
    stops_done++;
  }
  else if (coord1+coord2 < 1.8) {
    delay(10000);
  }
  else if (coord2 > coord1 + 0.15) {
    speed = 0;
    bl_speed = 100;
    setMotor(speed, bl_speed, 1);
    Serial.println("Turning left");
    delay(200);
    speed = 80;
    bl_speed = speed;
    setMotor(speed, bl_speed, 1);
    Serial.println("Going straight");
    delay(2000);
  }
  else if (coord1 > coord2 + 0.15) {
    bl_speed = 0;
    speed = 100;
    setMotor(speed, bl_speed, 1);
    Serial.println("Turning right");
    delay(200);
    speed = 80;
    bl_speed = speed;
    setMotor(speed, bl_speed, 1);
    Serial.println("Going straight");
    delay(2000);
  }
  else {
    speed = 80;
    bl_speed = speed;
    setMotor(speed, bl_speed, 1);
    Serial.println("Going straight");
    delay(2000);
  }

  // Serial.println("Moving forward...");
  // while (position < targetDistance) {
  //   speed = map(position, 0, targetDistance, accelerationStep, maxSpeed); // Gradually increase speed
  //   bl_speed = speed;
  //   setMotor(speed, bl_speed, 1);
  //   delay(delayPerStep);
  //   // Serial.println(speed);
  //   Serial.print("Position: ");
  //   Serial.println(position);
  //   Serial.print("BL Position: ");
  //   Serial.println(bl_position);
  // }

  // Serial.println("Decelerating to stop...");
  // while (speed > 0) {
  //   speed -= accelerationStep;
  //   bl_speed -= accelerationStep;
  //   if (speed < 0) speed = 0;
  //   setMotor(speed, bl_speed, 1);
  //   delay(delayPerStep);
  // }

  // setMotor(0, 0, 1);
  // delay(1000);

  // Serial.println("Starting again...");
  // speed = 0;
  // bl_speed = 0;
  // position = 0;
  // bl_position = 0;
  // while (position < targetDistance) {
  //   speed = map(position, 0, targetDistance, accelerationStep, maxSpeed);
  //   bl_speed = speed;
  //   setMotor(speed, bl_speed, 1);
  //   delay(delayPerStep);
  // }

  // Serial.println("Decelerating to stop...");
  // while (speed > 0) {
  //   speed -= accelerationStep;
  //   bl_speed -= accelerationStep;
  //   if (speed < 0) speed = 0;
  //   setMotor(speed, bl_speed, 1);
  //   delay(delayPerStep);
  // }

  // setMotor(0, 0, 1);
  // delay(1000);


  //////////////////////////

  int analogValue = analogRead(ADC_PIN);
  int analogValue2 = analogRead(ADC_PIN2);

  // Convert the analog value to match the DAC range (0-255)
  int receivedValue = map(analogValue, 0, 4095, 0, 255);
  int receivedValue2 = map(analogValue2, 0, 4095, 0, 255);

  coord1 = receivedValue/255.0 * 5.0;
  coord2 = receivedValue2/255.0 * 5.0;

  // Print the received value for debugging
  Serial.println("Received value 1: " + String(coord1));
  Serial.println("Received value 2: " + String(coord2));

  // if (!SerialBT.connected()) {
  //   Serial.println("Attempting to connect to Makerfabs-Anchor...");
  //   if (SerialBT.connect("Makerfabs-Tag")) {
  //       Serial.println("Successfully connected to Makerfabs-Anchor!");
  //   } else {
  //       Serial.println("Failed to connect to Makerfabs-Anchor.");
  //   }
  // }

  // if (SerialBT.available()) {
  //   String receivedData = SerialBT.readStringUntil('\n'); // Read incoming data

  //   // Parse the distance if needed and use it in your application logic
  //   Serial.print("Parsed Distance String: ");
  //   Serial.println(receivedData);
  //   SerialBT.println("ACK");
  //   delay(1000);
  //   // if (receivedData.startsWith("Distance: ")) {
  //   //   Serial.print("Parsed Distance String: ");
  //   //   Serial.println(receivedData);
  //   //   // float distance = receivedData.substring(10).toFloat();
  //   //   // Serial.print("Parsed Distance: ");
  //   //   // Serial.println(distance);

  //   //   // SerialBT.println("ACK");
  //   //   // You can use this distance to adjust motor behavior
  //   //   // delay(500);
  //   // }
  // }
}


