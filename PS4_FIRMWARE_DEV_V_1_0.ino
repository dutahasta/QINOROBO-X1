/*   [Motor 1] ---- [Motor 2]
        |               |
        |               |
        |               |
     [Motor 3] ---- [Motor 4]
*/
#include <PS4Controller.h>
#include <FastLED.h>

// Define motor pins for Motor 1 (Front Left)
#define MOTOR1_IN1 12
#define MOTOR1_IN2 23

// Define motor pins for Motor 2 (Front Right)
#define MOTOR2_IN1 17
#define MOTOR2_IN2 16

// Define motor pins for Motor 3 (Back Left)
#define MOTOR3_IN1 26
#define MOTOR3_IN2 25

// Define motor pins for Motor 4 (Back Right)
#define MOTOR4_IN1 14 //Must Pull-down 10K
#define MOTOR4_IN2 27

#define LED_PIN    15
#define NUM_LEDS   4

#define MAX_MOTOR_SPEED 225
#define MIN_INPUT_VALUE 20

//For connection esp32 with joystick PS4
unsigned long lastReconnectAttempt = 0;
const unsigned long RECONNECT_INTERVAL = 5000; 

CRGB leds[NUM_LEDS];
bool policeMode = false;

void proccessPS4Input() {
  if (PS4.isConnected()) {
    int yAxisValue = PS4.data.analog.stick.ly;  // Left stick - y axis - forward/backward car movement
    int xAxisValue = PS4.data.analog.stick.rx;  // Right stick - x axis - left/right car movement

    //Data for serial monitor
    Serial.print("yAxisValue: ");
    Serial.print(yAxisValue);

    Serial.print(", xAxisValue: ");
    Serial.println(xAxisValue);

    int leftMotorSpeed = yAxisValue;
    int rightMotorSpeed = yAxisValue;
    //Turn left & Right
    if (xAxisValue > MIN_INPUT_VALUE){
      leftMotorSpeed = -xAxisValue;
      rightMotorSpeed = xAxisValue;
    }
    else if (xAxisValue < -MIN_INPUT_VALUE){
      leftMotorSpeed = -xAxisValue;
      rightMotorSpeed = xAxisValue;
    }
    else {
      // Move forward/backward
      if (abs(yAxisValue) > MIN_INPUT_VALUE) {
        leftMotorSpeed = yAxisValue;
        rightMotorSpeed = yAxisValue;
      } else {
        leftMotorSpeed = 0;
        rightMotorSpeed = 0;
      }
    }
    if (PS4.data.button.l2)// the robots moving left
    {
      moveLeft();
      Serial.println("L2 button pressed: Moving Left");
    }
    else if (PS4.data.button.r2)// the robots moving right
    {
      moveRight();
      Serial.println("R2 button pressed: Moving Right");
    }
    else{
    
    leftMotorSpeed =  constrain(leftMotorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
    rightMotorSpeed = constrain(rightMotorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);

    leftMotorSpeed =  map(leftMotorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, -255, 255);
    rightMotorSpeed = map(rightMotorSpeed, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED, -255, 255);

    // Set motor speeds and directions
    setMotorSpeed(MOTOR1_IN1, MOTOR1_IN2, leftMotorSpeed);
    setMotorSpeed(MOTOR3_IN1, MOTOR3_IN2, leftMotorSpeed);
    setMotorSpeed(MOTOR2_IN1, MOTOR2_IN2, rightMotorSpeed);
    setMotorSpeed(MOTOR4_IN1, MOTOR4_IN2, rightMotorSpeed);

    }
    if (PS4.data.button.l1) {
      policeMode = true;
      Serial.println("Police mode : ON");
    }
    if (PS4.data.button.r1){
      policeMode = false;
      Serial.println("Police mode : OFF");
      FastLED.clear();
      FastLED.show();
    }
    
    else {
      Serial.println("Controller is not connected");
    }
  }
}

void setMotorSpeed(int in1Pin, int in2Pin, int speed) {
  if (speed > 0) {
    analogWrite(in1Pin, speed);
    analogWrite(in2Pin, 0);
  } else {
    analogWrite(in1Pin, 0);
    analogWrite(in2Pin, -speed);
  }
}

void stopMotor(int in1Pin, int in2Pin) {
  analogWrite(in1Pin, 0);
  analogWrite(in2Pin, 0);
}

void onConnect() {
  Serial.println("Connected!");
}

void onDisConnect() {
  Serial.println("Disconnected !");
  // Stop motors when disconnected
  stopMotor(MOTOR1_IN1, MOTOR1_IN2);
  stopMotor(MOTOR2_IN1, MOTOR2_IN2);
  stopMotor(MOTOR3_IN1, MOTOR3_IN2);
  stopMotor(MOTOR4_IN1, MOTOR4_IN2);
}

void setup() {
  Serial.begin(115200);
  
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);
  PS4.begin("d0:ab:d5:93:45:bc"); // Change with your MAC Address Joystick
  Serial.println("Ready.");

  pinMode(MOTOR1_IN1, OUTPUT);
  pinMode(MOTOR1_IN2, OUTPUT);
  pinMode(MOTOR2_IN1, OUTPUT);
  pinMode(MOTOR2_IN2, OUTPUT);
  pinMode(MOTOR3_IN1, OUTPUT);
  pinMode(MOTOR3_IN2, OUTPUT);
  pinMode(MOTOR4_IN1, OUTPUT);
  pinMode(MOTOR4_IN2, OUTPUT);

  FastLED.addLeds<NEOPIXEL, LED_PIN>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();

}

void loop() {
  if (PS4.isConnected()) {
    proccessPS4Input();
  } else {
    unsigned long now = millis();
    if (now - lastReconnectAttempt > RECONNECT_INTERVAL) {
      Serial.println("Attempting to reconnect...");
      PS4.begin("d0:ab:d5:93:45:bc");
      lastReconnectAttempt = now;
    }
  }
  if (policeMode){
    police();
  }
  else{
    FastLED.clear();
    FastLED.show();
  }
}

/*{
  PS4.attach(proccessPS4Input);
  unsigned long now = millis();
    if (now - lastReconnectAttempt > RECONNECT_INTERVAL){
      Serial.println("Attempting to reconnect...");
      PS4.begin("d0:ab:d5:93:45:bc");
      lastReconnectAttempt = now;    
    }
}*/

void moveRight(void){
  //This function setting the motor for move right with MAX_MOTOR_SPEED define for direction
  setMotorSpeed(MOTOR1_IN1, MOTOR1_IN2, MAX_MOTOR_SPEED);
  setMotorSpeed(MOTOR2_IN1, MOTOR2_IN2, -MAX_MOTOR_SPEED);
  setMotorSpeed(MOTOR3_IN1, MOTOR3_IN2, -MAX_MOTOR_SPEED);
  setMotorSpeed(MOTOR4_IN1, MOTOR4_IN2, MAX_MOTOR_SPEED);
}
void moveLeft(void){
  //This function setting the motor for move left with MAX_MOTOR_SPEED define for direction
  setMotorSpeed(MOTOR1_IN1, MOTOR1_IN2, -MAX_MOTOR_SPEED);
  setMotorSpeed(MOTOR2_IN1, MOTOR2_IN2, MAX_MOTOR_SPEED);
  setMotorSpeed(MOTOR3_IN1, MOTOR3_IN2, MAX_MOTOR_SPEED);
  setMotorSpeed(MOTOR4_IN1, MOTOR4_IN2, -MAX_MOTOR_SPEED);
}
//Led Police features function
void police() {
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 255); // Blue
  }
  FastLED.show();
  delay(40);

  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(255, 0, 0); // Red
  }
  FastLED.show();
  delay(40);
}
