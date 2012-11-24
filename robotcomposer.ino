#include <Servo.h>

// Config
boolean KEYBOARD_ENABLED = false;
boolean LOG_COMMANDS = false;
boolean LOG_SENSOR_VALS = true;

// Misc. pin assignments
int ledPin = 13;

// Sensor pin assignments -- digital
int buttonPin = 7;
int magButtonPin = 4;

// Sensor pin assignments -- analog
int irPin = 0;
int flexPin = 1;
int potentiometerPin = 2;

// Servo pin
int servoPin = 6;

// Motor pin assigments
int STBY = 10;
int MOTOR_A_PWM = 3;
int MOTOR_A_IN1 = 9;
int MOTOR_A_IN2 = 8;
int MOTOR_B_PWM = 5;
int MOTOR_B_IN1 = 11;
int MOTOR_B_IN2 = 12;

// Sensor readings
int irVal;
int buttonVal;
int magButtonVal;
int flexVal;
int potentiometerVal;

// Servo setup
Servo myservo;
int servoPos = 0;

void setup() {
  // Digital pin setup
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(magButtonPin, INPUT);

  // Digigal pin setup -- motors
  pinMode(STBY, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Setup the servo
  myservo.attach(servoPin);

  // Initialize the the button pins to the HIGH position.
  // The button is ON when we read a LOW voltage.
  digitalWrite(buttonPin, HIGH);
  digitalWrite(magButtonPin, HIGH);

  // Serial Baud Rate setup
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for Leonardo only.
  }

  // Keyboard setup
  Keyboard.begin();
}

void updateSensorVals(String &logMsg) {
  // Analog
  flexVal = map(analogRead(flexPin), 768, 853, 0, 90);
  potentiometerVal = analogRead(potentiometerPin);
  irVal = analogRead(irPin);

  // Digital
  buttonVal = digitalRead(buttonPin);
  magButtonVal = digitalRead(magButtonPin);

  if (LOG_SENSOR_VALS) {
    logMsg += "\n\n-- Sensor Values --";
    logMsg += "\nFlex: ";
    logMsg += flexVal;
    logMsg += "\nPontentiometer: ";
    logMsg += potentiometerVal;
    logMsg += "\nIR: ";
    logMsg += irVal;
    logMsg += "\nButton: ";
    logMsg += buttonVal;
    logMsg += "\nMagButton: ";
    logMsg += magButtonVal;
  }
}

void prepareMessage() {
  if (buttonVal == LOW) send(String("wobble + 1"));
  if (magButtonVal == LOW) send(String("wobble reset 1"));
}

void moveServo() {
  for(servoPos = 0; servoPos < 180; servoPos += 1) {
    myservo.write(servoPos);
  }
  for(servoPos = 180; servoPos>=1; servoPos-=1) {
    myservo.write(servoPos);
  }
}

void move(int motor, int speed, int direction) {
  //Move specific motor at speed and direction
  //motor: 0 for B 1 for A
  //speed: 0 is off, and 255 is full speed
  //direction: 0 clockwise, 1 counter-clockwise

  digitalWrite(STBY, HIGH); //disable standby

  boolean inPin1 = LOW;
  boolean inPin2 = HIGH;

  if (direction == 1) {
    inPin1 = HIGH;
    inPin2 = LOW;
  }

  if (motor == 1) {
    digitalWrite(MOTOR_A_IN1, inPin1);
    digitalWrite(MOTOR_A_IN2, inPin2);
    analogWrite(MOTOR_A_PWM, speed);
  }
  else {
    digitalWrite(MOTOR_B_IN1, inPin1);
    digitalWrite(MOTOR_B_IN2, inPin2);
    analogWrite(MOTOR_B_PWM, speed);
  }
}

void runMotor() {
  move(1, 100, 2); //motor 1, full speed, left
  move(2, 255, 1); //motor 2, full speed, left
}

void stopMotors() {
  digitalWrite(STBY, LOW);
}

void send(const String &msg) {
  if (LOG_COMMANDS) Serial.println(msg);
  if (KEYBOARD_ENABLED) Keyboard.println(msg);
}

void loop() {
  String logMsg = "";
  updateSensorVals(logMsg);
  prepareMessage();
  runMotor();
  moveServo();
  Serial.println(logMsg);
  delay(500);
}
