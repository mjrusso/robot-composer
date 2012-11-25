#include <Servo.h>

// Config
boolean KEYBOARD_ENABLED = false;
boolean LOG_COMMANDS = false;
boolean LOG_SENSOR_VALS = true;

// Sensor pin assignments -- digital
int buttonPin = 7;
int magButtonPin = 4;

// Sensor pin assignments -- analog
int irPin = 0;
int flexPin = 1;
int flexPin2 = 2;
int photoPin1 = 3;
int photoPin2 = 4;

// Servo pin
int servoPin = 6;
int servo2Pin = 13;

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
int flex2Val;
int photo1Val;
int photo2Val;

// Servo setup
Servo myservo;
Servo myservo2;
int servoPos = 0;
int servo2Pos = 0;

void setup() {
  // Digital pin setup
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

  // Setup the servos
  myservo.attach(servoPin);
  myservo.attach(servo2Pin);

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
  flex2Val = analogRead(flexPin2);
  photo1Val = analogRead(photoPin1);
  photo2Val = analogRead(photoPin2);
  irVal = analogRead(irPin);

  // Digital
  buttonVal = digitalRead(buttonPin);
  magButtonVal = digitalRead(magButtonPin);

  if (LOG_SENSOR_VALS) {
    logMsg += "\n\n-- Sensor Values --";
    logMsg += "\nFlex 1: ";
    logMsg += flexVal;
    logMsg += "\nFlex 2: ";
    logMsg += flex2Val;
    logMsg += "\nPhoto 1: ";
    logMsg += photo1Val;
    logMsg += "\nPhoto 2: ";
    logMsg += photo2Val;
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
  /* for(servoPos = 0; servoPos < 180; servoPos += 1) { */
  /*   myservo.write(servoPos); */
  /* } */
  /* for(servoPos = 180; servoPos>=1; servoPos-=1) { */
  /*   myservo.write(servoPos); */
  /* } */
  myservo.write(90);
}

void moveServo2() {
  /* for(servo2Pos = 0; servo2Pos < 180; servo2Pos += 1) { */
  /*   myservo2.write(servo2Pos); */
  /* } */
  /* for(servo2Pos = 180; servo2Pos>=1; servo2Pos-=1) { */
  /*   myservo2.write(servo2Pos); */
  /* } */
  myservo2.write(90);
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
  moveServo2();
  Serial.println(logMsg);
  delay(500);
}
