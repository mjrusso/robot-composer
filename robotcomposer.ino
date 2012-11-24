// Config
boolean SERIAL_ENABLED = true;
boolean KEYBOARD_ENABLED = true;

// Misc. pin assignments
int ledPin = 13;

// Sensor pin assignments
int irPin = 0;
int buttonPin = 7;
int magButtonPin = 4;

// Motor pin assigments
int STBY = 10;
int MOTOR_A_PWM = 3;
int MOTOR_A_IN1 = 9;
int MOTOR_A_IN2 = 8;
int MOTOR_B_PWM = 5;
int MOTOR_B_IN1 = 11;
int MOTOR_B_IN2 = 12;

// Sensor input values
int irVal;
int buttonVal;
int magButtonVal;

// Messages
String startMsg;
String buttonMsg;
String magButtonMsg;

void setup() {
  // Pin setup
  pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  pinMode(magButtonPin, INPUT);

  // Pin setup -- motors
  pinMode(STBY, OUTPUT);
  pinMode(MOTOR_A_PWM, OUTPUT);
  pinMode(MOTOR_A_IN1, OUTPUT);
  pinMode(MOTOR_A_IN2, OUTPUT);
  pinMode(MOTOR_B_PWM, OUTPUT);
  pinMode(MOTOR_B_IN1, OUTPUT);
  pinMode(MOTOR_B_IN2, OUTPUT);

  // Initialize the the button pins to the HIGH position.
  // The button is ON when we read a LOW voltage.
  digitalWrite(buttonPin, HIGH);
  digitalWrite(magButtonPin, HIGH);

  // Serial Baud Rate setup
  Serial.begin(9600);
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for Leonardo only.
  }

  Keyboard.begin();

  startMsg = String (">  ");
  buttonMsg = String("(but)");
  magButtonMsg = String("(mag)");
}

void updateSensorVals() {
  irVal = analogRead(irPin);
  buttonVal = digitalRead(buttonPin);
  magButtonVal = digitalRead(magButtonPin);
}

void prepareMessage(String &msg) {
  msg += startMsg;
  if (buttonVal == LOW) {
    send(String("wobble + 1"));
  }
  if (magButtonVal == LOW) {
    send(String("wobble reset 1"));
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
  if (SERIAL_ENABLED) Serial.println(msg);
  if (KEYBOARD_ENABLED) Keyboard.println(msg);
}

void loop() {
  String msg("");
  updateSensorVals();
  prepareMessage(msg);
  /* if (msg.length() > 0) { */
  /*   Serial.println(msg); */
  /* } */
  runMotor();
  delay(500);
}
