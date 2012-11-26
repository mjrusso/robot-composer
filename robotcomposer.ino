#include <Servo.h>


template<int HSIZE>
class SensorHistory {
public:
  int datapts;
  int data[HSIZE];
  int wpos;
  int rpos;

  SensorHistory() {
    this->datapts = 0;
    this->wpos = 0;
    this->rpos = 0;
  };

  inline bool ready() {
    return this->datapts >= HSIZE;
  };

  inline int next_position(int currentPos) {
    return (currentPos + 1) % HSIZE;
  };

  inline int current_value() {
     if (this->datapts > 0)
	return this->data[this->rpos];
     return 0;
  };

  inline void add_point(int data) {
    if (!this->ready()) {
        this->datapts += 1;
    } else {
        this->rpos = next_position(this->rpos);
    }
    (this->data)[this->wpos] = data;
    this->wpos = next_position(this->wpos);
  };

  //
  inline void get_data(int* outbuf) {
    int* outp = outbuf;
    for (int* intp = this->data + this->rpos,
            * endp = this->data + HSIZE; intp < endp;)
            {
               *outp++ = *intp++;
            }
    for (int* intp = this->data,
            * endp = this->data + this->rpos;
            intp <  endp; )
        {
            *outp++ = *intp++;
        }
  };

};


int do_get_avg_reading(int* buf, int size) {
    // Gets an average reading for sensors
};

template<int HSIZE>
class AveragingSensor {
public:
    SensorHistory<HSIZE>& m_hist;

    AveragingSensor(SensorHistory<HSIZE>& hist)
    : m_hist(hist)
    {};

    inline int get_average() {
        int baseline = m_hist.data[0];
        if (m_hist.ready()) {
            int delta_sum = 0;
            for (int i = 1; i < HSIZE; ++i) {
               delta_sum += (m_hist.data[i] - baseline);
            }
            return baseline + delta_sum / HSIZE;
        } else {
            return baseline;
        }
    };
};

template<int HSIZE>
class AvgTrigger {
public:
  AveragingSensor<HSIZE>& m_hist;
  int m_pcdiff;
  int m_avg;
  AvgTrigger(int avg, int pcdiff, AveragingSensor<HSIZE>& hist)
    : m_hist(hist), m_pcdiff(pcdiff), m_avg(avg)
  {
  };

  inline bool is_active() {
    int current =  m_hist.get_average();
    int delta = (current - m_avg);
    int absavg = m_avg > 0? m_avg: - m_avg;
    delta = delta > 0? delta : -delta;
    if (delta * 100 >= absavg * m_pcdiff) {
      return true;
    }
    return false;
  };

};


template<int HSIZE>
class SensorChangeTrigger {
private:
    int m_dir;
    int m_pcdiff;
    SensorHistory<HSIZE> &m_hist;

public:
    SensorChangeTrigger(int direction, int pcdiff, SensorHistory<HSIZE> &hist)
    : m_dir(direction), m_pcdiff(pcdiff), m_hist(hist)
    {
    };

    inline void get_delta_averages(int& out_prev, int& out_next) {
        int buf[HSIZE];
        m_hist.get_data(buf);
        {
           int base_prev = buf[0];
           int base_next = buf[HSIZE/2];
           int deltas_prev = 0;
           int deltas_next = 0;
           for (int i = 0; i < HSIZE/2; ++i) {
                deltas_prev += (buf[i] - base_prev);
            }
          out_prev = base_prev + deltas_prev / (HSIZE/2);
          for (int i = HSIZE/2; i <HSIZE; ++i) {
                deltas_next += (buf[i] - base_next);
           }
          out_next = base_next + deltas_next / (HSIZE - HSIZE/2);
        }
    };

    inline bool is_active() {
        //1. Compute averages over half-ranges
        int avg_prev = 0;
        int avg_next = 0;
        get_delta_averages(avg_prev, avg_next);
        //2. Make triggering decisions!
        //2.1 Check percentage threshold
        int delta_avgs = avg_next - avg_prev;
        int abs_delta = delta_avgs > 0? delta_avgs : - delta_avgs;
        int absaprev = avg_prev > 0? avg_prev : - avg_prev;
        if ((abs_delta * 100) >= (absaprev * m_pcdiff)) {
            //2.2 Check direction
            if (0 != m_dir) {
                return ((delta_avgs > 0) && (m_dir > 0))
                        || ((delta_avgs < 0) && (m_dir < 0));
            } else {
                return true;
            }
        }
        return false;
    };
};

class LinearDecayingActuatorOutput {
public:
    int m_min;
    int m_max;
    float m_internal;
    float m_decay;

    LinearDecayingActuatorOutput(int min, int max, float decay, float start)
    : m_min(min), m_max(max), m_internal(start), m_decay(decay)
    {
    };

    // -1 <= val <= 1
    inline void reset_value(float val) {
        m_internal = val;
    };

    inline int tick() {
        int result = static_cast<int>((m_min + m_max) / 2.0
                                    + (m_max - m_min) / 2.0 * m_internal);
        if (result < m_min) result = m_min;
        else if (result > m_max) result = m_max;
        m_internal *= m_decay;
        if (m_internal > 1.0)
            m_internal = 1.0;
        return result;
    };

};




// Config
boolean KEYBOARD_ENABLED = false;
boolean LOG_COMMANDS = false;
boolean LOG_SENSOR_VALS = true;

// Sensor pin assignments -- digital

int buttonPin = 7;
int magButtonPin = 4;

// Sensor pin assignments -- analog
int irPin = 0;
int flexPinL = 2;
int flexPinR = 1;
int photoPinL = 4;
int photoPinR = 3;

int potentiometerPin = 2;

// Servo pin
int servoPinL = 6;
int servoPinR = 13;

// Motor pin assigments
int STBY = 10;
int MOTOR_A_PWM = 3;
int MOTOR_A_IN1 = 9;
int MOTOR_A_IN2 = 8;
int MOTOR_B_PWM = 5;
int MOTOR_B_IN1 = 11;
int MOTOR_B_IN2 = 12;

// Sensor readings
SensorHistory<10> irVals;
SensorHistory<10> flexValsL;
SensorHistory<10> flexValsR;
SensorHistory<10> photoValsL;
SensorHistory<10> photoValsR;

int irVal;
int buttonVal;
int magButtonVal;
int flexVal;
int potentiometerVal;

// Action condition triggers
//
AveragingSensor<10> irAvg(irVals);
SensorChangeTrigger<10> irCloser(1, 5, irVals);
SensorChangeTrigger<10> irFurther(-1, 5, irVals);

AveragingSensor<10> flexLeft(flexValsL);
AvgTrigger<10> flexLBent(-150, 20, flexLeft);

AveragingSensor<10> flexRight(flexValsR);
AvgTrigger<10> flexRBent(-180, 20, flexRight);

AveragingSensor<10> photoRight(photoValsR);
AvgTrigger<10> photoRTrigger(-550, 17, photoRight);

AveragingSensor<10> photoLeft(photoValsL);
AvgTrigger<10> photoLTrigger(-550, 17, photoLeft);

 //... TODO: and rest


// Servo setup
Servo servoL;
Servo servoR;
int servoPos = 0;

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

  // Setup the servo
  servoL.attach(servoPinL);
  servoR.attach(servoPinR);

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
  int flexValL = map(analogRead(flexPinL), 768, 853, 0, 90);
  int flexValR = map(analogRead(flexPinR), 768, 853, 0, 90);

  flexValsL.add_point(flexValL);
  flexValsR.add_point(flexValR);

  int photoValL = map(analogRead(photoPinL), 768, 853, 0, 90);
  int photoValR = map(analogRead(photoPinR), 768, 853, 0, 90);

  photoValsL.add_point(photoValL);
  photoValsR.add_point(photoValR);

  irVal = analogRead(irPin);
  irVals.add_point(irVal);

  /// ... TODO: Add rest of values

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

// void moveServo() {
//   for(servoPos = 0; servoPos < 180; servoPos += 1) {
//     myservo.write(servoPos);
//   }
//   for(servoPos = 180; servoPos>=1; servoPos-=1) {
//     myservo.write(servoPos);
//   }
// }

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


void forward() {
  move(1, 150, 2);
  move(2, 150, 1);
};

void backward() {
  move(1, 150, 1);
  move(2, 150, 2);
};

void rotateLeft() {
  move(1, 150, 1);
  move(2, 150, 1);
};

void rotateRight() {
  move(1, 150, 2);
  move(2, 150, 2);
};

void loop() {
  String logMsg = "";
  updateSensorVals(logMsg);
  // Ensure all sensors have warmed-up buffers of readings
  bool shortloop = !(irVals.ready()
                     //&& flexVals1.ready() && flexVals2.ready()
                     //    && photo1.ready() && photo2.ready()
    );

  if (shortloop) {
    delay(5);
    return;
  }

  bool motorKill = magButtonVal == 1;
  bool motorActive = false;

  if (buttonVal == 0) {
    //Serial.println("Reset!");
    send(String("v println"));
  }

  if (magButtonVal == 1) {
    // send(String("v reset 0"));
    KEYBOARD_ENABLED = false;
  } else {
    KEYBOARD_ENABLED = true;
    // send(String("v reset 1"));
  }

  //if (irAvg.get_average() <= 300) {
    // DO NOTHING
  if (irCloser.is_active()) {
    Serial.println("DBUG: Robot moving away (scared)");
    forward();
    send(String("bpm - 10"));
    motorActive = true;
  } else if (irFurther.is_active()) {
    Serial.println("DBUG: Robot moving closer (attracted)");
    send(String("bpm + 10"));
    backward();
    motorActive = true;
  }

  if (flexLBent.is_active()) {
    send(String("wobble + 0.1"));
    Serial.println("DBUG: FlexLeft! ");
    Serial.println(String(flexLeft.get_average()));
    rotateLeft();
    motorActive = true;
  } else if (flexRBent.is_active()) {
    send(String("wobble - 0.1"));
    Serial.println("DBUG: FlexRight! ");
    Serial.println(String(flexRight.get_average()));
    Serial.println(String(photoLeft.get_average()));
    Serial.println(String(photoRight.get_average()));
    rotateRight();
    motorActive = true;
  }

  if (photoLTrigger.is_active()) {
    Serial.println("DBUG: PhotoLeft! ");
    Serial.println(String(photoLeft.get_average()));
    send(String("note + 0.1"));
  }

  if (photoRTrigger.is_active()) {
    send(String("note - 0.1"));
    Serial.println("DBUG: PhotoRight! ");
    Serial.println(String(photoRight.get_average()));
  }

  if (motorKill || !motorActive)
    stopMotors();

  //Serial.println(photoLeft.get_average());
  //prepareMessage();
  //runMotor();
  //moveServo();
  //Serial.println(logMsg);
  delay(5);
}
