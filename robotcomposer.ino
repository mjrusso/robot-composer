#include <Servo.h>

#define MY_MAX_INT 1024
#define MY_MIN_INT -1024

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

  inline bool ready() {
    return m_hist.ready();
  };

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
class TimeStabilizingSensor {
public:
    AveragingSensor<HSIZE>& m_sensor;
    int m_clockIAt;
    long m_stabilize;
    long m_samples;
    float m_data;
    int m_max;
    int m_min;

    TimeStabilizingSensor(AveragingSensor<HSIZE>& sense, int clockIAt,
                          long stabilize_time)
      :
      m_sensor(sense), m_clockIAt(clockIAt), m_stabilize(stabilize_time),
      m_samples(0), m_data(0.0), m_max(MY_MIN_INT), m_min(MY_MAX_INT)
    {
    };

    void sample() {
        int sampl = m_sensor.get_average();
        if (this->ready())
          this->m_samples -= 1;

        this->m_data = ((float)(sampl + (this->m_data) * (this->m_samples)))
          / (++(this->m_samples));
        if (sampl > this->m_max)
            this->m_max = sampl;
        if (sampl < this->m_min)
            this->m_min = sampl;
    };

    int get_stable_reading() { return this->m_data; };
    int get_current_reading() { return m_sensor.get_average(); };

    int get_max_reading() {return this->m_max; };
    int get_min_reading() {return this->m_min; };


    int get_average() { return this->m_data; };
    bool ready() {
        return m_sensor.ready() && (this->m_samples * m_clockIAt >= m_stabilize);
    };

    void reset() {
        this->m_data = 0;
        this->m_samples = 0;
    };

};

float linear_map_1s(float val, float range_min, float avg, float range_max,
                    float* params)
{
  if (val > avg) {
    float r_size  = (range_max - avg);
    return min(1.0f, (val - avg) / r_size);
  } else {
    float r_size  = (avg - range_min);
    return max(-1.0f, (val - avg) / r_size);
  }
};

float linear_map_01(float val, float range_min, float avg, float range_max,
                    float* params)
{
  if (val > avg) {
    float delta = 2*(range_max - avg);
    return min(1.0f, 0.5f + (avg - val) / delta);
  } else {
    float delta = 2*(avg - range_min);
    return max(0.0f, (val - avg) / delta);
  }
};

float exp_map_am(float val, float range_min, float avg, float range_max,
                 float* params)
{
  float dx = range_max - range_min;
  float thresh = 0.05f;
  float lambda = -(log(thresh))/dx;
  return exp(- lambda * (range_max - val));
};

float log_map_am(float val, float range_min, float avg, float range_max,
                 float* params)
{
  float lambda = - log(0.05);
  float y = (range_max - val) / (range_max - range_min);
  return -log(y) / lambda;
};

typedef float sensor_transform_t (float, float, float, float, float*);

template<int HSIZE>
class RelativeSensor {
public:
    TimeStabilizingSensor<HSIZE>& m_sensor;
    sensor_transform_t* m_transform;
    float* m_fparams;

    RelativeSensor(TimeStabilizingSensor<HSIZE>& sense, sensor_transform_t* trans, float* extra_params)
        : m_sensor(sense), m_transform(trans), m_fparams(extra_params)
    {
    };

  void sample() {
    return m_sensor.sample();
  }

    bool ready() {
        return m_sensor.ready();
    };

    float get_relative_value() {
       return (*m_transform) (m_sensor.get_current_reading(),
                              m_sensor.get_min_reading(),
                              m_sensor.get_average(),
                              m_sensor.get_max_reading(),
                              m_fparams);
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

    LinearDecayingActuatorOutput(int min, int max, float start)
    : m_min(min), m_max(max), m_internal(start), m_decay(0)
    {
    };

    // -1 <= val <= 1
  inline void reset_value(float val, float decay) {
        m_internal = val;
        m_decay = 0;
    };

    inline int tick() {
        int result = static_cast<int>((m_min + m_max) / 2.0
                                    + (m_max - m_min) / 2.0 * m_internal);
        if (result < m_min) result = m_min;
        else if (result > m_max) result = m_max;
        m_internal *= m_decay;
        if (m_internal > 1.0)
            m_internal = 1.0;
        if (m_internal < -1.0)
          m_internal = -1.0;
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
TimeStabilizingSensor<10> irStab(irAvg, 5, 1000L*20);
RelativeSensor<10> irRel(irStab, &exp_map_am, NULL);

SensorChangeTrigger<10> irCloser(1, 5, irVals);
SensorChangeTrigger<10> irFurther(-1, 5, irVals);

AveragingSensor<10> flexLeft(flexValsL);
TimeStabilizingSensor<10> flexLStab(flexLeft ,5, 1000L*20);
RelativeSensor<10> flexLRel(flexLStab, &linear_map_1s, NULL);

AveragingSensor<10> flexRight(flexValsR);
TimeStabilizingSensor<10> flexRStab(flexRight, 5, 1000L*20);
RelativeSensor<10> flexRRel(flexRStab, &linear_map_1s, NULL);

AveragingSensor<10> photoRight(photoValsR);
TimeStabilizingSensor<10> photoRStab(photoRight, 5, 1000L*20);
RelativeSensor<10> photoRRel(photoRStab, &linear_map_1s, NULL);

AveragingSensor<10> photoLeft(photoValsL);
TimeStabilizingSensor<10> photoLStab(photoLeft, 5, 1000L*20);
RelativeSensor<10> photoLRel(photoLStab, &linear_map_1s, NULL);

AvgTrigger<10> flexRBent(-180, 20, flexRight);
AvgTrigger<10> flexLBent(-150, 20, flexLeft);
AvgTrigger<10> photoRTrigger(-550, 17, photoRight);
AvgTrigger<10> photoLTrigger(-550, 17, photoLeft);

 //... TODO: and rest

LinearDecayingActuatorOutput motor_left(-150, 150, 0);
LinearDecayingActuatorOutput motor_right(-150, 150, 0);

void engines() {
  int leftMotor = motor_left.tick();
  int rightMotor = motor_right.tick();
  int leftDir = leftMotor > 0? 1 : 2;
  int rightDir = rightMotor > 0? 2 : 1;
  move(1, abs(leftMotor), leftDir);
  move(2, abs(rightMotor), rightDir);
}

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
  // move(1, 150, 1);
  // move(2, 150, 2);
  motor_left.reset_value(1, 0.9995);
  motor_right.reset_value(1, 0.9995);
};

void backward() {
  // move(1, 150, 2);
  // move(2, 150, 1);
  motor_left.reset_value(-1, 0.9995);
  motor_right.reset_value(-1, 0.999);
};

void rotateLeft() {
  motor_left.reset_value(1, 0.9995);
  motor_right.reset_value(-1, 0.9995);

  // move(1, 150, 1);
  // move(2, 150, 1);
};

void rotateRight() {
  motor_left.reset_value(-1, 0.9995);
  motor_right.reset_value(1, 0.9995);
  // move(1, 150, 2);
  // move(2, 150, 2);
};


int clockDiv = 0;
void loop() {
  String logMsg = "";
  updateSensorVals(logMsg);
  // Ensure all sensors have warmed-up buffers of readings
  bool shortloop = !(irRel.ready()
                     //&& flexVals1.ready() && flexVals2.ready()
                     //    && photo1.ready() && photo2.ready()
    );

  if (shortloop) {
    if (irAvg.ready()) irRel.sample();
    if (flexLeft.ready()) flexLRel.sample();
    if (flexRight.ready()) flexRRel.sample();
    if (photoLeft.ready()) photoLRel.sample();
    if (photoRight.ready()) photoRRel.sample();
    delay(5);
    return;
  }

  // Flex sensors are bad, so keep sampling
  flexLRel.sample();
  flexRRel.sample();

      // Serial.println(String("Stabilized IR:"));
      // Serial.println(String(irStab.get_current_reading()));
      // Serial.println(String((int) (irRel.get_relative_value() * 1000)));
      // Serial.println(String("Stabilized Flexleft:"));
      // Serial.println(String(flexLStab.get_current_reading()));
      // Serial.println(String((int) (flexLRel.get_relative_value() * 1000)));
      // Serial.println(String("Stabilized Flexright:"));
      // Serial.println(String(flexRStab.get_current_reading()));
      // Serial.println(String((int) (flexRRel.get_relative_value() * 1000)));
      // Serial.println(String("Stabilized Photoleft:"));
      // Serial.println(String(photoLStab.get_current_reading()));
      // Serial.println(String((int) (photoLRel.get_relative_value() * 1000)));
      // Serial.println(String("Stabilized Photoright:"));
      // Serial.println(String(photoRStab.get_current_reading()));
      // Serial.println(String((int) (photoRRel.get_relative_value() * 1000)));

      // Serial.println(String(""));
      // delay(1000);
      // return;
  bool motorKill = magButtonVal == 1;
  bool motorActive = false;

  if (buttonVal == 0) {
    Serial.println("Reset!");
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

  if (irRel.get_relative_value() > 0.5) {
    // if (clockDiv % 30 == 0)
    //   Serial.println("DBUG: Robot moving away (scared)");
    backward();
    if (clockDiv % 30 == 0)
      send(String("bpm + 10"));
    motorActive = true;
  } else if (irRel.get_relative_value() > 0.26) {
    // if (clockDiv % 60 == 0)
    //   Serial.println("DBUG: Robot moving closer (attracted)");
    forward();
    if (clockDiv % 30 == 0)
      send(String("bpm - 10"));
    motorActive = true;
  }

  if (abs(flexLRel.get_relative_value()) > 0.7f) {
    //if (clockDiv % 60 == 0)
    //  Serial.println("DBUG: Robot rotating left!");

    rotateLeft();
    if (clockDiv % 10 == 0)
      send(String("wobble - 0.1"));

    motorActive = true;
  } else if (abs(flexRRel.get_relative_value()) > 0.7f) {
    //if (clockDiv % 60 == 0)
    //  Serial.println("DBUG: Robot rotating right!");

    rotateRight();
    if (clockDiv % 10 == 0)
      send(String("wobble + 0.1"));

    motorActive = true;
  }

  if (photoLRel.get_relative_value() < -0.4) {
    if (clockDiv % 10 == 0)
        send(String("note - 0.4"));
  }

  if (photoRRel.get_relative_value() < -0.4) {
    if (clockDiv % 10 == 0)
        send(String("note + 0.4"));
  }

/*
  if (irCloser.is_active()) {
    //Serial.println("DBUG: Robot moving away (scared)");
    forward();
    send(String("bpm - 10"));
    motorActive = true;
  } else if (irFurther.is_active()) {
    //Serial.println("DBUG: Robot moving closer (attracted)");
    send(String("bpm + 10"));
    backward();
    motorActive = true;
  }
*/

  // if (flexLBent.is_active()) {
  //   send(String("wobble + 0.1"));
  //   Serial.println("DBUG: FlexLeft! ");
  //   Serial.println(String(flexLeft.get_average()));
  //   rotateLeft();
  //   motorActive = true;
  // } else if (flexRBent.is_active()) {
  //   send(String("wobble - 0.1"));
  //   Serial.println("DBUG: FlexRight! ");
  //   Serial.println(String(flexRight.get_average()));
  //   Serial.println(String(photoLeft.get_average()));
  //   Serial.println(String(photoRight.get_average()));
  //   rotateRight();
  //   motorActive = true;
  // }

  // if (photoLTrigger.is_active()) {
  //   Serial.println("DBUG: PhotoLeft! ");
  //   Serial.println(String(photoLeft.get_average()));
  //   send(String("note + 0.1"));
  // }

  // if (photoRTrigger.is_active()) {
  //   send(String("note - 0.1"));
  //   Serial.println("DBUG: PhotoRight! ");
  //   Serial.println(String(photoRight.get_average()));
  // }

  if (motorKill)
    stopMotors();
  else
    engines();


  //Serial.println(photoLeft.get_average());
  //prepareMessage();
  //runMotor();
  //moveServo();
  //Serial.println(logMsg);
  clockDiv = (clockDiv + 1) % 200;
  delay(5);

}
