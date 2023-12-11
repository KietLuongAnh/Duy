#include "DFRobot_TFmini.h"
#include <Stepper.h>
/**
 * \brief Common Pin Define
 */
#define largeDirPin 2
#define largePin 3
#define STEPPER_PIN_1 8
#define STEPPER_PIN_2 9
#define STEPPER_PIN_3 10
#define STEPPER_PIN_4 11
#define PINRELAY 7

#define MODE2DMAPPING 1
#define MODE3DMAPPING 2

#define ANTI_CLOCKWISE 0
#define CLOCKWISE 1


/**
 * \brief set up global var
 */
int i = 0;
int pulse_per_round = 6400;  // Number of pulse to one round
double posX = 0;
double posY = 0;
double posY_inner = 0;
int timer_2_counter = 0;
int count_posY = 0;
bool measure_flag = false;  // To check the status of measure the distance of lidar
bool stop_flag = false;     // To check for interrupt timer to read data
int round_2D = 0;           /* Round to draw 2D Mapping */
bool runMotor = false;
/* Mode 0: Waiting Select Mode || Mode 1: 2D Mapping || Mode 2: 3D Mapping */
int modeWorking = 0;
static int motorNhoStepCnt = 0;
SoftwareSerial mySerial(0, 1);  // RX, TX

DFRobot_TFmini TFmini;
uint16_t distance;
unsigned long timer = 0;

void setup() {
  delay(5000);
  Serial.begin(115200);
  TFmini.begin(mySerial);
  /* Declare Pins As Output */
  pinMode(largeDirPin, OUTPUT);
  pinMode(largePin, OUTPUT);
  digitalWrite(largeDirPin, LOW);

  pinMode(PINRELAY, OUTPUT);
  digitalWrite(PINRELAY, LOW);

  // Light Sensor input signal
  pinMode(5, INPUT_PULLUP);

  // Motor nho control module output pin
  for (int i = 8; i <= 11; i++) {
    pinMode(i, OUTPUT);
  }
  // Set position of Motor nho to initial
  Init_MotorNho();

  cli();  // stop interrupts

  // set timer1 interrupt at 200Hz
  TCCR1A = 0;  // set entire TCCR1A register to 0
  TCCR1B = 0;  // same for TCCR1B
  TCNT1 = 0;   // initialize counter value to 0
  // set compare match register for 16000hz increments
  OCR1A = 9999;  // = (16*10^6) / (200*8) - 1 (must be <65536)
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS12 bits for 256 prescaler
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (0 << OCIE1A);

  sei();  // allow interrupts
}

ISR(TIMER1_COMPA_vect) {
  if (!stop_flag) {
    if (i < (pulse_per_round)) {
      /* If i%2 ==0 -> Set High */
      if (0 == i % 2) {
        digitalWrite(largePin, HIGH);
      }
      /* If i%2 ==0 -> Set Low */
      else {
        digitalWrite(largePin, LOW);
        posX += double(360) / (pulse_per_round / 2);
        if (360 <= posX) {
          posX = 0;
          Serial.println("Round Up");
        }
      }

      if ((i >= (pulse_per_round - 2)) && (i <= (pulse_per_round - 1))) {
        if (motorNhoStepCnt >= 36) {
          Init_MotorNho();
          motorNhoStepCnt = 0;
        }
        if (MODE3DMAPPING == modeWorking) {
          s = true;
          Serial.println("Run oki");
          if (i == (pulse_per_round - 1))
            posY = posY_inner;
        }

        else if (MODE2DMAPPING == modeWorking) {
          if (i == (pulse_per_round - 1)) {
            round_2D += 1;
          }
        }
      }
    }

    else {
      i = -1;  // Set i=-1 then ++ i =0
    }

    i++;

    if (MODE3DMAPPING == modeWorking) {
      /* Send string for python to extract and get the posX posY and the distance */
      // Serial.println(posY);
      if (posY < 90) {
      }
      // Serial.println(String(posX, 2) + "," + String(posY, 2) + "," + String(distance));
      else {
        Serial.println("End Data");
        stop_flag = true;
        measure_flag = false;
      }
    }

    else if (MODE2DMAPPING == modeWorking) {
      /* To get 3 round of 2D  */
      if (round_2D < 5)
        /* Send string for python to extract and get the posX, posY and distance */
        Serial.println(String(posX, 2) + "," + String(posY, 2) + "," + String(distance));
      else {
        /* Send End Data to stop read  */
        Serial.println("End Data");

        /* Set relay to low to disconnect */
        digitalWrite(PINRELAY, LOW);

        /* Set stop_flag to true to end interrupt to read data */
        stop_flag = true;

        /* set measure_flag to false to  */
        measure_flag = false;
      }
    }
  }

  else {
    /* Do nothing */
  }
}

void loop() {

  if (runMotor) {
    
    motorNhoStepCnt++;
    runMotorNho(28, 1);
    Serial.print("cnt: ");
    Serial.println(motorNhoStepCnt);
    runMotor = false;
  }
  if (false == measure_flag) {
    if (Serial.available() > 0) {
      /* To receive the mode from the python  */
      modeWorking = Serial.readString().toInt();

      if (modeWorking == MODE2DMAPPING) {
        /* Reset all parameters */
        stop_flag = false;
        i = 0;
        posX = 0;
        posY = 0;
        round_2D = 0;

        /* Set the relay to on */
        digitalWrite(PINRELAY, HIGH);

        TIMSK1 |= (1 << OCIE1A);
        measure_flag = true;
      } else if (modeWorking == MODE3DMAPPING) {
        stop_flag = false;
        i = 0;
        posX = 0;
        posY = 0;

        digitalWrite(PINRELAY, HIGH);

        TIMSK1 |= (1 << OCIE1A);
        measure_flag = true;
      }
    }
  }

  else if (true == measure_flag) {
    if (TFmini.measure()) {
      distance = TFmini.getDistance();  // Get distance data
    }
  }
}

/**
 * @brief 
 * 
 * @param Pulse 
 * @param direction 
 */
void runMotorNho(int Pulse, int direction) {
  static int cnt = 0;
  (direction == 1) ? cnt = 0 : cnt = Pulse;
  while (1) {

    if (direction == 1) {
      cnt++;
      if (cnt == Pulse) {
        cnt = 0;
        break;
      }
    } else {
      cnt--;
      if (cnt == 0) {
        cnt = Pulse;
        break;
      }
    }

    switch (cnt % 4) {
      case 0:
        digitalWrite(STEPPER_PIN_1, HIGH);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 1:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, HIGH);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 2:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, HIGH);
        digitalWrite(STEPPER_PIN_4, LOW);
        break;
      case 3:
        digitalWrite(STEPPER_PIN_1, LOW);
        digitalWrite(STEPPER_PIN_2, LOW);
        digitalWrite(STEPPER_PIN_3, LOW);
        digitalWrite(STEPPER_PIN_4, HIGH);
        break;
    }
    delayMicroseconds(2000);
  }
  // Update Y axis rotate
  posY_inner += ((float)360 / 2048);
}

/**
 * @brief Init MotorNho function
 * @details Set position motor nho to the position of light sensor
 */
void Init_MotorNho() {
  auto pnpSensor = digitalRead(5);

  while (pnpSensor) {
    static int tmp = 0;
    tmp++;
    runMotorNho(tmp, ANTI_CLOCKWISE);
    pnpSensor = digitalRead(5);
    Serial.println(pnpSensor);
  }
}