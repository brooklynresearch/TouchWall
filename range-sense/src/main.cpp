#include <Arduino.h>
#include "TFMini.h"
#include <Wire.h>
#include <LIDARLite.h>

enum sensor_type {LIDAR, IR_TOF, IR_SHARP, ULTRASONIC};

/* ***** CHOOSE SENSOR TYPE HERE (LIDAR, IR_TOF, IR_SHARP, ULTRASONIC) ****** */
sensor_type sensor = LIDAR;

// SHARP IR SENSOR
const int PIN_IR_SHARP = A9;
float ir_sharp_baseline;
float ir_sharp_threshold;
float ir_sharp_sensitivity = 5;   // lower is more sensitive

// TIME OF FLIGHT IR SENSOR
TFMini tfmini;
uint16_t ir_tof_threshold = 50;   // lower is more sensitive
uint16_t ir_tof_max_dist = 250;   // max sensing distance in cm

// LIDAR LITE
LIDARLite lidar;
int lidar_max_dist = 250;         // max sensing distance in cm

// ULTRASONIC
const int PIN_ULTRASONIC = A8;
int usonic_threshold = 51000;     // higher is more sensitive

void run_lidar();
void run_ir_sharp();
void run_ir_tof();
void run_ultrasonic();

void setup() {

  Serial.begin(115200);

  // status LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  // short delay before we start doing anything
  delay(3000);
  Serial.println("getting ready...");

  switch (sensor) {

    case LIDAR:
      Serial.println("init LIDAR LITE...");
      lidar.begin(0, true);
      lidar.configure(0);
      break;

    case IR_SHARP:
      Serial.println("init IR_SHARP...");
      pinMode(PIN_IR_SHARP, INPUT);
      analogReadResolution(16);

      // take a small sample of readings to construct a baseline
      ir_sharp_baseline = 0;
      for (int i = 0; i < 1000; i++) {
        ir_sharp_baseline += (float) analogRead(PIN_IR_SHARP);
        delay(5);
      }
      ir_sharp_baseline /= 1000;
      ir_sharp_threshold = ir_sharp_baseline * (ir_sharp_sensitivity/100+1);

      Serial.print("baseline reading is ");
      Serial.println(ir_sharp_baseline);

      break;

    case IR_TOF:
      Serial.println("init IR_TOF...");
      // initialize UART for communication w/ ToF IR sensor
      Serial1.begin(TFMINI_BAUDRATE);
      tfmini.begin(&Serial1);

      break;

    case ULTRASONIC:
      pinMode(PIN_ULTRASONIC, INPUT);
      analogReadResolution(16);
      break;

    default:
      break;
  }

  Serial.print("ready");

}

void loop() {
  switch (sensor) {
    case LIDAR:
      run_lidar();
      break;
    case IR_SHARP:
      run_ir_sharp();
      break;
    case IR_TOF:
      run_ir_tof();
      break;
    case ULTRASONIC:
      run_ultrasonic();
      break;
    default:
      break;
  }
}

void log_touch() {
  Serial.println("TOUCH");
}

void log_no_touch() {
  Serial.println("");
}

void run_ir_sharp() {
  // get reading w/ 16-bit ADC
  int val = analogRead(PIN_IR_SHARP);

  // detect touch if above threshold
  if (val > ir_sharp_threshold) log_touch();
  else log_no_touch();

  delay(100);
}

void run_ir_tof() {
  uint16_t dist = tfmini.getDistance();
  uint16_t strength = tfmini.getRecentSignalStrength();

  if (dist < ir_tof_max_dist && strength > ir_tof_threshold) log_touch();
  else log_no_touch();

  delay(10);
}

void run_lidar() {
  // get distance (w/ bias correction)
  int dist = lidar.distance();

  if (dist < lidar_max_dist) log_touch();
  else log_no_touch();

  delay(10);
}

void run_ultrasonic() {
  // read distance as analog value
  uint16_t val = analogRead(PIN_ULTRASONIC);
  Serial.println(val);
  // if (val < usonic_threshold) log_touch();
  // else log_no_touch();
  delay(50);
}
