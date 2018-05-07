#include <Arduino.h>

static const int PIN_COL_A = 11;
static const int PIN_COL_B = 12;

int touching_col_a = 0;
int touching_col_b = 0;

void setup() {

  pinMode(PIN_COL_A, INPUT);
  pinMode(PIN_COL_B, INPUT);

  // status LED
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);

  Serial.begin(115200);
}

void loop() {

  int touch_a = digitalRead(PIN_COL_A);
  int touch_b = digitalRead(PIN_COL_B);

  if (touch_a && !touching_col_a) {
    touching_col_a = 1;
    // Serial.println("new touch, column A");
  } else if (!touch_a && touching_col_a){
    touching_col_a = 0;
    // Serial.println("release, column A");
  }

  if (touch_b && !touching_col_b) {
    touching_col_b = 1;
    // Serial.println("new touch, column B");
  } else if (!touch_b && touching_col_b) {
    touching_col_b = 0;
    // Serial.println("release, column B");
  }

}
