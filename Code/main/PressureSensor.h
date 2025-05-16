#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#define PRESSURE_1_PIN 32
#define PRESSURE_2_PIN 33
#define PRESSURE_3_PIN 19


void setupPressurePins() {
  pinMode(PRESSURE_1_PIN, INPUT_PULLUP);
  pinMode(PRESSURE_2_PIN, INPUT_PULLUP);
  pinMode(PRESSURE_3_PIN, INPUT_PULLUP);
}


bool allPressureSensorsPressed() {
  return digitalRead(PRESSURE_1_PIN) == LOW &&
         digitalRead(PRESSURE_2_PIN) == LOW &&
         digitalRead(PRESSURE_3_PIN) == LOW;
}


void checkPressureStatus() {
  static unsigned long lastPrintTime = 0;
  unsigned long now = millis();
  if (now - lastPrintTime > 500) {
    lastPrintTime = now;
    if (digitalRead(PRESSURE_1_PIN) == LOW) Serial.println("PRESSURE 1 pressed");
    if (digitalRead(PRESSURE_2_PIN) == LOW) Serial.println("PRESSURE 2 pressed");
    if (digitalRead(PRESSURE_3_PIN) == LOW) Serial.println("PRESSURE 3 pressed");
  }
}


#endif