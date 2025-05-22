#ifndef PRESSURESENSOR_H
#define PRESSURESENSOR_H

#define PRESSURE_1_PIN 19
#define PRESSURE_2_PIN 33
#define PRESSURE_3_PIN 32


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

  // 当有压力传感器处于被按下状态时，对应的灯条被点亮
  extern CRGB leds1[LEDS_PER_STRIP];
  extern CRGB leds2[LEDS_PER_STRIP];
  extern CRGB leds3[LEDS_PER_STRIP];

  fill_solid(leds1, LEDS_PER_STRIP, digitalRead(PRESSURE_1_PIN) == LOW ? CRGB::Green : CRGB::Black);
  fill_solid(leds2, LEDS_PER_STRIP, digitalRead(PRESSURE_2_PIN) == LOW ? CRGB::Green : CRGB::Black);
  fill_solid(leds3, LEDS_PER_STRIP, digitalRead(PRESSURE_3_PIN) == LOW ? CRGB::Green : CRGB::Black);

  FastLED.show();
}


#endif