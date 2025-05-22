#ifndef LED_H
#define LED_H

#include <FastLED.h>

#define NUM_STRIPS 3
#define LEDS_PER_STRIP 60
#define DATA_PIN_1 2
#define DATA_PIN_2 4
#define DATA_PIN_3 5

extern CRGB leds1[LEDS_PER_STRIP];
extern CRGB leds2[LEDS_PER_STRIP];
extern CRGB leds3[LEDS_PER_STRIP];

extern int activeStrip;
extern int activeLedCount;
extern unsigned long currentDelay;
extern unsigned long countdownStartTime;
extern unsigned long lastCountdownUpdate;
extern unsigned long timePerLed;


void setupLED() {
  FastLED.addLeds<WS2812B, DATA_PIN_1, GRB>(leds1, LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PIN_2, GRB>(leds2, LEDS_PER_STRIP);
  FastLED.addLeds<WS2812B, DATA_PIN_3, GRB>(leds3, LEDS_PER_STRIP);
  fill_solid(leds1, LEDS_PER_STRIP, CRGB::Black);
  fill_solid(leds2, LEDS_PER_STRIP, CRGB::Black);
  fill_solid(leds3, LEDS_PER_STRIP, CRGB::Black);
  FastLED.show();
}



void resetAllStrips() {
  fill_solid(leds1, LEDS_PER_STRIP, CRGB::Black);
  fill_solid(leds2, LEDS_PER_STRIP, CRGB::Black);
  fill_solid(leds3, LEDS_PER_STRIP, CRGB::Black);
  activeLedCount = LEDS_PER_STRIP;
}


void updateActiveStrip() {
  if (activeStrip < 0) return;
  CRGB* currentStrip;
  switch (activeStrip) {
    case 0: currentStrip = leds1; break;
    case 1: currentStrip = leds2; break;
    case 2: currentStrip = leds3; break;
    default: return;
  }
  for (int i = 0; i < LEDS_PER_STRIP; i++) {
    currentStrip[i] = (i < activeLedCount) ? CRGB(CHSV(i * 10, 255, 255)) : CRGB::Black;
  }
}



void updateLedCountdown() {
  if (activeStrip >= 0) {
    unsigned long currentTime = millis();
    if (currentTime - lastCountdownUpdate >= 50) {
      lastCountdownUpdate = currentTime;
      unsigned long elapsedTime = currentTime - countdownStartTime;
      timePerLed = currentDelay / LEDS_PER_STRIP;
      int newCount = max(0, LEDS_PER_STRIP - (int)(elapsedTime / timePerLed));
      if (newCount != activeLedCount) {
        activeLedCount = newCount;
        updateActiveStrip();
        FastLED.show();
      }
      if (activeLedCount <= 0) {
        activeStrip = -1;
        resetAllStrips();
        FastLED.show();
      }
    }
  }
}


void startCountdown(int strip, unsigned long delay) {
  if (activeStrip >= 0) resetAllStrips();
  activeStrip = strip;
  currentDelay = delay;
  activeLedCount = LEDS_PER_STRIP;
  countdownStartTime = millis();
  lastCountdownUpdate = millis();
  updateActiveStrip();
  FastLED.show();
}


void adjustCurrentDelay(unsigned long newDelay) {
  if (activeStrip < 0) return;
  unsigned long now = millis();
  unsigned long elapsed = now - countdownStartTime;
  float proportion = 1.0 - ((float)elapsed / currentDelay);
  unsigned long newRemain = newDelay * proportion;
  currentDelay = newDelay;
  countdownStartTime = now - (newDelay - newRemain);
  activeLedCount = LEDS_PER_STRIP * proportion;
  updateActiveStrip();
  FastLED.show();
}


// 闪烁蓝光两次以显示系统启动
void flashAllStrips(CRGB color = CRGB::Blue, int times = 2, int delayMs = 300) {
  for (int t = 0; t < times; t++) {
    fill_solid(leds1, LEDS_PER_STRIP, color);
    fill_solid(leds2, LEDS_PER_STRIP, color);
    fill_solid(leds3, LEDS_PER_STRIP, color);
    FastLED.show();
    delay(delayMs);

    resetAllStrips();
    FastLED.show();
    delay(delayMs);
  }
}


#endif