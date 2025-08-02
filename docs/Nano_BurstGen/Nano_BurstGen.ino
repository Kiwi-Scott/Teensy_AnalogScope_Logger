#define USE_LCD 1  // Set to 0 for OLED, 1 for LCD

#include <Encoder.h>

#if USE_LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#else
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define OLED_RESET -1
Adafruit_SSD1306 display(128, 64, &Wire, OLED_RESET);
#endif

Encoder encFreq(2, 3);
Encoder encPulses(4, 5);
Encoder encGate(6, 7);

#define BTN_FREQ   8
#define BTN_PULSE  A0
#define BTN_GATE   10

#define OUT_PORT PORTB
#define OUT_BIT  _BV(1)

volatile uint16_t pulseCount = 0;
volatile uint16_t pulseLimit = 10;
volatile bool gateActive = true;
unsigned long gateDelay = 5000;
unsigned long gateTimer = 0;

volatile uint32_t freq = 20000;
volatile uint16_t ocrVal = 399;

uint16_t freqStep =  1;
uint8_t pulseStep = 1;
uint16_t gateStep = 10;

void setup() {
  pinMode(9, OUTPUT);
  pinMode(BTN_FREQ, INPUT_PULLUP);
  pinMode(BTN_PULSE, INPUT_PULLUP);
  pinMode(BTN_GATE, INPUT_PULLUP);

  TCCR1A = _BV(COM1A0);
  TCCR1B = _BV(WGM12);
  TIMSK1 = _BV(OCIE1A);
  OCR1A = ocrVal;

  TCNT1 = 0;
  TCCR1B = _BV(WGM12) | _BV(CS10);
  gateActive = true;

#if USE_LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Nano Burst Gen");
  delay(1000);
#else
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Nano Burst Gen");
  display.display();
  delay(1000);
#endif

  Serial.begin(9600);
}

ISR(TIMER1_COMPA_vect) {
  if (!gateActive) return;
  if (++pulseCount >= pulseLimit * 2) {
    TCCR1B &= 0b11111000;
    gateActive = false;
    pulseCount = 0;
    OUT_PORT &= ~OUT_BIT;
    gateTimer = micros();
  }
}

void loop() {
  static long lastFreq = 0, lastPulse = 0, lastGate = 0;
  long f = encFreq.read() / 4;
  long p = encPulses.read() / 4;
  long g = encGate.read() / 4;

  if (f != lastFreq) {
    freq = constrain(freq + (f - lastFreq) * freqStep, 100, 500000);
    lastFreq = f;
    ocrVal = (F_CPU / (2UL * freq)) - 1;
    OCR1A = ocrVal;
  }

  if (p != lastPulse) {
    pulseLimit = constrain(pulseLimit + (p - lastPulse) * pulseStep, 1, 1000);
    lastPulse = p;
  }

  if (g != lastGate) {
    gateDelay = constrain(gateDelay + (g - lastGate) * gateStep, 100, 1000000);
    lastGate = g;
  }

  if (!gateActive && (micros() - gateTimer >= gateDelay)) {
    TCNT1 = 0;
    TCCR1B = _BV(WGM12) | _BV(CS10);
    gateActive = true;
  }

  static bool lastBF = 1, lastBP = 1, lastBG = 1;
  bool bf = digitalRead(BTN_FREQ);
  bool bp = digitalRead(BTN_PULSE);
  bool bg = digitalRead(BTN_GATE);

  if (bf == LOW && lastBF == HIGH)
    freqStep = (freqStep == 1) ? 10 : (freqStep == 10) ? 100 : (freqStep == 100) ? 1000 : 1;
  if (bp == LOW && lastBP == HIGH)
    pulseStep = (pulseStep == 1) ? 5 : (pulseStep == 5) ? 10 : 1;
  if (bg == LOW && lastBG == HIGH)
    gateStep = (gateStep == 10) ? 100 : (gateStep == 100) ? 1000 : 10;

  lastBF = bf;
  lastBP = bp;
  lastBG = bg;

  static unsigned long lastDisplay = 0;
  if (millis() - lastDisplay > 1000) {
    lastDisplay = millis();
#if USE_LCD
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("F:"); lcd.print(freq); lcd.print("Hz ");
    lcd.print("P:"); lcd.print(pulseLimit);
    lcd.setCursor(0, 1);
    lcd.print("G:"); lcd.print(gateDelay); lcd.print("us");
#else
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("Freq: "); display.print(freq); display.println(" Hz");
    display.print("Pulses: "); display.println(pulseLimit);
    display.print("Gate: "); display.print(gateDelay); display.println(" us");
    display.display();
#endif
  }
}
