// Teensy_AnalogScope_Logger.ino
// Teensy 4.1 Dual-Channel Analog Logger
// Triggered buffered acquisition, 500 samples per channel

#include <ADC.h>
#include <ADC_util.h>

#define CH1 A0
#define CH2 A1
#define TRIGGER_THRESHOLD 600  // ADC units (~0.8V)

#define SAMPLES 500
#define BAUD_RATE 115200

ADC *adc = new ADC();
uint16_t buffer_ch1[SAMPLES];
uint16_t buffer_ch2[SAMPLES];

bool triggered = false;
uint16_t sampleIndex = 0;

void setup() {
  Serial.begin(BAUD_RATE);
  while (!Serial) ;  // Wait for serial port
  delay(100);
  
  adc->adc0->setAveraging(4);
  adc->adc0->setResolution(10);
  adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  adc->adc1->setAveraging(4);
  adc->adc1->setResolution(10);
  adc->adc1->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
  adc->adc1->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);

  Serial.println("Teensy Analog Logger Ready");
}

void loop() {
  if (!triggered) {
    // Capture sample into circular buffer
    buffer_ch1[sampleIndex] = adc->adc0->analogRead(CH1);
    buffer_ch2[sampleIndex] = adc->adc1->analogRead(CH2);

    // Trigger detection on rising edge
    if (buffer_ch1[sampleIndex] > TRIGGER_THRESHOLD &&
        buffer_ch1[(sampleIndex + SAMPLES - 1) % SAMPLES] <= TRIGGER_THRESHOLD) {
      triggered = true;
    }

    sampleIndex = (sampleIndex + 1) % SAMPLES;
  } else {
    // Transmit all captured samples as CSV
    Serial.println("CH1,CH2");
    for (uint16_t i = 0; i < SAMPLES; i++) {
      uint16_t index = (sampleIndex + i) % SAMPLES;
      Serial.print(buffer_ch1[index]);
      Serial.print(",");
      Serial.println(buffer_ch2[index]);
    }

    triggered = false;
    sampleIndex = 0;
    delay(500);  // Prevent spamming — adjust or remove if needed
  }
}

