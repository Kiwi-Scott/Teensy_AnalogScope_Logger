
#include <Arduino.h>
#include "imxrt.h"                // Provides CoreDebug, DWT, etc., for Teensy 4.1
// Minimal definitions for cycle counter access
typedef struct {
    volatile uint32_t DEMCR;
} CoreDebug_Type;
#define CoreDebug ((CoreDebug_Type*)0xE000EDF0)
#ifndef CoreDebug_DEMCR_TRCENA_Msk
  #define CoreDebug_DEMCR_TRCENA_Msk (1UL << 24)
#endif

typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t CYCCNT;
} DWT_Type;
#define DWT ((DWT_Type*)0xE0001000)
#ifndef DWT_CTRL_CYCCNTENA_Msk
  #define DWT_CTRL_CYCCNTENA_Msk (1UL << 0)
#endif
#include <ADC.h>                  // Teensy ADC library with DMA support
#include <arduino_freertos.h>     // FreeRTOS for Arduino
#include <queue.h>


// Do not redefine F_CPU if already defined by Teensyduino
#ifndef F_CPU
#define F_CPU 600000000UL         // Teensy 4.1 runs at 600 MHz
#endif

// Pre-calculate ticks per microsecond (600 ticks per µs)
#define TICKS_PER_US (F_CPU / 1000000UL)

// Scaling factors
const float VOLTAGE_SCALING_FACTOR = 5.0 / 1023.0;  
const float PHASE_ANGLE_SCALING_FACTOR = 1.0 / 360.0;

// ----- Pin Definitions -----
const uint8_t pulsePin = 2;         // Digital output for pulse (direct register access)
const uint8_t senseAnalogPin = A0;    // ADC channel for sensing voltage (output of ISO124)
const uint8_t senseDigitalPin = 3;    // Digital input for peak detection (external interrupt)
const uint8_t buttonPin = 4;          // (Unused in this example)

// ----- Global Signal Parameters & Variables -----
// These are used for the optimization process.
volatile bool peakDetected = false;    // Set by external interrupt
volatile uint16_t adcValue = 0;          // Latest ADC reading (0–1023)

float previousPeakVoltage = 0;           // Best voltage measured so far
float currentPhaseAngle = 0;             // Fraction (0.0 to 1.0) representing phase offset
volatile uint32_t lastPeakTime = 0;        // Tick count from last detected peak
uint32_t frequencyPeriod = 0;            // Measured period in ticks

// Global pulse duration variable (in ticks); start with 10 µs.
uint32_t pulseWidthTicks = TICKS_PER_US * 10;

// Bidirectional optimization step sizes
uint32_t freqStep = 10;                   // Frequency step (ticks)
float phaseStep = PHASE_ANGLE_SCALING_FACTOR * 10;  // Phase step (fraction units)
uint32_t pulseStep = 100;                 // Pulse duration step (ticks)

// Fail counters for adaptive step-size reduction
uint8_t failCountFreq = 0, failCountPhase = 0, failCountPulse = 0;

// Variables for direct register I/O
volatile uint32_t *pulseOutReg;          // Pointer to pulse pin's port register
uint32_t pulseBitMask;                   // Bit mask for pulse pin

// ----- ADC Object -----
ADC *adc = new ADC();

// ----- DWT Cycle Counter Setup -----
void initCycleCounter() {
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;  // Enable trace
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;               // Enable cycle counter
}

uint32_t getTicks() {
  return DWT->CYCCNT;
}

// ----- Direct Register I/O Macros -----
// Use direct register writes for speed (set: offset 0x04, clear: offset 0x08)
#define PULSE_SET()  (*(volatile uint32_t *)((uint32_t)pulseOutReg + 0x04)) = pulseBitMask
#define PULSE_CLEAR() (*(volatile uint32_t *)((uint32_t)pulseOutReg + 0x08)) = pulseBitMask

// ----- FreeRTOS Optimization Event Logging -----
enum EventType {
  FREQ_EVENT = 0,
  PHASE_EVENT = 1,
  PULSE_EVENT = 2,
  GATE_EVENT = 3   // Indicates gateTime event
};

struct OptEvent {
  uint32_t tick;       // Tick count when the event occurred
  uint8_t eventType;   // Type of event
  uint32_t value;      // For FREQ: period in ticks; PHASE: phase (in milli-units); PULSE: pulse width in ticks
  uint16_t adcVal;     // ADC reading at time of event
};

QueueHandle_t eventQueue;      // FreeRTOS queue for optimization events
const int QUEUE_LENGTH = 64;   // Adjust as needed

// ----- External Interrupt for Peak Detection -----
void senseISR() {
  static uint32_t lastInterruptTime = 0;
  uint32_t currentTime = getTicks();
  uint32_t debounceTicks = TICKS_PER_US * 50; // 50 µs debounce period
  if ((currentTime - lastInterruptTime) > debounceTicks) {
    peakDetected = true;
    lastPeakTime = currentTime;
  }
  lastInterruptTime = currentTime;
}

// ----- ADC ISR (called by ADC library DMA interrupt) -----
void adc_isr() {
  adcValue = adc->readSingle();
}

// ----- Non-blocking Wait for a Peak -----
// Instead of busy-waiting, yield using vTaskDelay.
void waitForPeak() {
  peakDetected = false;
  while (!peakDetected) {
    vTaskDelay(1);
  }
}

// ----- Send Pulse at Specified Phase Angle -----
// Wait until a calculated target time then generate a pulse of specified duration.
void sendPulseAtPhaseAngle() {
  uint32_t targetTime = lastPeakTime + (uint32_t)(frequencyPeriod * currentPhaseAngle);
  while (getTicks() < targetTime) {
    vTaskDelay(0);
  }
  PULSE_SET();
  uint32_t startPulse = getTicks();
  while ((getTicks() - startPulse) < pulseWidthTicks) {
    vTaskDelay(0);
  }
  PULSE_CLEAR();
}

// ----- Circuit Initialization -----
// Sends an initial pulse and measures the period between two peaks.
// Resets the phase angle and updates previousPeakVoltage.
void initializeCircuit() {
  sendPulseAtPhaseAngle();
  waitForPeak();
  uint32_t firstPeakTime = lastPeakTime;
  waitForPeak();
  uint32_t secondPeakTime = lastPeakTime;
  frequencyPeriod = (secondPeakTime - firstPeakTime) * 2;
  currentPhaseAngle = 0.0;
  previousPeakVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
}

// ----- Bidirectional Optimization Functions -----
// Each parameter is tested for both an increase and decrease; the best result is kept.

// Optimize Frequency Period
void optimizeFrequency() {
  uint32_t original = frequencyPeriod;
  uint32_t bestFrequency = original;
  float bestVoltage = previousPeakVoltage;
  float testVoltage;

  // Test increasing frequency (i.e. larger period)
  frequencyPeriod = original + freqStep;
  sendPulseAtPhaseAngle();
  waitForPeak();
  testVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
  if (testVoltage > bestVoltage) {
    bestVoltage = testVoltage;
    bestFrequency = frequencyPeriod;
    // Log event for increase
    OptEvent ev = { getTicks(), FREQ_EVENT, frequencyPeriod, adcValue };
    xQueueSend(eventQueue, &ev, 0);
  }

  // Test decreasing frequency (if possible)
  if (original > freqStep) {
    frequencyPeriod = original - freqStep;
    sendPulseAtPhaseAngle();
    waitForPeak();
    testVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
    if (testVoltage > bestVoltage) {
      bestVoltage = testVoltage;
      bestFrequency = frequencyPeriod;
      // Log event for decrease
      OptEvent ev = { getTicks(), FREQ_EVENT, frequencyPeriod, adcValue };
      xQueueSend(eventQueue, &ev, 0);
    }
  }
  frequencyPeriod = bestFrequency;
  previousPeakVoltage = bestVoltage;
  // Adaptive step-size: if no improvement, increase fail count.
  if (bestFrequency == original) {
    failCountFreq++;
    if (failCountFreq > 5 && freqStep > 1) { 
      freqStep /= 2;
      failCountFreq = 0;
    }
  } else {
    failCountFreq = 0;
  }
}

// Optimize Phase Angle
void optimizePhase() {
  float original = currentPhaseAngle;
  float bestPhase = original;
  float bestVoltage = previousPeakVoltage;
  float testVoltage;
  float testPhase;

  // Test increasing phase angle
  testPhase = original + phaseStep;
  currentPhaseAngle = testPhase;
  sendPulseAtPhaseAngle();
  waitForPeak();
  testVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
  if (testVoltage > bestVoltage) {
    bestVoltage = testVoltage;
    bestPhase = testPhase;
    OptEvent ev = { getTicks(), PHASE_EVENT, (uint32_t)(testPhase * 1000), adcValue };
    xQueueSend(eventQueue, &ev, 0);
  }
  
  // Test decreasing phase angle (ensuring it does not drop below 0)
  if (original >= phaseStep) {
    testPhase = original - phaseStep;
    currentPhaseAngle = testPhase;
    sendPulseAtPhaseAngle();
    waitForPeak();
    testVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
    if (testVoltage > bestVoltage) {
      bestVoltage = testVoltage;
      bestPhase = testPhase;
      OptEvent ev = { getTicks(), PHASE_EVENT, (uint32_t)(testPhase * 1000), adcValue };
      xQueueSend(eventQueue, &ev, 0);
    }
  }
  currentPhaseAngle = bestPhase;
  previousPeakVoltage = bestVoltage;
  if (bestPhase == original) {
    failCountPhase++;
    if (failCountPhase > 5 && phaseStep > (PHASE_ANGLE_SCALING_FACTOR / 10)) {
      phaseStep /= 2;
      failCountPhase = 0;
    }
  } else {
    failCountPhase = 0;
  }
}

// Optimize Pulse Duration
void optimizePulseWidth() {
  uint32_t original = pulseWidthTicks;
  uint32_t bestPulse = original;
  float bestVoltage = previousPeakVoltage;
  float testVoltage;
  uint32_t testPulse;

  // Test increasing pulse width
  testPulse = original + pulseStep;
  pulseWidthTicks = testPulse;
  sendPulseAtPhaseAngle();
  waitForPeak();
  testVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
  if (testVoltage > bestVoltage) {
    bestVoltage = testVoltage;
    bestPulse = testPulse;
    OptEvent ev = { getTicks(), PULSE_EVENT, testPulse, adcValue };
    xQueueSend(eventQueue, &ev, 0);
  }
  
  // Test decreasing pulse width (if valid)
  if (original > pulseStep) {
    testPulse = original - pulseStep;
    pulseWidthTicks = testPulse;
    sendPulseAtPhaseAngle();
    waitForPeak();
    testVoltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
    if (testVoltage > bestVoltage) {
      bestVoltage = testVoltage;
      bestPulse = testPulse;
      OptEvent ev = { getTicks(), PULSE_EVENT, testPulse, adcValue };
      xQueueSend(eventQueue, &ev, 0);
    }
  }
  pulseWidthTicks = bestPulse;
  previousPeakVoltage = bestVoltage;
  if (bestPulse == original) {
    failCountPulse++;
    if (failCountPulse > 5 && pulseStep > 10) {
      pulseStep /= 2;
      failCountPulse = 0;
    }
  } else {
    failCountPulse = 0;
  }
}

// ----- Optimization Task -----
// Runs the bidirectional optimization for each parameter in sequence.
void vOptimizationTask(void *pvParameters) {
  (void) pvParameters;
  for (;;) {
    // Optimize frequency, phase, and pulse width
    optimizeFrequency();
    optimizePhase();
    optimizePulseWidth();
    
    // If none of the tests improved the parameters (i.e. all remain unchanged), then assume local optimum.
    if (failCountFreq > 0 && failCountPhase > 0 && failCountPulse > 0) {
      // Signal that gate time monitoring should begin.
      OptEvent gateEv = { getTicks(), GATE_EVENT, 0, adcValue };
      xQueueSend(eventQueue, &gateEv, 0);
      // Allow some delay before re-checking.
      vTaskDelay(50);
    }
    vTaskDelay(1);
  }
}

// ----- Gate Time Task -----
// Waits for a GATE_EVENT, then flushes the event queue over Serial,
// monitors ADC during gate time, and finally re-initializes the circuit.
void vGateTimeTask(void *pvParameters) {
  (void) pvParameters;
  OptEvent ev;
  for (;;) {
    // Wait for a gate event from the optimization task.
    if (xQueueReceive(eventQueue, &ev, portMAX_DELAY) == pdTRUE) {
      if (ev.eventType == GATE_EVENT) {
        Serial.println("----- Gate Time: Flushing Optimization Log -----");
        while (uxQueueMessagesWaiting(eventQueue) > 0) {
          if (xQueueReceive(eventQueue, &ev, 0) == pdTRUE) {
            Serial.print("Tick: ");
            Serial.print(ev.tick);
            Serial.print(" | ");
            if (ev.eventType == FREQ_EVENT) {
              Serial.print("Freq Event - New Frequency: ");
              Serial.print(ev.value);
              Serial.print(" ticks");
            } else if (ev.eventType == PHASE_EVENT) {
              Serial.print("Phase Event - New Phase Angle: ");
              Serial.print(((float)ev.value)/1000.0, 3);
              Serial.print(" units");
            } else if (ev.eventType == PULSE_EVENT) {
              Serial.print("Pulse Event - New Pulse Width: ");
              Serial.print(ev.value);
              Serial.print(" ticks (");
              Serial.print(ev.value / TICKS_PER_US);
              Serial.println(" us)");
            }
            Serial.print(" | ADC: ");
            Serial.println(ev.adcVal);
          }
        }
        Serial.println("----- End of Optimization Log -----");
        // Monitor gate time: log ADC values until voltage falls below threshold or timeout.
        uint32_t startTicks = getTicks();
        while ((getTicks() - startTicks) < (2000UL * (F_CPU / 1000UL))) {
          Serial.print("Tick: ");
          Serial.print(getTicks());
          Serial.print(" | ADC: ");
          Serial.println(adcValue);
          float voltage = ((float)adcValue) * VOLTAGE_SCALING_FACTOR;
          if (voltage < 0.1) {
            break;
          }
          vTaskDelay(10);
        }
        // Reinitialize the circuit to capture any environmental changes.
        Serial.println("Reinitializing circuit...");
        initializeCircuit();
      }
    }
    vTaskDelay(1);
  }
}

// ----- Setup -----
void setup() {
  Serial.begin(115200);
  while (!Serial) {}  // Wait for Serial monitor to open
  
  initCycleCounter();
  
  // Configure pulsePin for direct register I/O.
  pinMode(pulsePin, arduino::OUTPUT);
  pulseOutReg = portOutputRegister(digitalPinToPort(pulsePin));
  pulseBitMask = digitalPinToBitMask(pulsePin);
  
  // Configure senseDigitalPin for external interrupts.
  pinMode(senseDigitalPin, arduino::INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(senseDigitalPin), senseISR, arduino::FALLING);
  
  // ADC Setup with Teensy ADC library.
  adc->adc0->setAveraging(16);
adc->adc0->setResolution(10);
adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
adc->adc0->enableDMA();
adc->adc0->startContinuous(senseAnalogPin);
adc->adc0->enableInterrupts(adc_isr);

  
  // Initial circuit measurement.
  initializeCircuit();
  
  // Create the FreeRTOS event queue.
  eventQueue = xQueueCreate(QUEUE_LENGTH, sizeof(OptEvent));
  
  // Create FreeRTOS tasks.
  xTaskCreate(vOptimizationTask, "OptimizationTask", 256, NULL, 2, NULL);
  xTaskCreate(vGateTimeTask, "GateTimeTask", 256, NULL, 2, NULL);
}

// ----- Loop -----
// In a FreeRTOS environment, loop() is not used.
void loop() {
  // Nothing needed here.
}
