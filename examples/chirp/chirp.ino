/*
  chirp.ino

  Generates a continuous sawtooth FMCW chirp, matching the datasheet's
  FMCW Radar Ramp Settings worked example: 50 MHz sweep centered around
  5800-5850 MHz, repeating every 2 ms.

  This single call handles the deviation word, deviation offset,
  CLK1/CLK2 timer, step count, and the full R7->R0 write sequence.
*/

#include <SPI.h>
#include "ADF4159.h"

const int LE_PIN = 5;
const uint32_t REFIN_HZ = 25000000UL;        // 25 MHz reference, matches worked example
const uint64_t START_FREQ_HZ = 5800000000ULL;
const uint64_t BANDWIDTH_HZ  = 50000000ULL;
const double   SWEEP_TIME_MS = 2.0;
const uint16_t NUM_STEPS     = 200;

ADF4159 synth;

void setup() {
  Serial.begin(115200);
  delay(200);

  synth.begin(LE_PIN, REFIN_HZ);

  // One call: stages start frequency, deviation, timer and step count,
  // then activates a continuous sawtooth ramp.
  synth.configureFMCWChirp(START_FREQ_HZ, BANDWIDTH_HZ, SWEEP_TIME_MS, NUM_STEPS);

  Serial.println("Continuous sawtooth chirp running: 5800-5850 MHz every 2 ms.");
  Serial.println();
  synth.printStatus();

  // Other ramp shapes available:
  //   synth.configureTriangleRamp(START_FREQ_HZ, START_FREQ_HZ + BANDWIDTH_HZ,
  //                                NUM_STEPS, (SWEEP_TIME_MS*1000.0)/NUM_STEPS, true);
  //   synth.configureDualRamp(centerFreq, stepTimeUs, devHz1, steps1, devHz2, steps2);
  //   synth.configureFastRamp(centerFreq, upDevHz, upSteps, upStepUs, downDevHz, downSteps, downStepUs);
  //   synth.configureParabolicRamp(centerFreq, devHz, numSteps, stepTimeUs);
}

void loop() {
  // The ramp runs autonomously on-chip once activated.
}
