/*
  fsk.ino

  Frequency Shift Keying: locks to a center frequency, then hops by
  +/-deviationHz each time the TXDATA pin is toggled.

  TXDATA is a dedicated chip pin, separate from the SPI (LE/CLK/DATA)
  interface - this library does not drive it. Wire it to a spare GPIO
  and toggle it directly with digitalWrite().

  Matches the datasheet's FSK Settings worked example:
  5.8 GHz center, 25 MHz PFD, 250 kHz deviation.
*/

#include <SPI.h>
#include "ADF4159.h"

const int LE_PIN = 5;
const int TXDATA_PIN = 4; // wire directly to the ADF4159 TXDATA pin

const uint32_t REFIN_HZ = 100000000UL;
const uint64_t CENTER_FREQ_HZ = 5800000000ULL;
const double DEVIATION_HZ = 250000.0; // deviation stays a double - precision
                                       // there is not sub-Hz critical
const unsigned long HOP_INTERVAL_MS = 50;

ADF4159 synth;
bool txState = false;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TXDATA_PIN, OUTPUT);
  digitalWrite(TXDATA_PIN, LOW);

  synth.begin(LE_PIN, REFIN_HZ);
  synth.configureFSK(CENTER_FREQ_HZ, DEVIATION_HZ);

  Serial.println("FSK configured: 5.8 GHz center, +/-250 kHz deviation.");
}

void loop() {
  txState = !txState;
  digitalWrite(TXDATA_PIN, txState ? HIGH : LOW);
  Serial.println(txState ? "TXDATA high -> +250 kHz" : "TXDATA low -> -250 kHz");
  delay(HOP_INTERVAL_MS);
}
