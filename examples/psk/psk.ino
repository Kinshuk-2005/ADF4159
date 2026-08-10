/*
  psk.ino

  Phase Shift Keying: TXDATA high shifts the output phase by
  +phaseDeg, TXDATA low shifts it by -phaseDeg.

  Matches the datasheet's PSK description: phase value = 1024 gives
  a 90 degree shift ((1024 * 360) / 4096 = 90).

  As with FSK, TXDATA is a dedicated chip pin - wire it to a spare
  GPIO and toggle it directly.
*/

#include <SPI.h>
#include "ADF4159.h"

const int LE_PIN = 5;
const int TXDATA_PIN = 4;

const uint32_t REFIN_HZ = 100000000UL;
const uint64_t RFOUT_HZ = 12000000000ULL;
const double PHASE_SHIFT_DEG = 90.0;
const unsigned long TOGGLE_INTERVAL_MS = 100;

ADF4159 synth;
bool txState = false;

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(TXDATA_PIN, OUTPUT);
  digitalWrite(TXDATA_PIN, LOW);

  synth.begin(LE_PIN, REFIN_HZ);

  // Lock to the carrier first, then enable PSK with the desired phase step.
  synth.setFrequency(RFOUT_HZ);
  synth.configurePSK(PHASE_SHIFT_DEG);

  Serial.println("PSK configured: +/-90 degree phase shift on TXDATA toggle.");
}

void loop() {
  txState = !txState;
  digitalWrite(TXDATA_PIN, txState ? HIGH : LOW);
  Serial.println(txState ? "TXDATA high -> +90 deg" : "TXDATA low -> -90 deg");
  delay(TOGGLE_INTERVAL_MS);
}
