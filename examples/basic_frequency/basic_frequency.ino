/*
  basic_frequency.ino

  Locks the ADF4159 to a fixed RF output frequency, and dumps the full
  status (raw registers + decoded fields + derived RFOUT/fPFD) to the
  Serial monitor.

  Wiring (ESP32 example, default VSPI pins):
    LE    -> GPIO 5   (any GPIO, passed to begin())
    CLK   -> GPIO 18  (SPI SCK)
    DATA  -> GPIO 23  (SPI MOSI)
    CE    -> tie high (chip enable, or drive with a GPIO)
    REFIN -> your reference source (e.g. TCXO, or SI5351 output)
    RFINA -> VCO output, AC-coupled
    RFINB -> decoupled to ground with a 100 pF cap (single-ended use)

  NOTE: the ADF4159's digital I/O is 1.8V logic - see the README's
  voltage warning before wiring this to a 3.3V/5V MCU directly.
*/

#include <SPI.h>
#include "ADF4159.h"

const int LE_PIN = 5;
const uint32_t REFIN_HZ = 100000000UL;      // 100 MHz reference
const uint64_t RFOUT_HZ = 12002000000ULL;   // 12.002 GHz target output

ADF4159 synth;

void setup() {
  Serial.begin(115200);
  delay(200);

  // Uses the default SPI bus and this platform's default SPI clock
  // (2 MHz on AVR, 4 MHz elsewhere). To use a non-default bus, e.g.
  // ESP32 HSPI:  SPIClass hspi(HSPI); hspi.begin(14,12,13,-1);
  //              synth.begin(LE_PIN, REFIN_HZ, hspi);
  synth.begin(LE_PIN, REFIN_HZ);

  bool ok = synth.setFrequency(RFOUT_HZ);

  Serial.println(ok ? "RFOUT programmed." : "Requested frequency out of range (0.5-13 GHz).");
  Serial.println();

  synth.printStatus(); // dumps every register + decoded field + RFOUT/fPFD
}

void loop() {
  // Nothing to do - the ADF4159 free-runs at the programmed frequency.
  // To retune, just call synth.setFrequency(newFreqHz) again; it only
  // rewrites R1 and R0 after the first call, no power cycling involved.
}
