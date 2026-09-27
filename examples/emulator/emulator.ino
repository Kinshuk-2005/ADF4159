/*
  emulator.ino

  Emulates the ADF4159's 3-wire register-write interface (CLK, DATA, LE)
  on a second board, so you can test the ADF4159 controller library
  (this repo's ADF4159.h/.cpp) against something other than the real
  chip - no RF hardware needed to exercise the SPI/register logic.

  This single sketch runs unmodified as the emulator on EITHER an ESP32
  or a Raspberry Pi Pico (RP2040) - pin numbers and the ISR-attribute
  macro below are selected automatically based on which board you
  compile it for in the Arduino IDE. It works as the emulator no matter
  what's controlling it (an ESP32 or a Pico both running this repo's
  ADF4159 library, since RP2040 is already one of the architectures
  that library supports directly - no separate "Pico controller" sketch
  is needed for that role).

  This does NOT emulate RF output, MUXOUT, or readback. Every time it
  receives a 32-bit write, it updates its own shadow copy of all 8
  registers and prints a FULL snapshot: every register's raw hex value,
  plus every register fully decoded (R0-R7) - not just the one that was
  just written - so you always see the complete current configuration,
  mirroring the controller library's own printStatus().

  ---------------------------------------------------------------
  WHY THIS IS BIT-BANGED, NOT HARDWARE SPI
  ---------------------------------------------------------------
  The ADF4159 has no MISO-equivalent pin and no chip-select in the
  usual sense. LE acts like an inverted, delayed chip-select: it's
  held LOW while 32 bits are clocked in on the rising edges of CLK,
  then pulsed HIGH afterward to latch the shift register into the
  addressed internal register. That's a different shape than a normal
  SPI slave transaction (CS low-for-duration, not low-then-pulse-high-
  after), so this sketch just uses two GPIO interrupts - one on CLK to
  shift bits in, one on LE's rising edge to latch and flag a complete
  word - rather than either board's SPI slave/peripheral hardware.

  Because this is interrupt-driven, keep the controller's SPI clock
  modest for reliable capture. On the controller side:
      synth.begin(LE_PIN, REFIN_HZ, SPI, 200000UL); // 200 kHz
  The library's default (2-4 MHz) is too fast for interrupt-based
  sampling on the Arduino framework to reliably keep up with, on
  either board.

  ---------------------------------------------------------------
  WIRING
  ---------------------------------------------------------------
  ESP32 and RP2040 are both 3.3V logic, so any ESP32<->ESP32,
  ESP32<->Pico, or Pico<->Pico pairing connects directly - no level
  shifter needed. (The real ADF4159 chip is 1.8V logic and DOES need
  one - that requirement is about the real chip, not about any of
  these board-to-board test pairings.)

    Controller                    Emulator (this sketch)
    -----------------------       -----------------------------
    LE_PIN (per library call) --> PIN_LE
    SPI SCK                   --> PIN_CLK
    SPI MOSI                  --> PIN_DATA
    GND                       --- GND   (must be common to both boards)

  If the controller is an ESP32: SCK=GPIO18, MOSI=GPIO23 by default
  (unless you remapped the SPI bus - see the "Multiple SPI buses"
  section of the library README).

  If the controller is a Pico: SCK=GP18, MOSI=GP19 by default (the
  arduino-pico core's default SPI0 pins) - just call synth.begin() the
  same way as on ESP32, since RP2040 is already a supported
  architecture; no separate controller sketch is needed for that role.
  Pick any free GPIO (e.g. GP5) for LE_PIN.

  Emulator pin numbers below (PIN_CLK/PIN_DATA/PIN_LE) are chosen per
  board: on a Pico, GPIO23-25/29 aren't broken out to header pins on
  the official board, so this sketch uses low-numbered, always-exposed
  GPIOs there instead of reusing the ESP32 pin numbers.

  MOSI only, one direction - this sketch never drives DATA back, so
  there's no MISO wiring to worry about.
*/

#if defined(ARDUINO_ARCH_RP2040)
  const int PIN_CLK  = 2;
  const int PIN_DATA = 3;
  const int PIN_LE   = 4;
#else // ESP32 (default)
  const int PIN_CLK  = 18;
  const int PIN_DATA = 23;
  const int PIN_LE   = 5;
#endif

// IRAM_ATTR is an ESP32-specific macro (keeps the ISR out of flash, to
// avoid a conflict when flash is briefly disabled during flash access).
// RP2040 doesn't need or define it, so this expands to nothing there.
#if defined(ESP32)
  #define ISR_ATTR IRAM_ATTR
#else
  #define ISR_ATTR
#endif

// Set this to whatever REFIN your controller sketch is actually using,
// so the emulator can decode an approximate RFOUT, not just raw bits.
const uint32_t REFIN_HZ = 100000000UL; // 100 MHz

volatile uint32_t bitBuffer = 0;
volatile int bitCount = 0;
volatile uint32_t latchedWord = 0;
volatile bool wordReady = false;

// Shadow copy of all 8 registers, indexed by control bits (R0..R7).
// Updated on every write; printAllRegisters() decodes the whole set
// together so you always see the complete current configuration, not
// just whichever register happened to arrive most recently.
uint32_t regs[8] = {0, 0, 0, 0, 0, 0, 0, 0};

void ISR_ATTR onClkRising() {
  int bit = digitalRead(PIN_DATA);
  bitBuffer = (bitBuffer << 1) | (uint32_t)bit;
  bitCount++;
  if (bitCount > 32) bitCount = 32; // clamp; a full word is always 32 clocks
}

void ISR_ATTR onLeRising() {
  if (bitCount == 32) {
    latchedWord = bitBuffer;
    wordReady = true;
  }
  bitBuffer = 0;
  bitCount = 0;
}

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(PIN_CLK, INPUT);
  pinMode(PIN_DATA, INPUT);
  pinMode(PIN_LE, INPUT);

  attachInterrupt(digitalPinToInterrupt(PIN_CLK), onClkRising, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_LE), onLeRising, RISING);

  Serial.println(F("ADF4159 emulator ready - waiting for register writes..."));
}

void loop() {
  if (wordReady) {
    uint32_t word;
    noInterrupts();
    word = latchedWord;
    wordReady = false;
    interrupts();

    onWordReceived(word);
  }
}

// ---------------------------------------------------------------
// Decoding - field positions match the ADF4159 datasheet register
// maps (Figures 25-32), same bit layout this repo's ADF4159.cpp uses.
// ---------------------------------------------------------------

int32_t signExtend(uint32_t value, int bits) {
  uint32_t signBit = 1UL << (bits - 1);
  return (int32_t)((value ^ signBit) - signBit);
}

void onWordReceived(uint32_t w) {
  uint8_t ctrl = w & 0x7; // C3 C2 C1 = DB[2:0]
  regs[ctrl] = w;

  Serial.println(F("=========================================="));
  Serial.print(F("New write -> R")); Serial.print(ctrl);
  Serial.print(F(" = 0x")); Serial.println(w, HEX);
  Serial.println();

  printAllRegisters();
}

void printAllRegisters() {
  Serial.println(F("---- Raw register snapshot ----"));
  for (int i = 0; i < 8; i++) {
    Serial.print(F("R")); Serial.print(i);
    Serial.print(F(": 0x")); Serial.println(regs[i], HEX);
  }
  Serial.println();

  Serial.println(F("---- Decoded (all registers) ----"));
  // R2 then R1 first, since printR0()'s RFOUT calculation depends on
  // fPFD (from R2) and FRAC_LSB (from R1) being current.
  printR2(regs[2]);
  printR1(regs[1]);
  printR0(regs[0]);
  printR3(regs[3]);
  printR4(regs[4]);
  printR5(regs[5]);
  printR6(regs[6]);
  printR7(regs[7]);
  Serial.println();
}

double lastFPFD = 0;
uint16_t lastFracLsb = 0;

void printR0(uint32_t w) {
  bool rampOn    = (w >> 31) & 0x1;
  uint8_t mux    = (w >> 27) & 0xF;
  uint16_t intV  = (w >> 15) & 0xFFF;
  uint16_t fMsb  = (w >> 3)  & 0xFFF;

  Serial.println(F("REGISTER R0 (FRAC/INT)"));
  Serial.print(F("  RAMP_ON   : ")); Serial.println(rampOn);
  Serial.print(F("  MUXOUT    : 0b")); Serial.println(mux, BIN);
  Serial.print(F("  INT       : ")); Serial.println(intV);
  Serial.print(F("  FRAC_MSB  : ")); Serial.println(fMsb);

  if (lastFPFD > 0) {
    uint32_t fracTotal = ((uint32_t)fMsb << 13) | lastFracLsb;
    double rfOut = lastFPFD * ((double)intV + (double)fracTotal / 33554432.0);
    Serial.print(F("  -> RFOUT approx: "));
    Serial.print(rfOut / 1e6, 6);
    Serial.println(F(" MHz"));
  }
}

void printR1(uint32_t w) {
  bool phaseEn   = (w >> 28) & 0x1;
  uint16_t fLsb  = (w >> 15) & 0x1FFF;
  int32_t phase  = signExtend((w >> 3) & 0xFFF, 12);

  lastFracLsb = fLsb; // cached for the R0 decode above

  Serial.println(F("REGISTER R1 (LSB FRAC)"));
  Serial.print(F("  PHASE_ADJ_EN : ")); Serial.println(phaseEn);
  Serial.print(F("  FRAC_LSB     : ")); Serial.println(fLsb);
  Serial.print(F("  PHASE_VALUE  : ")); Serial.println(phase);
}

void printR2(uint32_t w) {
  bool csr        = (w >> 28) & 0x1;
  uint8_t cpIdx   = (w >> 24) & 0xF;
  bool presc89    = (w >> 22) & 0x1;
  bool rdiv2      = (w >> 21) & 0x1;
  bool doubler    = (w >> 20) & 0x1;
  uint8_t rCount  = (w >> 15) & 0x1F;
  uint16_t clk1   = (w >> 3)  & 0xFFF;

  uint32_t rFull = (uint32_t)(rCount == 0 ? 32 : rCount) * (rdiv2 ? 2 : 1);
  uint32_t dFull = REFIN_HZ * (doubler ? 2 : 1);
  lastFPFD = (rFull > 0) ? ((double)dFull / (double)rFull) : 0;

  Serial.println(F("REGISTER R2 (R DIVIDER)"));
  Serial.print(F("  CSR           : ")); Serial.println(csr);
  Serial.print(F("  CP_CURRENT_IDX: ")); Serial.println(cpIdx);
  Serial.print(F("  PRESCALER     : ")); Serial.println(presc89 ? "8/9" : "4/5");
  Serial.print(F("  RDIV2         : ")); Serial.println(rdiv2);
  Serial.print(F("  DOUBLER       : ")); Serial.println(doubler);
  Serial.print(F("  R_COUNTER     : ")); Serial.println(rCount == 0 ? 32 : rCount);
  Serial.print(F("  CLK1          : ")); Serial.println(clk1);
  Serial.print(F("  -> fPFD approx: ")); Serial.print(lastFPFD / 1e6, 6); Serial.println(F(" MHz"));
}

void printR3(uint32_t w) {
  uint8_t negBleed = (w >> 22) & 0x7;
  bool negBleedEn  = (w >> 21) & 0x1;
  bool lol         = (w >> 16) & 0x1;
  bool nSel        = (w >> 15) & 0x1;
  bool sdReset     = (w >> 14) & 0x1;
  uint8_t rampMode = (w >> 10) & 0x3;
  bool psk         = (w >> 9)  & 0x1;
  bool fsk         = (w >> 8)  & 0x1;
  bool ldp         = (w >> 7)  & 0x1;
  bool pdPol       = (w >> 6)  & 0x1;
  bool pwrDown     = (w >> 5)  & 0x1;
  bool cpTri       = (w >> 4)  & 0x1;
  bool cntReset    = (w >> 3)  & 0x1;

  const char* rampModeNames[4] = {
    "CONT_SAWTOOTH", "CONT_TRIANGLE", "SINGLE_SAWTOOTH_BURST", "SINGLE_RAMP_BURST"
  };

  Serial.println(F("REGISTER R3 (FUNCTION)"));
  Serial.print(F("  NEG_BLEED_IDX : ")); Serial.println(negBleed);
  Serial.print(F("  NEG_BLEED_EN  : ")); Serial.println(negBleedEn);
  Serial.print(F("  LOL           : ")); Serial.println(lol);
  Serial.print(F("  N_SEL         : ")); Serial.println(nSel);
  Serial.print(F("  SD_RESET      : ")); Serial.println(sdReset);
  Serial.print(F("  RAMP_MODE     : ")); Serial.println(rampModeNames[rampMode]);
  Serial.print(F("  PSK           : ")); Serial.println(psk);
  Serial.print(F("  FSK           : ")); Serial.println(fsk);
  Serial.print(F("  LDP           : ")); Serial.println(ldp ? "6ns" : "14ns");
  Serial.print(F("  PD_POLARITY   : ")); Serial.println(pdPol ? "POSITIVE" : "NEGATIVE");
  Serial.print(F("  POWER_DOWN    : ")); Serial.println(pwrDown);
  Serial.print(F("  CP_THREE_STATE: ")); Serial.println(cpTri);
  Serial.print(F("  COUNTER_RESET : ")); Serial.println(cntReset);
}

void printR4(uint32_t w) {
  bool leSel         = (w >> 31) & 0x1;
  uint8_t sdMode     = (w >> 26) & 0x1F;
  uint8_t rampStat   = (w >> 21) & 0x1F;
  uint8_t clkDivMode = (w >> 19) & 0x3;
  uint16_t clk2      = (w >> 7)  & 0xFFF;
  bool clkDivSel     = (w >> 6)  & 0x1;

  Serial.println(F("REGISTER R4 (CLOCK)"));
  Serial.print(F("  LE_SEL       : ")); Serial.println(leSel);
  Serial.print(F("  SD_MODE      : 0b")); Serial.println(sdMode, BIN);
  Serial.print(F("  RAMP_STATUS  : 0b")); Serial.println(rampStat, BIN);
  Serial.print(F("  CLK_DIV_MODE : 0b")); Serial.println(clkDivMode, BIN);
  Serial.print(F("  CLK2         : ")); Serial.println(clk2);
  Serial.print(F("  CLK_DIV_SEL  : ")); Serial.println(clkDivSel ? "Ramp2" : "Ramp1");
}

void printR5(uint32_t w) {
  bool txInvert    = (w >> 30) & 0x1;
  bool txRampClk   = (w >> 29) & 0x1;
  bool parabolic   = (w >> 28) & 0x1;
  uint8_t intMode  = (w >> 26) & 0x3;
  bool fskRamp     = (w >> 25) & 0x1;
  bool dualRamp    = (w >> 24) & 0x1;
  bool devSel      = (w >> 23) & 0x1;
  uint8_t devOff   = (w >> 19) & 0xF;
  int32_t devWord  = signExtend((w >> 3) & 0xFFFF, 16);

  Serial.println(F("REGISTER R5 (DEVIATION)"));
  Serial.print(F("  TXDATA_INVERT   : ")); Serial.println(txInvert);
  Serial.print(F("  TXDATA_RAMP_CLK : ")); Serial.println(txRampClk);
  Serial.print(F("  PARABOLIC_RAMP  : ")); Serial.println(parabolic);
  Serial.print(F("  INTERRUPT_MODE  : 0b")); Serial.println(intMode, BIN);
  Serial.print(F("  FSK_RAMP_EN     : ")); Serial.println(fskRamp);
  Serial.print(F("  DUAL_RAMP_EN    : ")); Serial.println(dualRamp);
  Serial.print(F("  DEV_SEL         : ")); Serial.println(devSel ? "Word 2" : "Word 1");
  Serial.print(F("  DEV_OFFSET      : ")); Serial.println(devOff);
  Serial.print(F("  DEV_WORD        : ")); Serial.println(devWord);
}

void printR6(uint32_t w) {
  bool stepSel      = (w >> 23) & 0x1;
  uint32_t stepWord = (w >> 3)  & 0xFFFFF;

  Serial.println(F("REGISTER R6 (STEP)"));
  Serial.print(F("  STEP_SEL  : ")); Serial.println(stepSel ? "Word 2" : "Word 1");
  Serial.print(F("  STEP_WORD : ")); Serial.println(stepWord);
}

void printR7(uint32_t w) {
  bool txTrigDelay   = (w >> 23) & 0x1;
  bool triDelay      = (w >> 22) & 0x1;
  bool singleFullTri = (w >> 21) & 0x1;
  bool txTrigger     = (w >> 20) & 0x1;
  bool fastRamp      = (w >> 19) & 0x1;
  bool rampDelayFL   = (w >> 18) & 0x1;
  bool rampDelay     = (w >> 17) & 0x1;
  bool delClkSel     = (w >> 16) & 0x1;
  bool delStartEn    = (w >> 15) & 0x1;
  uint16_t delayWord = (w >> 3)  & 0xFFF;

  Serial.println(F("REGISTER R7 (DELAY)"));
  Serial.print(F("  TXDATA_TRIG_DELAY: ")); Serial.println(txTrigDelay);
  Serial.print(F("  TRI_DELAY        : ")); Serial.println(triDelay);
  Serial.print(F("  SINGLE_FULL_TRI  : ")); Serial.println(singleFullTri);
  Serial.print(F("  TXDATA_TRIGGER   : ")); Serial.println(txTrigger);
  Serial.print(F("  FAST_RAMP        : ")); Serial.println(fastRamp);
  Serial.print(F("  RAMP_DELAY_FL    : ")); Serial.println(rampDelayFL);
  Serial.print(F("  RAMP_DELAY       : ")); Serial.println(rampDelay);
  Serial.print(F("  DEL_CLK_SEL      : ")); Serial.println(delClkSel);
  Serial.print(F("  DEL_START_EN     : ")); Serial.println(delStartEn);
  Serial.print(F("  DELAY_START_WORD : ")); Serial.println(delayWord);
}
