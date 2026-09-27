/*
  ADF4159.cpp - Arduino library for the Analog Devices ADF4159
  See ADF4159.h for full notes on wiring, register sequencing, and
  the single-ended/differential RF input handling.
*/

#include "ADF4159.h"

static const uint64_t ADF4159_MODULUS = 33554432ULL; // 2^25

// Portable round-to-long helper. Avoids relying on C99 lround()/llround()
// being present in every core's math library (older AVR toolchains have
// been inconsistent about this) - a plain compare-and-add is enough for
// the value ranges used here (deviation words, phase words, CLK dividers).
static inline long adf4159_round(double x) {
  return (long)(x >= 0.0 ? (x + 0.5) : (x - 0.5));
}

ADF4159::ADF4159()
: _spi(&SPI), _lePin(-1), _refIn(0), _spiHz(ADF4159_DEFAULT_SPI_HZ),
  _fPFD(0), _initialized(false),
  _r0(0), _r1(0), _r2(0), _r3(0), _r4(0), _r5(0), _r6(0), _r7(0),
  _r4b(0), _r5b(0), _r6b(0)
{
}

void ADF4159::begin(int lePin, uint32_t refInHz, SPIClass &spiPort,
                     uint32_t spiHz, bool initSPI) {
  _lePin = lePin;
  _refIn = refInHz;
  _spi = &spiPort;
  _spiHz = spiHz;
  _initialized = false;

  pinMode(_lePin, OUTPUT);
  digitalWrite(_lePin, LOW);

  if (initSPI) {
    _spi->begin();
  }
}

// ---------------------------------------------------------------
// Low level
// ---------------------------------------------------------------
void ADF4159::writeRegister(uint32_t value) {
  _spi->beginTransaction(SPISettings(_spiHz, MSBFIRST, SPI_MODE0));
  digitalWrite(_lePin, LOW);
  _spi->transfer((uint8_t)((value >> 24) & 0xFF));
  _spi->transfer((uint8_t)((value >> 16) & 0xFF));
  _spi->transfer((uint8_t)((value >> 8) & 0xFF));
  _spi->transfer((uint8_t)(value & 0xFF));
  _spi->endTransaction();
  // LE rising edge latches the shift register into the target latch
  digitalWrite(_lePin, HIGH);
  delayMicroseconds(1);
  digitalWrite(_lePin, LOW);
}

// ---------------------------------------------------------------
// Per-register shadow setters
// ---------------------------------------------------------------
void ADF4159::setR0(bool rampOn, uint8_t muxout, uint16_t intVal, uint16_t fracMsb) {
  _r0 = ((uint32_t)(rampOn ? 1 : 0) << 31)
      | (((uint32_t)muxout & 0xF) << 27)
      | (((uint32_t)intVal & 0xFFF) << 15)
      | (((uint32_t)fracMsb & 0xFFF) << 3)
      | 0b000;
}

void ADF4159::setR1(bool phaseAdjEn, uint16_t fracLsb, int16_t phaseVal) {
  _r1 = ((uint32_t)(phaseAdjEn ? 1 : 0) << 28)
      | (((uint32_t)fracLsb & 0x1FFF) << 15)
      | (((uint32_t)phaseVal & 0xFFF) << 3)
      | 0b001;
}

void ADF4159::setR2(bool csr, uint8_t cpCurrentIdx, bool prescaler89, bool rdiv2,
                     bool doubler, uint8_t rCounter, uint16_t clk1) {
  _r2 = ((uint32_t)(csr ? 1 : 0) << 28)
      | (((uint32_t)cpCurrentIdx & 0xF) << 24)
      | ((uint32_t)(prescaler89 ? 1 : 0) << 22)
      | ((uint32_t)(rdiv2 ? 1 : 0) << 21)
      | ((uint32_t)(doubler ? 1 : 0) << 20)
      | (((uint32_t)rCounter & 0x1F) << 15)
      | (((uint32_t)clk1 & 0xFFF) << 3)
      | 0b010;
}

void ADF4159::setR3(uint8_t negBleedCurrentIdx, bool negBleedEn, bool lol, bool nSel,
                     bool sdReset, uint8_t rampMode, bool psk, bool fsk, bool ldp,
                     bool pdPolarityPos, bool powerDown, bool cpThreeState,
                     bool counterReset) {
  _r3 = (((uint32_t)negBleedCurrentIdx & 0x7) << 22)
      | ((uint32_t)(negBleedEn ? 1 : 0) << 21)
      | (1UL << 17)  // reserved, must be 1 per datasheet
      | ((uint32_t)(lol ? 1 : 0) << 16)
      | ((uint32_t)(nSel ? 1 : 0) << 15)
      | ((uint32_t)(sdReset ? 1 : 0) << 14)
      | (((uint32_t)rampMode & 0x3) << 10)
      | ((uint32_t)(psk ? 1 : 0) << 9)
      | ((uint32_t)(fsk ? 1 : 0) << 8)
      | ((uint32_t)(ldp ? 1 : 0) << 7)
      | ((uint32_t)(pdPolarityPos ? 1 : 0) << 6)
      | ((uint32_t)(powerDown ? 1 : 0) << 5)
      | ((uint32_t)(cpThreeState ? 1 : 0) << 4)
      | ((uint32_t)(counterReset ? 1 : 0) << 3)
      | 0b011;
}

void ADF4159::setR4(bool leSel, uint8_t sdMode, uint8_t rampStatus, uint8_t clkDivMode,
                     uint16_t clk2, bool clkDivSel) {
  _r4 = ((uint32_t)(leSel ? 1 : 0) << 31)
      | (((uint32_t)sdMode & 0x1F) << 26)
      | (((uint32_t)rampStatus & 0x1F) << 21)
      | (((uint32_t)clkDivMode & 0x3) << 19)
      | (((uint32_t)clk2 & 0xFFF) << 7)
      | ((uint32_t)(clkDivSel ? 1 : 0) << 6)
      | 0b100;
}

void ADF4159::setR5(bool txDataInvert, bool txDataRampClk, bool parabolicRamp,
                     uint8_t interruptMode, bool fskRampEn, bool dualRampEn,
                     bool devSel, uint8_t devOffset, int16_t devWord) {
  _r5 = ((uint32_t)(txDataInvert ? 1 : 0) << 30)
      | ((uint32_t)(txDataRampClk ? 1 : 0) << 29)
      | ((uint32_t)(parabolicRamp ? 1 : 0) << 28)
      | (((uint32_t)interruptMode & 0x3) << 26)
      | ((uint32_t)(fskRampEn ? 1 : 0) << 25)
      | ((uint32_t)(dualRampEn ? 1 : 0) << 24)
      | ((uint32_t)(devSel ? 1 : 0) << 23)
      | (((uint32_t)devOffset & 0xF) << 19)
      | (((uint32_t)devWord & 0xFFFF) << 3)
      | 0b101;
}

void ADF4159::setR6(bool stepSel, uint32_t stepWord) {
  _r6 = ((uint32_t)(stepSel ? 1 : 0) << 23)
      | ((stepWord & 0xFFFFF) << 3)
      | 0b110;
}

void ADF4159::setR7(bool txDataTrigDelay, bool triDelay, bool singleFullTri,
                     bool txDataTrigger, bool fastRamp, bool rampDelayFL,
                     bool rampDelay, bool delClkSel, bool delStartEn,
                     uint16_t delayStartWord) {
  _r7 = ((uint32_t)(txDataTrigDelay ? 1 : 0) << 23)
      | ((uint32_t)(triDelay ? 1 : 0) << 22)
      | ((uint32_t)(singleFullTri ? 1 : 0) << 21)
      | ((uint32_t)(txDataTrigger ? 1 : 0) << 20)
      | ((uint32_t)(fastRamp ? 1 : 0) << 19)
      | ((uint32_t)(rampDelayFL ? 1 : 0) << 18)
      | ((uint32_t)(rampDelay ? 1 : 0) << 17)
      | ((uint32_t)(delClkSel ? 1 : 0) << 16)
      | ((uint32_t)(delStartEn ? 1 : 0) << 15)
      | (((uint32_t)delayStartWord & 0xFFF) << 3)
      | 0b111;
}

// ---------------------------------------------------------------
// Status / readback (of this library's own shadow state)
// ---------------------------------------------------------------
ADF4159_Status ADF4159::readStatus() const {
  ADF4159_Status s;

  s.r0 = _r0; s.r1 = _r1; s.r2 = _r2; s.r3 = _r3;
  s.r4 = _r4; s.r5 = _r5; s.r6 = _r6; s.r7 = _r7;

  s.fPFD = _fPFD;
  s.resolutionHz = getResolution();

  s.rampOn = _cfg.rampOn;
  s.muxout = _cfg.muxout;
  s.intVal = _cfg.intVal;
  s.fracMsb = _cfg.fracMsb;
  s.fracLsb = _cfg.fracLsb;
  s.fracTotal = ((uint32_t)_cfg.fracMsb << 13) | _cfg.fracLsb;
  s.phaseAdjEn = _cfg.phaseAdjEn;
  s.phaseVal = _cfg.phaseVal;

  s.csr = _cfg.csr;
  s.cpCurrentIdx = _cfg.cpCurrentIdx;
  s.prescaler89 = _cfg.prescaler89;
  s.rdiv2 = _cfg.rdiv2;
  s.doubler = _cfg.doubler;
  s.rCounter = _cfg.rCounter;
  s.clk1 = _cfg.clk1;

  s.fsk = _cfg.fsk;
  s.psk = _cfg.psk;
  s.powerDown = _cfg.powerDown;
  s.rampMode = _cfg.rampMode;

  s.clk2 = _cfg.clk2;
  s.clk2b = _cfg.clk2b;
  s.rampStatus = _cfg.rampStatus;

  s.dualRampEn = _cfg.dualRampEn;
  s.fastRamp = _cfg.fastRamp;
  s.parabolicRamp = _cfg.parabolicRamp;
  s.interruptMode = _cfg.interruptMode;
  s.devOffset = _cfg.devOffset;
  s.devWord = _cfg.devWord;
  s.devOffset2 = _cfg.devOffset2;
  s.devWord2 = _cfg.devWord2;

  s.stepWord = _cfg.stepWord;
  s.stepWord2 = _cfg.stepWord2;

  s.rampDelay = _cfg.rampDelay;
  s.rampDelayFL = _cfg.rampDelayFL;
  s.delStartEn = _cfg.delStartEn;
  s.delClkSel = _cfg.delClkSel;
  s.delayStartWord = _cfg.delayStartWord;

  if (s.fPFD > 0) {
    s.rfOutHz = s.fPFD * ((double)s.intVal + (double)s.fracTotal / (double)ADF4159_MODULUS);
  } else {
    s.rfOutHz = 0;
  }

  return s;
}

void ADF4159::printStatus(Print &out) const {
  ADF4159_Status s = readStatus();

  out.println(F("=== ADF4159 status ==="));
  out.print(F("R0: 0x")); out.println(s.r0, HEX);
  out.print(F("R1: 0x")); out.println(s.r1, HEX);
  out.print(F("R2: 0x")); out.println(s.r2, HEX);
  out.print(F("R3: 0x")); out.println(s.r3, HEX);
  out.print(F("R4: 0x")); out.println(s.r4, HEX);
  out.print(F("R5: 0x")); out.println(s.r5, HEX);
  out.print(F("R6: 0x")); out.println(s.r6, HEX);
  out.print(F("R7: 0x")); out.println(s.r7, HEX);

  out.print(F("RFOUT      : ")); out.print(s.rfOutHz / 1e6, 6); out.println(F(" MHz"));
  out.print(F("fPFD       : ")); out.print(s.fPFD / 1e6, 6); out.println(F(" MHz"));
  out.print(F("Resolution : ")); out.print(s.resolutionHz, 4); out.println(F(" Hz"));

  out.print(F("INT=")); out.print(s.intVal);
  out.print(F(" FRAC_MSB=")); out.print(s.fracMsb);
  out.print(F(" FRAC_LSB=")); out.print(s.fracLsb);
  out.print(F(" FRAC_TOTAL=")); out.println(s.fracTotal);

  out.print(F("R_COUNTER=")); out.print(s.rCounter);
  out.print(F(" DOUBLER=")); out.print(s.doubler);
  out.print(F(" RDIV2=")); out.print(s.rdiv2);
  out.print(F(" PRESCALER=")); out.println(s.prescaler89 ? "8/9" : "4/5");

  out.print(F("RAMP_ON=")); out.print(s.rampOn);
  out.print(F(" RAMP_MODE=")); out.print(s.rampMode);
  out.print(F(" DUAL_RAMP=")); out.print(s.dualRampEn);
  out.print(F(" FAST_RAMP=")); out.print(s.fastRamp);
  out.print(F(" PARABOLIC=")); out.println(s.parabolicRamp);

  out.print(F("FSK=")); out.print(s.fsk);
  out.print(F(" PSK=")); out.print(s.psk);
  out.print(F(" POWER_DOWN=")); out.println(s.powerDown);

  out.print(F("CLK1=")); out.print(s.clk1);
  out.print(F(" CLK2(Ramp1)=")); out.print(s.clk2);
  out.print(F(" CLK2(Ramp2)=")); out.println(s.clk2b);

  out.print(F("STEP_WORD(Ramp1)=")); out.print(s.stepWord);
  out.print(F(" STEP_WORD(Ramp2)=")); out.println(s.stepWord2);

  out.print(F("DEV(Ramp1) offset=")); out.print(s.devOffset);
  out.print(F(" word=")); out.print(s.devWord);
  out.print(F(" | DEV(Ramp2) offset=")); out.print(s.devOffset2);
  out.print(F(" word=")); out.println(s.devWord2);

  out.print(F("PHASE_ADJ_EN=")); out.print(s.phaseAdjEn);
  out.print(F(" PHASE_VAL=")); out.println(s.phaseVal);

  out.print(F("RAMP_DELAY=")); out.print(s.rampDelay);
  out.print(F(" RAMP_DELAY_FL=")); out.print(s.rampDelayFL);
  out.print(F(" DEL_START_EN=")); out.print(s.delStartEn);
  out.print(F(" DEL_CLK_SEL=")); out.print(s.delClkSel);
  out.print(F(" DELAY_START_WORD=")); out.println(s.delayStartWord);

  out.print(F("MUXOUT=0b")); out.println(s.muxout, BIN);
  out.print(F("RAMP_STATUS=0b")); out.println(s.rampStatus, BIN);
  out.print(F("INTERRUPT_MODE=0b")); out.println(s.interruptMode, BIN);
  out.print(F("CSR=")); out.print(s.csr);
  out.print(F(" CP_CURRENT_IDX=")); out.println(s.cpCurrentIdx);
}

// ---------------------------------------------------------------
// Sequencing
// ---------------------------------------------------------------
void ADF4159::writeFullSequence() {
  writeRegister(_r7);
  writeRegister(_r6);   // Step Word 1 (Ramp 1)
  writeRegister(_r6b);  // Step Word 2 (Ramp 2)
  writeRegister(_r5);   // Deviation Word 1 (Ramp 1)
  writeRegister(_r5b);  // Deviation Word 2 (Ramp 2)
  writeRegister(_r4);   // CLK2 for Ramp 1
  writeRegister(_r4b);  // CLK2 for Ramp 2
  writeRegister(_r3);
  writeRegister(_r2);
  writeRegister(_r1);
  writeRegister(_r0);
}

void ADF4159::updateFrequencyOnly() {
  writeRegister(_r1);
  writeRegister(_r0);
}

// ---------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------
bool ADF4159::stageFrequency(uint64_t freqHz, uint8_t rCounter, bool doubler, bool rdiv2) {
  _cfg.rCounter = rCounter;
  _cfg.doubler = doubler;
  _cfg.rdiv2 = rdiv2;

  // Effective REFIN after doubler ("dFull"), and effective R-divide
  // factor after RDIV2 ("rFull") - both exact integers.
  //   fPFD = dFull / rFull
  //   N    = freqHz / fPFD = freqHz * rFull / dFull
  uint64_t dFull = (uint64_t)_refIn * (doubler ? 2ULL : 1ULL);
  uint32_t rFull = (uint32_t)rCounter * (rdiv2 ? 2U : 1U);
  if (dFull == 0 || rFull == 0) return false;

  // Approximate fPFD (double) - fine for display, ramp timing and
  // deviation math, which don't need sub-Hz precision. The exact
  // synthesis below never uses this value.
  _fPFD = (double)dFull / (double)rFull;

  _cfg.prescaler89 = freqHz > 8000000000ULL;

  // Exact 64-bit integer INT/FRAC split - no floating point, so this
  // is bit-exact on every platform including AVR.
  uint64_t product = freqHz * (uint64_t)rFull;
  uint64_t intVal64 = product / dFull;
  uint64_t remainder = product % dFull;

  uint64_t fracScaled = remainder * ADF4159_MODULUS;
  uint64_t fracTotal = (fracScaled + dFull / 2) / dFull; // rounding division

  if (fracTotal >= ADF4159_MODULUS) {
    fracTotal -= ADF4159_MODULUS;
    intVal64 += 1;
  }

  uint16_t nMin = _cfg.prescaler89 ? 75 : 23;
  if (intVal64 < nMin || intVal64 > 4095) return false;

  _cfg.intVal = (uint16_t)intVal64;
  _cfg.fracMsb = (uint16_t)((fracTotal >> 13) & 0xFFF);
  _cfg.fracLsb = (uint16_t)(fracTotal & 0x1FFF);
  return true;
}

void ADF4159::computeDeviation(double devHz, uint8_t &devOffsetOut, int16_t &devWordOut) {
  double fRES = _fPFD / (double)ADF4159_MODULUS;

  double devOffsetF = log10(fabs(devHz) / (fRES * 32768.0)) / log10(2.0);
  int devOffset = (int)adf4159_round(devOffsetF);
  if (devOffset < 0) devOffset = 0;
  if (devOffset > 15) devOffset = 15;

  long devWordL = adf4159_round(devHz / (fRES * (double)(1UL << devOffset)));
  if (devWordL > 32767) devWordL = 32767;
  if (devWordL < -32768) devWordL = -32768;

  devOffsetOut = (uint8_t)devOffset;
  devWordOut = (int16_t)devWordL;
}

void ADF4159::computeRampTimer(double stepTimeUs, uint16_t &clk1Out, uint16_t &clk2Out) {
  double timerTicks = (stepTimeUs * 1.0e-6) * _fPFD;
  uint32_t clk2 = 1;
  uint32_t clk1 = (uint32_t)adf4159_round(timerTicks);
  if (clk1 < 1) clk1 = 1;
  if (clk1 > 4095) {
    clk2 = (clk1 + 4094) / 4095;
    clk1 = (uint32_t)adf4159_round(timerTicks / (double)clk2);
    if (clk1 < 1) clk1 = 1;
    if (clk1 > 4095) clk1 = 4095;
  }
  if (clk2 > 4095) clk2 = 4095;

  clk1Out = (uint16_t)clk1;
  clk2Out = (uint16_t)clk2;
}

uint16_t ADF4159::clk2For(double stepTimeUs, uint16_t clk1) {
  if (clk1 == 0) clk1 = 1;
  double ticks = (stepTimeUs * 1.0e-6 * _fPFD) / (double)clk1;
  long v = adf4159_round(ticks);
  if (v < 1) v = 1;
  if (v > 4095) v = 4095;
  return (uint16_t)v;
}

void ADF4159::pushAll() {
  if (!_cfg.independentRamp2) {
    // Normal (non dual/fast ramp) case: mirror Ramp 1 into Ramp 2 so both
    // SEL=0/SEL=1 latches get valid, identical data - matching the
    // datasheet's own power-up initialization sequence.
    _cfg.devOffset2 = _cfg.devOffset;
    _cfg.devWord2   = _cfg.devWord;
    _cfg.stepWord2  = _cfg.stepWord;
    _cfg.clk2b      = _cfg.clk2;
  }

  setR7(_cfg.txDataTrigDelay, _cfg.triDelay, _cfg.singleFullTri, _cfg.txDataTrigger,
        _cfg.fastRamp, _cfg.rampDelayFL, _cfg.rampDelay, _cfg.delClkSel,
        _cfg.delStartEn, _cfg.delayStartWord);

  setR6(false, _cfg.stepWord);
  uint32_t r6a = _r6;
  setR6(true, _cfg.stepWord2);
  uint32_t r6b = _r6;

  setR5(_cfg.txDataInvert, _cfg.txDataRampClk, _cfg.parabolicRamp, _cfg.interruptMode,
        _cfg.fskRampEn, _cfg.dualRampEn, false, _cfg.devOffset, _cfg.devWord);
  uint32_t r5a = _r5;
  setR5(_cfg.txDataInvert, _cfg.txDataRampClk, _cfg.parabolicRamp, _cfg.interruptMode,
        _cfg.fskRampEn, _cfg.dualRampEn, true, _cfg.devOffset2, _cfg.devWord2);
  uint32_t r5b = _r5;

  setR4(_cfg.leSel, _cfg.sdMode, _cfg.rampStatus, _cfg.clkDivMode, _cfg.clk2, false);
  uint32_t r4a = _r4;
  setR4(_cfg.leSel, _cfg.sdMode, _cfg.rampStatus, _cfg.clkDivMode, _cfg.clk2b, true);
  uint32_t r4b = _r4;

  setR3(_cfg.negBleedIdx, _cfg.negBleedEn, _cfg.lol, _cfg.nSel, _cfg.sdReset,
        _cfg.rampMode, _cfg.psk, _cfg.fsk, _cfg.ldp, _cfg.pdPolarityPos,
        _cfg.powerDown, _cfg.cpThreeState, _cfg.counterReset);
  setR2(_cfg.csr, _cfg.cpCurrentIdx, _cfg.prescaler89, _cfg.rdiv2, _cfg.doubler,
        _cfg.rCounter, _cfg.clk1);
  setR1(_cfg.phaseAdjEn, _cfg.fracLsb, _cfg.phaseVal);
  setR0(_cfg.rampOn, _cfg.muxout, _cfg.intVal, _cfg.fracMsb);

  _r6 = r6a; _r6b = r6b;
  _r5 = r5a; _r5b = r5b;
  _r4 = r4a; _r4b = r4b;

  writeFullSequence();
  _initialized = true;
}

// ---------------------------------------------------------------
// High level
// ---------------------------------------------------------------
bool ADF4159::setFrequency(uint64_t freqHz, uint8_t rCounter, bool doubler, bool rdiv2) {
  if (freqHz < 500000000ULL || freqHz > 13000000000ULL) return false;

  if (!stageFrequency(freqHz, rCounter, doubler, rdiv2)) return false;

  if (!_initialized) {
    pushAll();
  } else {
    setR1(_cfg.phaseAdjEn, _cfg.fracLsb, _cfg.phaseVal);
    setR0(_cfg.rampOn, _cfg.muxout, _cfg.intVal, _cfg.fracMsb);
    updateFrequencyOnly();
  }
  return true;
}

void ADF4159::configureSawtoothRamp(uint64_t startFreq, uint64_t stopFreq,
                                     uint16_t numSteps, double stepTimeUs,
                                     bool continuous) {
  if (numSteps == 0) numSteps = 1;

  if (!stageFrequency(startFreq, _cfg.rCounter, _cfg.doubler, _cfg.rdiv2)) return;

  double bandwidth = (double)((int64_t)stopFreq - (int64_t)startFreq);
  double devPerStepHz = bandwidth / (double)numSteps;

  computeDeviation(devPerStepHz, _cfg.devOffset, _cfg.devWord);
  computeRampTimer(stepTimeUs, _cfg.clk1, _cfg.clk2);

  _cfg.stepWord = numSteps;
  _cfg.clkDivMode = 0b11; // ramp divider mode, required for ramp timing
  _cfg.independentRamp2 = false;
  _cfg.dualRampEn = false;
  _cfg.fastRamp = false;
  _cfg.fskRampEn = false;
  _cfg.parabolicRamp = false;
  _cfg.rampMode = continuous ? ADF4159_RAMP_CONT_SAWTOOTH
                              : ADF4159_RAMP_SINGLE_SAWTOOTH_BURST;
  _cfg.singleFullTri = false;
  _cfg.rampOn = true;

  pushAll();
}

void ADF4159::configureTriangleRamp(uint64_t startFreq, uint64_t stopFreq,
                                     uint16_t numSteps, double stepTimeUs,
                                     bool continuous) {
  // Build all the same ramp parameters as the sawtooth case, then override
  // just the ramp-mode-related fields for triangle behaviour.
  configureSawtoothRamp(startFreq, stopFreq, numSteps, stepTimeUs, continuous);

  _cfg.rampMode = continuous ? ADF4159_RAMP_CONT_TRIANGLE
                              : ADF4159_RAMP_SINGLE_RAMP_BURST;
  _cfg.singleFullTri = !continuous; // single full triangle needs Ramp Mode = 11
  pushAll();
}

void ADF4159::configureFMCWChirp(uint64_t startFreq, uint64_t bandwidthHz,
                                  double sweepTimeMs, uint16_t numSteps) {
  uint64_t stopFreq = startFreq + bandwidthHz;
  double stepTimeUs = (sweepTimeMs * 1000.0) / (double)numSteps;
  configureSawtoothRamp(startFreq, stopFreq, numSteps, stepTimeUs, true);
}

void ADF4159::configureDualRamp(uint64_t centerFreq, double stepTimeUs,
                                 double devHz1, uint16_t numSteps1,
                                 double devHz2, uint16_t numSteps2,
                                 ADF4159_RampMode rampMode) {
  if (numSteps1 == 0) numSteps1 = 1;
  if (numSteps2 == 0) numSteps2 = 1;

  if (!stageFrequency(centerFreq, _cfg.rCounter, _cfg.doubler, _cfg.rdiv2)) return;

  uint16_t clk1, clk2;
  computeRampTimer(stepTimeUs, clk1, clk2);
  _cfg.clk1 = clk1;
  _cfg.clk2 = clk2;
  _cfg.clk2b = clk2; // shared timer, per the datasheet's own dual-ramp worked example

  computeDeviation(devHz1, _cfg.devOffset, _cfg.devWord);
  computeDeviation(devHz2, _cfg.devOffset2, _cfg.devWord2);

  _cfg.stepWord = numSteps1;
  _cfg.stepWord2 = numSteps2;
  _cfg.independentRamp2 = true;
  _cfg.dualRampEn = true;
  _cfg.fastRamp = false;
  _cfg.fskRampEn = false;
  _cfg.parabolicRamp = false;
  _cfg.clkDivMode = 0b11;
  _cfg.rampMode = rampMode;
  _cfg.singleFullTri = false;
  _cfg.rampOn = true;

  pushAll();
}

void ADF4159::configureFastRamp(uint64_t centerFreq,
                                 double upDevHz, uint16_t upSteps, double upStepTimeUs,
                                 double downDevHz, uint16_t downSteps, double downStepTimeUs) {
  if (upSteps == 0) upSteps = 1;
  if (downSteps == 0) downSteps = 1;

  if (!stageFrequency(centerFreq, _cfg.rCounter, _cfg.doubler, _cfg.rdiv2)) return;

  // CLK1 (Register R2) is a single shared register - it can't differ
  // between the up and down ramps. Size it for the up-ramp's step time,
  // then solve CLK2 (which *can* differ per ramp via R4's SEL bit) for
  // each side against that shared CLK1.
  uint16_t clk1, clk2Up;
  computeRampTimer(upStepTimeUs, clk1, clk2Up);
  uint16_t clk2Down = clk2For(downStepTimeUs, clk1);

  _cfg.clk1 = clk1;
  _cfg.clk2 = clk2Up;
  _cfg.clk2b = clk2Down;

  computeDeviation(upDevHz, _cfg.devOffset, _cfg.devWord);
  computeDeviation(downDevHz, _cfg.devOffset2, _cfg.devWord2);

  _cfg.stepWord = upSteps;
  _cfg.stepWord2 = downSteps;
  _cfg.independentRamp2 = true;
  _cfg.fastRamp = true;
  _cfg.dualRampEn = false;
  _cfg.fskRampEn = false;
  _cfg.parabolicRamp = false;
  _cfg.clkDivMode = 0b11;
  _cfg.rampMode = ADF4159_RAMP_CONT_TRIANGLE; // fast ramp requires continuous triangle
  _cfg.singleFullTri = false;
  _cfg.rampOn = true;

  pushAll();
}

void ADF4159::configureParabolicRamp(uint64_t centerFreq, double devHz,
                                      uint16_t numSteps, double stepTimeUs,
                                      bool continuous) {
  if (numSteps == 0) numSteps = 1;

  if (!stageFrequency(centerFreq, _cfg.rCounter, _cfg.doubler, _cfg.rdiv2)) return;

  computeRampTimer(stepTimeUs, _cfg.clk1, _cfg.clk2);
  computeDeviation(devHz, _cfg.devOffset, _cfg.devWord);

  _cfg.stepWord = numSteps;
  _cfg.independentRamp2 = false; // parabolic ramp doesn't use the dual-ramp latches
  _cfg.dualRampEn = false;
  _cfg.fastRamp = false;
  _cfg.fskRampEn = false;
  _cfg.parabolicRamp = true;
  _cfg.clkDivMode = 0b11;
  _cfg.rampMode = continuous ? ADF4159_RAMP_CONT_TRIANGLE : ADF4159_RAMP_SINGLE_RAMP_BURST;
  _cfg.singleFullTri = false;
  _cfg.rampOn = true;

  pushAll();

  // Datasheet: "Set the counter reset (Bit DB3 in Register R3) to 1 and
  // then set it to 0" after activating the parabolic ramp.
  _cfg.counterReset = true;
  setR3(_cfg.negBleedIdx, _cfg.negBleedEn, _cfg.lol, _cfg.nSel, _cfg.sdReset,
        _cfg.rampMode, _cfg.psk, _cfg.fsk, _cfg.ldp, _cfg.pdPolarityPos,
        _cfg.powerDown, _cfg.cpThreeState, _cfg.counterReset);
  writeRegister(_r3);

  _cfg.counterReset = false;
  setR3(_cfg.negBleedIdx, _cfg.negBleedEn, _cfg.lol, _cfg.nSel, _cfg.sdReset,
        _cfg.rampMode, _cfg.psk, _cfg.fsk, _cfg.ldp, _cfg.pdPolarityPos,
        _cfg.powerDown, _cfg.cpThreeState, _cfg.counterReset);
  writeRegister(_r3);
}

void ADF4159::enableRampDelay(double delayUs, bool useClk1Multiplier, bool fastLockDuringDelay) {
  double tPFD = _fPFD > 0 ? 1.0 / _fPFD : 0;
  double denom = tPFD * (useClk1Multiplier ? (double)_cfg.clk1 : 1.0);
  long word = denom > 0 ? adf4159_round((delayUs * 1.0e-6) / denom) : 0;
  if (word < 0) word = 0;
  if (word > 4095) word = 4095;

  _cfg.rampDelay = true;
  _cfg.rampDelayFL = fastLockDuringDelay;
  _cfg.delClkSel = useClk1Multiplier;
  _cfg.delStartEn = false; // shares the delay word/clock-select with delayed start
  _cfg.delayStartWord = (uint16_t)word;

  pushAll();
}

void ADF4159::disableRampDelay() {
  _cfg.rampDelay = false;
  _cfg.rampDelayFL = false;
  pushAll();
}

void ADF4159::enableDelayedStart(double delayUs, bool useClk1Multiplier) {
  double tPFD = _fPFD > 0 ? 1.0 / _fPFD : 0;
  double denom = tPFD * (useClk1Multiplier ? (double)_cfg.clk1 : 1.0);
  long word = denom > 0 ? adf4159_round((delayUs * 1.0e-6) / denom) : 0;
  if (word < 0) word = 0;
  if (word > 4095) word = 4095;

  _cfg.delStartEn = true;
  _cfg.delClkSel = useClk1Multiplier;
  _cfg.rampDelay = false; // shares the delay word/clock-select with ramp delay
  _cfg.delayStartWord = (uint16_t)word;

  pushAll();
}

void ADF4159::disableDelayedStart() {
  _cfg.delStartEn = false;
  pushAll();
}

void ADF4159::enableInterruptReadback(bool stopOnInterrupt) {
  _cfg.muxout = ADF4159_MUX_READBACK;                      // R0 DB[30:27] = 1111
  _cfg.rampStatus = ADF4159_RAMPSTATUS_INTERRUPT_READBACK; // R4 DB[25:21] = 00010
  _cfg.interruptMode = stopOnInterrupt ? 0b11 : 0b01;      // R5 DB[27:26], Table 8
  pushAll();
}

void ADF4159::disableInterruptReadback() {
  _cfg.muxout = ADF4159_MUX_THREE_STATE;
  _cfg.rampStatus = ADF4159_RAMPSTATUS_NORMAL;
  _cfg.interruptMode = 0;
  pushAll();
}

bool ADF4159::readFrequency(int clkPin, int muxoutPin, int txDataPin, double &outFreqHz) {
  if (_fPFD <= 0) return false;

  pinMode(clkPin, OUTPUT);
  pinMode(muxoutPin, INPUT);
  pinMode(txDataPin, OUTPUT);
  digitalWrite(clkPin, LOW);

  // Rising edge on TXDATA triggers the interrupt and latches the current
  // ramp frequency for readback.
  digitalWrite(txDataPin, LOW);
  delayMicroseconds(2);
  digitalWrite(txDataPin, HIGH);
  delayMicroseconds(2);

  // "LE SHOULD BE KEPT HIGH DURING READBACK" - datasheet Figure 3 note.
  digitalWrite(_lePin, HIGH);

  // 37 bits total: 12-bit INT then 25-bit FRAC, MSB first. Data updates
  // on MUXOUT on CLK's rising edge and is read on the falling edge
  // (datasheet Figure 51 caption).
  uint64_t word = 0;
  for (int i = 0; i < 37; i++) {
    digitalWrite(clkPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(clkPin, LOW);
    delayMicroseconds(1);
    int bit = digitalRead(muxoutPin);
    word = (word << 1) | (uint64_t)(bit ? 1 : 0);
  }

  digitalWrite(_lePin, LOW);
  digitalWrite(txDataPin, LOW);

  uint32_t intVal = (uint32_t)((word >> 25) & 0xFFF);
  uint32_t fracVal = (uint32_t)(word & 0x1FFFFFF);

  outFreqHz = _fPFD * ((double)intVal + (double)fracVal / (double)ADF4159_MODULUS);
  return true;
}

void ADF4159::configureFSK(uint64_t centerFreq, double deviationHz, uint8_t rCounter) {
  if (!stageFrequency(centerFreq, rCounter, _cfg.doubler, _cfg.rdiv2)) return;

  computeDeviation(deviationHz, _cfg.devOffset, _cfg.devWord);

  _cfg.independentRamp2 = false;
  _cfg.fskRampEn = false;
  _cfg.dualRampEn = false;
  _cfg.parabolicRamp = false;
  _cfg.fsk = true;
  _cfg.psk = false;
  _cfg.clkDivMode = 0; // no ramp timer needed for plain FSK
  _cfg.rampOn = false;

  pushAll();
}

void ADF4159::configurePSK(double phaseDeg) {
  int phaseVal = (int)adf4159_round((phaseDeg / 360.0) * 4096.0);
  if (phaseVal > 2047) phaseVal = 2047;
  if (phaseVal < -2048) phaseVal = -2048;

  _cfg.phaseAdjEn = true;
  _cfg.phaseVal = (int16_t)phaseVal;
  _cfg.psk = true;
  _cfg.fsk = false;
  _cfg.rampOn = false;

  pushAll();
}

void ADF4159::activateRamp() {
  _cfg.rampOn = true;
  setR0(_cfg.rampOn, _cfg.muxout, _cfg.intVal, _cfg.fracMsb);
  writeRegister(_r0);
}

void ADF4159::deactivateRamp() {
  _cfg.rampOn = false;
  setR0(_cfg.rampOn, _cfg.muxout, _cfg.intVal, _cfg.fracMsb);
  writeRegister(_r0);
}

void ADF4159::enableCycleSlipReduction() {
  // CSR requires positive PD polarity - datasheet is explicit that it
  // cannot be used with negative polarity.
  _cfg.pdPolarityPos = true;
  _cfg.csr = true;
  pushAll();
}

void ADF4159::disableCycleSlipReduction() {
  _cfg.csr = false;
  pushAll();
}

void ADF4159::powerDown() {
  _cfg.powerDown = true;
  setR3(_cfg.negBleedIdx, _cfg.negBleedEn, _cfg.lol, _cfg.nSel, _cfg.sdReset,
        _cfg.rampMode, _cfg.psk, _cfg.fsk, _cfg.ldp, _cfg.pdPolarityPos,
        _cfg.powerDown, _cfg.cpThreeState, _cfg.counterReset);
  writeRegister(_r3);
}

void ADF4159::powerUp() {
  _cfg.powerDown = false;
  setR3(_cfg.negBleedIdx, _cfg.negBleedEn, _cfg.lol, _cfg.nSel, _cfg.sdReset,
        _cfg.rampMode, _cfg.psk, _cfg.fsk, _cfg.ldp, _cfg.pdPolarityPos,
        _cfg.powerDown, _cfg.cpThreeState, _cfg.counterReset);
  writeRegister(_r3);
}
