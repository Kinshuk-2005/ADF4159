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
  _r0(0), _r1(0), _r2(0), _r3(0), _r4(0), _r5(0), _r6(0), _r7(0)
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
// Sequencing
// ---------------------------------------------------------------
void ADF4159::writeFullSequence() {
  // R7
  writeRegister(_r7);
  // R6: STEP_SEL bit is DB23
  writeRegister(_r6 & ~(1UL << 23));   // Step Word 1
  writeRegister(_r6 |  (1UL << 23));   // Step Word 2
  // R5: DEV_SEL bit is DB23
  writeRegister(_r5 & ~(1UL << 23));   // Deviation Word 1
  writeRegister(_r5 |  (1UL << 23));   // Deviation Word 2
  // R4: CLK_DIV_SEL bit is DB6
  writeRegister(_r4 & ~(1UL << 6));    // CLK2 for Ramp 1
  writeRegister(_r4 |  (1UL << 6));    // CLK2 for Ramp 2
  // R3, R2, R1, R0
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

  // Effective REFIN after doubler ("Dfull"), and effective R-divide
  // factor after RDIV2 ("Rfull") - both exact integers.
  //   fPFD = Dfull / Rfull
  //   N    = freqHz / fPFD = freqHz * Rfull / Dfull
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
  //   product = freqHz * Rfull                (<= ~13e9 * 64  ~ 2^40, safe)
  //   INT     = product / Dfull               (exact integer division)
  //   rem     = product % Dfull               (< Dfull <= ~520e6)
  //   FRAC    = round(rem * 2^25 / Dfull)     (rem*2^25 <= ~1.75e16, safe)
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

void ADF4159::pushAll() {
  setR7(_cfg.txDataTrigDelay, _cfg.triDelay, _cfg.singleFullTri, _cfg.txDataTrigger,
        _cfg.fastRamp, _cfg.rampDelayFL, _cfg.rampDelay, _cfg.delClkSel,
        _cfg.delStartEn, _cfg.delayStartWord);
  setR6(false, _cfg.stepWord);
  setR5(_cfg.txDataInvert, _cfg.txDataRampClk, _cfg.parabolicRamp, _cfg.interruptMode,
        _cfg.fskRampEn, _cfg.dualRampEn, false, _cfg.devOffset, _cfg.devWord);
  setR4(_cfg.leSel, _cfg.sdMode, _cfg.rampStatus, _cfg.clkDivMode, _cfg.clk2, false);
  setR3(_cfg.negBleedIdx, _cfg.negBleedEn, _cfg.lol, _cfg.nSel, _cfg.sdReset,
        _cfg.rampMode, _cfg.psk, _cfg.fsk, _cfg.ldp, _cfg.pdPolarityPos,
        _cfg.powerDown, _cfg.cpThreeState, _cfg.counterReset);
  setR2(_cfg.csr, _cfg.cpCurrentIdx, _cfg.prescaler89, _cfg.rdiv2, _cfg.doubler,
        _cfg.rCounter, _cfg.clk1);
  setR1(_cfg.phaseAdjEn, _cfg.fracLsb, _cfg.phaseVal);
  setR0(_cfg.rampOn, _cfg.muxout, _cfg.intVal, _cfg.fracMsb);

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
    // R2 also changed (INT range affects prescaler / rCounter may differ) -
    // if the prescaler or R-divider settings changed since last push, do a
    // full sequence to be safe; otherwise the fast R1->R0 path is enough.
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
  double fRES = _fPFD / (double)ADF4159_MODULUS;

  // DEV_OFFSET = log2(fDEV / (fRES * 2^15)), clamped to 4-bit range
  double devOffsetF = log10(fabs(devPerStepHz) / (fRES * 32768.0)) / log10(2.0);
  int devOffset = (int)adf4159_round(devOffsetF);
  if (devOffset < 0) devOffset = 0;
  if (devOffset > 15) devOffset = 15;

  long devWordL = adf4159_round(devPerStepHz / (fRES * (double)(1UL << devOffset)));
  if (devWordL > 32767) devWordL = 32767;
  if (devWordL < -32768) devWordL = -32768;

  // Timer = CLK1 * CLK2 / fPFD ; keep CLK2 = 1 unless CLK1 would overflow 12 bits
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

  _cfg.clk1 = (uint16_t)clk1;
  _cfg.clk2 = (uint16_t)clk2;
  _cfg.devOffset = (uint8_t)devOffset;
  _cfg.devWord = (int16_t)devWordL;
  _cfg.stepWord = numSteps;
  _cfg.clkDivMode = 0b11; // ramp divider mode, required for ramp timing
  _cfg.dualRampEn = false;
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

void ADF4159::configureFSK(uint64_t centerFreq, double deviationHz, uint8_t rCounter) {
  if (!stageFrequency(centerFreq, rCounter, _cfg.doubler, _cfg.rdiv2)) return;

  double fRES = _fPFD / (double)ADF4159_MODULUS;
  double devOffsetF = log10(fabs(deviationHz) / (fRES * 32768.0)) / log10(2.0);
  int devOffset = (int)adf4159_round(devOffsetF);
  if (devOffset < 0) devOffset = 0;
  if (devOffset > 15) devOffset = 15;

  long devWordL = adf4159_round(deviationHz / (fRES * (double)(1UL << devOffset)));
  if (devWordL > 32767) devWordL = 32767;
  if (devWordL < -32768) devWordL = -32768;

  _cfg.devOffset = (uint8_t)devOffset;
  _cfg.devWord = (int16_t)devWordL;
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
