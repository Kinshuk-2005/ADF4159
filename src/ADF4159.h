/*
  ADF4159.h - Arduino library for the Analog Devices ADF4159
  13 GHz Direct Modulation / Fast Waveform Generating Fractional-N
  Frequency Synthesizer.

  Author: kinny (Kinshuk-2005)
  License: MIT

  ---------------------------------------------------------------
  HARDWARE NOTES
  ---------------------------------------------------------------
  REFIN   : 10 MHz - 260 MHz (10-50 MHz if reference doubler used)
  RFINA   : 0.5 GHz - 13 GHz, -10 dBm to 0 dBm, AC-coupled from VCO
  RFINB   : complementary prescaler input

  Single-ended vs differential RF input is NOT a register setting.
  It is a PCB/wiring choice: for single-ended operation, RFINB is
  simply decoupled to ground with a ~100 pF capacitor and left
  undriven, while RFINA carries the AC-coupled VCO signal. There is
  no bit on this chip that switches input mode - it is fixed by how
  RFINA/RFINB are wired on the board.

  TXDATA is a dedicated physical pin (not part of the 3-wire SPI
  interface) used to trigger FSK hops, PSK phase flips, and ramp
  step/interrupt functions. This library does not drive TXDATA -
  toggle it yourself with digitalWrite() on your MCU, wired directly
  to the ADF4159 TXDATA pin.

  ---------------------------------------------------------------
  REGISTER WRITE SEQUENCE
  ---------------------------------------------------------------
  Every full (re)configuration - power-up or any ramp parameter
  change - MUST use this order, per the ADF4159 datasheet's
  "Ramp Programming Sequence":

      R7 -> R6 -> R6 -> R5 -> R5 -> R4 -> R4 -> R3 -> R2 -> R1 -> R0

  R6, R5 and R4 are each written twice because they carry
  double-selected latches (Step Word 1/2, Deviation Word 1/2, and
  CLK2 for Ramp1/Ramp2 respectively) that must both be loaded before
  R0 finalizes the configuration. This library's writeFullSequence()
  enforces this automatically.

  A pure frequency retune (same ramp/modulation configuration, new
  RFOUT) only needs R1 -> R0, since FRAC/INT are double-buffered and
  take effect on the R0 write. updateFrequencyOnly() does this.

  ---------------------------------------------------------------
  MULTI-PLATFORM NOTES (AVR / ESP32 / Teensy / STM32 / RP2040)
  ---------------------------------------------------------------
  This library only depends on the standard Arduino SPIClass /
  digitalWrite / pinMode APIs, which all five core families provide,
  but two things needed platform-specific care:

  1. FREQUENCY PRECISION ON AVR
     On AVR, `double` is the same 32-bit IEEE-754 type as `float`
     (~24-bit mantissa). A 12 GHz value cannot be represented to
     better than roughly 1 kHz in that format - the number itself
     rounds before any register math even runs. To avoid this on
     every platform (not just AVR), all frequency arguments that need
     exact synthesis (setFrequency, ramp/chirp start & stop, REFIN)
     are integer Hz (uint64_t for RF, uint32_t for REFIN), and the
     INT/FRAC split is computed with pure 64-bit integer arithmetic
     (no floating point in that path at all). This is exact on every
     platform in this list, AVR included. Deviation (FSK/PSK/ramp
     step size) and ramp timing stay as `double` - those don't need
     sub-Hz precision, so the AVR float-precision limit there is
     immaterial in practice.

  2. MULTIPLE SPI BUSES
     ESP32 (VSPI/HSPI), Teensy (SPI/SPI1/SPI2), STM32, and RP2040
     (SPI0/SPI1) can all expose more than one hardware SPI peripheral.
     begin() takes an SPIClass& (defaulting to the global `SPI`) so
     you can point the library at whichever bus you've set up, e.g.:

       SPIClass hspi(HSPI);              // ESP32
       hspi.begin(14, 12, 13, -1);
       synth.begin(LE_PIN, refHz, hspi);

     If you've already called spiPort.begin() yourself (e.g. to remap
     pins on ESP32/RP2040), pass initSPI=false so this library doesn't
     re-initialize the bus.

  3. DEFAULT SPI CLOCK
     AVR parts are typically run at 8-16 MHz system clock and are
     usually wired with longer/breadboard-style RF-eval-board leads,
     so the default SPI clock is a conservative 2 MHz there. All
     other families default to 4 MHz. Both are safely under the
     datasheet's write-timing limits (25 ns min CLK high/low, i.e.
     up to 20 MHz) and can be overridden via the spiHz parameter.
*/

#ifndef ADF4159_H
#define ADF4159_H

#include <Arduino.h>
#include <SPI.h>

#if defined(__AVR__)
  static const uint32_t ADF4159_DEFAULT_SPI_HZ = 2000000UL;
#else
  static const uint32_t ADF4159_DEFAULT_SPI_HZ = 4000000UL;
#endif

// ---------------- Ramp mode (Register R3, DB[11:10]) ----------------
enum ADF4159_RampMode {
  ADF4159_RAMP_CONT_SAWTOOTH        = 0b00,
  ADF4159_RAMP_CONT_TRIANGLE        = 0b01,
  ADF4159_RAMP_SINGLE_SAWTOOTH_BURST = 0b10,
  ADF4159_RAMP_SINGLE_RAMP_BURST    = 0b11
};

// ---------------- MUXOUT select (Register R0, DB[30:27]) ----------------
enum ADF4159_MuxOut {
  ADF4159_MUX_THREE_STATE = 0b0000,
  ADF4159_MUX_DVDD        = 0b0001,
  ADF4159_MUX_DGND        = 0b0010,
  ADF4159_MUX_RDIV_OUT    = 0b0011,
  ADF4159_MUX_NDIV_OUT    = 0b0100,
  ADF4159_MUX_LOCK_DETECT = 0b0110,
  ADF4159_MUX_SDO         = 0b0111,
  ADF4159_MUX_CLKDIV_OUT  = 0b1010,
  ADF4159_MUX_RDIV_DIV2   = 0b1101,
  ADF4159_MUX_NDIV_DIV2   = 0b1110,
  ADF4159_MUX_READBACK    = 0b1111
};

// ---------------- Ramp status select (Register R4, DB[25:21]) ----------------
enum ADF4159_RampStatus {
  ADF4159_RAMPSTATUS_NORMAL             = 0b00000,
  ADF4159_RAMPSTATUS_INTERRUPT_READBACK = 0b00010,
  ADF4159_RAMPSTATUS_RAMP_COMPLETE      = 0b00011
};

class ADF4159 {
  public:
    ADF4159();

    // lePin is wired to the ADF4159 LE (Load Enable) pin.
    // refInHz is the exact REFIN frequency in Hz (integer - see the
    // MULTI-PLATFORM NOTES above on why this isn't a double).
    // spiPort lets you select a non-default SPI peripheral (ESP32
    // HSPI/VSPI, Teensy SPI1/SPI2, RP2040 SPI1, etc). If you've
    // already called spiPort.begin() yourself, pass initSPI=false.
    void begin(int lePin, uint32_t refInHz, SPIClass &spiPort = SPI,
               uint32_t spiHz = ADF4159_DEFAULT_SPI_HZ, bool initSPI = true);

    // ---------------- low level ----------------
    void writeRegister(uint32_t value);

    // ---------------- per-register shadow setters ----------------
    // These only update the in-memory shadow registers (_r0.._r7).
    // Call writeFullSequence() (or updateFrequencyOnly() for a pure
    // frequency change) to actually commit them over SPI.
    void setR0(bool rampOn, uint8_t muxout, uint16_t intVal, uint16_t fracMsb);
    void setR1(bool phaseAdjEn, uint16_t fracLsb, int16_t phaseVal);
    void setR2(bool csr, uint8_t cpCurrentIdx, bool prescaler89, bool rdiv2,
               bool doubler, uint8_t rCounter, uint16_t clk1);
    void setR3(uint8_t negBleedCurrentIdx, bool negBleedEn, bool lol, bool nSel,
               bool sdReset, uint8_t rampMode, bool psk, bool fsk, bool ldp,
               bool pdPolarityPos, bool powerDown, bool cpThreeState,
               bool counterReset);
    void setR4(bool leSel, uint8_t sdMode, uint8_t rampStatus, uint8_t clkDivMode,
               uint16_t clk2, bool clkDivSel);
    void setR5(bool txDataInvert, bool txDataRampClk, bool parabolicRamp,
               uint8_t interruptMode, bool fskRampEn, bool dualRampEn,
               bool devSel, uint8_t devOffset, int16_t devWord);
    void setR6(bool stepSel, uint32_t stepWord);
    void setR7(bool txDataTrigDelay, bool triDelay, bool singleFullTri,
               bool txDataTrigger, bool fastRamp, bool rampDelayFL,
               bool rampDelay, bool delClkSel, bool delStartEn,
               uint16_t delayStartWord);

    // ---------------- sequencing ----------------
    // Full power-up / ramp-reconfig write order:
    // R7, R6(sel0), R6(sel1), R5(sel0), R5(sel1), R4(sel0), R4(sel1),
    // R3, R2, R1, R0
    void writeFullSequence();

    // Fast path for a pure frequency retune: R1, R0 only.
    void updateFrequencyOnly();

    // ---------------- high level ----------------

    // Programs RFOUT = (INT + FRAC/2^25) * fPFD. freqHz is exact integer
    // Hz - see MULTI-PLATFORM NOTES for why this isn't a double. First
    // call after begin() performs a full sequence write; subsequent
    // calls use the R1->R0 fast path only (no power-down/up cycle either way).
    bool setFrequency(uint64_t freqHz, uint8_t rCounter = 1,
                       bool doubler = false, bool rdiv2 = false);

    // Dedicated sawtooth chirp function. start/stopFreq in exact Hz.
    void configureSawtoothRamp(uint64_t startFreq, uint64_t stopFreq,
                                uint16_t numSteps, double stepTimeUs,
                                bool continuous = true);

    // Dedicated triangle ramp function. start/stopFreq in exact Hz.
    void configureTriangleRamp(uint64_t startFreq, uint64_t stopFreq,
                                uint16_t numSteps, double stepTimeUs,
                                bool continuous = true);

    // Convenience wrapper matching the datasheet's FMCW Radar Ramp
    // Settings worked example: sawtooth ramp of given bandwidth,
    // swept in sweepTimeMs, split into numSteps hops.
    void configureFMCWChirp(uint64_t startFreq, uint64_t bandwidthHz,
                             double sweepTimeMs, uint16_t numSteps = 200);

    // Frequency Shift Keying: locks to centerFreq (exact Hz), hops by
    // +/-deviationHz each time TXDATA is toggled (rising edge = +dev,
    // falling = -dev, unless invertTxData is set).
    void configureFSK(uint64_t centerFreq, double deviationHz,
                       uint8_t rCounter = 1);

    // Phase Shift Keying: TXDATA high shifts output phase by +phaseDeg,
    // TXDATA low shifts by -phaseDeg, relative to current programmed phase.
    void configurePSK(double phaseDeg);

    // Two independent ramps (different deviation/step count) sharing the
    // same CLK1/CLK2 timer, per the datasheet's "Dual Ramps with
    // Different Ramp Rates" worked example. devHz1/devHz2 are per-step
    // deviation in Hz (not total bandwidth).
    void configureDualRamp(uint64_t centerFreq, double stepTimeUs,
                            double devHz1, uint16_t numSteps1,
                            double devHz2, uint16_t numSteps2,
                            ADF4159_RampMode rampMode = ADF4159_RAMP_CONT_SAWTOOTH);

    // Triangular ramp with independent up-slope and down-slope timing/
    // deviation/step-count (Ramp 1 = up, Ramp 2 = down), per the
    // datasheet's "Fast Ramp Mode" section - mitigates the overshoot a
    // plain sawtooth gets on the fast retrace. Note from the datasheet:
    // the total frequency change of the up and down ramps must be equal
    // for stability, i.e. upDevHz*upSteps should match downDevHz*downSteps
    // in magnitude.
    void configureFastRamp(uint64_t centerFreq,
                            double upDevHz, uint16_t upSteps, double upStepTimeUs,
                            double downDevHz, uint16_t downSteps, double downStepTimeUs);

    // Nonlinear (parabolic) ramp: fOUT(n+1) = fOUT(n) + n*fDEV. Matches
    // the datasheet's "Parabolic (Nonlinear) Ramp Mode" section,
    // including the required counter-reset toggle after activation.
    void configureParabolicRamp(uint64_t centerFreq, double devHz,
                                 uint16_t numSteps, double stepTimeUs,
                                 bool continuous = true);

    // Delay inserted between successive ramp bursts (R7 "Delay Between
    // Ramps"). Shares its delay word/clock-select bits with
    // enableDelayedStart() in hardware, so enabling one clears the other.
    void enableRampDelay(double delayUs, bool useClk1Multiplier = false,
                          bool fastLockDuringDelay = false);
    void disableRampDelay();

    // Delay before the very first ramp step starts (R7 "Delayed Start").
    // Shares its delay word/clock-select bits with enableRampDelay() in
    // hardware, so enabling one clears the other.
    void enableDelayedStart(double delayUs, bool useClk1Multiplier = false);
    void disableDelayedStart();

    // Enables MUXOUT-based frequency interrupt/readback (Register R0
    // MUXOUT=1111, R4 ramp status=00010, R5 interrupt mode per Table 8).
    // Requires an active ramp - call after a configure*Ramp() call.
    void enableInterruptReadback(bool stopOnInterrupt = false);
    void disableInterruptReadback();

    // Bit-bangs the 37-bit interrupt readback sequence (Figure 51):
    // pulses TXDATA to latch the current ramp frequency, holds LE high,
    // and clocks 37 bits out of MUXOUT (12-bit INT, then 25-bit FRAC),
    // reading each bit on CLK's falling edge as the datasheet specifies.
    // This bypasses the hardware SPI object, since it needs direct
    // control of LE while reading a pin outside the normal write path -
    // pass the raw CLK pin plus the MUXOUT and TXDATA pins here.
    // enableInterruptReadback() must be called first, and a ramp must be
    // running. Returns false if no valid PFD frequency has been staged
    // yet (i.e. before any setFrequency()/configure*() call).
    bool readFrequency(int clkPin, int muxoutPin, int txDataPin, double &outFreqHz);

    // Sets RAMP_ON (R0, DB31) without touching any other staged config.
    void activateRamp();
    void deactivateRamp();

    // Enables cycle slip reduction. Requires PD polarity = positive,
    // which this function also forces.
    void enableCycleSlipReduction();
    void disableCycleSlipReduction();

    void powerDown();
    void powerUp();

    double getPFDFrequency() const { return _fPFD; }
    double getResolution() const { return _fPFD > 0 ? _fPFD / 33554432.0 : 0; }

  private:
    SPIClass *_spi;
    int _lePin;
    uint32_t _refIn;
    uint32_t _spiHz;
    double _fPFD;      // approximate; used only for display, ramp timing
                        // and deviation math, never for INT/FRAC synthesis
    bool _initialized;

    uint32_t _r0, _r1, _r2, _r3, _r4, _r5, _r6, _r7;
    uint32_t _r4b, _r5b, _r6b; // Ramp 2 (SEL=1) shadow copies of R4/R5/R6

    // Cached configuration - lets high-level functions modify a single
    // field without needing to know every other currently-programmed bit.
    struct Config {
      // R2
      bool     csr           = false;
      uint8_t  cpCurrentIdx  = 7;      // 0b0111 = 2.5 mA
      bool     prescaler89   = false;
      bool     rdiv2         = false;
      bool     doubler       = false;
      uint8_t  rCounter      = 1;
      uint16_t clk1          = 1;
      // R3
      uint8_t  negBleedIdx   = 0;
      bool     negBleedEn    = false;
      bool     lol           = true;
      bool     nSel          = false;
      bool     sdReset       = false;
      uint8_t  rampMode      = ADF4159_RAMP_CONT_SAWTOOTH;
      bool     psk           = false;
      bool     fsk           = false;
      bool     ldp           = true;
      bool     pdPolarityPos = true;
      bool     powerDown     = false;
      bool     cpThreeState  = false;
      bool     counterReset  = false;
      // R4
      bool     leSel         = false;
      uint8_t  sdMode        = 0;
      uint8_t  rampStatus    = 0;
      uint8_t  clkDivMode    = 0;      // 0b11 required when ramping
      uint16_t clk2          = 1;
      // R5
      bool     txDataInvert  = false;
      bool     txDataRampClk = false;
      bool     parabolicRamp = false;
      uint8_t  interruptMode = 0;
      bool     fskRampEn     = false;
      bool     dualRampEn    = false;
      uint8_t  devOffset     = 0;
      int16_t  devWord       = 0;
      // R6
      uint32_t stepWord      = 0;
      // Ramp 2 (SEL=1 latches) - mirrored from ramp 1 automatically by
      // pushAll() unless independentRamp2 is set (dual ramp / fast ramp).
      uint8_t  devOffset2    = 0;
      int16_t  devWord2      = 0;
      uint32_t stepWord2     = 0;
      uint16_t clk2b         = 1;
      bool     independentRamp2 = false;
      // R7
      bool     txDataTrigDelay = false;
      bool     triDelay        = false;
      bool     singleFullTri   = false;
      bool     txDataTrigger   = false;
      bool     fastRamp        = false;
      bool     rampDelayFL     = false;
      bool     rampDelay       = false;
      bool     delClkSel       = false;
      bool     delStartEn      = false;
      uint16_t delayStartWord  = 0;
      // R0
      bool     rampOn = false;
      uint8_t  muxout = ADF4159_MUX_THREE_STATE;
      // R1
      bool     phaseAdjEn = false;
      int16_t  phaseVal   = 0;
      // computed from setFrequency()/stageFrequency()
      uint16_t intVal   = 0;
      uint16_t fracMsb  = 0;
      uint16_t fracLsb  = 0;
    } _cfg;

    // Computes fPFD/INT/FRAC for freqHz and stores into _cfg. INT/FRAC
    // are derived with exact 64-bit integer arithmetic (portable to
    // AVR, no precision loss at any Hz value in the chip's 0.5-13 GHz
    // range). Does not touch SPI - caller decides whether to push via
    // writeFullSequence() or updateFrequencyOnly().
    bool stageFrequency(uint64_t freqHz, uint8_t rCounter, bool doubler, bool rdiv2);

    // Solves DEV_OFFSET/DEV for a target per-step deviation in Hz,
    // using the same log2 rearrangement as the datasheet's FSK/ramp
    // worked examples. Requires _fPFD to already be staged.
    void computeDeviation(double devHz, uint8_t &devOffsetOut, int16_t &devWordOut);

    // Solves CLK1 (and CLK2 if CLK1 alone would exceed 12 bits) for a
    // target step time in microseconds. Requires _fPFD to already be staged.
    void computeRampTimer(double stepTimeUs, uint16_t &clk1Out, uint16_t &clk2Out);

    // Solves CLK2 alone for a target step time, given a CLK1 that's
    // already fixed (used by configureFastRamp(), where CLK1 - a single
    // shared register - must match between the up and down ramps).
    uint16_t clk2For(double stepTimeUs, uint16_t clk1);

    // Builds _r0.._r7 (+ _r4b/_r5b/_r6b) from _cfg and writes the full
    // corrected sequence.
    void pushAll();
};

#endif
