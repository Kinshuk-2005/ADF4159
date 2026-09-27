# ADF4159

Arduino library for the Analog Devices [ADF4159](https://www.analog.com/ADF4159) —
a 13 GHz direct-modulation / fast-waveform-generating fractional-N frequency
synthesizer with FMCW ramp, FSK and PSK support.

Supports **AVR** (Uno/Nano/Mega), **ESP32**, **Teensy**, **STM32**, and
**Raspberry Pi Pico (RP2040)** — any Arduino-core board with a hardware
`SPIClass` object.

## WARNING - The code has only been tested using a hardware emulator - one ESP32 was set as controller and another was made to behave as an ADF4159 emulator that simply takes the SPI values and shows R0 and R2. The code has not been tested on actual hardware. Use this library at your own risk. I will be grateful to any one who can test this library on actual hardware and I am more than happy to receive any feedback.

## Hardware

| Pin | Role |
|---|---|
| LE | Load Enable — pass to `begin()`, drives the 3-wire write latch |
| CLK | SPI clock |
| DATA | SPI MOSI |
| CE | Chip Enable — tie high, or drive externally |
| TXDATA | Dedicated FSK/PSK/ramp-trigger pin — **not** driven by this library; wire it to a spare GPIO and toggle with `digitalWrite()` |
| REFIN | Reference input, 10–260 MHz (10–50 MHz if the on-chip doubler is used) |
| RFINA | RF input from VCO, 0.5–13 GHz, AC-coupled |
| RFINB | Complementary RF input |

## ⚠️ Voltage / logic level warning

**The ADF4159's digital I/O is 1.8 V logic, not 3.3 V or 5 V.** Per the
datasheet:

- `DVDD`, `SDVDD` (digital supply): **1.62 V – 1.98 V** (Table 1)
- `VINH` (input high): 1.17 V min · `VINL` (input low): 0.4 V max (Table 1)
- Absolute max digital I/O voltage to GND: **DVDD + 0.3 V** (Table 4) —
  with a 1.8 V DVDD, that's roughly **2.1 V max** on LE, CLK, DATA, CE,
  and TXDATA.

**Driving these pins directly from a 3.3 V MCU (ESP32, STM32, Teensy,
RP2040) or a 5 V MCU (classic AVR Uno/Nano/Mega) exceeds this absolute
maximum rating and can permanently damage the chip.** You need a
level shifter (or a simple resistive divider on unidirectional lines)
between your MCU and every digital pin on the ADF4159:

- **LE, CLK, DATA, CE, TXDATA** (MCU → ADF4159): use a bidirectional
  level shifter (e.g. TXB0104/TXB0108) or, for these push-pull outputs
  specifically, a simple resistive divider or a 74LVC-series buffer
  powered from 1.8 V.
- **MUXOUT / SDOUT** (ADF4159 → MCU, only if you use lock-detect or
  register readback): these only swing to ~1.8 V, which may not read
  reliably as a logic HIGH on a 5 V AVR input — level-shift this line
  too, or read it through a comparator.

This library has no way to detect or enforce correct logic levels in
software — get this right in hardware before applying power. (This
warning is specific to the real ADF4159 chip. If you're testing against
the software emulator in `examples/emulator/`, both boards in that
pairing are 3.3 V logic and connect directly — see that example's
comments.)

### Single-ended vs. differential RF input

This is **not a register setting** — there is no bit on the ADF4159 that
selects input mode. It's fixed entirely by PCB wiring: for single-ended
operation, RFINB is simply decoupled to ground through a ~100 pF capacitor
and left undriven, while RFINA carries the AC-coupled VCO signal. Both
pins always feed a fixed differential limiting amplifier internally.

## Wiring example (any microcontroller)

The ADF4159 only needs three digital lines for normal operation (LE,
CLK, DATA), plus CE and, optionally, TXDATA. This is board-agnostic —
substitute your MCU's own SPI SCK/MOSI pins and any free GPIOs; only
the pin *numbers* passed to `begin()` and `pinMode()` change between
boards, not the wiring topology.

```
                         1.8V LOGIC SIDE (via level shifter)
   MCU (3.3V or 5V)                                    ADF4159
  ------------------                              ------------------
   SPI SCK   -------->|  LEVEL SHIFTER  |-------->  CLK
   SPI MOSI  -------->|   (MCU side <-> |-------->  DATA
   GPIO (LE) -------->|   1.8V side)    |-------->  LE
   GPIO (CE) -------->|                 |-------->  CE   (or tie CE
   GPIO      -------->|                 |-------->  TXDATA    high on
                       |                 |<--------  MUXOUT     the
                        -----------------             (optional,   1.8V
                                                        lock detect) side)

   3.3V/1.8V regulator ---------------------------->  AVDD, DVDD, VP
   GND ---------------------------------------------> AGND, DGND, CPGND,
                                                        SDGND, exposed pad
```

Generic pin-role mapping (fill in real GPIO numbers for your board):

| ADF4159 pin | Connect to | Notes |
|---|---|---|
| LE | any MCU GPIO | passed as `lePin` to `begin()` |
| CLK | MCU hardware SPI SCK | via level shifter (see warning above) |
| DATA | MCU hardware SPI MOSI | via level shifter |
| CE | MCU GPIO, or tied high through the level shifter | low = power-down |
| TXDATA | any MCU GPIO | toggled with `digitalWrite()`, not driven by this library |
| MUXOUT | any MCU GPIO (input), via level shifter | optional: lock detect / readback |
| REFIN | your reference oscillator/TCXO output | 10–260 MHz, AC- or DC-coupled per datasheet |
| RFINA | VCO output, AC-coupled | 0.5–13 GHz |
| RFINB | 100 pF to ground (single-ended) or driven differentially | see note above |
| AVDD, VP | 2.7–3.45 V analog supply | decouple close to the pins |
| DVDD, SDVDD | 1.62–1.98 V digital supply | decouple close to the pins |
| AGND, DGND, CPGND, SDGND, exposed pad | common ground | tie together, connect exposed pad to AGND |

Board-specific SPI pins (SCK/MOSI) and how to instantiate a second SPI
bus are covered in the "Multiple SPI buses" section below — those are
the only genuinely per-microcontroller details.

## Register write sequence

Every full (re)configuration follows the datasheet's corrected order:

```
R7 -> R6 -> R6 -> R5 -> R5 -> R4 -> R4 -> R3 -> R2 -> R1 -> R0
```

R6, R5 and R4 are each written twice because they carry dual-selected
latches (Step Word 1/2, Deviation Word 1/2, CLK2 for Ramp 1/2) that must
both be loaded before the R0 write finalizes the configuration. This
applies to every ramp reconfiguration, not just power-up.
`writeFullSequence()` enforces this automatically, and every high-level
`configure*()` function calls it internally. For a plain single ramp
(sawtooth/triangle/parabolic/FSK/PSK), both latches get identical data;
`configureDualRamp()` and `configureFastRamp()` load genuinely different
values into the Ramp 1 and Ramp 2 latches.

A pure frequency retune (same ramp/modulation setup, new RFOUT) only
needs `R1 -> R0`, since FRAC/INT are double-buffered. `setFrequency()`
uses this fast path automatically after its first call — **no power-down
or power-up cycle happens on any update.**

## Multi-platform notes

### Why frequencies are integer Hz, not `double`

On AVR, `double` is the same 32-bit type as `float` (~24-bit mantissa),
which cannot represent a 12 GHz value to better than ~1 kHz — the number
itself would round before any register math runs. To keep frequency
synthesis exact on every platform (AVR included), `setFrequency()` and
the ramp/chirp start/stop arguments take `uint64_t` Hz, and the INT/FRAC
split is computed with pure 64-bit integer arithmetic — no floating
point in that path at all. Deviation (FSK/PSK/ramp step size) and ramp
timing remain `double`, since those don't need sub-Hz precision.

### Multiple SPI buses

ESP32 (VSPI/HSPI), Teensy (SPI/SPI1/SPI2), STM32, and RP2040 (SPI0/SPI1)
can expose more than one hardware SPI peripheral. `begin()` takes an
`SPIClass&` (defaults to the global `SPI`):

```cpp
SPIClass hspi(HSPI);              // ESP32 example
hspi.begin(14, 12, 13, -1);
synth.begin(LE_PIN, refHz, hspi); // pass initSPI=false if you already called hspi.begin()
```

Default SPI clock is 2 MHz on AVR, 4 MHz elsewhere (both are well under
the datasheet's 20 MHz write-timing limit); override via the `spiHz`
parameter.

## Quick start

```cpp
#include <SPI.h>
#include "ADF4159.h"

ADF4159 synth;

void setup() {
  synth.begin(/* LE pin */ 5, /* REFIN Hz */ 100000000UL);
  synth.setFrequency(12002000000ULL); // 12.002 GHz, exact integer Hz
  synth.printStatus(); // dump every register + decoded field + RFOUT/fPFD
}
```

## Functions

- `begin(lePin, refInHz, spiPort=SPI, spiHz=platform default, initSPI=true)`
- `writeRegister(uint32_t value)` — raw 32-bit SPI write + LE pulse
- `setR0`..`setR7(...)` — stage individual register fields (call `writeFullSequence()` to commit)
- `getR0()`..`getR7()` — return the raw 32-bit words currently staged in this library's memory (not a live SPI read from the chip — the ADF4159's 3-wire interface is write-mostly; see `readFrequency()` for the one thing it can actually read back)
- `readStatus()` — returns an `ADF4159_Status` struct with every raw register, every decoded field (INT/FRAC, ramp mode, deviation words for both ramp latches, CP current index, delay words, etc.), and derived values (current RFOUT, fPFD, frequency resolution)
- `printStatus(out=Serial)` — dumps `readStatus()` to any `Print` target in human-readable form
- `writeFullSequence()` / `updateFrequencyOnly()`
- `setFrequency(freqHz, rCounter=1, doubler=false, rdiv2=false)` — `freqHz` is `uint64_t`
- `configureSawtoothRamp(startFreq, stopFreq, numSteps, stepTimeUs, continuous=true)` — Hz args are `uint64_t`
- `configureTriangleRamp(startFreq, stopFreq, numSteps, stepTimeUs, continuous=true)` — Hz args are `uint64_t`
- `configureFMCWChirp(startFreq, bandwidthHz, sweepTimeMs, numSteps=200)` — Hz args are `uint64_t`
- `configureDualRamp(centerFreq, stepTimeUs, devHz1, numSteps1, devHz2, numSteps2, rampMode=CONT_SAWTOOTH)` — two independent ramps sharing one CLK1/CLK2 timer, per the datasheet's "Dual Ramps with Different Ramp Rates" example
- `configureFastRamp(centerFreq, upDevHz, upSteps, upStepTimeUs, downDevHz, downSteps, downStepTimeUs)` — two-slope triangle (independent up/down timing), mitigates sawtooth retrace overshoot; total frequency change of both slopes should match for stability
- `configureParabolicRamp(centerFreq, devHz, numSteps, stepTimeUs, continuous=true)` — nonlinear ramp, `fOUT(n+1) = fOUT(n) + n·fDEV`; includes the required counter-reset toggle after activation
- `enableRampDelay(delayUs, useClk1Multiplier=false, fastLockDuringDelay=false)` / `disableRampDelay()` — delay between successive ramp bursts
- `enableDelayedStart(delayUs, useClk1Multiplier=false)` / `disableDelayedStart()` — delay before the first ramp step; shares its delay word with `enableRampDelay()` in hardware, so only one of the two can be active
- `enableInterruptReadback(stopOnInterrupt=false)` / `disableInterruptReadback()` — arms MUXOUT-based frequency readback; requires an active ramp
- `readFrequency(clkPin, muxoutPin, txDataPin, double &outFreqHz)` — bit-bangs the 37-bit readback sequence (pulses TXDATA, holds LE high, clocks INT/FRAC out of MUXOUT) and returns the frequency at the moment of interrupt; bypasses the hardware SPI object since it needs direct pin control outside the normal write path
- `configureFSK(centerFreq, deviationHz, rCounter=1)` — `centerFreq` is `uint64_t`, `deviationHz` stays `double`
- `configurePSK(phaseDeg)`
- `activateRamp()` / `deactivateRamp()`
- `enableCycleSlipReduction()` / `disableCycleSlipReduction()`
- `powerDown()` / `powerUp()`
- `getPFDFrequency()` / `getResolution()`

## Testing without RF hardware

`examples/emulator/emulator.ino` emulates the chip's 3-wire register
interface on a second board (ESP32 or Pico) — it decodes and prints
every register write it receives, with no RF hardware needed. See the
comments in that file for wiring between two boards in either
controller/emulator role, and how to set a slow enough `spiHz` for
reliable interrupt-based capture.

## Examples

- `basic_frequency` — lock to a fixed RF output, print full status
- `chirp` — continuous sawtooth FMCW chirp (datasheet worked example: 5800–5850 MHz / 2 ms)
- `fsk` — frequency shift keying via TXDATA
- `psk` — phase shift keying via TXDATA
- `emulator` — chip emulator for SPI/register testing without RF hardware (ESP32 and Pico)

## Datasheet

This library implements the register map, timing, and worked examples
from:

> Analog Devices, Inc. *ADF4159: Direct Modulation/Fast Waveform
> Generating, 13 GHz, Fractional-N Frequency Synthesizer*, Data Sheet,
> Rev. E, ©2013–2014.
> <https://www.analog.com/en/products/adf4159.html>

Register bit positions, timing figures, and pin voltage limits referenced
throughout this README and the library source come directly from that
document — refer to it for anything not covered here (e.g. loop filter
design, spur mechanisms, or PCB layout guidelines).

## License

MIT
