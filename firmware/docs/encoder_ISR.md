# Encoder ISR — Implementation Reference

> **Applies to:** Rev. B hardware · Firmware v0.9.7 · `arduino.ino` + `EncoderCounter.{h,cpp}` + `ISRScheduler.{h,cpp}`

---

## 1. Overview

The encoder subsystem provides interrupt-driven quadrature counting for all four DC motor channels. It is split across three layers:

| Layer | Files | Role |
|-------|-------|------|
| Hardware abstraction | `EncoderCounter.h / .cpp` | Counting logic, state machine, atomic reads |
| ISR wiring | `arduino.ino` (trampolines + PCINT vectors) | Vector → object dispatch |
| Interrupt attachment | `ISRScheduler.cpp` | `attachInterrupt()` and PCINT register setup |

The interface `IEncoderCounter` lets the rest of the firmware call `getCount()`, `snapshot()`, and `resetCount()` without caring whether a specific motor uses 2x or 4x mode, or whether its pins are on hardware INT or PCINT.

---

## 2. Hardware Pin Assignments

### 2.1 Motor 1 and Motor 2 — Dedicated Hardware INT Pins

M1 and M2 encoders sit on the six external interrupt pins of the Mega 2560. Both phases of each motor are on hardware INT pins, which is required for 4x quadrature decoding.

| Signal | Arduino Pin | AVR Vector | Mode |
|--------|-------------|------------|------|
| M1_ENC_A | 2 | INT0 | CHANGE |
| M1_ENC_B | 3 | INT1 | CHANGE |
| M2_ENC_A | 18 | INT5 | CHANGE |
| M2_ENC_B | 19 | INT4 | CHANGE |

> **Why INT4/INT5 for M2 and not INT2/INT3?**
> Pins 18/19 are also the hardware UART1 (Serial1) TX/RX lines. Serial1 is intentionally left unused so these pins are available as dedicated encoder interrupts. See `pins.h` note: `Serial1 NOT AVAILABLE — used by M2_ENC_A / M2_ENC_B`.

### 2.2 Motor 3 and Motor 4 — Pin-Change Interrupt (PCINT) Pins

M3 and M4 encoders were relocated to PCINT pins in Rev. B to free the hardware INT bank for M1/M2 full 4x decoding.

| Signal | Arduino Pin | Port | PCINT Bit | PCINT Group |
|--------|-------------|------|-----------|-------------|
| M3_ENC_A | A14 | Port K | PCINT22 | PCIE2 (PCINT2_vect) |
| M3_ENC_B | A15 | Port K | PCINT23 | PCIE2 (PCINT2_vect) |
| M4_ENC_A | 11 | Port B | PCINT5 | PCIE0 (PCINT0_vect) |
| M4_ENC_B | 12 | Port B | PCINT6 | PCIE0 (PCINT0_vect) |

PCINT vectors fire on any pin change within the entire port group. The ISR wrappers in `arduino.ino` dispatch directly to the encoder objects without further demultiplexing (only one encoder pin per group is active per interrupt in the current configuration).

---

## 3. Encoder Resolution Modes

Two counting strategies are compiled in via `config.h`:

```c
#define ENCODER_2X    2   // Interrupt on phase A only (CHANGE)
#define ENCODER_4X    4   // Interrupt on both phases (CHANGE on A and B)
```

### Active configuration (firmware v0.9.7)

```c
#define ENCODER_1_MODE    ENCODER_4X
#define ENCODER_2_MODE    ENCODER_4X
#define ENCODER_3_MODE    ENCODER_4X
#define ENCODER_4_MODE    ENCODER_4X
```

With `ENCODER_PPR = 1440` pulses per revolution (manufacturer spec):

| Mode | Edges counted | Counts per revolution | Effective CPR |
|------|--------------|----------------------|---------------|
| 2x | Rising + falling on A only | 720 | 720 |
| 4x | All edges on A and B | 1440 | 1440 |

### Direction inversion

```c
#define ENCODER_1_DIR_INVERTED  0   // normal
#define ENCODER_2_DIR_INVERTED  0   // normal
#define ENCODER_3_DIR_INVERTED  1   // inverted (H-bridge wiring)
#define ENCODER_4_DIR_INVERTED  1   // inverted (H-bridge wiring)
```

Direction inversion is applied inside the ISR (`invertDir_` flag), not in the PID layer.

---

## 4. Class Hierarchy

```
IEncoderCounter  (pure virtual interface)
├── EncoderCounter2x   — CHANGE on phase A; reads B for direction
└── EncoderCounter4x   — CHANGE on both phases; full state-machine decoding
```

Global instances in `arduino.ino`:

```cpp
// Selected at compile time from ENCODER_N_MODE defines
EncoderCounter4x encoder1;   // M1
EncoderCounter4x encoder2;   // M2
EncoderCounter4x encoder3;   // M3
EncoderCounter4x encoder4;   // M4
```

---

## 5. EncoderCounter2x — Implementation Detail

### 5.1 Initialisation (`init`)

`init()` sets both pins `INPUT_PULLUP` (required for open-collector encoder outputs; harmless for push-pull). It caches direct port pointers to avoid `digitalRead()` overhead inside the ISR:

```cpp
pinAInReg_ = portInputRegister(digitalPinToPort(pinA_));
pinBInReg_ = portInputRegister(digitalPinToPort(pinB_));
pinAMask_  = digitalPinToBitMask(pinA_);
pinBMask_  = digitalPinToBitMask(pinB_);
```

### 5.2 ISR Handler (`onInterruptA`)

Called on every CHANGE of phase A. Phase B is sampled via the cached port register to determine direction. Target time: **< 10 CPU cycles**.

```
Direction logic (2x):
  A XOR B == 1  →  forward  (count++)
  A XOR B == 0  →  reverse  (count--)
```

Truth table for forward (CW) rotation:

| A state | B state | A XOR B | Action |
|---------|---------|---------|--------|
| 0 (falling) | 1 | 1 | count++ |
| 1 (rising)  | 0 | 1 | count++ |
| 0 (falling) | 0 | 0 | count-- |
| 1 (rising)  | 1 | 0 | count-- |

`onInterruptB()` is a no-op in 2x mode.

---

## 6. EncoderCounter4x — Implementation Detail

### 6.1 Initialisation (`init`)

Same port-register caching as 2x, plus capture of the initial A/B state as `prevState_`:

```cpp
uint8_t a = (*pinAInReg_ & pinAMask_) ? 1U : 0U;
uint8_t b = (*pinBInReg_ & pinBMask_) ? 1U : 0U;
prevState_ = (a << 1) | b;   // 2-bit state: [A|B]
```

### 6.2 State Machine (`processEdge`)

Both `onInterruptA()` and `onInterruptB()` call the same `processEdge()` helper. Target time: **~15–20 CPU cycles**.

State encoding: bits `[prevA | prevB | currA | currB]` form a 4-bit index (0–15) into a lookup table.

```cpp
static const int8_t stateTable[16] = {
    0,   // 00 → 00  (no change)
    1,   // 00 → 01  forward
   -1,   // 00 → 10  reverse
    0,   // 00 → 11  invalid (skip-2 glitch)
   -1,   // 01 → 00  reverse
    0,   // 01 → 01  (no change)
    0,   // 01 → 10  invalid
    1,   // 01 → 11  forward
    1,   // 10 → 00  forward
    0,   // 10 → 01  invalid
    0,   // 10 → 10  (no change)
   -1,   // 10 → 11  reverse
    0,   // 11 → 00  invalid
   -1,   // 11 → 01  reverse
    1,   // 11 → 10  forward
    0    // 11 → 11  (no change)
};
```

Valid forward sequence: `00 → 01 → 11 → 10 → 00 …`
Valid reverse sequence: `00 → 10 → 11 → 01 → 00 …`

Invalid transitions (two bits change simultaneously) yield `delta = 0` — the count is not modified, preventing glitch accumulation.

After the lookup, `invertDir_` flips the sign before applying to `count_`:

```cpp
count_ += invertDir_ ? -delta : delta;
if (delta != 0) {
    lastEdgeUs_ = micros();
}
prevState_ = currState;
```

---

## 7. Atomicity and Thread Safety

`count_` and `lastEdgeUs_` are `volatile int32_t` / `volatile uint32_t`. On AVR (8-bit), a 32-bit read or write is not atomic — it takes four load/store instructions. If `loop()` reads while the ISR is mid-write, it gets a torn value.

### Reading from loop context

**`getCount()`** uses `ATOMIC_BLOCK(ATOMIC_RESTORESTATE)` to disable interrupts for the duration of the 32-bit copy:

```cpp
int32_t EncoderCounter2x::getCount() const {
    int32_t value;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        value = count_;
    }
    return value;
}
```

**`snapshot(count, lastEdgeUs)`** is the preferred API for velocity feedback because it captures both `count_` and `lastEdgeUs_` in a single atomic block, guaranteeing the pair is consistent (same interrupt context):

```cpp
void EncoderCounter2x::snapshot(int32_t &count, uint32_t &lastEdgeUs) const {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count      = count_;
        lastEdgeUs = lastEdgeUs_;
    }
}
```

### Writing from loop context

`resetCount()` and `setCount()` use `noInterrupts()` / `interrupts()` directly (functionally equivalent to `ATOMIC_BLOCK` for writes with no return value).

---

## 8. ISR Trampolines in `arduino.ino`

`attachInterrupt()` requires a plain C function pointer; it cannot bind a C++ member function directly. A set of thin global wrappers forward to the encoder object methods:

```cpp
// Hardware INT — M1 (both phases)
void encoderISR_M1_A() { encoder1.onInterruptA(); }
void encoderISR_M1_B() { encoder1.onInterruptB(); }

// Hardware INT — M2 (both phases)
void encoderISR_M2_A() { encoder2.onInterruptA(); }
void encoderISR_M2_B() { encoder2.onInterruptB(); }

// PCINT dispatchers — M3 (Port K) and M4 (Port B)
void encoderISR_M3()   { encoder3.onInterruptA(); }
void encoderISR_M4()   { encoder4.onInterruptA(); }

ISR(PCINT2_vect) { encoderISR_M3(); }   // A14/A15 — M3 encoder
ISR(PCINT0_vect) { encoderISR_M4(); }   // pin11/12 — M4 encoder
```

`encoderISR_M1_A()` also toggles `DEBUG_PIN_ENCODER_ISR` (A7) when `DEBUG_PINS_ENABLED = 1`, allowing oscilloscope measurement of encoder ISR activity.

> **PCINT limitation:** `PCINT2_vect` and `PCINT0_vect` fire whenever *any* pin in the group changes. Because only one encoder phase per group is unmasked in 2x mode (`PCMSK` only enables the A-phase bit), a single ISR dispatch is sufficient. In 4x mode the B-phase bit is also unmasked (`PCMSK2 |= _BV(PCINT23)` for M3, `PCMSK0 |= _BV(PCINT6)` for M4), so the same vector handles both phase edges — the `processEdge()` state machine correctly handles edges from either phase.

---

## 9. Interrupt Attachment (`ISRScheduler`)

### 9.1 Hardware INT — M1 and M2

```cpp
// ISRScheduler.cpp
void ISRScheduler::attachDcEncoderInterrupts(
        void (*m1a)(void), void (*m1b)(void),
        void (*m2a)(void), void (*m2b)(void))
{
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), m1a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_B), m1b, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), m2a, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_B), m2b, CHANGE);
}
```

All four channels use `CHANGE` mode to capture both rising and falling edges.

### 9.2 PCINT — M3 and M4

```cpp
void ISRScheduler::attachDcEncoderPcints() {
    // M3: PCINT2 group (Port K) — A14=PCINT22, A15=PCINT23
    PCIFR  |= _BV(PCIF2);          // clear any pending flag
    PCMSK2 |= _BV(PCINT22);        // enable A14 (phase A always)
#if ENCODER_3_MODE == ENCODER_4X
    PCMSK2 |= _BV(PCINT23);        // enable A15 (phase B, 4x only)
#endif
    PCICR  |= _BV(PCIE2);          // enable PCINT2 group

    // M4: PCINT0 group (Port B) — pin11=PCINT5, pin12=PCINT6
    PCIFR  |= _BV(PCIF0);
    PCMSK0 |= _BV(PCINT5);
#if ENCODER_4_MODE == ENCODER_4X
    PCMSK0 |= _BV(PCINT6);
#endif
    PCICR  |= _BV(PCIE0);
}
```

The pending interrupt flag is cleared (`PCIFR`) before enabling the group to avoid a spurious fire on startup.

---

## 10. Initialisation Order in `setup()`

Encoder interrupts must not fire before the encoder objects are fully initialised. The setup sequence in `arduino.ino` enforces this order:

```
Step 9   DCMotorBringup::initAll(...)        ← calls encoder.init() for all 4 motors
Step 10a ISRScheduler::attachDcEncoderInterrupts(...)  ← hardware INT armed
Step 10b ISRScheduler::attachDcEncoderPcints()         ← PCINT masked and enabled
Step 12  ISRScheduler::configureTimer1DcSlotISR()      ← global interrupts enabled (sei())
```

`ISRScheduler::init()` (called last) re-enables global interrupts with `interrupts()` after configuring the timers. Encoder ISRs begin firing from this point.

---

## 11. Velocity Estimation

Encoder velocity is computed in `DCMotor::refreshFeedback()`, called by `taskMotorFeedback()` at 200 Hz. It uses `snapshot()` to atomically read both `count_` and `lastEdgeUs_`:

```cpp
int32_t count;
uint32_t lastEdgeUs;
encoder_->snapshot(count, lastEdgeUs);
```

If no edge has arrived within `VELOCITY_ZERO_TIMEOUT = 50 ms`, velocity is forced to zero regardless of `lastEdgeUs`. A first-order low-pass filter is applied with weight `VELOCITY_LOWPASS_CONST = 0.50`.

---

## 12. Stall Detection

Controlled by `config.h`:

```c
#define ENCODER_STALL_DETECTION     0    // disabled by default
#define ENCODER_FAIL_PWM_THRESHOLD  51   // ~20% of 255
#define ENCODER_FAIL_TIMEOUT_MS     500  // ms without encoder movement
```

When enabled, `SafetyManager` disables a motor channel if `|PWM| > threshold` and encoder position has not changed for the timeout period. Currently disabled (`0`) to avoid false positives during PID tuning.

---

## 13. Debug Pin

When `DEBUG_PINS_ENABLED = 1` in `config.h`, `DEBUG_PIN_ENCODER_ISR` (A7) is toggled HIGH at the start of `encoderISR_M1_A()` and LOW at exit. This pin can be probed with an oscilloscope to measure encoder ISR latency and firing rate.

Scope setup: A7 — time-base 500 µs/div, trigger on rising edge. At 100 RPM with 1440 CPR in 4x mode, edges arrive at ~9600 Hz (one edge every ~104 µs).

---

## 14. Key Configuration Reference

| Symbol | Location | Default | Effect |
|--------|----------|---------|--------|
| `ENCODER_PPR` | `config.h` | 1440 | Encoder pulses per revolution (manufacturer spec) |
| `ENCODER_N_MODE` | `config.h` | `ENCODER_4X` | 2x or 4x counting mode per motor (N = 1–4) |
| `ENCODER_N_DIR_INVERTED` | `config.h` | 0 / 0 / 1 / 1 | Flip count polarity per motor |
| `VELOCITY_ZERO_TIMEOUT` | `config.h` | 50 ms | Zero-velocity timeout for velocity estimator |
| `VELOCITY_LOWPASS_CONST` | `config.h` | 0.50 | Low-pass weight on raw velocity estimate |
| `ENCODER_STALL_DETECTION` | `config.h` | 0 | Enable/disable stall fault logic |
| `PIN_M1_ENC_A/B` | `pins.h` | 2 / 3 | Arduino pin numbers for M1 encoder |
| `PIN_M2_ENC_A/B` | `pins.h` | 18 / 19 | Arduino pin numbers for M2 encoder |
| `PIN_M3_ENC_A/B` | `pins.h` | A14 / A15 | Arduino pin numbers for M3 encoder |
| `PIN_M4_ENC_A/B` | `pins.h` | 11 / 12 | Arduino pin numbers for M4 encoder |
