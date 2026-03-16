# Sensorless Homing with TMC2209 Drivers

This document covers the hardware wiring, firmware configuration, and runtime
tuning required to use TMC2209 stallGuard sensorless homing on the X and Y axes
of the pyuscope microscope stage.

Z axis uses a physical limit switch and is not affected by any of this.

---

## How it works

Each TMC2209 driver has a stallGuard feature that monitors motor load.  When the
motor stalls (stage hits a mechanical end-stop), the driver asserts its DIAG pin.
That DIAG pin is wired to the same input as the axis MIN limit switch, so Grbl
sees it as a limit trigger and stops the homing move.

The driver is also configured over a single-wire UART interface (PDN_UART) so
the firmware can set the stallGuard threshold, motor current, and operating mode
at runtime rather than relying on fixed hardware resistors.

---

## Hardware wiring

### UART interface (AUX-2 connector on MKS GEN V1.4 / RAMPS 1.4)

Each axis needs two connections from the driver carrier board to the RAMPS AUX-2
header, plus one 1 kΩ resistor:

```
MCU TX pin ──┬── 1kΩ ── MCU RX pin
             │
         PDN_UART pin on TMC2209 carrier
```

| Axis | MCU TX     | MCU RX     | Arduino pin names |
|------|-----------|-----------|-------------------|
| X    | PG1 / D40 | PK1 / A9  | Digital 40, Analog 9  |
| Y    | PF5 / A5  | PK2 / A10 | Analog 5, Analog 10   |

Steps:

1. Fit a **1 kΩ resistor** between D40 and A9 on the AUX-2 header (X axis).
2. Fit a **1 kΩ resistor** between A5 and A10 on the AUX-2 header (Y axis).
3. On each TMC2209 carrier board, connect the **PDN_UART** pad to the
   corresponding MCU RX pin (A9 for X, A10 for Y).  On most carriers this is
   the same pad as MS3; check your carrier's silkscreen.

> **Note:** A9 and A10 are permanently claimed by the UART interface on this
> board.  The RESET and FEED_HOLD control inputs are therefore moved to A13 and
> A14 (also on AUX-2).

### DIAG output (stall signal)

Wire each driver's DIAG output to the corresponding axis MIN limit input:

| Axis | DIAG → | RAMPS connector | Arduino pin |
|------|--------|-----------------|-------------|
| X    | X MIN  | X endstop MIN   | D3          |
| Y    | Y MIN  | Y endstop MIN   | D14         |

The DIAG pin is open-drain and active-high: it goes high on a stall.  RAMPS
endstop inputs are normally pulled high, so the DIAG signal must pull the pin
**low** on stall — you may need a pull-up and an inverting buffer, or verify that
your carrier board's DIAG output is active-low/open-drain compatible with the
RAMPS endstop input.  Check your carrier board datasheet.

### Microstepping jumpers (MS1 / MS2)

The TMC2209 UART node address is set by the MS1 and MS2 pins:

| MS1 | MS2 | Address | Microstepping |
|-----|-----|---------|---------------|
| LOW | LOW | 0       | 1/8 step      |
| HIGH| LOW | 1       | 1/2 step      |
| LOW | HIGH| 2       | 1/4 step      |
| HIGH| HIGH| 3       | 1/16 step (standard RAMPS jumpers) |

With standard RAMPS 1/16-step jumpers fitted, **MS1=MS2=HIGH → address 3**.

The firmware auto-detects the address at startup by scanning 0–3, so the
jumper state does not need to match any firmware setting.  If you want to skip
the scan and pin a fixed address, set `TMC2209_X_ADDR` / `TMC2209_Y_ADDR` in
`grbl/config.h`.

---

## Firmware configuration

All TMC2209 settings live in `grbl/config.h` inside the
`#ifdef CPU_MAP_2560_RAMPS_BOARD_PYUSCOPE` block.

### Enable sensorless homing

```c
#define TMC2209_SENSORLESS_HOMING   // comment out to disable
```

Disabling this reverts to standard Grbl behaviour (physical limit switches
required on X and Y).

### Motor current

```c
#define TMC2209_IRUN   16   // run current, 0–31 (31 = 100 % of Vref)
#define TMC2209_IHOLD   4   // hold current, 0–31
```

Start conservative (16 / 4) and increase IRUN if the motor skips steps during
normal moves.  Higher run current also changes the stall force, so re-tune the
stallGuard thresholds (see below) after changing IRUN.

### StealthChop / SpreadCycle threshold

```c
#define TMC2209_TPWMTHRS_NORMAL  300
```

stallGuard only works in SpreadCycle mode.  During homing the firmware forces
SpreadCycle by setting TPWMTHRS=0; after homing it restores this value so the
motor runs quietly (StealthChop) at low speeds.  Increase this value if you
want StealthChop active at higher speeds; decrease it if you want SpreadCycle
(louder but more torque) at all speeds during normal operation.

### Default stallGuard thresholds (EEPROM initial values)

```c
#define DEFAULT_TMC_X_SEEK_SGTHRS  30   // $40
#define DEFAULT_TMC_X_FEED_SGTHRS  60   // $41
#define DEFAULT_TMC_Y_SEEK_SGTHRS  30   // $42
#define DEFAULT_TMC_Y_FEED_SGTHRS  60   // $43
```

These are written to EEPROM only when the EEPROM is wiped (`$RST=*`).  After
that they are tuned at runtime with `$40`–`$43` (see below).

### Alarm on driver failure

```c
#define TMC2209_ALARM_ON_FAIL
```

When defined, homing is blocked (ALARM:10) if a TMC2209 driver does not respond
on UART at startup.  Comment out if you want homing to proceed even when a
driver is not communicating (stallGuard will not work on that axis, but a
physical end-stop wired to the DIAG/limit pin will still function).

---

## Startup diagnostics

At power-on the firmware prints a status line for each axis:

```
[MSG:TMC2209 X lb=ok addr=3 rx=08/05FF062100004CDF ver=21 OK]
[MSG:TMC2209 Y lb=ok addr=3 rx=08/05FF062100004CDF ver=21 OK]
```

| Field   | Meaning |
|---------|---------|
| `lb=ok` | Loopback test passed — 1 kΩ coupling resistor is present and wired |
| `lb=FAIL` | Resistor missing or TX/RX pins not connected |
| `addr=3` | Driver responded at UART address 3 (MS1=MS2=HIGH) |
| `addr=?` | No driver responded at any address |
| `rx=08/…` | 8 bytes received; hex dump of the raw datagram |
| `ver=21` | TMC2209 silicon version 0x21 confirmed |
| `OK` / `FAIL` | Overall pass/fail |

If you see `lb=FAIL`: check the 1 kΩ resistor and the TX/RX wiring.

If you see `lb=ok addr=? FAIL`: the resistor is present but the driver is not
responding.  Check the PDN_UART connection from the carrier to the RX pin, and
verify the driver is powered.

---

## Runtime tuning — stallGuard thresholds

The four stallGuard settings control how sensitive the stall detection is:

| Setting | Default | Meaning |
|---------|---------|---------|
| `$40`   | 30      | X axis seek phase (fast approach) threshold |
| `$41`   | 60      | X axis feed phase (slow locate) threshold |
| `$42`   | 30      | Y axis seek phase threshold |
| `$43`   | 60      | Y axis feed phase threshold |

**Higher value = less sensitive** (harder stall needed to trigger).
**Lower value = more sensitive** (lighter touch triggers a stall).

Homing runs in two phases:
1. **Seek** — fast approach at `$25` (homing seek rate).  Uses the seek
   threshold (`$40` / `$42`).
2. **Feed** — slow locate at `$24` (homing feed rate).  Uses the feed
   threshold (`$41` / `$43`).

### Tuning procedure

1. Home an axis with `$H` and watch for false triggers (axis stops before
   reaching the end-stop) or missed triggers (axis keeps moving and crashes).

2. **False trigger (stops too early):** increase the threshold value.
   ```
   $40=40
   ```

3. **Missed trigger / crash:** decrease the threshold value.
   ```
   $40=20
   ```

4. Tune the seek threshold first (it covers the fast approach), then the feed
   threshold (slow locate pass where precision matters).

5. Typical starting range is 10–80.  Exact values depend on motor current,
   stage friction, and homing speed.

6. Once happy, verify the values are stored:
   ```
   $$
   ```
   Settings are written to EEPROM immediately so they survive a power cycle.

### Interaction with motor current

stallGuard sensitivity scales with run current.  If you change `TMC2209_IRUN`
in `config.h` and reflash, re-tune `$40`–`$43` from scratch.

---

## Complete wiring summary

```
MKS GEN V1.4 / RAMPS 1.4
─────────────────────────
AUX-2 pin A5  (PF5) ─── TX for Y UART
AUX-2 pin A9  (PK1) ─── RX for X UART  [also: X DIAG goes to X MIN endstop connector]
AUX-2 pin A10 (PK2) ─── RX for Y UART  [also: Y DIAG goes to Y MIN endstop connector]
AUX-2 pin A13 (PK5) ─── RESET control input (remapped from A9)
AUX-2 pin A14 (PK6) ─── FEED HOLD control input (remapped from A10)
Digital pin D40 (PG1) ── TX for X UART

X driver carrier
────────────────
PDN_UART ─── 1kΩ ─── D40 / A9 junction

Y driver carrier
────────────────
PDN_UART ─── 1kΩ ─── A5 / A10 junction
```
