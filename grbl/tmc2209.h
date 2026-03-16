/*
  tmc2209.h - TMC2209 stepper driver support for sensorless homing
  Part of Grbl

  Provides software (bit-bang) UART communication with TMC2209 drivers on the
  MKS GEN V1.4 / RAMPS 1.4 board. Configures stallGuard for sensorless homing
  on the X and Y axes. The DIAG output of each driver is wired directly to the
  axis MIN limit pin; no other changes to the limit-switch reading code are needed.

  Wiring (AUX-2 connector on MKS GEN V1.4):
    X axis: D40 (TX) --[1k]-- A9 (RX) <-- X driver MS3 (PDN_UART)
    Y axis: A5  (TX) --[1k]-- A10(RX) <-- Y driver MS3 (PDN_UART)
    X DIAG wired to X MIN limit input (D3)
    Y DIAG wired to Y MIN limit input (D14)

  UART address (0-3) is set by MS1/MS2 jumpers on the driver carrier board.
  If TMC2209_X_ADDR / TMC2209_Y_ADDR are defined in config.h the address is
  used directly.  If left undefined, the address is auto-detected at startup
  by scanning all four addresses and using the first that responds.
*/

#ifndef tmc2209_h
#define tmc2209_h

#ifdef TMC2209_SENSORLESS_HOMING

#include <stdbool.h>
#include <stdint.h>

// ---------------------------------------------------------------------------
// TMC2209 register addresses
// ---------------------------------------------------------------------------
#define TMC_REG_GCONF      0x00   // Global configuration
#define TMC_REG_GSTAT      0x01   // Global status (clear-on-read)
#define TMC_REG_IOIN       0x06   // Reads input pin states and chip version
#define TMC_REG_IHOLD_IRUN 0x10   // Motor current: hold / run / power-down delay
#define TMC_REG_TPOWERDOWN 0x11   // Delay before standstill current reduction
#define TMC_REG_TSTEP      0x12   // Actual measured time between steps (read)
#define TMC_REG_TPWMTHRS   0x13   // Upper velocity for StealthChop (below = quiet)
#define TMC_REG_TCOOLTHRS  0x14   // Lower velocity threshold for stallGuard/CoolStep
#define TMC_REG_CHOPCONF   0x6C   // Chopper configuration
#define TMC_REG_DRV_STATUS 0x6F   // Driver status flags (read)
#define TMC_REG_PWMCONF    0x70   // StealthChop PWM configuration
#define TMC_REG_SGTHRS     0x40   // stallGuard detection threshold (0-255)
#define TMC_REG_SG_RESULT  0x41   // stallGuard result (read; 0 = stall)

// ---------------------------------------------------------------------------
// GCONF bit positions
// ---------------------------------------------------------------------------
#define TMC_GCONF_I_SCALE_ANALOG  (1u << 0)  // Use external Vref (clear = internal)
#define TMC_GCONF_EN_SPREADCYCLE  (1u << 2)  // 1 = force SpreadCycle; 0 = StealthChop (velocity-dependent via TPWMTHRS)
#define TMC_GCONF_PDN_DISABLE     (1u << 6)  // Disable automatic standstill reduction via PDN pin; required for UART control

// ---------------------------------------------------------------------------
// IOIN version field (bits 31:24 should be 0x21 for TMC2209)
// ---------------------------------------------------------------------------
#define TMC_IOIN_VERSION_SHIFT 24
#define TMC_IOIN_VERSION_MASK  0xFF
#define TMC2209_VERSION        0x21

// ---------------------------------------------------------------------------
// IHOLD_IRUN bit fields
//   bits  4:0  = IHOLD (standstill current, 0-31)
//   bits 12:8  = IRUN  (run current, 0-31)
//   bits 19:16 = IHOLDDELAY (power-down delay steps, 0-15)
// ---------------------------------------------------------------------------
#define TMC_IHOLD_IRUN_VAL(ihold, irun, delay) \
    (((uint32_t)(ihold) & 0x1F) | \
     (((uint32_t)(irun)  & 0x1F) << 8) | \
     (((uint32_t)(delay) & 0x0F) << 16))

// ---------------------------------------------------------------------------
// Phase indices for tmc_sgthrs[axis][phase]
// ---------------------------------------------------------------------------
#define TMC_PHASE_SEEK 0   // Fast approach (coarse)
#define TMC_PHASE_FEED 1   // Slow locate   (fine)

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

// Initialise a TMC2209 axis driver: configure pins, enable UART mode, set
// motor current, and verify the chip version via IOIN read.
// Returns true on success; false if the chip does not respond or the version
// byte is wrong. On failure an error message is written to the serial port and
// sensorless homing should not be attempted on that axis.
bool tmc2209_init(uint8_t axis);

// Configure the driver for the homing approach pass: force SpreadCycle by
// setting GCONF.en_SpreadCycle=1 (stallGuard is silent in StealthChop),
// enable stallGuard at all speeds (TCOOLTHRS=max, TPWMTHRS=max), and load the
// seek-phase threshold from settings.tmc_sgthrs[axis][TMC_PHASE_SEEK].
void tmc2209_homing_start(uint8_t axis);

// Switch SGTHRS to the feed-phase (locate) value without changing TCOOLTHRS.
// Called by limits_go_home() at the seek→locate phase transition.
void tmc2209_set_sgthrs(uint8_t axis, uint8_t phase);

// Restore normal operation after homing: clear GCONF.en_SpreadCycle (restores
// velocity-dependent StealthChop), disable stallGuard (TCOOLTHRS=0, SGTHRS=0),
// and restore TPWMTHRS to TMC2209_TPWMTHRS_NORMAL.
void tmc2209_homing_end(uint8_t axis);

// Returns true if the last tmc2209_init() call for this axis succeeded.
// Used by mc_homing_cycle() to gate homing when TMC2209_ALARM_ON_FAIL is set.
bool tmc2209_axis_ok(uint8_t axis);

#endif // TMC2209_SENSORLESS_HOMING
#endif // tmc2209_h
