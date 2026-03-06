/*
  tmc2209.c - TMC2209 stepper driver support for sensorless homing
  Part of Grbl

  Implements a minimal blocking software (bit-bang) UART to configure TMC2209
  drivers. Only used at startup and around homing cycles, so blocking is fine.

  Baud rate: TMC2209_BAUD_RATE (default 19200) defined in config.h.
  At 16 MHz / 19200 baud: one bit = 52.08 µs  ≈ 833 CPU cycles.
  At 16 MHz / 115200 baud: one bit = 8.68 µs  ≈ 139 CPU cycles (workable but tight).

  TMC2209 single-wire UART protocol:
    Write datagram (8 bytes): SYNC(0x05) | ADDR | REG|0x80 | DATA[3..0] | CRC8
    Read request  (4 bytes):  SYNC(0x05) | ADDR | REG      | CRC8
    Read response (8 bytes):  SYNC(0x05) | 0xFF | REG      | DATA[3..0] | CRC8

  CRC polynomial: 0x07 (standard UART CRC-8).

  Because TX and RX are separate physical pins (connected via 1 kΩ), the MCU
  holds TX high (idle) while receiving the driver's response, so there is no
  bus contention.
*/

#include "grbl.h"

#ifdef TMC2209_SENSORLESS_HOMING

#include <util/delay.h>
#include <avr/interrupt.h>

// ---------------------------------------------------------------------------
// Bit period in microseconds (compile-time constant for _delay_us).
// ---------------------------------------------------------------------------
#define TMC_BIT_US   (1000000.0 / TMC2209_BAUD_RATE)
#define TMC_HALF_BIT_US (TMC_BIT_US / 2.0)

// Timeout for waiting for a RX start bit: 15 ms expressed as loop iterations.
// Each iteration is ~5 µs (conservative); 15 ms / 5 µs = 3000 iterations.
#define TMC_RX_TIMEOUT_LOOPS  3000

// In single-wire UART mode the TMC2209 echoes every byte it receives back on
// the same wire before sending a read reply.  We must account for these echo
// bytes in both writes and reads:
//
//   Write (8 bytes TX) → IC echoes 8 bytes.  Echo starts after a 4-bit
//   inter-frame gap and lasts (8 × 10) bit-periods.
//
//   Read request (4 bytes TX) → IC echoes 4 bytes, then sends 8-byte reply.
//
// For writes we add a post-TX delay (outside cli) so the echo clears before
// the next register access.  Overlapping writes corrupt each other because
// when the master's TX output is HIGH (idle) but the IC is echoing a 0 bit,
// the 1 kΩ coupling lets the IC pull the shared line LOW, corrupting the next
// byte.
//
// For reads we actively receive and discard the 4 echo bytes (inside cli)
// then receive the real 8-byte reply.
#define TMC_ECHO_GUARD_BITS   4                               // inter-frame gap before echo
#define TMC_WRITE_ECHO_US     (TMC_BIT_US * (TMC_ECHO_GUARD_BITS + 8*10.0))  // 8-byte write echo
#define TMC_READ_ECHO_US      (TMC_BIT_US * (TMC_ECHO_GUARD_BITS + 4*10.0))  // 4-byte read-req echo

// ---------------------------------------------------------------------------
// Per-axis initialisation result (set by tmc2209_init, read by tmc2209_axis_ok)
// ---------------------------------------------------------------------------
static bool tmc_init_ok[2] = {false, false};

// ---------------------------------------------------------------------------
// Axis-indexed pin accessors
// ---------------------------------------------------------------------------
typedef struct {
    volatile uint8_t *tx_ddr;
    volatile uint8_t *tx_port;
    uint8_t           tx_bit;
    volatile uint8_t *rx_ddr;
    volatile uint8_t *rx_port;  // write to enable pull-up
    volatile uint8_t *rx_pin;   // read for input state
    uint8_t           rx_bit;
    uint8_t           addr;
} tmc_axis_t;

static const tmc_axis_t tmc_axes[2] = {
    {   // X axis
        .tx_ddr  = &TMC_X_TX_DDR,
        .tx_port = &TMC_X_TX_PORT,
        .tx_bit  = TMC_X_TX_BIT,
        .rx_ddr  = &TMC_X_RX_DDR,
        .rx_port = &TMC_X_RX_PORT,
        .rx_pin  = &TMC_X_RX_PIN,
        .rx_bit  = TMC_X_RX_BIT,
        .addr    = TMC2209_X_ADDR,
    },
    {   // Y axis
        .tx_ddr  = &TMC_Y_TX_DDR,
        .tx_port = &TMC_Y_TX_PORT,
        .tx_bit  = TMC_Y_TX_BIT,
        .rx_ddr  = &TMC_Y_RX_DDR,
        .rx_port = &TMC_Y_RX_PORT,
        .rx_pin  = &TMC_Y_RX_PIN,
        .rx_bit  = TMC_Y_RX_BIT,
        .addr    = TMC2209_Y_ADDR,
    },
};

// ---------------------------------------------------------------------------
// CRC-8, polynomial 0x07 (standard for TMC UART datagrams)
// ---------------------------------------------------------------------------
static uint8_t tmc_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        uint8_t b = data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if ((crc ^ b) & 0x80) { crc = (crc << 1) ^ 0x07; }
            else                  { crc <<= 1; }
            b <<= 1;
        }
    }
    return crc;
}

// ---------------------------------------------------------------------------
// Low-level bit-bang TX: send one byte, LSB first, with start/stop bits.
// Interrupts MUST be disabled by the caller for timing accuracy.
// ---------------------------------------------------------------------------
static void tmc_send_byte(const tmc_axis_t *ax, uint8_t b)
{
    // Start bit
    *ax->tx_port &= ~(1 << ax->tx_bit);
    _delay_us(TMC_BIT_US);
    // Data bits, LSB first
    for (uint8_t i = 0; i < 8; i++) {
        if (b & 0x01) { *ax->tx_port |=  (1 << ax->tx_bit); }
        else          { *ax->tx_port &= ~(1 << ax->tx_bit); }
        b >>= 1;
        _delay_us(TMC_BIT_US);
    }
    // Stop bit (idle = HIGH)
    *ax->tx_port |= (1 << ax->tx_bit);
    _delay_us(TMC_BIT_US);
}

// ---------------------------------------------------------------------------
// Low-level bit-bang RX: receive one byte.
// Waits for a start bit falling edge, then samples at the centre of each bit.
// Returns true if a byte was received; false on timeout.
// Interrupts MUST be disabled by the caller.
// ---------------------------------------------------------------------------
static bool tmc_recv_byte(const tmc_axis_t *ax, uint8_t *out)
{
    // Wait for start bit (line goes LOW)
    uint16_t timeout = TMC_RX_TIMEOUT_LOOPS;
    while (*ax->rx_pin & (1 << ax->rx_bit)) {
        _delay_us(5);
        if (!--timeout) { return false; }
    }
    // We are at the falling edge of the start bit; wait to its midpoint
    _delay_us(TMC_HALF_BIT_US);
    // Verify it really is a start bit (should still be low)
    if (*ax->rx_pin & (1 << ax->rx_bit)) { return false; }
    // Sample 8 data bits at the centre of each bit period
    uint8_t b = 0;
    for (uint8_t i = 0; i < 8; i++) {
        _delay_us(TMC_BIT_US);
        if (*ax->rx_pin & (1 << ax->rx_bit)) { b |= (1 << i); }
    }
    // Consume the stop bit
    _delay_us(TMC_BIT_US);
    *out = b;
    return true;
}

// ---------------------------------------------------------------------------
// Write a 32-bit value to a TMC2209 register.
// ---------------------------------------------------------------------------
void tmc2209_write_reg(uint8_t axis, uint8_t reg, uint32_t val)
{
    if (axis > 1) { return; }
    const tmc_axis_t *ax = &tmc_axes[axis];

    uint8_t dgram[8];
    dgram[0] = 0x05;               // SYNC byte
    dgram[1] = ax->addr;           // Node address
    dgram[2] = reg | 0x80;         // Register address with write flag
    dgram[3] = (uint8_t)(val >> 24);
    dgram[4] = (uint8_t)(val >> 16);
    dgram[5] = (uint8_t)(val >>  8);
    dgram[6] = (uint8_t)(val      );
    dgram[7] = tmc_crc8(dgram, 7);

    uint8_t sreg = SREG;
    cli();
    for (uint8_t i = 0; i < 8; i++) { tmc_send_byte(ax, dgram[i]); }
    SREG = sreg;
    // Wait for the IC's echo to finish before the next register access.
    // The echo is open-drain on the shared wire; if the next TX starts while
    // the echo is still in progress the IC's 0-bits pull down the line and
    // corrupt the master's 1-bits.  This delay is outside cli() so the
    // stepper ISR continues to fire normally.
    _delay_us(TMC_WRITE_ECHO_US);
}

// ---------------------------------------------------------------------------
// Read a 32-bit value from a TMC2209 register.
// Returns true if a valid response was received (correct CRC and address).
// ---------------------------------------------------------------------------
bool tmc2209_read_reg(uint8_t axis, uint8_t reg, uint32_t *val)
{
    if (axis > 1) { return false; }
    const tmc_axis_t *ax = &tmc_axes[axis];

    // Send read request (4 bytes)
    uint8_t req[4];
    req[0] = 0x05;
    req[1] = ax->addr;
    req[2] = reg & 0x7F;  // Read bit = 0
    req[3] = tmc_crc8(req, 3);

    uint8_t sreg = SREG;
    cli();
    for (uint8_t i = 0; i < 4; i++) { tmc_send_byte(ax, req[i]); }
    // The IC echoes our 4-byte request before sending its 8-byte reply.
    // Receive and discard those echo bytes so we are aligned on the real response.
    { uint8_t dummy; for (uint8_t i = 0; i < 4; i++) { tmc_recv_byte(ax, &dummy); } }
    // Receive 8-byte response
    uint8_t resp[8];
    for (uint8_t i = 0; i < 8; i++) {
        if (!tmc_recv_byte(ax, &resp[i])) {
            SREG = sreg;
            return false;
        }
    }
    SREG = sreg;

    // Validate CRC
    if (resp[7] != tmc_crc8(resp, 7)) { return false; }
    // Validate SYNC and master address (0xFF for replies to master)
    if (resp[0] != 0x05 || resp[1] != 0xFF) { return false; }

    *val = ((uint32_t)resp[3] << 24) |
           ((uint32_t)resp[4] << 16) |
           ((uint32_t)resp[5] <<  8) |
            (uint32_t)resp[6];
    return true;
}

// ---------------------------------------------------------------------------
// Serial hex helper — writes one byte as two uppercase ASCII hex digits.
// ---------------------------------------------------------------------------
static void tmc_write_hex8(uint8_t b)
{
    const uint8_t hi = (b >> 4) & 0x0F;
    const uint8_t lo =  b       & 0x0F;
    serial_write(hi < 10 ? '0' + hi : 'A' + hi - 10);
    serial_write(lo < 10 ? '0' + lo : 'A' + lo - 10);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

bool tmc2209_init(uint8_t axis)
{
    if (axis > 1) { return false; }
    const tmc_axis_t *ax = &tmc_axes[axis];

    // --- Pin setup ----------------------------------------------------------
    *ax->tx_ddr  |=  (1 << ax->tx_bit);   // TX: output, idle HIGH
    *ax->tx_port |=  (1 << ax->tx_bit);
    *ax->rx_ddr  &= ~(1 << ax->rx_bit);   // RX: input with pull-up
    *ax->rx_port |=  (1 << ax->rx_bit);
    _delay_us(100);                        // let pull-up settle

    // --- Self-test: does TX LOW appear on RX? --------------------------------
    // Drive TX LOW and check RX; if RX stays HIGH the 1k coupling or the
    // RX→PDN_UART jumper is missing.  Restore TX HIGH before UART traffic.
    cli();
    *ax->tx_port &= ~(1 << ax->tx_bit);   // TX LOW
    _delay_us(5);
    bool loopback_ok = !(*ax->rx_pin & (1 << ax->rx_bit));  // RX should be LOW
    *ax->tx_port |=  (1 << ax->tx_bit);   // TX HIGH (idle)
    sei();

    // --- Configure driver via UART ------------------------------------------
    tmc2209_write_reg(axis, TMC_REG_GCONF, TMC_GCONF_PDN_DISABLE);
    tmc2209_write_reg(axis, TMC_REG_IHOLD_IRUN,
        TMC_IHOLD_IRUN_VAL(TMC2209_IHOLD, TMC2209_IRUN, 6));

    // --- Read IOIN: capture all bytes for diagnostics -----------------------
    // We send the 4-byte request, read back 4 echo bytes then the 8-byte reply,
    // storing everything so the diagnostic message can show exactly what failed.
    uint8_t req[4];
    req[0] = 0x05;
    req[1] = ax->addr;
    req[2] = TMC_REG_IOIN & 0x7F;
    req[3] = tmc_crc8(req, 3);

    uint8_t echo_buf[4] = {0, 0, 0, 0};
    uint8_t resp_buf[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    int8_t  echo_fail = -1;   // index of first timed-out echo byte, or -1 = all OK
    int8_t  resp_fail = -1;   // index of first timed-out response byte, or -1 = all OK

    uint8_t sreg = SREG;
    cli();
    for (uint8_t i = 0; i < 4; i++) { tmc_send_byte(ax, req[i]); }
    for (uint8_t i = 0; i < 4; i++) {
        if (!tmc_recv_byte(ax, &echo_buf[i])) { echo_fail = (int8_t)i; break; }
    }
    if (echo_fail < 0) {
        for (uint8_t i = 0; i < 8; i++) {
            if (!tmc_recv_byte(ax, &resp_buf[i])) { resp_fail = (int8_t)i; break; }
        }
    }
    SREG = sreg;

    // --- Parse result -------------------------------------------------------
    uint8_t ver = 0;
    bool crc_ok = false;
    bool ok = false;
    if (echo_fail < 0 && resp_fail < 0) {
        uint8_t exp_crc = tmc_crc8(resp_buf, 7);
        crc_ok = (resp_buf[7] == exp_crc)
               && (resp_buf[0] == 0x05)
               && (resp_buf[1] == 0xFF);
        if (crc_ok) {
            uint32_t ioin = ((uint32_t)resp_buf[3] << 24)
                          | ((uint32_t)resp_buf[4] << 16)
                          | ((uint32_t)resp_buf[5] <<  8)
                          |  (uint32_t)resp_buf[6];
            ver = (uint8_t)((ioin >> TMC_IOIN_VERSION_SHIFT) & TMC_IOIN_VERSION_MASK);
            ok  = (ver == TMC2209_VERSION);
        }
    }

    // --- Diagnostic message -------------------------------------------------
    // Examples:
    //   [MSG:TMC2209 X lb=ok echo=05000600 resp=05FF0600000021B8 crc=ok ver=21 OK]
    //   [MSG:TMC2209 X lb=FAIL echo=to@0]
    //   [MSG:TMC2209 X lb=ok echo=05000600 resp=to@3]
    //   [MSG:TMC2209 X lb=ok echo=05000600 resp=XXXXXXXXXXXXXXXX crc=err ver=00 FAIL]
    #define TW(c) serial_write(c)
    TW('['); TW('M'); TW('S'); TW('G'); TW(':');
    TW('T'); TW('M'); TW('C'); TW('2'); TW('2'); TW('0'); TW('9'); TW(' ');
    TW(axis == 0 ? 'X' : 'Y'); TW(' ');
    // Loopback result
    TW('l'); TW('b'); TW('=');
    if (loopback_ok) { TW('o'); TW('k'); } else { TW('F'); TW('A'); TW('I'); TW('L'); }
    TW(' ');
    // Echo bytes
    TW('e'); TW('c'); TW('h'); TW('o'); TW('=');
    if (echo_fail >= 0) {
        TW('t'); TW('o'); TW('@'); TW('0' + (uint8_t)echo_fail);
    } else {
        for (uint8_t i = 0; i < 4; i++) { tmc_write_hex8(echo_buf[i]); }
        // Response bytes
        TW(' '); TW('r'); TW('e'); TW('s'); TW('p'); TW('=');
        if (resp_fail >= 0) {
            TW('t'); TW('o'); TW('@'); TW('0' + (uint8_t)resp_fail);
        } else {
            for (uint8_t i = 0; i < 8; i++) { tmc_write_hex8(resp_buf[i]); }
            TW(' '); TW('c'); TW('r'); TW('c'); TW('=');
            if (crc_ok) { TW('o'); TW('k'); } else { TW('e'); TW('r'); TW('r'); }
            TW(' '); TW('v'); TW('e'); TW('r'); TW('=');
            tmc_write_hex8(ver);
        }
    }
    TW(' ');
    if (ok) { TW('O'); TW('K'); } else { TW('F'); TW('A'); TW('I'); TW('L'); }
    TW(']'); TW('\r'); TW('\n');
    #undef TW

    tmc_init_ok[axis] = ok;
    return ok;
}

bool tmc2209_axis_ok(uint8_t axis)
{
    if (axis > 1) { return false; }
    return tmc_init_ok[axis];
}

void tmc2209_homing_start(uint8_t axis)
{
    if (axis > 1) { return; }
    // Force SpreadCycle at all speeds (stallGuard is silent in StealthChop)
    tmc2209_write_reg(axis, TMC_REG_TPWMTHRS, 0);
    // Enable stallGuard across the full speed range
    tmc2209_write_reg(axis, TMC_REG_TCOOLTHRS, TMC2209_TCOOLTHRS);
    // Load seek-phase threshold
    tmc2209_write_reg(axis, TMC_REG_SGTHRS,
        (uint32_t)settings.tmc_sgthrs[axis][TMC_PHASE_SEEK]);
}

void tmc2209_set_sgthrs(uint8_t axis, uint8_t phase)
{
    if (axis > 1 || phase > 1) { return; }
    tmc2209_write_reg(axis, TMC_REG_SGTHRS,
        (uint32_t)settings.tmc_sgthrs[axis][phase]);
}

void tmc2209_homing_end(uint8_t axis)
{
    if (axis > 1) { return; }
    // Disable stallGuard so normal moves don't false-trigger
    tmc2209_write_reg(axis, TMC_REG_TCOOLTHRS, 0);
    tmc2209_write_reg(axis, TMC_REG_SGTHRS, 0);
    // Re-enable StealthChop for quiet operation during imaging
    tmc2209_write_reg(axis, TMC_REG_TPWMTHRS, TMC2209_TPWMTHRS_NORMAL);
}

#endif // TMC2209_SENSORLESS_HOMING
