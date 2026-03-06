/*
  led_control.c - RGB LED control via M150 command
  Part of Grbl-Mega-pyuscope

  Controls RGB LEDs for microscope illumination using hardware PWM:
    Red:   D5  (PE3, Timer 3 OC3A) - Servo 3 header
    Green: D6  (PH3, Timer 4 OC4A) - Servo 2 header
    Blue:  D44 (PL5, Timer 5 OC5C) - AUX-2 header

  Timer 3 is configured here for LED Red PWM (was previously used for sleep counter,
  which has been moved to Timer 2).

  Timer 4 is shared with spindle PWM (OCR4C on D8). The spindle init already configures
  Timer 4 in Fast PWM mode 14 (TOP=ICR4). We just enable the OC4A output here.

  Timer 5 is configured here for LED Blue PWM (was previously unused).
*/

#include "grbl.h"

#ifdef LED_RED_DDR  // Only compile if LED pins are defined (pyuscope config)


void led_init()
{
  // Configure LED pins as outputs
  LED_RED_DDR |= (1<<LED_RED_BIT);
  LED_GREEN_DDR |= (1<<LED_GREEN_BIT);
  LED_BLUE_DDR |= (1<<LED_BLUE_BIT);

  // Timer 3: Red LED on OC3A (D5)
  // Fast PWM mode 14 (WGM3:0 = 1110), TOP = ICR3, 1/8 prescaler
  // TCCR3A: WGM31=1, WGM30=0 (COM3A bits set later when LED is turned on)
  // TCCR3B: WGM33=1, WGM32=1, CS31=1 (1/8 prescaler)
  TCCR3A = (1<<WGM31);
  TCCR3B = (1<<WGM33) | (1<<WGM32) | (1<<CS31);
  ICR3 = 0x0400;  // TOP = 1024, ~1.9kHz PWM
  OCR3A = 0;

  // Timer 4: Green LED on OC4A (D6)
  // Timer 4 is already configured by spindle_init() in Fast PWM mode 14 (TOP=ICR4).
  // We just need to set OCR4A to 0 initially. OC4A output is enabled when LED is turned on.
  OCR4A = 0;

  // Timer 5: Blue LED on OC5C (D44)
  // Fast PWM mode 14 (WGM5:0 = 1110), TOP = ICR5, 1/8 prescaler
  TCCR5A = (1<<WGM51);
  TCCR5B = (1<<WGM53) | (1<<WGM52) | (1<<CS51);
  ICR5 = 0x0400;  // TOP = 1024, ~1.9kHz PWM
  OCR5C = 0;

  // Start with all LEDs off.
  // For inverted channels, connect OC outputs now (they stay connected permanently)
  // so the pin is driven HIGH (LED off) rather than floating low.
  #ifdef LED_RED_INVERT
    TCCR3A |= (1<<COM3A1);
  #endif
  #ifdef LED_GREEN_INVERT
    TCCR4A |= (1<<COM4A1);
  #endif
  #ifdef LED_BLUE_INVERT
    TCCR5A |= (1<<COM5C1);
  #endif
  led_stop();
}


void led_set_color(uint8_t red, uint8_t green, uint8_t blue)
{
  // Map 0-255 to 0-1024 PWM range (multiply by 4, with 255 mapping to 1024)
  uint16_t r_pwm = (uint16_t)red * 4;
  uint16_t g_pwm = (uint16_t)green * 4;
  uint16_t b_pwm = (uint16_t)blue * 4;

  // For inverted (reverse-acting) channels, invert the PWM value.
  // OCR=TOP gives constant HIGH output (LED off for active-low driver).
  // OCR near 0 gives mostly-LOW output (LED on for active-low driver).
  // OC output stays connected at all times so the pin doesn't float low.

  // Red channel: Timer 3, OC3A (D5)
  #ifdef LED_RED_INVERT
    OCR3A = 0x0400 - r_pwm;
    TCCR3A |= (1<<COM3A1);   // Always connected for inverted output
  #else
    OCR3A = r_pwm;
    if (r_pwm > 0) {
      TCCR3A |= (1<<COM3A1);
    } else {
      TCCR3A &= ~(1<<COM3A1);  // Disconnect OC3A (pin goes low)
    }
  #endif

  // Green channel: Timer 4, OC4A (D6)
  // Note: Timer 4 shared with spindle (OCR4C/D8). Only touch OC4A bits.
  #ifdef LED_GREEN_INVERT
    OCR4A = 0x0400 - g_pwm;
    TCCR4A |= (1<<COM4A1);
  #else
    OCR4A = g_pwm;
    if (g_pwm > 0) {
      TCCR4A |= (1<<COM4A1);
    } else {
      TCCR4A &= ~(1<<COM4A1);
    }
  #endif

  // Blue channel: Timer 5, OC5C (D44)
  #ifdef LED_BLUE_INVERT
    OCR5C = 0x0400 - b_pwm;
    TCCR5A |= (1<<COM5C1);
  #else
    OCR5C = b_pwm;
    if (b_pwm > 0) {
      TCCR5A |= (1<<COM5C1);
    } else {
      TCCR5A &= ~(1<<COM5C1);
    }
  #endif
}


void led_stop()
{
  // Turn all LEDs off.
  // For inverted channels: set OCR=TOP so pin stays HIGH (LED off), keep OC connected.
  // For non-inverted channels: disconnect OC so pin goes LOW (LED off).

  #ifdef LED_RED_INVERT
    OCR3A = 0x0400;
  #else
    TCCR3A &= ~(1<<COM3A1);
    OCR3A = 0;
  #endif

  #ifdef LED_GREEN_INVERT
    OCR4A = 0x0400;
  #else
    TCCR4A &= ~(1<<COM4A1);  // Leave OC4C for spindle
    OCR4A = 0;
  #endif

  #ifdef LED_BLUE_INVERT
    OCR5C = 0x0400;
  #else
    TCCR5A &= ~(1<<COM5C1);
    OCR5C = 0;
  #endif
}


// Parse integer value from string, advancing the pointer. Returns 0 if no digits found.
static uint16_t parse_int(char **p)
{
  uint16_t val = 0;
  while (**p >= '0' && **p <= '9') {
    val = val * 10 + (**p - '0');
    (*p)++;
  }
  return val;
}


uint8_t led_parse_m150(char *line)
{
  // line is already uppercase with spaces stripped by protocol layer.
  // Expected format: "M150R<n>U<n>B<n>" (parameters optional, default to 0)
  uint8_t red = 0, green = 0, blue = 0;

  // Skip past "M150"
  char *p = line + 4;

  while (*p) {
    char letter = *p++;
    uint16_t val = parse_int(&p);
    if (val > 255) val = 255;

    switch (letter) {
      case 'R': red = val; break;
      case 'U': green = val; break;
      case 'B': blue = val; break;
      default:
        return STATUS_GCODE_UNSUPPORTED_COMMAND;
    }
  }

  led_set_color(red, green, blue);
  return STATUS_OK;
}


#endif // LED_RED_DDR
