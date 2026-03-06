/*
  led_control.h - RGB LED control via M150 command
  Part of Grbl-Mega-pyuscope

  Controls RGB LEDs for microscope illumination using hardware PWM:
    Red:   D5  (PE3, Timer 3 OC3A) - Servo 3 header
    Green: D6  (PH3, Timer 4 OC4A) - Servo 2 header
    Blue:  D45 (PL4, Timer 5 OC5B) - AUX-2 header

  Usage: M150 R<0-255> U<0-255> B<0-255>
    R = Red brightness (0-255)
    U = Green brightness (0-255, 'U' because 'G' is reserved for G-codes)
    B = Blue brightness (0-255)
    Omitted parameters default to 0 (off).
*/

#ifndef led_control_h
#define led_control_h

// Initialize LED pins and timers. Call after spindle_init() since they share Timer 4.
void led_init();

// Set RGB LED brightness. Values 0-255 mapped to 10-bit PWM (0-1024).
void led_set_color(uint8_t red, uint8_t green, uint8_t blue);

// Turn all LEDs off.
void led_stop();

// Parse and execute M150 command line. Returns status code.
uint8_t led_parse_m150(char *line);

#endif
