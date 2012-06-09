#ifndef _MICROP_KSC_H
#define _MICROP_KSC_H

// OR'ed with scancode on the release event
#define MICROP_KSC_RELEASED_BIT 0x80
#define MICROP_KSC_SCANCODE_MASK (MICROP_KSC_RELEASED_BIT - 1)

#define MICROP_KSC_ID_SCANCODE	0x10
#define MICROP_KSC_ID_MODIFIER	0x11
#define MICROP_KSC_ID_VERSION	0x12
#define MICROP_KSC_ID_LED	0x13

enum {
	MICROP_KSC_LED_RESET,	// Resets LEDs to off
	MICROP_KSC_LED_CAPS,	// Caps Lock
	MICROP_KSC_LED_FN,	// FN lock
	MICROP_KSC_LED_MAX,
	MICROP_KSC_LED_FN_RAPH800 = 1,
	MICROP_KSC_LED_CAPS_RAPH800 = 2,
};

extern int micropksc_read_scancode(unsigned char *scancode, unsigned char *isdown);
extern int micropksc_set_led(unsigned int led, int value);

/* Modified by WisTilt2
   Kovsky and Rhod use the same Microp ids for keyboard backlight
   brightness and control, different bitmasks and number of params.
*/
#define MICROP_KSC_ID_QWERTY_BRIGHTNESS_KOVS_RHOD	0x32
#define MICROP_KSC_ID_QWERTY_ENABLE_KOVS_RHOD		0x30
#define QWERTY_RHOD_BRIGHTNESS_PARAM1			0xF3
#define QWERTY_RHOD_BRIGHTNESS_PARAM2			0x27
#define QWERTY_RHOD_BRIGHTNESS_PARAM3			0xF8

#define MICROP_KSC_ID_KEYPAD_LIGHT_RHOD         0x14

extern int micropksc_set_kbd_led_state(int on);
extern int micropksc_set_rhod_kpd_led_state(int on);
extern int micropksc_read_scancode_kovsky(unsigned char *scancode, unsigned char *isdown, unsigned char *clamshell);

#endif
