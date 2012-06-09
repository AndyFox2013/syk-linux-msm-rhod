#ifndef _BOOTCHECK_H_
#define _BOOTCHECK_H_


#define	POWERON_POWER_BUTTON_ON	0x42555454
#define	POWERON_RESET			0x52455354
#define	POWERON_CHARGING		0x43515247
#define	NO_BATTERY_B4			0x4E424234
#define	POWERON_SW_RESET		0x53525354

int is_a9_ready(void);
unsigned boot_reason_charge(void);

#endif /* _BOOTCHECK_H_ */
