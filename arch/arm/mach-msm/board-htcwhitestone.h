/* linux/arch/arm/mach-msm/board-htcwhitestone.h
 */
#ifndef __ARCH_ARM_MACH_MSM_BOARD_HTCWHITESTONE_H
#define __ARCH_ARM_MACH_MSM_BOARD_HTCWHITESTONE_H

#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>

//~ #define WHIT100_CAM_DAT0	0
//~ #define WHIT100_CAM_DAT1	1
//~ #define WHIT100_CAM_DAT2	2
//~ #define WHIT100_CAM_DAT3	3
//~ #define WHIT100_CAM_DAT4	4
//~ #define WHIT100_CAM_DAT5	5
//~ #define WHIT100_CAM_DAT6	6
//~ #define WHIT100_CAM_DAT7	7
//~ #define WHIT100_CAM_DAT8	8
//~ #define WHIT100_CAM_DAT9	9
//~ #define WHIT100_CAM_DAT10	10
//~ #define WHIT100_CAM_DAT11	11
//~ #define WHIT100_CAM_PCLK	12
//~ #define WHIT100_CAM_HSYNC_IN	13
//~ #define WHIT100_CAM_VSYNC_N	14
//~ #define WHIT100_CAM_MCLK	15

//~ #define WHIT100_CAM_PWR1	30

//~ #define WHIT100_CAM_VCMPDP  91
//~ #define WHIT100_CAM1_RST	92
//~ #define WHIT100_CAM2_RST	93

#define WHIT100_BAT_IN				28			// UNCONFIRMED
#define WHIT100_USB_AC_PWR			32			// UNCONFIRMED
#define WHIT100_CHARGE_EN_N			44			// UNCONFIRMED

#define WHIT100_SEND_KEY			41
#define WHIT100_VOLDOWN_KEY			40
#define WHIT100_VOLUP_KEY			39
#define WHIT100_POWER_KEY			18

#define WHIT100_KPD_IRQ				27	

//~ #define WHIT100_UART2DM_RTS			19	/* BT */
//~ #define WHIT100_UART2DM_CTS			20	/* BT */
//~ #define WHIT100_UART2DM_RX			21	/* BT */

//~ #define WHIT100_CABLE_IN1			18
//~ #define WHIT100_CABLE_IN2			45
//~ #define WHIT100_H2W_DATA			31
//~ #define WHIT100_H2W_CLK				46
//~ #define WHIT100_AUD_HSMIC_DET_N		17
#define WHIT100_USBPHY_RST			100			// UNCONFIRMED

#define WHIT100_LCD_VSYNC			97			// UNCONFIRMED
#define WHIT100_LCD_RESET			82

//~ #define WHIT100_CAMERA_SWITCH		30
//~ #define WHIT100_CAMERA_VCMPDP		91
//~ #define WHIT100_CAMERA1_RST			92
//~ #define WHIT100_CAMERA2_RST			93
//~ #define WHIT100_CAMERA1_PWR			107
//~ #define WHIT100_CAMERA2_PWR			109

#define WHIT100_MICROP_KLT_VERSION_REG		0x30
#define WHIT100_MICROP_KLT_VERSION		0x0a88

#endif /* GUARD */
