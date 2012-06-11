/* linux/arch/arm/mach-msm/board-htctopaz.h
 */
#ifndef __ARCH_ARM_MACH_MSM_BOARD_HTCRHODIUM_H
#define __ARCH_ARM_MACH_MSM_BOARD_HTCRHODIUM_H

#include <mach/board.h>


#define DECLARE_MSM_IOMAP
#include <mach/msm_iomap.h>


#define RHODIUM_CABLE_IN1		42
#define RHODIUM_CABLE_IN2		45
#define RHODIUM_H2W_CLK			46
#define RHODIUM_H2W_DATA		92
#define RHODIUM_H2W_UART_MUX		103	/* pretty sure this is right */
#define RHODIUM_GSENSOR_MOT		49	/* microp interrupt for 3.5mm detect */

#define RHODIUM_HEADSET_MIC		17	//0x11
#define RHODIUM_GPIO_UP_RESET_N	43	//0x2B H2W detection enable.

#define RHODIUM_BAT_IRQ			28  // GPIO IRQ
#define RHODIUM_USB_AC_PWR		32
#define RHODIUM_AC_DETECT		82  // ????
#define RHODIUM_CHARGE_EN_N		44
#define RHODIUM_KPD_IRQ			27  //Keyboard IRQ
#define RHODIUM_KB_SLIDER_IRQ		37  //Keyboard Slider IRQ //Currently Unknown, using stylus detect GPIO right now (37).
#define RHODIUM_BKL_PWR			86  //Keyboard blacklight  //Currently Unknown if this is right

#define RHODIUM_END_KEY			18
#define RHODIUM_VOLUMEUP_KEY            39
#define RHODIUM_VOLUMEDOWN_KEY          40
#define RHODIUM_POWER_KEY               83  //Power key
#define RHODIUM_SPKR_PWR        	84
#define RHODIUM_HS_AMP_PWR      	85


#define RHOD_LCD_VSYNC			97
#define RHOD_LCD_PWR1			99	//0x63
#define RHOD_LCD_PWR2			98	//0x62
#define RHOD_LCD_RST			82	//0x52

#define RHODIUM_USBPHY_RST		100

#define RHODIUM_GPIO_PROXIMITY_EN	102
#define RHODIUM_GPIO_PROXIMITY_EN_GSM	99	//0x63	/* this is possibly something differnt on cdma */
#define RHODIUM_GPIO_PROXIMITY_INT_N	90

#define RHODIUM_BT_SHUTDOWN_N   31
#define RHODIUM_BT_nRST         91
#define RHODIUM_BT_HOST_WAKE    94
#define RHODIUM_BT_WAKE         35

#define RHODIUM_PCM_DOUT        68
#define RHODIUM_PCM_DIN         69
#define RHODIUM_PCM_SYNC        70
#define RHODIUM_PCM_CLK         71

#define RHODIUM_CAM_PWR1_CDMA   26
#define RHODIUM_CAM_PWR1        30
#define RHODIUM_VCM_PDP         33
#define RHODIUM_MT9T013_RST     34
#define RHODIUM_VGACAM_RST      109

#define RHOD_MICROP_KLT_VERSION_REG		0x30
#define RHOD_MICROP_KLT_VERSION			0x0a88
#define RHOD_MICROP_KLT_LED_STATE       0x25 // Set LED enables (rhod)
#define RHOD_MICROP_KSC_VERSION_REG		0x12
#define RHOD_MICROP_KSC_VERSION			0x0488

#define RHOD_MICROP_KLT_DUALMIC_EN_BIT (1<<2)
#define RHOD_MICROP_KLT_CODEC_EN_BIT   (1<<4)
#endif 
