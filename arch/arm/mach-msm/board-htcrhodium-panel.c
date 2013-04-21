/* linux/arch/arm/mach-msm/board-htcrhod-panel.c
 * Based on board-trout-panel.c by: Brian Swetland <swetland@google.com>
 * Remodelled based on board-supersonic-panel.c by: Jay Tu <jay_tu@htc.com>
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/leds.h>
#include <linux/clk.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/microp-klt.h>

#include <asm/io.h>
#include <asm/gpio.h>
#include <asm/mach-types.h>

#include <mach/msm_fb.h>
#include <mach/vreg.h>
#include <linux/microp-klt.h>

#include "board-htctopaz.h"
#include "board-htcrhodium.h"
#include "proc_comm_wince.h"
#include "devices.h"

#define REG_WAIT (0xffff)

static void suc_backlight_switch(int on);

static unsigned int auto_bl=0;
int panel_id;

static struct cabc_t {
	struct led_classdev lcd_backlight;
	struct msm_mddi_client_data *client_data;
	struct mutex lock;
	unsigned long status;
} cabc;

enum {
	GATE_ON = 1 << 0,
};

/* Panel IDs for reference
1 : 0x01 AUO ES1
5 : 0x05 Hitachi
7 : 0x07 EID
13: 0xD  Sharp EVT
18: 0x12 Does not exist on CDMA
19: 0x13 AUO ES2
20: 0x14 EID ES3
21: 0x15 SHARP DVT
*/

#define PANEL_AUO_ES1	0x1
#define PANEL_HITACHI	0x5
#define PANEL_EID		0x7   // Named SEID in GSM
#define PANEL_SHARP_DVT	0xD
#define PANEL_AUO_ES2	0x13
#define PANEL_EID_ES3	0x14  // Named EID in GSM
#define PANEL_SHARP_EVT	0x15

struct nov_regs {
	unsigned reg;
	unsigned val;
} nov_init_eid_es3[] = {
	/* EID Rhod100,210,	// Table modified by WisTilt2
	       400,500 */
	{0x2900, 0x01},		// display on
	{0x1200, 0x01},		// partial mode
	{0x1100, 0x01},		// sleep-out
	{0x1300, 0x01},		// normal mode
	{0x3500, 0x00},		// tearing effect
	{0x3a00, 0x55},		// bits per pixel
	{0x3b00, 0x40},		// rgb mode 2, hsp/vsp high trigger
	{0x3b02, 0x00},		// vbp 1 clk
	{0x3b03, 0x00},		// vfp 1 clk
	{0x3b04, 0x00},		// hbp 1 clk
	{0x3b05, 0x00},		// hfp 1 clk
	{0x4400, 0x00},		// set tear line 1st param
	{0x4401, 0x00},		// set tear line 2nd param
	{0x4e00, 0x00},		// set spi/i2c mode
	{0x5301, 0x10},		// led control pins
	{0x5302, 0x01},		// labc dimming on/off
	{0x5303, 0x01},		// labc rising dimming style
	{0x5304, 0x01},		// labc falling dimming style
	{0x5500, 0x02},		// cabc enable & image type
	{0x5e00, 0x04},		// cabc minimum brightness
	{0x5e03, 0x05},		// labc adc & hysteresis enable
//	{0x5e04, 0x01},		// labc reference voltage 1.7v
	{0x6a01, 0x00},		// pwm duty/freq control
	{0x6a02, 0x01},		// pwm freq
	{0x6a17, 0x00},		// cabc pwm force off
	{0x6a18, 0xff},		// cabc pwm duty
	{0x6f00, 0x00},		// clear ls lsb
	{0x6f01, 0x00},		// clear ls msb
	{0x6f02, 0x00},		// force cabc pwm off
	{0x6f03, 0x00},		// force pwm duty
	{0x5300, 0x2c},		// no autobl bit during init
};

struct nov_regs nov_init_eid[] = {
	/* EID */
	{0x5100, 0x00},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
    {0x680, 0x35},
    {0x2280, 0xC},
    {0x2480, 0x2B},
    {0x2580, 0x5B},
    {0x2680, 0x78},
    {0x2780, 0x5C},
    {0x2880, 0x3E},
    {0x2980, 4},
    {0xD080, 0x1A},
    {0xD180, 0x16},
    {0xD280, 6},
    {0xD380, 8},
    {0xD480, 0x57},
    {0xD580, 0x15},
    {0xD680, 0x5C},
    {0xD780, 2},
    {0xD880, 0xFC},
    {0xDC80, 6},
    {0xDD80, 2},
    {0xDE80, 2},
    {0x4080, 0},
    {0x4180, 0x13},
    {0x4280, 0x2C},
    {0x4380, 0x40},
    {0x4480, 0x1C},
    {0x4580, 0x30},
    {0x4680, 0x60},
    {0x4780, 0x54},
    {0x4880, 0x20},
    {0x4980, 0x27},
    {0x4A80, 0xAC},
    {0x4B80, 0x20},
    {0x4C80, 0x4B},
    {0x4D80, 0x63},
    {0x4E80, 0xA0},
    {0x4F80, 0xBC},
    {0x5080, 0x5A},
    {0x5180, 0x73},
    {0x5280, 0},
    {0x5880, 0},
    {0x5980, 0x18},
    {0x5A80, 0x36},
    {0x5B80, 0x52},
    {0x5C80, 0x1C},
    {0x5D80, 0x35},
    {0x5E80, 0x5F},
    {0x5F80, 0x46},
    {0x6080, 0x17},
    {0x6180, 0x1E},
    {0x6280, 0x9F},
    {0x6380, 0x1E},
    {0x6480, 0x4F},
    {0x6580, 0x63},
    {0x6680, 0xB2},
    {0x6780, 0xC6},
    {0x6880, 0x5F},
    {0x6980, 0x72},
    {0x6A80, 0},
    {0x7080, 0},
    {0x7180, 0x13},
    {0x7280, 0x2C},
    {0x7380, 0x40},
    {0x7480, 0x1C},
    {0x7580, 0x30},
    {0x7680, 0x60},
    {0x7780, 0x54},
    {0x7880, 0x20},
    {0x7980, 0x27},
    {0x7A80, 0xAC},
    {0x7B80, 0x20},
    {0x7C80, 0x4B},
    {0x7D80, 0x63},
    {0x7E80, 0xA0},
    {0x7F80, 0xBC},
    {0x8080, 0x5A},
    {0x8180, 0x73},
    {0x8280, 0},
    {0x8880, 0},
    {0x8980, 0x18},
    {0x8A80, 0x36},
    {0x8B80, 0x52},
    {0x8C80, 0x1C},
    {0x8D80, 0x35},
    {0x8E80, 0x5F},
    {0x8F80, 0x46},
    {0x9080, 0x17},
    {0x9180, 0x1E},
    {0x9280, 0x9F},
    {0x9380, 0x1E},
    {0x9480, 0x4F},
    {0x9580, 0x63},
    {0x9680, 0xB2},
    {0x9780, 0xC6},
    {0x9880, 0x5F},
    {0x9980, 0x72},
    {0x9A80, 0},
    {0xA080, 0},
    {0xA180, 0x10},
    {0xA280, 0x28},
    {0xA380, 0x3D},
    {0xA480, 0x14},
    {0xA580, 0x28},
    {0xA680, 0x5B},
    {0xA780, 0x47},
    {0xA880, 0x21},
    {0xA980, 0x29},
    {0xAA80, 0xA2},
    {0xAB80, 0x20},
    {0xAC80, 0x4B},
    {0xAD80, 0x62},
    {0xAE80, 0xA2},
    {0xAF80, 0xBC},
    {0xB080, 0x5A},
    {0xB180, 0x73},
    {0xB280, 0},
    {0xB880, 0},
    {0xB980, 0x19},
    {0xBA80, 0x37},
    {0xBB80, 0x51},
    {0xBC80, 0x1D},
    {0xBD80, 0x33},
    {0xBE80, 0x60},
    {0xBF80, 0x50},
    {0xC080, 0x17},
    {0xC180, 0x1E},
    {0xC280, 0xAC},
    {0xC380, 0x24},
    {0xC480, 0x58},
    {0xC580, 0x6B},
    {0xC680, 0xB5},
    {0xC780, 0xCA},
    {0xC880, 0x62},
    {0xC980, 0x72},
    {0xCA80, 0},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{0x3500, 0x00},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5e03, 0x05},
};

struct nov_regs nov_init_auo_es1_1[] = {
	/* Auo es1 part 1*/
    {0x2200, 3},
    {0xF200,0xFC},
    {0xF207, 4},
    {0xB600,0x40},
	{REG_WAIT, 0x1},
    {0xF000,0x18},
    {0x6A02, 1},
    {0xF007,0x11},
    {0xC100,0x45},
    {0xC200,0x31},
    {0xC202,0x30},
    {0xC000,0x90},
    {0xC001, 0},
    {0xC002,0x6B},
    {0xC003, 0},
    {0xC700,0x83},
    {0xE000, 0},
    {0xE001, 5},
    {0xE002,0x15},
    {0xE003,0x24},
    {0xE004,0x1B},
    {0xE005,0x2F},
    {0xE006,0x60},
    {0xE007,0x27},
    {0xE008,0x20},
    {0xE009,0x27},
    {0xE00A,0x6F},
    {0xE00B,0x15},
    {0xE00C,0x39},
    {0xE00D,0x4E},
    {0xE00E,0x61},
    {0xE00F,0x82},
    {0xE010,0x2B},
    {0xE011,0x32},
    {0xE100, 0},
    {0xE101, 7},
    {0xE102,0x19},
    {0xE103,0x2B},
    {0xE104,0x1B},
    {0xE105,0x2E},
    {0xE106,0x60},
    {0xE107,0x3E},
    {0xE108,0x20},
    {0xE109,0x27},
    {0xE10A,0x8F},
    {0xE10B,0x15},
    {0xE10C,0x38},
    {0xE10D,0x4D},
    {0xE10E,0x90},
    {0xE10F,0xB4},
    {0xE110,0x63},
    {0xE111,0x69},
    {0xE200, 0},
    {0xE201, 5},
    {0xE202,0x15},
    {0xE203,0x24},
    {0xE204,0x1B},
    {0xE205,0x2F},
    {0xE206,0x60},
    {0xE207,0x27},
    {0xE208,0x20},
    {0xE209,0x27},
    {0xE20A,0x6F},
    {0xE20B,0x15},
    {0xE20C,0x39},
    {0xE20D,0x4E},
    {0xE20E,0x61},
    {0xE20F,0x82},
    {0xE210,0x2B},
    {0xE211,0x32},
    {0xE300, 0},
    {0xE301, 7},
    {0xE302,0x19},
    {0xE303,0x2B},
    {0xE304,0x1B},
    {0xE305,0x2E},
    {0xE306,0x60},
    {0xE307,0x3E},
    {0xE308,0x20},
    {0xE309,0x27},
    {0xE30A,0x8F},
    {0xE30B,0x15},
    {0xE30C,0x38},
    {0xE30D,0x4D},
    {0xE30E,0x90},
    {0xE30F,0xB4},
    {0xE310,0x63},
    {0xE311,0x69},
    {0xE400, 0},
    {0xE401, 5},
    {0xE402,0x15},
    {0xE403,0x24},
    {0xE404,0x1B},
    {0xE405,0x2F},
    {0xE406,0x60},
    {0xE407,0x27},
    {0xE408,0x20},
    {0xE409,0x27},
    {0xE40A,0x6F},
    {0xE40B,0x15},
    {0xE40C,0x39},
    {0xE40D,0x4E},
    {0xE40E,0x61},
    {0xE40F,0x82},
    {0xE410,0x2B},
    {0xE411,0x32},
    {0xE500, 0},
    {0xE501, 7},
    {0xE502,0x19},
    {0xE503,0x2B},
    {0xE504,0x1B},
    {0xE505,0x2E},
    {0xE506,0x60},
    {0xE507,0x3E},
    {0xE508,0x20},
    {0xE509,0x27},
    {0xE50A,0x8F},
    {0xE50B,0x15},
    {0xE50C,0x38},
    {0xE50D,0x4D},
    {0xE50E,0x90},
    {0xE50F,0xB4},
    {0xE510,0x63},
    {0xE511,0x69},
};

struct nov_regs nov_init_auo_es1_2[] = {
	/* Auo es1 part 2*/
	{0x5100, 0x00},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{REG_WAIT, 0x1},
	{0x3500, 0x02},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5e03, 0x05},
};

struct nov_regs nov_init_auo_es2_1[] = {
	/* Auo es2 part 1*/
    {0xC000, 0x86},
    {0xC001, 0},
    {0xC002, 0x86},
    {0xC003, 0},
    {0xB600, 0x30},
    {0xB602, 0x30},
    {0xC100, 0x40},
    {0xC200, 0x21},
    {0xC202, 2},
    {0xE000, 0},
    {0xE001, 0x14},
    {0xE002, 0x29},
    {0xE003, 0x3A},
    {0xE004, 0x1D},
    {0xE005, 0x30},
    {0xE006, 0x61},
    {0xE007, 0x3E},
    {0xE008, 0x21},
    {0xE009, 0x28},
    {0xE00A, 0x85},
    {0xE00B, 0x16},
    {0xE00C, 0x3B},
    {0xE00D, 0x4C},
    {0xE00E, 0x78},
    {0xE00F, 0x96},
    {0xE010, 0x4A},
    {0xE011, 0x4D},
    {0xE100, 0},
    {0xE101, 0x14},
    {0xE102, 0x29},
    {0xE103, 0x3A},
    {0xE104, 0x1D},
    {0xE105, 0x30},
    {0xE106, 0x61},
    {0xE107, 0x3E},
    {0xE108, 0x21},
    {0xE109, 0x28},
    {0xE10A, 0x85},
    {0xE10B, 0x16},
    {0xE10C, 0x3B},
    {0xE10D, 0x4C},
    {0xE10E, 0x78},
    {0xE10F, 0x96},
    {0xE110, 0x4A},
    {0xE111, 0x4D},
    {0xE200, 0x60},
    {0xE201, 0x62},
    {0xE202, 0x68},
    {0xE203, 0x6F},
    {0xE204, 0x13},
    {0xE205, 0x26},
    {0xE206, 0x59},
    {0xE207, 0x49},
    {0xE208, 0x1E},
    {0xE209, 0x26},
    {0xE20A, 0x88},
    {0xE20B, 0xF},
    {0xE20C, 0x2A},
    {0xE20D, 0x3D},
    {0xE20E, 0x9C},
    {0xE20F, 0xC2},
    {0xE210, 0x4C},
    {0xE211, 0x4D},
    {0xE300, 0x60},
    {0xE301, 0x62},
    {0xE302, 0x68},
    {0xE303, 0x6F},
    {0xE304, 0x13},
    {0xE305, 0x26},
    {0xE306, 0x59},
    {0xE307, 0x49},
    {0xE308, 0x1E},
    {0xE309, 0x26},
    {0xE30A, 0x88},
    {0xE30B, 0xF},
    {0xE30C, 0x2A},
    {0xE30D, 0x3D},
    {0xE30E, 0x9C},
    {0xE30F, 0xC2},
    {0xE310, 0x4C},
    {0xE311, 0x4D},
    {0xE400, 0x7E},
    {0xE401, 0x7F},
    {0xE402, 0x80},
    {0xE403, 0x8C},
    {0xE404, 0x18},
    {0xE405, 0x27},
    {0xE406, 0x5A},
    {0xE407, 0x56},
    {0xE408, 0x1E},
    {0xE409, 0x26},
    {0xE40A, 0x8E},
    {0xE40B, 0x16},
    {0xE40C, 0x3B},
    {0xE40D, 0x5B},
    {0xE40E, 0x80},
    {0xE40F, 0x84},
    {0xE410, 6},
    {0xE411, 0x4D},
    {0xE500, 0x7E},
    {0xE501, 0x7F},
    {0xE502, 0x80},
    {0xE503, 0x8C},
    {0xE504, 0x18},
    {0xE505, 0x27},
    {0xE506, 0x5A},
    {0xE507, 0x56},
    {0xE508, 0x1E},
    {0xE509, 0x26},
    {0xE50A, 0x8E},
    {0xE50B, 0x16},
    {0xE50C, 0x3B},
    {0xE50D, 0x5B},
    {0xE50E, 0x80},
    {0xE50F, 0x84},
    {0xE510, 6},
    {0xE511, 0x4D},
    {0xF402, 0x14},
    {0xF100, 0xC},
};

struct nov_regs nov_init_auo_es2_2[] = {
	/* Auo es2 part 2*/
	{0x5100, 0x00},
	{0x1100, 0x01},
	{REG_WAIT, 0x64},
	{0x4e00, 0x00},
	{0x3a00, 0x05},
	{REG_WAIT, 0x1},
	{0x3500, 0x02},
	{0x4400, 0x00},
	{0x4401, 0x00},
	{0x5e00, 0x00},
	{0x6a01, 0x00},
	{0x6a02, 0x01},
	{0x5301, 0x10},
	{0x5500, 0x02},
	{0x6a17, 0x01},
	{0x6a18, 0xff},
	{0x2900, 0x01},
	{0x5300, 0x2c},
	{0x5e03, 0x05},
};

struct nov_regs nov_init_sharp[] = {
	/* Common to both Sharp panels */
    {0x1100, 0},
	{REG_WAIT, 0x64},
    {0x4080,0x42},
    {0x4180,0x4F},
    {0x4280,0x57},
    {0x4380,0x61},
    {0x4480,0x1F},
    {0x4580,0x32},
    {0x4680,0x63},
    {0x4780,0x46},
    {0x4880,0x28},
    {0x4980,0x30},
    {0x4A80,0x88},
    {0x4B80,0x1D},
    {0x4C80,0x45},
    {0x4D80,0x50},
    {0x4E80,0x6F},
    {0x4F80,0x8D},
    {0x5080,0x39},
    {0x5180,0x5A},
    {0x5880,0x1D},
    {0x5980,0x3E},
    {0x5A80,0x6B},
    {0x5B80,0x88},
    {0x5C80,0x2F},
    {0x5D80,0x3D},
    {0x5E80,0x62},
    {0x5F80,0x74},
    {0x6080,0xF},
    {0x6180,0x17},
    {0x6280,0xB8},
    {0x6380,0x1D},
    {0x6480,0x4E},
    {0x6580,0x60},
    {0x6680,0x9F},
    {0x6780,0xA9},
    {0x6880,0x31},
    {0x6980,0x3E},
    {0x7080,0x63},
    {0x7180,0x6F},
    {0x7280,0x76},
    {0x7380,0x7F},
    {0x7480,0x1F},
    {0x7580,0x30},
    {0x7680,0x65},
    {0x7780,0x56},
    {0x7880,0x26},
    {0x7980,0x2F},
    {0x7A80,0x91},
    {0x7B80,0x1B},
    {0x7C80,0x3F},
    {0x7D80,0x51},
    {0x7E80,0x7A},
    {0x7F80,0x9C},
    {0x8080,0x52},
    {0x8180,0x5A},
    {0x8880,0x1D},
    {0x8980,0x25},
    {0x8A80,0x5B},
    {0x8B80,0x7E},
    {0x8C80,0x2E},
    {0x8D80,0x40},
    {0x8E80,0x65},
    {0x8F80,0x6A},
    {0x9080,0x10},
    {0x9180,0x18},
    {0x9280,0xA7},
    {0x9380,0x1D},
    {0x9480,0x4E},
    {0x9580,0x60},
    {0x9680,0x81},
    {0x9780,0x8A},
    {0x9880,0x11},
    {0x9980,0x1D},
    {0xA080,0x22},
    {0xA180,0x38},
    {0xA280,0x50},
    {0xA380,0x64},
    {0xA480,0x27},
    {0xA580,0x3D},
    {0xA680,0x69},
    {0xA780,0x5A},
    {0xA880,0x28},
    {0xA980,0x31},
    {0xAA80,0x97},
    {0xAB80,0x1B},
    {0xAC80,0x41},
    {0xAD80,0x50},
    {0xAE80,0x7B},
    {0xAF80,0x98},
    {0xB080,0x3C},
    {0xB180,0x5A},
    {0xB880,0x1D},
    {0xB980,0x3B},
    {0xBA80,0x5F},
    {0xBB80,0x7D},
    {0xBC80,0x2F},
    {0xBD80,0x3F},
    {0xBE80,0x64},
    {0xBF80,0x64},
    {0xC080,0xE},
    {0xC180,0x17},
    {0xC280,0xA3},
    {0xC380,0x17},
    {0xC480,0x43},
    {0xC580,0x58},
    {0xC680,0x9C},
    {0xC780,0xB0},
    {0xC880,0x49},
    {0xC980,0x5F},
    {0x4E00,0x00},
    {0x3A00,0x05},
    {0x3500,0x00},
    {0x4400,0x00},
    {0x4401,0x00},
    {0x5E00,0x00},
    {0x6A01,0x00},
    {0x6A02,0x01},
    {0x5301,0x10},
    {0x5500,0x02},
    {0x6A17,0x01},
    {0x6A18,0xFF},
    {0x2900,0x01},
    {0x5300,0x2C},
    {0x5E03,0x05},
};

struct nov_regs nov_init_sharp_evt[] = {
	/* Sharp evt*/
	{0x5100, 0x00},
    {0x2A00, 0x00},
    {0x2A01, 0x00},
    {0x2A02, 0x01},
    {0x2A03, 0xDF},
    {0x2B00, 0x00},
    {0x2B01, 0x00},
    {0x2B02, 0x03},
    {0x2B03, 0x1F},
    {0x2D00, 0x00},
    {0x2D01, 0x00},
    {0x2D02, 0x00},
    {0x2D03, 0x00},
    {0x3600, 0x00},
    {0x3601, 0x01},
    {0x180, 0x02},
    {0x2080,0x43},
    {0x2C80, 0x00},
    {0x2A80, 0x00},
    {0x2E80, 0x00},
};

struct nov_regs nov_init_sharp_dvt[] = {
	/* Sharp dvt*/
	{0x5100, 0x00},
    {0x2A00, 0x00},
    {0x2A01, 0x00},
    {0x2A02, 0x01},
    {0x2A03, 0xDF},
    {0x2B00, 0x00},
    {0x2B01, 0x00},
    {0x2B02, 0x03},
    {0x2B03, 0x1F},
    {0x2D00, 0x00},
    {0x2D01, 0x00},
    {0x2D02, 0x00},
    {0x2D03, 0x00},
    {0x3600, 0x00},
    {0x3601, 0x01},
    {0x180, 0x02},
    {0x2080,0x43},
    {0x2C80, 0x00},
    {0x2A80, 0x00},
    {0xF280,0x55},
    {0xF281,0xAA},
    {0xF282,0x66},
    {0xF38E,0x25},
};

//Extra init needed for hardware auto-backlight
//TODO make hysteresis ranges calculated and easy to update...
struct nov_regs labc_init_seq[] = {
	//Display Profiles, PWM output (low) 00h-FFh (bright)
	{0x5000, 0x0D},	//step 0
	{0x5001, 0x23},
	{0x5002, 0x3C},
	{0x5003, 0x5F},
	{0x5004, 0x8C},
	{0x5005, 0xA9},
	{0x5006, 0xC6},
	{0x5007, 0xE3},
	{0x5008, 0xFF},	//step 8
	//Hysteresis Function, input range (dark)0000h-03FFh(bright)
	{0x5700, 0x00},	//step0 Increment Value
	{0x5701, 0x05},
	{0x5702, 0x00},	//step0 Decrement Value
	{0x5703, 0x05},
	{0x5704, 0x00},	//step1 Increment Value
	{0x5705, 0x0A},
	{0x5706, 0x00},	//step1 Decrement Value
	{0x5707, 0x0A},
	{0x5708, 0x00},	//step2 Increment Value
	{0x5709, 0x14},
	{0x570a, 0x00},	//step2 Decrement Value
	{0x570b, 0x14},
	{0x570c, 0x00},	//step3 Increment Value
	{0x570d, 0x1E},
	{0x570e, 0x00},	//step3 Decrement Value
	{0x570f, 0x1E},
	{0x5710, 0x00},	//step4 Increment Value
	{0x5711, 0x3C},
	{0x5712, 0x00},	//step4 Decrement Value
	{0x5713, 0x3C},
	{0x5714, 0x01},	//step5 Increment Value
	{0x5715, 0x05},
	{0x5716, 0x01},	//step5 Decrement Value
	{0x5717, 0x05},
	{0x5718, 0x02},	//step6 Increment Value
	{0x5719, 0xBA},
	{0x571a, 0x02},	//step6 Decrement Value
	{0x571b, 0xBA},
	{0x571c, 0x02},	//step7 Increment Value
	{0x571d, 0x6E},
	{0x571e, 0x02},	//step7 Decrement Value
	{0x571f, 0x6E},
	{0x5720, 0x03},	//step8 Increment Value
	{0x5721, 0xFF},
	{0x5722, 0x03},	//step8 Decrement Value
	{0x5723, 0xFF},
};


struct nov_regs nov_deinit_seq[] = {
	{0x2800, 0x01},		// display off
   	{0x5300, 0x28},
    {0x5500, 0x00},
    {0x5300, 0x08},
    {0x5E03, 0x00},
    {0x5300, 0x00},
	{0x1000, 0x01},		// sleep-in
	{REG_WAIT, 0x5},
};

static void lacb_toggle(int lacb_on)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	if (lacb_on) {
		auto_bl = 1;
		client->remote_write(client, 0x00, 0x6a17);
		client->remote_write(client, 0x3c, 0x5300);
	} else {
		auto_bl = 0;
		client->remote_write(client, 0x01, 0x6a17);
		client->remote_write(client, 0x24, 0x5300);
		client->remote_write(client, 0x00, 0x5303);
	}
}

static void process_mddi_table(
	struct msm_mddi_client_data *client_data,
	struct nov_regs *table, size_t count)
{
    int i;
	unsigned reg, val;
    for(i = 0; i < count; i++) {
		reg = cpu_to_le32(table[i].reg);
        val = cpu_to_le32(table[i].val);
		if (reg == REG_WAIT)
			msleep(val);
		else
			client_data->remote_write(client_data, val, reg);
    }
}

static int htcrhod_mddi_client_init(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	printk(KERN_DEBUG "%s\n", __func__);
	switch (panel_id)
	{
	case	PANEL_EID:		/* 0x07 - EID */
		client_data->auto_hibernate(client_data, 0);
		process_mddi_table(client_data, nov_init_eid, ARRAY_SIZE(nov_init_eid));
		client_data->auto_hibernate(client_data, 1);
		break;
	case	PANEL_EID_ES3:	/* 0x14 - EID ES3*/
		client_data->auto_hibernate(client_data, 0);
		process_mddi_table(client_data, nov_init_eid_es3, ARRAY_SIZE(nov_init_eid_es3));
		client_data->auto_hibernate(client_data, 1);
		break;
	case	PANEL_AUO_ES1:	/* 0x01 - AUO ES1 */
		client_data->auto_hibernate(client_data, 0);
		/* todo: set clock a cleaner way.*/
		writel((readl(MSM_CLK_CTL_BASE + 0x8c) &0xfffff000) | 0xA41, MSM_CLK_CTL_BASE + 0x8c);
		process_mddi_table(client_data, nov_init_auo_es1_1, ARRAY_SIZE(nov_init_auo_es1_1));
		writel((readl(MSM_CLK_CTL_BASE + 0x8c) &0xfffff000) | 0xA21, MSM_CLK_CTL_BASE + 0x8c);
		process_mddi_table(client_data, nov_init_auo_es1_2, ARRAY_SIZE(nov_init_auo_es1_2));
		client_data->auto_hibernate(client_data, 1);
		break;
	case	PANEL_AUO_ES2:	/* 0x13 - AUO ES2 */
		client_data->auto_hibernate(client_data, 0);
		/* todo: set clock a cleaner way. This panel has been problematic.*/
		writel((readl(MSM_CLK_CTL_BASE + 0x8c) &0xfffff000) | 0xA41, MSM_CLK_CTL_BASE + 0x8c);
		process_mddi_table(client_data, nov_init_auo_es2_1, ARRAY_SIZE(nov_init_auo_es2_1));
		writel((readl(MSM_CLK_CTL_BASE + 0x8c) &0xfffff000) | 0xA21, MSM_CLK_CTL_BASE + 0x8c);
		process_mddi_table(client_data, nov_init_auo_es2_2, ARRAY_SIZE(nov_init_auo_es2_2));
		client_data->auto_hibernate(client_data, 1);
		break;
	case	PANEL_SHARP_EVT:	/* 0x15 - SHARP EVT*/
		client_data->auto_hibernate(client_data, 0);
		process_mddi_table(client_data, nov_init_sharp_evt, ARRAY_SIZE(nov_init_sharp_evt));
		process_mddi_table(client_data, nov_init_sharp, ARRAY_SIZE(nov_init_sharp));
		client_data->auto_hibernate(client_data, 1);
		break;
	default:
		printk(KERN_WARNING "%s: FIXME! Don't know panel_id %d?\n", __func__, panel_id);
		return 0;
	}

	//TODO: do all panels use the same init for labc?
	process_mddi_table(client_data, labc_init_seq, ARRAY_SIZE(labc_init_seq));

	return 0;
}

static int htcrhod_mddi_client_uninit(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	int i;
	unsigned reg, val;
	for (i = 0; i < ARRAY_SIZE(nov_deinit_seq); i++) {
		reg = cpu_to_le32(nov_deinit_seq[i].reg);
		val = cpu_to_le32(nov_deinit_seq[i].val);
		client_data->remote_write(client_data, val, reg);
		mdelay(5);
	}
	return 0;
}

static int htcrhod_mddi_panel_blank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	client_data->remote_write(client_data, 0x24, 0x5300);
	suc_backlight_switch(LED_OFF);
	return 0;
}

static int htcrhod_mddi_panel_unblank(
	struct msm_mddi_bridge_platform_data *bridge_data,
	struct msm_mddi_client_data *client_data)
{
	lacb_toggle(auto_bl);
	suc_backlight_switch(LED_FULL);
	if (!auto_bl) client_data->remote_write(client_data, 0x2c, 0x5300);	// toggle autobl bit
	return 0;
}

static struct vreg *vreg_lcd_1;	/* LCD1 */
static struct vreg *vreg_lcd_2;	/* LCD2 */

static void htcrhod_mddi_power_client(
	struct msm_mddi_client_data *client_data,
	int on)
{
	printk(KERN_DEBUG "%s (%s)\n", __func__, on ? "on" : "off");

	if (panel_id == PANEL_AUO_ES2)return;

	if (on) {
		if (get_machine_variant_type() != MACHINE_VARIANT_RHOD_4XX
			&& get_machine_variant_type() != MACHINE_VARIANT_RHOD_5XX) {
			vreg_enable(vreg_lcd_1);
			msleep(3);
			vreg_enable(vreg_lcd_2);
			msleep(20);
		} else {
			gpio_direction_output(RHOD_LCD_PWR1, 1);
			gpio_direction_output(RHOD_LCD_PWR2, 1);
			msleep(20);
		}
	} else {
		if (get_machine_variant_type() != MACHINE_VARIANT_RHOD_4XX
			&& get_machine_variant_type() != MACHINE_VARIANT_RHOD_5XX) {
			msleep(25);
			vreg_disable(vreg_lcd_1);
			msleep(3);
			vreg_disable(vreg_lcd_2);
			msleep(3);
		} else {
			msleep(25);
			gpio_direction_output(RHOD_LCD_PWR2, 0);
			gpio_direction_output(RHOD_LCD_PWR1, 0);
			msleep(3);
		}
	}
}

extern struct resource resources_msm_fb[];

static struct msm_mddi_bridge_platform_data novatec_client_data = {
	.init = htcrhod_mddi_client_init,
	.uninit = htcrhod_mddi_client_uninit,
	.blank = htcrhod_mddi_panel_blank,
	.unblank = htcrhod_mddi_panel_unblank,
	.fb_data = {
		.xres = 480,
		.yres = 800,
		.output_format = 0,
	},
};

static struct msm_mddi_platform_data mddi_pdata = {
	.vsync_irq = MSM_GPIO_TO_INT(RHOD_LCD_VSYNC),
	.clk_rate = 192000000,
	.power_client = htcrhod_mddi_power_client,
	.fb_resource = resources_msm_fb,
	.num_clients = 2,
	.client_platform_data = {
		{
			// rhod+topa
			.product_id = (0xb9f6 << 16 | 0x5580),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		},
		{
			// rhod+topa
			.product_id = (0xb9f6 << 16 | 0x5582),
			.name = "mddi_c_b9f6_5582",
			.id = 0,
			.client_data = &novatec_client_data,
			.clk_rate = 0,
		}
	},
};

static void suc_set_brightness(struct led_classdev *led_cdev,
				enum led_brightness val)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	unsigned int shrink_br = val;

	printk(KERN_DEBUG "set brightness = %d\n", val);
	if (test_bit(GATE_ON, &cabc.status) == 0)
		return;

	if (val < 30)
		shrink_br = 5;
	else if ((val >= 30) && (val <= 143))
		shrink_br = 104 * (val - 30) / 113 + 5;
	else
		shrink_br = 145 * (val - 144) / 111 + 110;
	mutex_lock(&cabc.lock);
	client->remote_write(client, 0, 0x5500);
	client->remote_write(client, shrink_br, 0x5100);	
	mutex_unlock(&cabc.lock);
}

static enum led_brightness
suc_get_brightness(struct led_classdev *led_cdev)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	return client->remote_read(client, 0x5100);
}

#define DEFAULT_BRIGHTNESS 100
static void suc_backlight_switch(int on)
{
	enum led_brightness val;
	if (on) {
		printk(KERN_DEBUG "turn on backlight\n");
		set_bit(GATE_ON, &cabc.status);
		val = cabc.lcd_backlight.brightness;

		/* LED core uses get_brightness for default value
 		If the physical layer is not ready, we should
		not count on it */
		if (val == 0)
			val = DEFAULT_BRIGHTNESS;
		suc_set_brightness(&cabc.lcd_backlight, val);
	} else {
		clear_bit(GATE_ON, &cabc.status);
		suc_set_brightness(&cabc.lcd_backlight, 0);
	}
}
static unsigned mddi_read_addr;
static ssize_t mddi_remote_read_get(struct device *dev,
	struct device_attribute *attr, char *ret_buf)
{
	struct msm_mddi_client_data *client = cabc.client_data;
	return sprintf(ret_buf,"0x%x\n",client->remote_read(client, mddi_read_addr));
}

static ssize_t mddi_remote_read_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	mddi_read_addr = simple_strtoul(in_buf, NULL, 16);
	printk(KERN_DEBUG "%s: setting mddi_read_addr = %x", __func__, mddi_read_addr);

	return count;
}

static DEVICE_ATTR(remote_read, 0666, mddi_remote_read_get, mddi_remote_read_set);

static ssize_t mddi_remote_store_set(struct device *dev,
	struct device_attribute *attr, const char *in_buf, size_t count)
{
	unsigned val, reg;
	struct msm_mddi_client_data *client = cabc.client_data;
	val = cpu_to_le32(simple_strtoul(in_buf, NULL, 16));
	client->auto_hibernate(client, 0);
	reg = cpu_to_le32(mddi_read_addr);
			
	client->remote_write(client, val, reg);
	
	printk(KERN_DEBUG "%s: setting {register,value}=0x%x 0x%x", __func__, reg,val);

	return count;
}
static DEVICE_ATTR(remote_store, 0666, NULL, mddi_remote_store_set);

static ssize_t autobl_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	int ret;
	ret = sprintf(buf, "%d\n",auto_bl);
	return ret;
}

//board-htcrhodium-led.c provides pieces for i2c
void queue_auto_backlight_update(int ls_auto);

static ssize_t autobl_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	lacb_toggle(ls_auto);
	queue_auto_backlight_update(ls_auto);

	return count;
}

static DEVICE_ATTR(auto_backlight, 0666, autobl_enable_show, autobl_enable_store);

static int suc_backlight_probe(struct platform_device *pdev)
{
	int err = -EIO;
	printk(KERN_DEBUG "%s", __func__);
	mutex_init(&cabc.lock);
	cabc.client_data = pdev->dev.platform_data;
	cabc.lcd_backlight.name = "lcd-backlight";
	cabc.lcd_backlight.brightness_set = suc_set_brightness;
	cabc.lcd_backlight.brightness = 0;
	err = led_classdev_register(&pdev->dev, &cabc.lcd_backlight);
	if (err)
		goto err_register_lcd_bl;
	err = device_create_file(cabc.lcd_backlight.dev, &dev_attr_remote_read);
	err = device_create_file(cabc.lcd_backlight.dev, &dev_attr_remote_store);
	err = device_create_file(cabc.lcd_backlight.dev, &dev_attr_auto_backlight);
	if (err < 0) {
		printk(KERN_ERR "%s: Failed registering suc_backlight attribute (%d)",
			__func__, err);
		goto err_register_lcd_bl;
	}

	printk(KERN_DEBUG "%s successful\n", __func__);	
	return 0;

err_register_lcd_bl:
	led_classdev_unregister(&cabc.lcd_backlight);
	return err;
}

static struct platform_driver suc_backlight_driver = {
	.probe = suc_backlight_probe,
	.driver = {
		.owner = THIS_MODULE,
	},
};

int __init htcrhod_init_panel(void)
{
	int rc;

	if(!machine_is_htcrhodium()) {
		printk(KERN_INFO "%s: panel does not apply to this device, aborted\n", __func__);
		return 0;
	}

        panel_id = readl(MSM_SPL_BASE+0x81034);
	printk("Panel type detected: %x\n",panel_id);

	printk(KERN_INFO "%s: Initializing panel\n", __func__);

	rc = platform_device_register(&msm_device_mdp);
	if (rc)
		return rc;

	vreg_lcd_1 = vreg_get(0, "rftx");
	if (IS_ERR(vreg_lcd_1))
		return PTR_ERR(vreg_lcd_1);

	vreg_lcd_2 = vreg_get(0, "rfrx2");
	if (IS_ERR(vreg_lcd_2))
		return PTR_ERR(vreg_lcd_2);

	rc = gpio_request(RHOD_LCD_PWR1, NULL);
	if (rc)
		return rc;
	rc = gpio_request(RHOD_LCD_PWR2, NULL);
	if (rc)
		return rc;
	rc = gpio_request(RHOD_LCD_VSYNC, "vsync");
	if (rc)
		return rc;
	rc = gpio_direction_input(RHOD_LCD_VSYNC);
	if (rc)
		return rc;

	suc_backlight_driver.driver.name = "nov_cabc";

	rc = platform_driver_register(&suc_backlight_driver);
	if (rc)
		return rc;

	msm_device_mddi0.dev.platform_data = &mddi_pdata;
	return platform_device_register(&msm_device_mddi0);
}
device_initcall(htcrhod_init_panel);
