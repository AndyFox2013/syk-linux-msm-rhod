/* arch/arm/mach-msm/include/mach/board.h
 *
 * Copyright (C) 2007 Google, Inc.
 * Author: Brian Swetland <swetland@google.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __ASM_ARCH_MSM_BOARD_H
#define __ASM_ARCH_MSM_BOARD_H

#include <linux/types.h>
#include <mach/mmc.h>

#define MACHINE_VARIANT_UNDEFINED	0x0
#define MACHINE_VARIANT_RHOD_1XX	0x0101
#define MACHINE_VARIANT_RHOD_2XX	0x0102
#define MACHINE_VARIANT_RHOD_3XX	0x0103
#define MACHINE_VARIANT_RHOD_4XX	0x0104
#define MACHINE_VARIANT_RHOD_5XX	0x0105

/* platform device data structures */

/* platform device data structures */
struct msm_acpu_clock_platform_data {
	uint32_t acpu_switch_time_us;
	uint32_t max_speed_delta_khz;
	uint32_t vdd_switch_time_us;
	unsigned long power_collapse_khz;
	unsigned long wait_for_irq_khz;
	unsigned int max_axi_khz;
};

struct msm_camera_io_ext {
    uint32_t mdcphy;
    uint32_t mdcsz;
    uint32_t appphy;
    uint32_t appsz;
    unsigned long camifpadphy;
    unsigned long camifpadsz;
};

struct msm_camera_device_platform_data {
    void (*camera_gpio_on) (void);
    void (*camera_gpio_off)(void);
    struct msm_camera_io_ext ioext;
};

struct msm_camera_legacy_device_platform_data {
    int sensor_reset;
    int sensor_pwd;
    int vcm_pwd;
    void (*config_gpio_on) (void);
    void (*config_gpio_off)(void);
};

#define MSM_CAMERA_FLASH_NONE 0
#define MSM_CAMERA_FLASH_LED  1

struct camera_flash_cfg {
    int num_flash_levels;
    int (*camera_flash)(int level);
    uint16_t low_temp_limit;
    uint16_t low_cap_limit;
    uint8_t postpone_led_mode;
};

struct msm_camera_sensor_info {
    const char *sensor_name;
    int sensor_node;                // 0 for main camera, 1 for front camera
    int sensor_reset;
    int sensor_pwd;
    int vcm_pwd;
    int vcm_reversed_polarity;      // 0 = "normal" use, 1 = "reversed" use
    int vcm_enable;
    void(*camera_clk_switch)(void);
    int mclk;
    int need_suspend;
    struct msm_camera_device_platform_data *pdata;
    struct resource *resource;
    uint8_t num_resources;
    uint32_t waked_up;
    wait_queue_head_t event_wait;
    uint32_t kpi_sensor_start;
    uint32_t kpi_sensor_end;
    int flash_type; /* for back support */
    struct camera_flash_cfg* flash_cfg;
};

struct snd_endpoint {
	int id;
	const char *name;
};

struct msm_snd_endpoints {
	struct snd_endpoint *endpoints;
	unsigned num;
};

struct clk;
struct msm_mmc_platform_data;

extern struct sys_timer msm_timer;

/* common init routines for use by arch/arm/mach-msm/board-*.c */

void __init msm_add_devices(void);
void __init msm_map_common_io(void);
void __init msm_init_irq(void);
void __init msm_init_gpio(void);
void __init msm_clock_init(struct clk *clock_tbl, unsigned num_clocks);
void __init msm_acpu_clock_init(struct msm_acpu_clock_platform_data *);
int __init msm_add_sdcc(unsigned int controller,
			struct msm_mmc_platform_data *plat,
			unsigned int stat_irq, unsigned long stat_irq_flags);
int get_machine_variant_type(void);
#endif
