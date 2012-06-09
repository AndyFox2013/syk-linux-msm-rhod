/* Author: Abel C. Laura <abel.laura@gmail.com> 
 *
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

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/proc_fs.h>
#include <linux/string.h>
#include <linux/module.h>
#include <asm/setup.h>
#include <asm/mach/map.h>
#include <mach/msm_iomap.h>
#include <linux/delay.h>
#include <mach/bootcheck.h>
#include "proc_comm_wince.h"
//int boot_reason;

unsigned int get_boot_reason(void)
{	
    return readl(MSM_SPL_BASE + 0x8105C);
}

int is_a9_ready(void)
{
	int i;
	/* No info found below, so use smem */
	/*readl(MSM_SPL_BASE + 0x81524) */
	for(i=0; i < 6; i++)
	{
		if (!!readl(MSM_SHARED_RAM_BASE + 0xfc13c)) 
		{
			printk(KERN_INFO "(%d)ARM9 DEX READY\n",i); 
			return 1;
		}
		mdelay(1000);
	}
	printk(KERN_INFO "(6)ARM9 DEX READY\n"); 
	return 0;
}
EXPORT_SYMBOL(is_a9_ready);

unsigned boot_reason_charge(void)
{
    if(POWERON_CHARGING == get_boot_reason())
        return 1;
    else
        return 0;
}
EXPORT_SYMBOL(boot_reason_charge);


static int get_boot_info(char *buf, char **start,
		off_t offset, int count, int *eof, void *data)
{
    int len = 0;
    char reasondesc[30];
    int reason = get_boot_reason();

    switch (reason)
    {
		case POWERON_POWER_BUTTON_ON:
			sprintf(reasondesc, "POWERON_POWER_BUTTON_ON");
			break;
		case POWERON_RESET:
			sprintf(reasondesc, "POWERON_RESET");
			break;
		case POWERON_CHARGING:
			sprintf(reasondesc, "POWERON_CHARGING");
			break;
		case NO_BATTERY_B4:
			sprintf(reasondesc, "NO_BATTERY_B4");
			break;
		case POWERON_SW_RESET:
			sprintf(reasondesc, "POWERON_SW_RESET");
			break;
		default:
			sprintf(reasondesc, "UNKNOWN(%08x)",reason);
			break;
    }

    len += sprintf(buf, "%s", reasondesc);
    return len;
}

static struct proc_dir_entry *proc_bootcheck = NULL;
extern struct proc_dir_entry proc_root;

int __init bootcheck_init_module(void)
{

    proc_bootcheck = &proc_root;
    create_proc_read_entry("bootcheck", 0, NULL,
			   get_boot_info, NULL);
    return 0;
}

void __exit bootcheck_cleanup_module(void)
{
    if (proc_bootcheck) {
		remove_proc_entry("bootcheck", proc_bootcheck);
		proc_bootcheck = NULL;
    }
}

module_init(bootcheck_init_module);
module_exit(bootcheck_cleanup_module);

MODULE_AUTHOR("Abel C. Laura <abel.laura@gmail.com>");
MODULE_LICENSE("GPL");
