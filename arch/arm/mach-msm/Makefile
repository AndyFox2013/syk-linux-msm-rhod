obj-y += io.o idle-v6.o timer.o nand_partitions.o
obj-y += clock.o
#obj-$(CONFIG_DEBUG_FS) += clock-debug.o

obj-$(CONFIG_MSM_VIC) += irq-vic.o
obj-$(CONFIG_MSM_IOMMU) += iommu.o iommu_dev.o devices-iommu.o

obj-$(CONFIG_ARCH_MSM7X00A) += dma.o irq.o acpuclock-arm11.o cpufreq.o
obj-$(CONFIG_ARCH_MSM7X30) += dma.o
obj-$(CONFIG_ARCH_QSD8X50) += dma.o sirc.o

obj-$(CONFIG_MSM_PROC_COMM) += proc_comm_wince.o clock-wince.o vreg.o

obj-$(CONFIG_MSM_SMD) += smd.o 
obj-$(CONFIG_MSM_SMD) += smd_tty.o smd_qmi.o

obj-$(CONFIG_MSM_SCM) += scm.o scm-boot.o

obj-$(CONFIG_HOTPLUG_CPU) += hotplug.o
obj-$(CONFIG_SMP) += headsmp.o platsmp.o

obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_device.o
obj-$(CONFIG_MSM_ONCRPCROUTER) += smd_rpcrouter_servers.o

obj-$(CONFIG_MACH_HTCRHODIUM) += board-htcrhodium.o board-htcrhodium-led.o board-htcrhodium-panel.o pm.o msm_vibrator.o pmem.o generic_gpio.o mmc.o devices_htc.o devices-msm7x00.o htc_tp_cal.o htc_acoustic_wince.o htc_battery.o fiq_glue.o msm7200a-rfkill.o board-htcrhodium-audio.o memory.o bootcheck.o htc_wifi_nvs.o 
obj-$(CONFIG_SCBS) += scbs.o
obj-$(CONFIG_MACH_HTCTOPAZ) += board-htctopaz.o board-htctopaz-keypad.o board-htctopaz-panel.o
obj-$(CONFIG_MACH_HTCWHITESTONE) += board-htcwhitestone.o board-htcwhitestone-panel.o board-htcwhitestone-led.o
obj-$(CONFIG_HTC_HEADSET) += htc_headset.o htc_headset_microp.o
obj-$(CONFIG_MSM_HW3D) += hw3d.o
obj-$(CONFIG_MSM_ADSP) += qdsp5/ 
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o board-trout-mmc.o devices-msm7x00.o
obj-$(CONFIG_MACH_TROUT) += board-trout.o board-trout-gpio.o board-trout-mmc.o board-trout-panel.o devices-msm7x00.o
obj-$(CONFIG_MACH_HALIBUT) += board-halibut.o devices-msm7x00.o
obj-$(CONFIG_ARCH_MSM7X30) += board-msm7x30.o devices-msm7x30.o
obj-$(CONFIG_ARCH_QSD8X50) += board-qsd8x50.o devices-qsd8x50.o
obj-$(CONFIG_ARCH_MSM8X60) += board-msm8x60.o
obj-$(CONFIG_ARCH_MSM8960) += board-msm8960.o devices-msm8960.o

obj-$(CONFIG_ARCH_MSM7X30) += gpiomux-v1.o gpiomux.o
obj-$(CONFIG_ARCH_QSD8X50) += gpiomux-8x50.o gpiomux-v1.o gpiomux.o
obj-$(CONFIG_ARCH_MSM8X60) += gpiomux-8x60.o gpiomux-v2.o gpiomux.o
ifdef CONFIG_MSM_V2_TLMM
ifndef CONFIG_ARCH_MSM8960
# TODO: TLMM Mapping issues need to be resolved
obj-y	+= gpio-v2.o
endif
else
obj-y	+= gpio.o
endif
