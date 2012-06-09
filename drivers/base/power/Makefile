obj-$(CONFIG_PM)	+= sysfs.o generic_ops.o qos.o
obj-$(CONFIG_PM_SLEEP)	+= main.o wakeup.o
obj-$(CONFIG_PM_RUNTIME)	+= runtime.o
obj-$(CONFIG_PM_TRACE_RTC)	+= trace.o
obj-$(CONFIG_PM_OPP)	+= opp.o
obj-$(CONFIG_PM_GENERIC_DOMAINS)	+=  domain.o domain_governor.o
ifndef CONFIG_MSM_AMSS_VERSION_WINCE
obj-$(CONFIG_HAVE_CLK)	+= clock_ops.o
obj-$(CONFIG_PM)	+= common.o
endif
ccflags-$(CONFIG_DEBUG_DRIVER) := -DDEBUG
