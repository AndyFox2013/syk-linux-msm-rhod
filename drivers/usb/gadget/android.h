#include <linux/if_ether.h>

#ifndef __GADGET_ANDROID_H
#define __GADGET_ANDROID_H


struct android_usb_function {
	char *name;
	void *config;

	struct device *dev;
#ifdef CONFIG_USB_G_ANDROID_COMPAT
	struct device *compat_dev;
#endif
	char *dev_name;
	struct device_attribute **attributes;

	/* for android_dev.enabled_functions */
	struct list_head enabled_list;

	/* Optional: initialization during gadget bind */
	int (*init)(struct android_usb_function *, struct usb_composite_dev *);
	/* Optional: cleanup during gadget unbind */
	void (*cleanup)(struct android_usb_function *);
	/* Optional: called when the function is added the list of
	 *		enabled functions */
	void (*enable)(struct android_usb_function *);
	/* Optional: called when it is removed */
	void (*disable)(struct android_usb_function *);

	int (*bind_config)(struct android_usb_function *,
			   struct usb_configuration *);

	/* Optional: called when the configuration is removed */
	void (*unbind_config)(struct android_usb_function *,
			      struct usb_configuration *);
	/* Optional: handle ctrl requests before the device is configured */
	int (*ctrlrequest)(struct android_usb_function *,
					struct usb_composite_dev *,
					const struct usb_ctrlrequest *);
};

struct android_dev {
	struct android_usb_function **functions;
	struct list_head enabled_functions;
	struct usb_composite_dev *cdev;
	struct device *dev;

	bool enabled;
	int disable_depth;
	struct mutex mutex;
	bool connected;
	bool sw_connected;
	struct work_struct work;
};

#define MAX_ACM_INSTANCES 4
struct acm_function_config {
	int instances;
};

struct rndis_function_config {
	u8      ethaddr[ETH_ALEN];
	u32     vendorID;
	char	manufacturer[256];
	bool	wceis;
};

struct mass_storage_function_config {
	struct fsg_config fsg;
	struct fsg_common *common;
};

static struct class *android_class;
static struct android_dev *_android_dev;
static int android_bind_config(struct usb_configuration *c);
static void android_unbind_config(struct usb_configuration *c);
static int android_check_function_enabled(struct android_dev *dev, char *name);
static int android_enable_function(struct android_dev *dev, char *name);
static int android_disable_function(struct android_dev *dev, char *name);
static int android_enable(struct android_dev *dev);
static int android_disable(struct android_dev *dev);
static void android_device_enable(struct android_dev *dev);
static void android_device_disable(struct android_dev *dev);

#endif

