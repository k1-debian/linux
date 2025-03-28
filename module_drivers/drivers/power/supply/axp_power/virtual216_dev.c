#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
//#include <mach/irqs.h>
#include <linux/power_supply.h>
#include <linux/module.h>

#include "axp-cfg.h"
#include "axp-mfd.h"

static struct platform_device virt[] = {
	{
		.name = "reg-216-cs-rtc",
		.id = -1,
		.dev        = {
			.platform_data = "rtc_on",
		}
	}, {
		.name = "reg-216-cs-aldo1",
		.id = -1,
		.dev        = {
			.platform_data = "cam_on",
		}
	}, {
		.name = "reg-216-cs-aldo2",
		.id = -1,
		.dev        = {
			.platform_data = "codec_on",
		}
	}, {
		.name = "reg-216-cs-aldo3",
		.id = -1,
		.dev        = {
			.platform_data = "ledir_on",
		}
	},/*{
            .name = "reg-22-cs-dldo1",
            .id = -1,
            .dev        = {
                .platform_data = "axp22_dldo1",
            }
    },{
            .name = "reg-22-cs-dldo2",
            .id = -1,
            .dev        = {
                .platform_data = "axp22_dldo2",
            }
    },{
            .name = "reg-22-cs-dldo3",
            .id = -1,
            .dev        = {
                .platform_data = "axp22_dldo3",
            }
    },{
            .name = "reg-22-cs-dldo4",
            .id = -1,
            .dev        = {
                .platform_data = "axp22_dldo4",
            }
    },*/{
		.name = "reg-216-cs-eldo1",
		.id = -1,
		.dev        = {
			.platform_data = "dvp_on",
		}
	}, {
		.name = "reg-216-cs-eldo2",
		.id = -1,
		.dev        = {
			.platform_data = "wlreg_on",
		}
	},/*{
            .name = "reg-22-cs-eldo3",
            .id = -1,
            .dev        = {
                .platform_data = "axp22_eldo3",
            }
    },*/{
		.name = "reg-216-cs-dcdc1",
		.id = -1,
		.dev        = {
			.platform_data = "cpu_vddio_3v3",
		}
	}, {
		.name = "reg-216-cs-dcdc2",
		.id = -1,
		.dev        = {
			.platform_data = "cpu_vmem",
		}
	}, {
		.name = "reg-216-cs-dcdc3",
		.id = -1,
		.dev        = {
			.platform_data = "cpu_core",
		}
	}, {
		.name = "reg-216-cs-dcdc4",
		.id = -1,
		.dev        = {
			.platform_data = "dcdc4",
		}
	}, {
		.name = "reg-216-cs-dcdc5",
		.id = -1,
		.dev        = {
			.platform_data = "cpu_vddio_1v8",
		}
	},/*{
            .name = "reg-22-cs-gpio0ldo",
            .id = -1,
            .dev        = {
                .platform_data = "axp22_ldoio0",
            }
    },*/{
		.name = "reg-216-cs-gpio1ldo",
		.id = -1,
		.dev        = {
			.platform_data = "dmic_on",
		}
	},
};

static int __init virtual_init(void)
{
	int j, ret;
	for (j = 0; j < ARRAY_SIZE(virt); j++) {
		ret =  platform_device_register(&virt[j]);
		if (ret) {
			goto creat_devices_failed;
		}
	}
	return ret;

creat_devices_failed:
	while (j--) {
		platform_device_register(&virt[j]);
	}
	return ret;

}

module_init(virtual_init);

static void __exit virtual_exit(void)
{
	int j;
	for (j = ARRAY_SIZE(virt) - 1; j >= 0; j--) {
		platform_device_unregister(&virt[j]);
	}
}
module_exit(virtual_exit);

MODULE_DESCRIPTION("X-POWER axp regulator test");
MODULE_AUTHOR("Weijin Zhong Krosspower");
MODULE_LICENSE("GPL");
