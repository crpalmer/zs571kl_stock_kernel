#include "asus_fp_id.h"
#include <linux/kernel.h>
#include <linux/types.h>

int asus_match_hw_id(struct device_node *np, int id)
{
	int id_gpio;
	int status;
	int value;
	pr_info("%s: ++", __func__);

	id_gpio = of_get_named_gpio_flags(np, "asus,id-gpio",
				0, NULL);
	if (id_gpio < 0) {
		pr_err("Failed to parse id-gpio, did you setup one in dts?\n");
		return 0;
	}

	status = gpio_request_one(id_gpio, GPIOF_DIR_IN, "id_gpio");
	if (status < 0) {
		pr_err("Failed to request gpio %d.\n", id_gpio);
		return 0;
	}
	value = gpio_get_value(id_gpio);
	pr_info("asus id pin read %d\n", value);


	gpio_free(id_gpio);
	return value == id;
}
