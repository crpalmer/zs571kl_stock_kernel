/******************************************************************************

Copyright (c) 2016, Analogix Semiconductor, Inc.

PKG Ver  : V2.1.11

Filename : 

Project  : ANX7688 

Created  : 28 Nov. 2016

Devices  : ANX7688

Toolchain: Android
 
Description:

Revision History:

******************************************************************************/

/*
 * Copyright(c) 2014, Analogix Semiconductor. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include "anx7688_driver.h"
#include "anx7688_private_interface.h"
#include "anx7688_public_interface.h"
#include "eeprom.h"

/* Use device tree structure data when defined "CONFIG_OF"  */
/* #define CONFIG_OF */

#define ANX7688_DRV_VERSION "2.1.12"

#include <linux/power_supply.h>
#include <linux/usb/class-dual-role.h>
#include <linux/completion.h>

#ifdef PD_CHARGING_DRIVER_SUPPORT
struct workqueue_struct *pd_workqueue;
struct work_struct pdwork;
extern void pd_send_rdo_resutl(void);
struct completion rdo_completion;
#endif
static struct regulator *usbc_1p0;
static struct regulator *usbc_1p8;
static struct regulator *usbc_3p3;
static struct regulator *usbc_vcon;
static bool vcon_en;
struct power_supply *usb_psy = NULL;
struct power_supply *smb_psy = NULL;
static unsigned char fw_ver[2];
extern unsigned char downstream_pd_cap;
struct completion prswap_completion;
extern int asus_otg_boost_enable(int, bool);

static int create_sysfs_interfaces(struct device *dev);

/* to access global platform data */
static struct anx7688_platform_data *g_pdata;

#define DONGLE_CABLE_INSERT  1

atomic_t anx7688_power_status;

struct i2c_client *anx7688_client;

struct anx7688_platform_data {
    int gpio_p_on;
    int gpio_reset;
    int gpio_cbl_det;
#ifdef SUP_INT_VECTOR
    int gpio_intr_comm;
#endif
    int gpio_redrv;
    int gpio_3v3;
#ifdef SUP_VBUS_CTL
    int gpio_vbus_ctrl;
#endif
    spinlock_t lock;
};

struct anx7688_data {
    struct anx7688_platform_data *pdata;
    struct delayed_work work;
    struct workqueue_struct *workqueue;
    struct mutex lock;
    struct wake_lock anx7688_lock;
    struct dual_role_phy_desc *desc;
    struct dual_role_phy_instance *dual_role;
};

unsigned char device_addr = OCM_SLAVE_I2C_ADDR1;
unsigned char debug_on = 0;
unsigned char ocm_bootload_done;
#if AUTO_UPDATE_OCM_FW
unsigned char auto_update = 1; // auto update OCM FW
static unsigned short int OcmFwVersion = 0;
#endif

/* anx7688 power status, sync with interface and cable detection thread */

inline unsigned char ReadReg(unsigned char RegAddr)
{
    int ret = 0;

    anx7688_client->addr = (device_addr >> 1);
    ret = i2c_smbus_read_byte_data(anx7688_client, RegAddr);
    if (ret < 0) {
        pr_err("%s %s: failed to read i2c addr=%x\n", LOG_TAG,
               __func__, device_addr);
    }
    return (uint8_t) ret;

}

inline int ReadBlockReg(u8 RegAddr, u8 len, u8 *dat)
{
    int ret = 0;

    anx7688_client->addr = (device_addr >> 1);
    ret = i2c_smbus_read_i2c_block_data(anx7688_client, RegAddr, len, dat);
    if (ret < 0) {
        pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
               __func__, device_addr);
        return -EPERM;
    }

    return (int)ret;
}


inline int WriteBlockReg(u8 RegAddr, u8 len, const u8 *dat)
{
    int ret = 0;

    anx7688_client->addr = (device_addr >> 1);
    ret = i2c_smbus_write_i2c_block_data(anx7688_client, RegAddr, len, dat);
    if (ret < 0) {
        pr_err("%s %s: failed to read i2c block addr=%x\n", LOG_TAG,
               __func__, device_addr);
        return -EPERM;
    }

    return (int)ret;
}

inline void WriteReg(unsigned char RegAddr, unsigned char RegVal)
{
    int ret = 0;
    anx7688_client->addr = (device_addr >> 1);
    ret = i2c_smbus_write_byte_data(anx7688_client, RegAddr, RegVal);
    if (ret < 0) {
        pr_err("%s %s: failed to write i2c addr=%x\n", LOG_TAG,
               __func__, device_addr);
    }
}

static inline int enable_vconn(void)
{
	int retv;

	if (!usbc_vcon || vcon_en)
		return 0;

	retv = regulator_enable(usbc_vcon);
	if (!retv)
		vcon_en = true;
	return retv;
}

static inline int disable_vconn(void)
{
	int retv;

	if (!usbc_vcon || !vcon_en)
		return 0;

	retv = regulator_disable(usbc_vcon);
	if (!retv)
		vcon_en = false;
	return retv;
}

void anx7688_redriver_enable(int enable)
{
	struct anx7688_platform_data *pdata = g_pdata;

	gpio_set_value(pdata->gpio_redrv, enable);

}

void anx7688_set_power_supply(struct power_supply *psy)
{
	if (psy) {
		smb_psy = psy;
		pr_info("%s : Set power supply\n", LOG_TAG);
		return;

	}
	pr_err("%s : Set power supply failed\n", LOG_TAG);
}
EXPORT_SYMBOL(anx7688_set_power_supply);

void MI1_power_on(void)
{
#ifdef CONFIG_OF
    struct anx7688_platform_data *pdata = g_pdata;
#else
    struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif

    /*power on pin enable */
    gpio_set_value(pdata->gpio_p_on, 1);
    mdelay(10);
    /*power reset pin enable */
    gpio_set_value(pdata->gpio_reset, 1);
    mdelay(10);
    pr_info("%s %s: MI-1 power on !\n", LOG_TAG, __func__);
}

void anx7688_hardware_reset(int enable)
{
#ifdef CONFIG_OF
    struct anx7688_platform_data *pdata = g_pdata;
#else
    struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif
    gpio_set_value(pdata->gpio_reset, enable);
}


void anx7688_power_standby(void)
{
    int retv;
#ifdef CONFIG_OF
    struct anx7688_platform_data *pdata = g_pdata;
#else
    struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif
    anx7688_redriver_enable(0);
    mdelay(1);
    gpio_set_value(pdata->gpio_reset, 0);
    mdelay(1);
    gpio_set_value(pdata->gpio_p_on, 0);
    mdelay(1);

    retv = disable_vconn();
    if (retv)
	pr_err("error in disable the usbc vcon (%d)\n", retv);

    pr_info("%s %s: anx7688 power down\n", LOG_TAG, __func__);
}

#define EEPROM_LOAD_STA 0x12
#define EEPROM_LOAD_STA_CHK	(1<<0)
#define EEPROM_LOAD_STA_CHK_DP  0x3f

void anx7688_hardware_poweron(void)
{
#ifdef CONFIG_OF
    struct anx7688_platform_data *pdata = g_pdata;
#else
    struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif
    int retry_count, i;

    pr_info("%s %s: anx7688 power on\n", LOG_TAG, __func__);

    i = enable_vconn();
    mdelay(10);
    if (i) {
	   pr_err("error in enable the usbc vcon (%d)\n", i);
    }

    for (retry_count = 0; retry_count < 3; retry_count++) {
#ifdef OCM_DEBUG_PD
        pr_info("%s %s: anx7688 check ocm loading...\n", LOG_TAG, __func__);
#endif
        /*power on pin enable */
        gpio_set_value(pdata->gpio_p_on, 1);
        mdelay(10);

        /*power reset pin enable */
        gpio_set_value(pdata->gpio_reset, 1);
        mdelay(10);

        /* load delay T3 : eeprom 3.2s,  OTP 20ms*/
        for (i = 0; i < OCM_LOADING_TIME; i++) {
            /*Interface work? */
            if ((ReadReg(EEPROM_LOAD_STA)&EEPROM_LOAD_STA_CHK) == EEPROM_LOAD_STA_CHK) {
#ifdef OCM_DEBUG_PD
                pr_info("%s %s: interface initialization\n", LOG_TAG, __func__);
#endif

                chip_register_init();
                interface_init();
		send_initialized_setting();

#ifdef OCM_DEBUG
                pr_info("%s %s: chip is power on! firmware version is %02x%02x, Driver version:%s\n", LOG_TAG, __func__,ReadReg(0x15),ReadReg(0x16),ANX7688_DRV_VERSION);
#endif
		ocm_bootload_done = 1;
		fw_ver[0] = ReadReg(0x15);
		fw_ver[1] = ReadReg(0x16);
                return;
            }
            mdelay(1);
            printk(".");
        }
        anx7688_power_standby();
        mdelay(10);
    }

}

void anx7688_vbus_control(bool value)
{
#ifdef SUP_VBUS_CTL

#ifdef CONFIG_OF
    struct anx7688_platform_data *pdata = g_pdata;
#else
    struct anx7688_platform_data *pdata = anx7688_client->dev.platform_data;
#endif


    if(value)
        gpio_set_value(pdata->gpio_vbus_ctrl, 1);
    else
        gpio_set_value(pdata->gpio_vbus_ctrl, 0);

#endif

}

static void anx7688_free_gpio(struct anx7688_data *platform)
{
    gpio_free(platform->pdata->gpio_cbl_det);
    gpio_free(platform->pdata->gpio_reset);
    gpio_free(platform->pdata->gpio_p_on);
#ifdef SUP_INT_VECTOR
    gpio_free(platform->pdata->gpio_intr_comm);
#endif
#ifdef SUP_VBUS_CTL
    gpio_free(platform->pdata->gpio_vbus_ctrl);
#endif
    if (gpio_is_valid(platform->pdata->gpio_3v3))
	    gpio_free(platform->pdata->gpio_3v3);
}

static int anx7688_init_gpio(struct anx7688_data *platform)
{
    int ret = 0;

    pr_info("%s %s: anx7688 init gpio\n", LOG_TAG, __func__);
    /*  gpio for chip power down  */
    ret = gpio_request(platform->pdata->gpio_p_on, "anx7688_p_on_ctl");
    if (ret) {
        pr_err("%s : failed to request gpio %d\n", __func__,
               platform->pdata->gpio_p_on);
        goto err0;
    }
    gpio_direction_output(platform->pdata->gpio_p_on, 0);
    /*  gpio for chip reset  */
    ret = gpio_request(platform->pdata->gpio_reset, "anx7688_reset_n");
    if (ret) {
        pr_err("%s : failed to request gpio %d\n", __func__,
               platform->pdata->gpio_reset);
        goto err1;
    }
    gpio_direction_output(platform->pdata->gpio_reset, 0);

    /*  gpio for cable detect  */
    ret = gpio_request(platform->pdata->gpio_cbl_det, "anx7688_cbl_det");
    if (ret) {
        pr_err("%s : failed to request gpio %d\n", __func__,
               platform->pdata->gpio_cbl_det);
        goto err2;
    }
    gpio_direction_input(platform->pdata->gpio_cbl_det);

#ifdef SUP_INT_VECTOR
    /*  gpio for chip interface communaction */
    ret = gpio_request(platform->pdata->gpio_intr_comm, "anx7688_intr_comm");
    if (ret) {
        pr_err("%s : failed to request gpio %d\n", __func__,
               platform->pdata->gpio_intr_comm);
        goto err3;
    }
    gpio_direction_input(platform->pdata->gpio_intr_comm);
#endif
    if (gpio_is_valid(platform->pdata->gpio_redrv)) {
    /*  gpio for DP redriver */
    	ret = gpio_request(platform->pdata->gpio_redrv, "anx7688_redrv");
    	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
			platform->pdata->gpio_redrv);
		goto err3;
   	}
   	gpio_direction_output(platform->pdata->gpio_redrv, 0);
   }
#ifdef PD_CHARGING_DRIVER_SUPPORT
    init_completion(&rdo_completion);
#endif
    init_completion(&prswap_completion);
#ifdef SUP_VBUS_CTL
    /*  gpio for vbus control  */
    ret = gpio_request(platform->pdata->gpio_vbus_ctrl, "anx7688_vbus_ctrl");
    if (ret) {
        pr_err("%s : failed to request gpio %d\n", __func__,
               platform->pdata->gpio_vbus_ctrl);
        goto err4;
    }
    gpio_direction_output(platform->pdata->gpio_vbus_ctrl, 0);
#endif

    goto out;

#ifdef SUP_VBUS_CTL
err4:
    gpio_free(platform->pdata->gpio_vbus_ctrl);
#endif
#ifdef SUP_INT_VECTOR
err3:
    gpio_free(platform->pdata->gpio_intr_comm);
#endif
err2:
    gpio_free(platform->pdata->gpio_cbl_det);
err1:
    gpio_free(platform->pdata->gpio_reset);
err0:
    gpio_free(platform->pdata->gpio_p_on);

    return 1;
out:
    return 0;
}

void anx7688_main_process(void)
{
    /* do main loop, do what you want to do */
#if AUTO_UPDATE_OCM_FW
    if(auto_update) {
        auto_update = 0;
        OcmFwVersion = burnhexauto();
    }
#endif
}
#if AUTO_UPDATE_OCM_FW
#define MAIN_PROC_RELOAD 0
static void anx7688_work_func(struct work_struct *work)
{
    struct anx7688_data *td = container_of(work, struct anx7688_data,
                                           work.work);
#if MAIN_PROC_RELOAD
    int workqueu_timer = 0;
    workqueu_timer = 1000;
#endif
    mutex_lock(&td->lock);
    anx7688_main_process();
    mutex_unlock(&td->lock);
#if MAIN_PROC_RELOAD
    queue_delayed_work(td->workqueue, &td->work,
                       msecs_to_jiffies(workqueu_timer));
#endif
}
#endif

#ifdef PD_CHARGING_DRIVER_SUPPORT
static void anx7688_pd_work_func(struct work_struct *work)
{
	u32 timeout;
	pr_info("wait for PD request response\n");
	timeout =wait_for_completion_timeout(&rdo_completion, msecs_to_jiffies(500));
	if (!timeout) {
		pr_err("PD response time out\n");
		return;
	}
	pd_send_rdo_resutl();

}
#endif

#ifdef CABLE_DET_PIN_HAS_GLITCH
static unsigned char confirmed_cable_det(void *data)
{
    struct anx7688_data *platform = data;
    unsigned int count = 9;
    unsigned int cable_det_count = 0;
    u8 val = 0;

    do {
        val = gpio_get_value(platform->pdata->gpio_cbl_det);
        if (DONGLE_CABLE_INSERT == val)
            cable_det_count++;
        mdelay(1);;
    } while (count--);

    if (cable_det_count > 7)
        return 1;
    else if (cable_det_count < 3)
        return 0;
    else
        return atomic_read(&anx7688_power_status);
}
#endif

static irqreturn_t anx7688_cbl_det_isr(int irq, void *data)
{
    struct anx7688_data *platform = data;
    int cable_connected = 0;

    if (debug_on) return IRQ_NONE;

#ifdef CABLE_DET_PIN_HAS_GLITCH
    cable_connected = confirmed_cable_det((void *)platform);
#else
    cable_connected = gpio_get_value(platform->pdata->gpio_cbl_det);
#endif

#ifdef OCM_DEBUG
    pr_info("%s %s : cable plug pin status %d\n", LOG_TAG, __func__, cable_connected);
#endif

    if (cable_connected == DONGLE_CABLE_INSERT) {
        if (atomic_read(&anx7688_power_status) == 1) {
#ifdef CABLE_DET_PIN_HAS_GLITCH
            mdelay(2);
            //anx7688_power_standby();
            return IRQ_HANDLED;
#else
            return IRQ_HANDLED;
#endif
        }
        atomic_set(&anx7688_power_status, 1);
	ocm_bootload_done = 0;
	pm_stay_awake(&anx7688_client->dev);
        anx7688_hardware_poweron();
	pm_relax(&anx7688_client->dev);

    } else {
        atomic_set(&anx7688_power_status, 0);
	anx7688_power_standby();
#ifdef SUP_VBUS_CTL
        gpio_set_value(platform->pdata->gpio_vbus_ctrl, 0);
#endif
	if (usb_psy)
		power_supply_set_usb_otg(usb_psy, 0);

	asus_otg_boost_enable(0, NULL);

    }
#ifdef CONFIG_DUAL_ROLE_USB_INTF
    if (platform->dual_role)
	    dual_role_instance_changed(platform->dual_role);
#endif
    return IRQ_HANDLED;
}

#ifdef SUP_INT_VECTOR
static irqreturn_t anx7688_intr_comm_isr(int irq, void *data)
{
    unsigned char c;

    if (unlikely(atomic_read(&anx7688_power_status) != 1))
        return IRQ_NONE;

    if (unlikely(ocm_bootload_done != 1))
        return IRQ_NONE;

    pm_stay_awake(&anx7688_client->dev);
    device_addr= OCM_SLAVE_I2C_ADDR2;
    //clear interrupt
    c=ReadReg(0x10);
    if (c!=0) WriteReg(0x10, c);
    device_addr= OCM_SLAVE_I2C_ADDR1;


    if (is_soft_reset_intr()) {
#ifdef OCM_DEBUG_PD
        pr_info("%s %s : ======I=====\n", LOG_TAG, __func__);
#endif

        handle_intr_vector();
    }
    pm_relax(&anx7688_client->dev);
    return IRQ_HANDLED;
}
#endif

#ifdef CONFIG_OF
static int anx7688_parse_dt(struct device *dev, struct anx7688_platform_data *pdata)
{
    struct device_node *np = dev->of_node;

    pdata->gpio_p_on =
        of_get_named_gpio_flags(np, "analogix,p-on-gpio", 0, NULL);

    pdata->gpio_reset =
        of_get_named_gpio_flags(np, "analogix,reset-gpio", 0, NULL);

    pdata->gpio_cbl_det =
        of_get_named_gpio_flags(np, "analogix,cbl-det-gpio", 0, NULL);

#ifdef SUP_VBUS_CTL
    pdata->gpio_vbus_ctrl =
        of_get_named_gpio_flags(np, "analogix,v33-ctrl-gpio", 0, NULL); /*reuse previous unless gpio(v33_ctrl) for vbus control*/
//	    of_get_named_gpio_flags(np, "analogix,vbus-ctrl-gpio", 0, NULL);
#endif
#ifdef SUP_INT_VECTOR
    pdata->gpio_intr_comm =
        of_get_named_gpio_flags(np, "analogix,intr-comm-gpio", 0, NULL);
#endif
    pdata->gpio_redrv = of_get_named_gpio_flags(np, "analogix,redriver-gpio", 0, NULL);

    pdata->gpio_3v3 = of_get_named_gpio_flags(np, "vcc3v3-gpio",0, NULL);

    pr_info("%s gpio p_on : %d, reset : %d,  gpio_cbl_det %d, redrv %d\n",
            LOG_TAG, pdata->gpio_p_on,
            pdata->gpio_reset, pdata->gpio_cbl_det, pdata->gpio_redrv);

    return 0;
}
#else
static int anx7688_parse_dt(struct device *dev, struct anx7688_platform_data *pdata)
{
    return -ENODEV;
}
#endif

#ifdef CONFIG_DUAL_ROLE_USB_INTF
static enum dual_role_property anx7688_drp_properties[] = {
	DUAL_ROLE_PROP_MODE,
	DUAL_ROLE_PROP_PR,
	DUAL_ROLE_PROP_DR,
};

static int dual_role_is_writeable(struct dual_role_phy_instance *drp,
				  enum dual_role_property prop)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return 1;
	else
		return 0;
}

static int dual_role_get_local_prop(struct dual_role_phy_instance *dual_role,
				    enum dual_role_property prop,
				    unsigned int *val)
{
	struct anx7688_data *anx7688 = dual_role_get_drvdata(dual_role);
	int mode = 0;

	if (!anx7688)
		return -EINVAL;

	if (atomic_read(&anx7688_power_status) == 0)
		mode = -1;
	else
		mode = get_power_role();

	if (mode == 1) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_DFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SRC;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_HOST;
		else
			return -EINVAL;
	} else if (mode == 0) {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_UFP;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_SNK;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_DEVICE;
		else
			return -EINVAL;
	} else {
		if (prop == DUAL_ROLE_PROP_MODE)
			*val = DUAL_ROLE_PROP_MODE_NONE;
		else if (prop == DUAL_ROLE_PROP_PR)
			*val = DUAL_ROLE_PROP_PR_NONE;
		else if (prop == DUAL_ROLE_PROP_DR)
			*val = DUAL_ROLE_PROP_DR_NONE;
		else
			return -EINVAL;
	}

	return 0;
}

static int dual_role_set_mode_prop(struct dual_role_phy_instance *dual_role,
				   enum dual_role_property prop,
				   const unsigned int *val)
{
	struct anx7688_data *anx7688 = dual_role_get_drvdata(dual_role);
	int ret = 0;
	int mode = 0;
	u32 timeout;

	if (!anx7688)
		return -EINVAL;

	if (*val != DUAL_ROLE_PROP_MODE_DFP && *val != DUAL_ROLE_PROP_MODE_UFP)
		return -EINVAL;

	if (atomic_read(&anx7688_power_status) == 0)
		return 0;

	mode = get_power_role();

	if (*val == DUAL_ROLE_PROP_MODE_DFP && mode == 1)
		return 0;

	if (*val == DUAL_ROLE_PROP_MODE_UFP && mode == 0)
		return 0;

	printk("%s: start %s PD command\n", __func__, downstream_pd_cap ? "with":"without");

	if (mode == 0) {
		pr_err("%s: try reversing, form Sink to Source\n", __func__);
		ret = try_source();
		if (!ret) {
			if(!downstream_pd_cap)
				pr_err("success power role is %s\n", get_power_role() ? "Source" : "Sink");
			else {
				reinit_completion(&prswap_completion);
				timeout =wait_for_completion_timeout(&prswap_completion, msecs_to_jiffies(1000));
				if (!timeout)
					pr_warn("power swap timeout\n");
				msleep(1000);
				if(get_data_role() == 0) {
					pr_info("data role is ufp\n");
					interface_dr_swap();
				}
			}
		} else {
			pr_err("failed power role is %s\n", get_power_role() ? "Source" : "Sink");
			ret = -EIO;
		}
	} else if (mode == 1) {
		pr_err("%s: try reversing, form Source to Sink\n", __func__);
		ret = try_sink();
		if (!ret) {
			if(!downstream_pd_cap)
				pr_err("success power role is %s\n", get_power_role() ? "Source" : "Sink");
			else {
				reinit_completion(&prswap_completion);
				timeout =wait_for_completion_timeout(&prswap_completion, msecs_to_jiffies(1000));
				if (!timeout)
					pr_warn("power swap timeout\n");
				msleep(1000);
				if(get_data_role() == 1) {
					pr_info("data role is dfp\n");
					interface_dr_swap();
				}

			}
		} else {
			if(!downstream_pd_cap) {
				asus_otg_boost_enable(1, NULL);
				pr_err("failed power role is %s\n", get_power_role() ? "Source" : "Sink");
				ret = -EIO;
			}
		}
	} else {
		pr_err("%s: get role failed\n", __func__);
		ret = -EIO;
	}

	printk("%s: end ret = %d\n", __func__, ret);

	return ret;
}

static int dual_role_set_prop(struct dual_role_phy_instance *dual_role,
			      enum dual_role_property prop,
			      const unsigned int *val)
{
	if (prop == DUAL_ROLE_PROP_MODE)
		return dual_role_set_mode_prop(dual_role, prop, val);
	else
		return -EINVAL;
}

#endif

static int anx7688_i2c_probe(struct i2c_client *client,
                             const struct i2c_device_id *id)
{

    struct anx7688_data *platform;
    struct anx7688_platform_data *pdata;
#ifdef CONFIG_DUAL_ROLE_USB_INTF
    struct dual_role_phy_desc *desc;
    struct dual_role_phy_instance *dual_role;
#endif
    int ret = 0;
    int cbl_det_irq = 0;

    if (asus_otg_boost_enable(0, NULL)) {
	    ret = -EPROBE_DEFER;
	    goto exit;
    }

    if (!i2c_check_functionality(client->adapter,
                                 I2C_FUNC_SMBUS_I2C_BLOCK)) {
        pr_err("%s:anx7688's i2c bus doesn't support\n", __func__);
        ret = -ENODEV;
        goto exit;
    }

    platform = kzalloc(sizeof(struct anx7688_data), GFP_KERNEL);
    if (!platform) {
        pr_err("%s: failed to allocate driver data\n", __func__);
        ret = -ENOMEM;
        goto exit;
    }

    if (client->dev.of_node) {
        pdata = devm_kzalloc(&client->dev,
                             sizeof(struct anx7688_platform_data),
                             GFP_KERNEL);
        if (!pdata) {
            pr_err("%s: Failed to allocate memory\n", __func__);
            return -ENOMEM;
        }

        client->dev.platform_data = pdata;

        /* device tree parsing function call */
        ret = anx7688_parse_dt(&client->dev, pdata);
        if (ret != 0)	/* if occurs error */
            goto err0;

        platform->pdata = pdata;
    } else {
        platform->pdata = client->dev.platform_data;
    }

    /* to access global platform data */
    g_pdata = platform->pdata;
    anx7688_client = client;
    anx7688_client->addr = (device_addr >> 1);

    atomic_set(&anx7688_power_status, 0);

    mutex_init(&platform->lock);

    if (!platform->pdata) {
        ret = -EINVAL;
        goto err0;
    }
    /*open l11*/
    usbc_1p0 = devm_regulator_get(&client->dev, "USBC_1p0");
    if (IS_ERR(usbc_1p0)) {
		pr_err("get usbc 1p0 fail %ld\n", PTR_ERR(usbc_1p0));
		goto err0;
    }

    ret = regulator_set_voltage(usbc_1p0, 1000000, 1000000);
    if (ret) {
	    pr_err("set voltage level for usbc 1p0 fail\n");
	    regulator_set_voltage(usbc_1p0, 0, 1000000);
	    goto err0;
    }

    //l11 load current 235mA
    ret = regulator_set_optimum_mode(usbc_1p0, 235000);
    if (ret < 0) {
	    pr_err("%s: Unable to set HPM of the regulator "
			    "usbc_1p0\n", __func__);
		return ret;
    }

    /*SR  open l12
     *SR2 open l6 */
    usbc_1p8 = devm_regulator_get(&client->dev, "USBC_1p8");
    if (IS_ERR(usbc_1p8)) {
		pr_err("get usbc 1p8 fail %ld\n", PTR_ERR(usbc_1p8));
		goto err0;
    }

    ret = regulator_set_voltage(usbc_1p8, 1800000, 1800000);
    if (ret) {
	    pr_err("set voltage level for usbc 1p8 fail\n");
	    regulator_set_voltage(usbc_1p8, 0, 1800000);
	    goto err0;
    }

    //SR l12 load current 135mA
    //SR2 l6 load current 30mA
    ret = regulator_set_optimum_mode(usbc_1p8, 135000);
    if (ret < 0) {
	    pr_err("%s: Unable to set HPM of the regulator "
			    "usbc_1p8\n", __func__);
		return ret;
    }

    /*open l17*/
    usbc_3p3 = devm_regulator_get(&client->dev, "USBC_3p3");
    if (IS_ERR(usbc_3p3)) {
		pr_err("get usbc 3p3 fail %ld\n", PTR_ERR(usbc_3p3));
		goto err0;
    }

    ret = regulator_set_voltage(usbc_3p3, 3300000, 3300000);
    if (ret) {
	    pr_err("set voltage level for usbc 3p3 fail\n");
	    regulator_set_voltage(usbc_3p3, 0, 3300000);
	    goto err0;
    }

    //l17 load curretn 85mA
    ret = regulator_set_optimum_mode(usbc_3p3, 85000);
    if (ret < 0) {
	    pr_err("%s: Unable to set HPM of the regulator "
			    "usbc_3p3\n", __func__);
		return ret;
    }

    usbc_vcon = devm_regulator_get(&client->dev, "USBC_vcon");
    if (IS_ERR(usbc_vcon)) {
		pr_err("get usbc vcon fail %ld\n", PTR_ERR(usbc_vcon));
		goto err0;
    }

    ret = anx7688_init_gpio(platform);
    if (ret) {
        pr_err("%s: failed to initialize gpio\n", __func__);
        goto err0;
    }

    /*when probe anx7688 device, enter standy mode */
    anx7688_power_standby();
    device_init_wakeup(&client->dev, true);

#if AUTO_UPDATE_OCM_FW
    INIT_DELAYED_WORK(&platform->work, anx7688_work_func);
#endif

    platform->workqueue = create_singlethread_workqueue("anx7688_work");
    if (platform->workqueue == NULL) {
        pr_err("%s: failed to create work queue\n", __func__);
        ret = -ENOMEM;
        goto err1;
    }

#ifdef PD_CHARGING_DRIVER_SUPPORT
    INIT_WORK(&pdwork, anx7688_pd_work_func);
    pd_workqueue = create_singlethread_workqueue("pd_work");
    if (pd_workqueue == NULL) {
        pr_err("%s: failed to create work queue\n", __func__);
        ret = -ENOMEM;
        goto err1;
    }
#endif

    cbl_det_irq = gpio_to_irq(platform->pdata->gpio_cbl_det);
    if (cbl_det_irq < 0) {
        pr_err("%s : failed to get gpio irq\n", __func__);
        goto err1;
    }

    wake_lock_init(&platform->anx7688_lock, WAKE_LOCK_SUSPEND, "anx7688_wake_lock");

    ret = request_threaded_irq(cbl_det_irq, NULL, anx7688_cbl_det_isr,
                               IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING
                               | IRQF_ONESHOT, "anx7688-cbl-det", platform);
    if (ret < 0) {
        pr_err("%s : failed to request irq\n", __func__);
        goto err3;
    }

    ret = irq_set_irq_wake(cbl_det_irq, 1);
    if (ret < 0) {
        pr_err("%s : Request irq for cable detect", __func__);
        pr_err("interrupt wake set fail\n");
        goto err4;
    }

    if (gpio_is_valid(platform->pdata->gpio_3v3)) {
	ret = gpio_request_one(platform->pdata->gpio_3v3, GPIOF_OUT_INIT_HIGH,"FP_3V3_EN");
	if (ret) {
		pr_err("%s : failed to request gpio %d\n", __func__,
			platform->pdata->gpio_3v3);
		gpio_free(platform->pdata->gpio_3v3);
		goto err4;
	}
    }

    ret = enable_irq_wake(cbl_det_irq);
    if (ret < 0) {
        pr_err("%s : Enable irq for cable detect", __func__);
        pr_err("interrupt wake enable fail\n");
        goto err4;
    }
#ifdef SUP_INT_VECTOR
    client->irq = gpio_to_irq(platform->pdata->gpio_intr_comm);
    if (client->irq < 0) {
        pr_err("%s : failed to get anx7688 gpio comm irq\n", __func__);
        goto err3;
    }

    ret = request_threaded_irq(client->irq, NULL, anx7688_intr_comm_isr,
                               IRQF_TRIGGER_FALLING  | IRQF_ONESHOT, "anx7688-intr-comm", platform);

    if (ret < 0) {
        pr_err("%s : failed to request interface irq\n", __func__);
        goto err4;
    }

    ret = irq_set_irq_wake(client->irq, 1);
    if (ret < 0) {
        pr_err("%s : Request irq for interface communaction", __func__);
        goto err4;
    }

    ret = enable_irq_wake(client->irq);
    if (ret < 0) {
        pr_err("%s : Enable irq for interface communaction", __func__);
        goto err4;
    }

    ret = regulator_enable(usbc_1p8);
    if (ret) {
	pr_err("enable the usbc 1p8 fail\n");
	goto err4;
    }

    ret = regulator_enable(usbc_3p3);
    if (ret) {
	pr_err("enable the usbc 3p3 fail\n");
	goto err4;
    }

    ret = regulator_enable(usbc_1p0);
    if (ret) {
	pr_err("enable the usbc 1p0 fail\n");
	goto err4;
    }

#endif
    ret = create_sysfs_interfaces(&client->dev);
    if (ret < 0) {
        pr_err("%s : sysfs register failed", __func__);
        goto err4;
    }

    usb_psy = power_supply_get_by_name("usb");
    if (!usb_psy) {
		pr_err("%s : USB supply not found\n",LOG_TAG);
		goto err4;
    }
#if AUTO_UPDATE_OCM_FW
    /*add work function*/
    queue_delayed_work(platform->workqueue, &platform->work, msecs_to_jiffies(4000));
#endif

#ifdef CONFIG_DUAL_ROLE_USB_INTF
    desc = devm_kzalloc(&client->dev, sizeof(struct dual_role_phy_desc),GFP_KERNEL);
    if (!desc) {
	pr_err("%s : alloctate dul_role_phy_desc failed", __func__);
	goto err4;
    }

    desc->name = "otg_default";
    desc->supported_modes = DUAL_ROLE_SUPPORTED_MODES_DFP_AND_UFP;
    desc->set_property = dual_role_set_prop;
    desc->get_property = dual_role_get_local_prop;
    desc->num_properties = ARRAY_SIZE(anx7688_drp_properties);
    desc->properties = anx7688_drp_properties;
    desc->property_is_writeable = dual_role_is_writeable;
    dual_role = devm_dual_role_instance_register(&client->dev, desc);
    dual_role->drv_data = platform;
    platform->dual_role = dual_role;
    platform->desc = desc;
#endif

    pr_info("anx7688_i2c_probe successfully %s %s end\n", LOG_TAG, __func__);
    goto exit;

err4:
    free_irq(client->irq, platform);
err3:
    free_irq(cbl_det_irq, platform);
err1:
    anx7688_free_gpio(platform);
    destroy_workqueue(platform->workqueue);
#ifdef PD_CHARGING_DRIVER_SUPPORT
    destroy_workqueue(pd_workqueue);
#endif
err0:
    anx7688_client = NULL;
    kfree(platform);
exit:
    return ret;
}

static void anx7688_shutdown(struct i2c_client *client)
{
	pr_info("anx7688 shutdown\n");
	disable_irq(client->irq);
	asus_otg_boost_enable(0,NULL);
}

static int anx7688_i2c_remove(struct i2c_client *client)
{
    struct anx7688_data *platform = i2c_get_clientdata(client);
    printk("anx7688_i2c_remove\n");
    free_irq(client->irq, platform);
    anx7688_free_gpio(platform);
    destroy_workqueue(platform->workqueue);
#ifdef PD_CHARGING_DRIVER_SUPPORT
    destroy_workqueue(pd_workqueue);
#endif
    wake_lock_destroy(&platform->anx7688_lock);
    kfree(platform);
    return 0;
}

static int anx7688_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
    return 0;
}

static int anx7688_i2c_resume(struct i2c_client *client)
{
    return 0;
}

static const struct i2c_device_id anx7688_id[] = {
    {"anx7688", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, anx7688_id);

#ifdef CONFIG_OF
static struct of_device_id anx_match_table[] = {
    {.compatible = "analogix,anx7688",},
    {},
};
#endif

static struct i2c_driver anx7688_driver = {
    .driver = {
        .name = "anx7688",
        .owner = THIS_MODULE,
#ifdef CONFIG_OF
        .of_match_table = anx_match_table,
#endif
    },
    .probe = anx7688_i2c_probe,
    .remove = anx7688_i2c_remove,
    .suspend = anx7688_i2c_suspend,
    .resume = anx7688_i2c_resume,
    .shutdown = anx7688_shutdown,
    .id_table = anx7688_id,
};

static void __init anx7688_init_async(void *data, async_cookie_t cookie)
{
    int ret = 0;

    ret = i2c_add_driver(&anx7688_driver);
    if (ret < 0)
        pr_err("%s: failed to register anx7688 i2c drivern", __func__);
}

static int __init anx7688_init(void)
{
    async_schedule(anx7688_init_async, NULL);
    return 0;
}

static void __exit anx7688_exit(void)
{
    i2c_del_driver(&anx7688_driver);
}

#ifdef OCM_DEBUG
void dump_reg(void)
{
    int i = 0;
    u8 val = 0;

    printk("dump registerad:\n");
    printk("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    for (i = 0; i < 256; i++) {
        val = ReadReg(i);

        if ((i) % 0x10 == 0x00)
            printk("\n[%x]:%02x ", i, val);
        else
            printk("%02x ", val);

    }
    printk("\n");
}

ssize_t anx7688_send_pd_cmd(struct device *dev,
                            struct device_attribute *attr,
                            const char *buf, size_t count)
{
    int cmd;
    int result;

    result = sscanf(buf, "%d", &cmd);
    switch (cmd) {
    case TYPE_PWR_SRC_CAP:
        send_pd_msg(TYPE_PWR_SRC_CAP, 0, 0);
        break;

    case TYPE_DP_SNK_IDENTITY:
        send_pd_msg(TYPE_DP_SNK_IDENTITY, 0, 0);
        break;

    case TYPE_PSWAP_REQ:
        send_pd_msg(TYPE_PSWAP_REQ, 0, 0);
        break;
    case TYPE_DSWAP_REQ:
        send_pd_msg(TYPE_DSWAP_REQ, 0, 0);
        break;

    case TYPE_GOTO_MIN_REQ:
        send_pd_msg(TYPE_GOTO_MIN_REQ, 0, 0);
        break;

    case TYPE_PWR_OBJ_REQ:
        interface_send_request();
        break;
    case TYPE_ACCEPT:
        interface_send_accept();
        break;
    case TYPE_REJECT:
        interface_send_reject();
        break;
    case TYPE_SOFT_RST:
        send_pd_msg(TYPE_SOFT_RST, 0, 0);
        break;
    case TYPE_HARD_RST:
        send_pd_msg(TYPE_HARD_RST, 0, 0);
        break;

    case 0xFD:
        pr_info("fetch powerrole: %d\n", get_power_role());
        break;
    case 0xFE:
        pr_info("fetch datarole: %d\n", get_data_role());
        break;

    case 0xff:
        dump_reg();
        break;
    }
    return count;
}


ssize_t anx7688_send_pswap(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    return snprintf(buf, sizeof(u8), "%d\n", send_power_swap());
}

ssize_t anx7688_send_dswap(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    return snprintf(buf, sizeof(u8), "%d\n", send_data_swap());
}

ssize_t anx7688_try_source(struct device *dev,
                           struct device_attribute *attr, char *buf)
{
    return snprintf(buf, sizeof(u8), "%d\n", try_source());
}

ssize_t anx7688_try_sink(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    return snprintf(buf, sizeof(u8), "%d\n", try_sink());
}

ssize_t anx7688_get_data_role(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    return snprintf(buf, sizeof(u8), "%d\n", get_data_role());
}

ssize_t anx7688_get_power_role(struct device *dev,
                               struct device_attribute *attr, char *buf)
{
    return snprintf(buf, sizeof(u8), "%d\n", get_power_role());
}

ssize_t anx7688_rd_reg(struct device *dev,
                       struct device_attribute *attr,
                       const char *buf, size_t count)
{
    int cmd;
    int result;

    result = sscanf(buf, "%x", &cmd);
    printk("reg[%x] = %x\n", cmd, ReadReg(cmd));

    return count;

}

ssize_t anx7688_wr_reg(struct device *dev,
                       struct device_attribute *attr,
                       const char *buf, size_t count)
{
    int cmd, val;
    int result;

    result = sscanf(buf, "%x  %x", &cmd, &val);
    pr_info("c %x val %x\n", cmd, val);
    WriteReg(cmd, val);
    pr_info("reg[%x] = %x\n", cmd, ReadReg(cmd));
    return count;
}

ssize_t anx7688_atd_test(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result;

	if ((ReadReg(EEPROM_LOAD_STA)&EEPROM_LOAD_STA_CHK_DP) == EEPROM_LOAD_STA_CHK_DP)
		result = 1;
	else
		result = 0;

	return snprintf(buf, PAGE_SIZE, "%d\n", result);
}

ssize_t anx7688_atd_cc_side(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result = 0;
	unsigned char ana_ctrl;

	ana_ctrl = ReadReg(0x48);

	pr_info("ATD CC side reg[48] = %x\n", ana_ctrl);

	if((ana_ctrl & 0x0c) == 0x04)
		result = 1;//cc1 rd connect
	else if((ana_ctrl & 0x03) == 0x01)
		result = 2;//cc2 rd connect

	return snprintf(buf, PAGE_SIZE, "%d\n", result);
}

ssize_t anx7688_atd_cc1_test(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result = 0;
	int power_ctrl;
	int ana_status;

	power_ctrl = ReadReg(0x0d);
	ana_status = ReadReg(0x40);

	pr_info("ATD CC1 reg[0d] = %x , reg[40] = %x\n", power_ctrl, ana_status);

	if((ana_status & 0x08) && (power_ctrl & 0x80))
		result = 1;

	return snprintf(buf, PAGE_SIZE, "%d\n", result);
}


ssize_t anx7688_atd_cc2_test(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	int result = 0;
	int power_ctrl;
	int ana_status;

	power_ctrl = ReadReg(0x0d);
	ana_status = ReadReg(0x40);

	pr_info("ATD CC2 reg[0d] = %x , reg[40] = %x\n", power_ctrl, ana_status);

	if((ana_status & 0x08) && (power_ctrl & 0x10))
		result = 1;

	return snprintf(buf, PAGE_SIZE, "%d\n", result);
}

ssize_t anx7688_dump_register(struct device *dev,
                              struct device_attribute *attr, char *buf)
{
    int i = 0;
    for (i = 0; i < 256; i++) {
        if (i % 0x10 == 0)
            pr_info("\n");
        printk(" %.2x", ReadReg(i));

        snprintf(&buf[i], sizeof(u8), "%d", ReadReg(i));
    }

    printk("\n");

    return i;
}

ssize_t anx7688_select_rdo_index(struct device *dev,
                                 struct device_attribute *attr,
                                 const char *buf, size_t count)
{
    int cmd;
    cmd = sscanf(buf, "%d", &cmd);
    if (cmd <= 0)
        return 0;

    pr_info("NewRDO idx %d, Old idx %d\n", cmd, sel_voltage_pdo_index);
    sel_voltage_pdo_index = cmd;
    return count;
}

ssize_t anx7688_chg_addr(struct device *dev,
                         struct device_attribute *attr,
                         const  char *buf, size_t count)
{
    int val;
    int result;

    result = sscanf(buf, "%x", &val);
    device_addr = (unsigned char) val;
    pr_info( "Change Device Address to 0x%x\n", (int)device_addr);
    return count;
}



ssize_t anx7688_burn_hex(struct device *dev,
                         struct device_attribute *attr,
                         const char *buf, size_t count)
{
    burnhex();

    return 256+32;
}

ssize_t anx7688_read_hex(struct device *dev,
                         struct device_attribute *attr, char *buf)
{
    printk( "readhex() \n");
    readhex();
    printk("\n");

    return 256+32;
}

ssize_t anx7688_debug(struct device *dev,
                      struct device_attribute *attr,
                      const char *buf, size_t count)
{
    int param[4];
    int result,i;
    char CommandName[10];
    extern unsigned char debug_on;

    memset(param,0,sizeof(param));
    result = sscanf(buf, "%s %d %d %d %d",CommandName, param, param+1, param+2, param+3);
    //printk("anx7688 count: %d\n", (int)count);
    //printk("anx7688 buf: %s", buf);
    //printk("anx7688 param no: %d\n", result);
    printk("anx7688 cmd[%s", CommandName);
    for(i=0; i<result-1; i++)
        printk(" %d", param[i]);
    printk("]\n");

    if(strcmp(CommandName, "poweron") == 0) {
        printk("MI1_power_on\n");
        MI1_power_on();
    } else if(strcmp(CommandName, "powerdown") == 0) {
        anx7688_power_standby();
    } else if(strcmp(CommandName, "debugon") == 0) {
        debug_on = 1;
        printk("debug_on = %d\n",debug_on);
    } else if(strcmp(CommandName, "debugoff") == 0) {
        debug_on = 0;
        printk("debug_on = %d\n",debug_on);
    } else if(strcmp(CommandName, "burntest") == 0) {
        extern void burntest(unsigned short int select);
        burntest(2);
    } else {
        printk("Usage:\n");
        printk("  echo poweron > cmd       : power on\n");
        printk("  echo powerdown > cmd     : power off\n");
        printk("  echo debugon > cmd       : debug on\n");
        printk("  echo debugoff > cmd      : debug off\n");
        printk("  echo burntest > cmd      : write test fw\n");
    }

    return count;
}

/* for debugging */
static struct device_attribute anx7688_device_attrs[] = {
    __ATTR(pdcmd, S_IWUSR, NULL,
    anx7688_send_pd_cmd),
    __ATTR(rdreg, S_IWUSR, NULL,
    anx7688_rd_reg),
    __ATTR(wrreg, S_IWUSR, NULL,
    anx7688_wr_reg),
    __ATTR(addr, S_IWUSR, NULL,
    anx7688_chg_addr),
    __ATTR(rdoidx, S_IWUSR, NULL,
    anx7688_select_rdo_index),
    __ATTR(atd, S_IRUGO , anx7688_atd_test,
    NULL),
        __ATTR(atd_cc_side, S_IRUGO , anx7688_atd_cc_side,
    NULL),
	__ATTR(atd_cc1, S_IRUGO , anx7688_atd_cc1_test,
    NULL),
        __ATTR(atd_cc2, S_IRUGO , anx7688_atd_cc2_test,
    NULL),
    __ATTR(dumpreg, S_IRUGO , anx7688_dump_register,
    NULL),
    __ATTR(prole, S_IRUGO, anx7688_get_power_role,
    NULL),
    __ATTR(drole, S_IRUGO, anx7688_get_data_role,
    NULL),
    __ATTR(trysrc, S_IRUGO, anx7688_try_source,
    NULL),
    __ATTR(trysink, S_IRUGO, anx7688_try_sink,
    NULL),
    __ATTR(pswap, S_IRUGO, anx7688_send_pswap,
    NULL),
    __ATTR(dswap, S_IRUGO, anx7688_send_dswap,
    NULL),
    __ATTR(burnhex, S_IWUSR, NULL,
    anx7688_burn_hex),
    __ATTR(readhex, S_IRUGO, anx7688_read_hex,
    NULL),
    __ATTR(cmd, S_IWUSR, NULL,
    anx7688_debug)
};
#else
static struct device_attribute anx7688_device_attrs[] = {  };
#endif

static int create_sysfs_interfaces(struct device *dev)
{
    int i;
    pr_info("anx7688 create system fs interface ...\n");
    for (i = 0; i < ARRAY_SIZE(anx7688_device_attrs); i++)
        if (device_create_file(dev, &anx7688_device_attrs[i]))
            goto error;
    pr_info("success\n");
    return 0;
error:

    for (; i >= 0; i--)
        device_remove_file(dev, &anx7688_device_attrs[i]);
    pr_err("%s %s: anx7688 Unable to create interface", LOG_TAG, __func__);
    return -EINVAL;
}

module_init(anx7688_init);
module_exit(anx7688_exit);

MODULE_DESCRIPTION("USB PD Anx7688 driver");
MODULE_AUTHOR("Xia Junhua <jxia@analogixsemi.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("ANX7688_DRV_VERSION");
