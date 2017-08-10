/*
 *
 * FocalTech fts TouchScreen driver.
 * 
 * Copyright (c) 2010-2015, Focaltech Ltd. All rights reserved.
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

 /*******************************************************************************
*
* File Name: Focaltech_ex_fun.c
*
* Author: Xu YongFeng
*
* Created: 2015-01-29
*   
* Modify by mshl on 2015-07-06
*
* Abstract:
*
* Reference:
*
*******************************************************************************/

/*******************************************************************************
* 1.Included header files
*******************************************************************************/
#include "focaltech_core.h"

/*******************************************************************************
* Private constant and macro definitions using #define
*******************************************************************************/
/*create apk debug channel*/
#define PROC_UPGRADE			0
#define PROC_READ_REGISTER		1
#define PROC_WRITE_REGISTER	2
#define PROC_AUTOCLB			4
#define PROC_UPGRADE_INFO		5
#define PROC_WRITE_DATA		6
#define PROC_READ_DATA			7
#define PROC_SET_TEST_FLAG				8
#define PROC_NAME	"ftxxxx-debug"

#define WRITE_BUF_SIZE		512
#define READ_BUF_SIZE		512

// add by leo ++
#define boolean unsigned char

extern struct fts_ts_data *private_ts;
extern int ft5726_touch_status;

extern int rawdata_test_result;
extern int uniformity_test_result;
extern int scap_cb_test_result;
extern int scap_rawdata_test_result;
extern int FT5726_TP_ID;
extern int FT5726_LCM_ID;

extern boolean start_test_tp(void);
extern void ft5726_irq_enable(void);
extern void ft5726_irq_disable(void);
extern void free_test_param_data(void);
extern void fts_update_fw_vendor_id(struct fts_ts_data *data);
extern int get_test_data(char *pTestData);
extern int set_param_data(char * TestParamData);
extern int fts_ctpm_fw_download_with_i_file(struct i2c_client *client, char *firmware_name);

extern void led_configure(int enable);
extern void focal3267_reset_ic(void);
extern void FreeStoreAllData(void);
extern int fts_sw_reset(struct i2c_client *client);

extern int ledstatus;
extern bool switch_mode;
extern bool gloved_mode;
// add by leo --
/*******************************************************************************
* Private enumerations, structures and unions using typedef
*******************************************************************************/


/*******************************************************************************
* Static variables
*******************************************************************************/
static unsigned char proc_operate_mode = PROC_UPGRADE;
static struct proc_dir_entry *fts_proc_entry;
/*******************************************************************************
* Global variable or extern global variabls/functions
*******************************************************************************/
#if GTP_ESD_PROTECT
int apk_debug_flag = 0;
#endif
/*******************************************************************************
* Static function prototypes
*******************************************************************************/

// add by leo ++
extern void focal3267_reset_ic(void);
// add by leo --

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static ssize_t fts_debug_write(struct file *filp, const char __user *buff, size_t count, loff_t *ppos)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = count;
	int writelen = 0;
	int ret = 0;
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
			#if GTP_ESD_PROTECT
			apk_debug_flag = 1;
			#endif
			
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
			apk_debug_flag = 0;
			#endif
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = count - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return count;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static ssize_t fts_debug_read(struct file *filp, char __user *buff, size_t count, loff_t *ppos)
{
	int ret = 0;
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	unsigned char buf[READ_BUF_SIZE];
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	if (copy_to_user(buff, buf, num_read_chars)) {
		dev_err(&fts_i2c_client->dev, "%s:copy to user error\n", __func__);
		return -EFAULT;
	}

	return num_read_chars;
}
static const struct file_operations fts_proc_fops = {
		.owner = THIS_MODULE,
		.read = fts_debug_read,
		.write = fts_debug_write,
		
};
#else
/*interface of write proc*/
/************************************************************************
*   Name: fts_debug_write
*  Brief:interface of write proc
* Input: file point, data buf, data len, no use
* Output: no
* Return: data len
***********************************************************************/
static int fts_debug_write(struct file *filp, 
	const char __user *buff, unsigned long len, void *data)
{
	unsigned char writebuf[WRITE_BUF_SIZE];
	int buflen = len;
	int writelen = 0;
	int ret = 0;
	
	
	if (copy_from_user(&writebuf, buff, buflen)) {
		dev_err(&fts_i2c_client->dev, "%s:copy from user error\n", __func__);
		return -EFAULT;
	}
	proc_operate_mode = writebuf[0];

	switch (proc_operate_mode) {
	
	case PROC_UPGRADE:
		{
			char upgrade_file_path[128];
			memset(upgrade_file_path, 0, sizeof(upgrade_file_path));
			sprintf(upgrade_file_path, "%s", writebuf + 1);
			upgrade_file_path[buflen-1] = '\0';
			FTS_DBG("%s\n", upgrade_file_path);
			disable_irq(fts_i2c_client->irq);
			#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
			ret = fts_ctpm_fw_upgrade_with_app_file(fts_i2c_client, upgrade_file_path);
			#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
			enable_irq(fts_i2c_client->irq);
			if (ret < 0) {
				dev_err(&fts_i2c_client->dev, "%s:upgrade failed.\n", __func__);
				return ret;
			}
		}
		break;
	case PROC_READ_REGISTER:
		writelen = 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_WRITE_REGISTER:
		writelen = 2;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	case PROC_AUTOCLB:
		FTS_DBG("%s: autoclb\n", __func__);
		fts_ctpm_auto_clb(fts_i2c_client);
		break;
	case PROC_READ_DATA:
	case PROC_WRITE_DATA:
		writelen = len - 1;
		ret = fts_i2c_write(fts_i2c_client, writebuf + 1, writelen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:write iic error\n", __func__);
			return ret;
		}
		break;
	default:
		break;
	}
	

	return len;
}

/*interface of read proc*/
/************************************************************************
*   Name: fts_debug_read
*  Brief:interface of read proc
* Input: point to the data, no use, no use, read len, no use, no use 
* Output: page point to data
* Return: read char number
***********************************************************************/
static int fts_debug_read( char *page, char **start,
	off_t off, int count, int *eof, void *data )
{
	int ret = 0;
	unsigned char buf[READ_BUF_SIZE];
	int num_read_chars = 0;
	int readlen = 0;
	u8 regvalue = 0x00, regaddr = 0x00;
	
	switch (proc_operate_mode) {
	case PROC_UPGRADE:
		//after calling fts_debug_write to upgrade
		regaddr = 0xA6;
		ret = fts_read_reg(fts_i2c_client, regaddr, &regvalue);
		if (ret < 0)
			num_read_chars = sprintf(buf, "%s", "get fw version failed.\n");
		else
			num_read_chars = sprintf(buf, "current fw version:0x%02x\n", regvalue);
		break;
	case PROC_READ_REGISTER:
		readlen = 1;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		} 
		num_read_chars = 1;
		break;
	case PROC_READ_DATA:
		readlen = count;
		ret = fts_i2c_read(fts_i2c_client, NULL, 0, buf, readlen);
		if (ret < 0) {
			dev_err(&fts_i2c_client->dev, "%s:read iic error\n", __func__);
			return ret;
		}
		
		num_read_chars = readlen;
		break;
	case PROC_WRITE_DATA:
		break;
	default:
		break;
	}
	
	memcpy(page, buf, num_read_chars);
	return num_read_chars;
}
#endif
/************************************************************************
* Name: fts_create_apk_debug_channel
* Brief:  create apk debug channel
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_apk_debug_channel(struct i2c_client * client)
{	
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
		fts_proc_entry = proc_create(PROC_NAME, 0777, NULL, &fts_proc_fops);		
	#else
		fts_proc_entry = create_proc_entry(PROC_NAME, 0777, NULL);
	#endif
	if (NULL == fts_proc_entry) 
	{
		dev_err(&client->dev, "Couldn't create proc entry!\n");
		
		return -ENOMEM;
	} 
	else 
	{
		dev_info(&client->dev, "Create proc entry success!\n");
		
		#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 10, 0))
			fts_proc_entry->write_proc = fts_debug_write;
			fts_proc_entry->read_proc = fts_debug_read;
		#endif
	}
	return 0;
}
/************************************************************************
* Name: fts_release_apk_debug_channel
* Brief:  release apk debug channel
* Input: no
* Output: no
* Return: no
***********************************************************************/
void fts_release_apk_debug_channel(void)
{
	
	if (fts_proc_entry)
		#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 0))
			proc_remove(fts_proc_entry);
		#else
			remove_proc_entry(NULL, fts_proc_entry);
		#endif
}

/************************************************************************
* Name: fts_tpfwver_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpfwver_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	u8 fwver = 0;
	mutex_lock(&fts_input_dev->mutex);
	if (fts_read_reg(fts_i2c_client, FTS_REG_FW_VER, &fwver) < 0)
		return -1;
	
	
	if (fwver == 255)
		num_read_chars = snprintf(buf, 128,"get tp fw version fail!\n");
	else
	{
		num_read_chars = snprintf(buf, 128, "%02X\n", fwver);
	}
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpfwver_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpfwver_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tpdriver_version_show
* Brief:  show tp fw vwersion
* Input: device, device attribute, char buf
* Output: no
* Return: char number
***********************************************************************/
static ssize_t fts_tpdriver_version_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	ssize_t num_read_chars = 0;
	
	mutex_lock(&fts_input_dev->mutex);
	
	num_read_chars = snprintf(buf, 128,"%s \n", FTS_DRIVER_INFO);
	
	mutex_unlock(&fts_input_dev->mutex);
	
	return num_read_chars;
}
/************************************************************************
* Name: fts_tpdriver_version_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tpdriver_version_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_tprwreg_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}
/************************************************************************
* Name: fts_tprwreg_store
* Brief:  read/write register
* Input: device, device attribute, char buf, char count
* Output: print register value
* Return: char count
***********************************************************************/
static ssize_t fts_tprwreg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	ssize_t num_read_chars = 0;
	int retval;
	long unsigned int wmreg=0;
	u8 regaddr=0xff,regvalue=0xff;
	u8 valbuf[5]={0};

	memset(valbuf, 0, sizeof(valbuf));
	mutex_lock(&fts_input_dev->mutex);	
	num_read_chars = count - 1;
	if (num_read_chars != 2) 
	{
		if (num_read_chars != 4) 
		{
			dev_err(dev, "please input 2 or 4 character\n");
			goto error_return;
		}
	}
	memcpy(valbuf, buf, num_read_chars);
	//retval = strict_strtoul(valbuf, 16, &wmreg);
	retval = kstrtoul(valbuf, 16, &wmreg);
	if (0 != retval) 
	{
		dev_err(dev, "%s() - ERROR: Could not convert the given input to a number. The given input was: \"%s\"\n", __FUNCTION__, buf);
		goto error_return;
	}
	if (2 == num_read_chars) 
	{
		/*read register*/
		regaddr = wmreg;
		printk("[focal][test](0x%02x)\n", regaddr);
		if (fts_read_reg(client, regaddr, &regvalue) < 0)
			printk("[Focal] %s : Could not read the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : the register(0x%02x) is 0x%02x\n", __func__, regaddr, regvalue);
	} 
	else 
	{
		regaddr = wmreg>>8;
		regvalue = wmreg;
		if (fts_write_reg(client, regaddr, regvalue)<0)
			printk("[Focal] %s : Could not write the register(0x%02x)\n", __func__, regaddr);
		else
			printk("[Focal] %s : Write 0x%02x into register(0x%02x) successful\n", __func__, regvalue, regaddr);

	}
	error_return:
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupdate_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupdate_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupdate_store
* Brief:  upgrade from *.i
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupdate_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	u8 uc_host_fm_ver;
	int i_ret;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	#if GTP_ESD_PROTECT
		apk_debug_flag = 1;
	#endif
	
	i_ret = fts_ctpm_fw_upgrade_with_i_file(client);
	if (i_ret == 0)
	{
		msleep(300);
		uc_host_fm_ver = fts_ctpm_get_i_file_ver();
		dev_dbg(dev, "%s [FTS] upgrade to new version 0x%x\n", __func__, uc_host_fm_ver);
	}
	else
	{
		dev_err(dev, "%s ERROR:[FTS] upgrade failed ret=%d.\n", __func__, i_ret);
	}
	
	#if GTP_ESD_PROTECT
		apk_debug_flag = 0;
	#endif
	enable_irq(client->irq);
	mutex_unlock(&fts_input_dev->mutex);
	
	return count;
}
/************************************************************************
* Name: fts_fwupgradeapp_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_fwupgradeapp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* place holder for future use */
	return -EPERM;
}

/************************************************************************
* Name: fts_fwupgradeapp_store
* Brief:  upgrade from app.bin
* Input: device, device attribute, char buf, char count
* Output: no
* Return: char count
***********************************************************************/
static ssize_t fts_fwupgradeapp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char fwname[128];
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	memset(fwname, 0, sizeof(fwname));
	sprintf(fwname, "%s", buf);
	fwname[count-1] = '\0';

	mutex_lock(&fts_input_dev->mutex);
	
	disable_irq(client->irq);
	#if GTP_ESD_PROTECT
				apk_debug_flag = 1;
			#endif
	fts_ctpm_fw_upgrade_with_app_file(client, fwname);
	#if GTP_ESD_PROTECT
				apk_debug_flag = 0;
			#endif
	enable_irq(client->irq);
	
	mutex_unlock(&fts_input_dev->mutex);
	return count;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_show
* Brief:  no
* Input: device, device attribute, char buf
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	
	return -EPERM;
}
/************************************************************************
* Name: fts_ftsgetprojectcode_store
* Brief:  no
* Input: device, device attribute, char buf, char count
* Output: no
* Return: EPERM
***********************************************************************/
static ssize_t fts_getprojectcode_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	/* place holder for future use */
	return -EPERM;
}

// add by leo ++
#if 0
static ssize_t ft5726_touch_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ft5726_touch_status);
}

static ssize_t ft5726_touch_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return -EPERM;
}
#endif
static ssize_t ft5726_disable_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[ft5726] %s: disable_irq\n", __func__);
	//ft5726_irq_disable();
	disable_irq_nosync(private_ts->client->irq);

	return sprintf(buf, "Disable IRQ \n");
}

static ssize_t ft5726_disable_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return -EPERM;
}

static ssize_t ft5726_enable_irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[ft5726] %s: enable_irq\n", __func__);
	//ft5726_irq_enable();
	enable_irq(private_ts->client->irq);

	return sprintf(buf, "Enable IRQ \n");
}
static ssize_t ft5726_enable_irq_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return -EPERM;
}

static ssize_t ft5726_reset_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->pdata->reset_gpio));
}

static ssize_t ft5726_reset_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rst_pin = 0;
	sscanf(buf, "%d\n", &rst_pin);

	switch(rst_pin)
	{
		case 0:
			gpio_direction_output(private_ts->pdata->reset_gpio, 0);
			printk("[ft5726] %s: set reset_gpio direction output low\n", __func__);
			break;

		case 1:
			gpio_direction_output(private_ts->pdata->reset_gpio, 1);
			printk("[ft5726] %s: set reset_gpio direction output high\n", __func__);
			break;

		case 2:
			gpio_direction_input(private_ts->pdata->reset_gpio);
			printk("[ft5726] %s: set reset_gpio direction input\n", __func__);
			break;

		default:
			focal3267_reset_ic();
			printk("[ft5726] %s: reset IC\n", __func__);
	}
	return count;
}

static ssize_t ft5726_rstpin_test_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 state = 0;
	struct i2c_client *client = container_of(dev, struct i2c_client, dev);

	mutex_lock(&fts_input_dev->mutex);	

	fts_write_reg(client, 0xd0, 0x01);
	msleep(20);
	fts_read_reg(client, 0xd0, &state);
	printk("[ft5726] %s: set 0xd0 = 0x%02X\n", __func__, state);

	if (gpio_is_valid(private_ts->pdata->reset_gpio))
	{
		gpio_set_value_cansleep(private_ts->pdata->reset_gpio, 0);
		msleep(private_ts->pdata->hard_rst_dly);
		gpio_set_value_cansleep(private_ts->pdata->reset_gpio, 1);
	}
	msleep(300);

	fts_read_reg(client, 0x0d, &state);
	printk("[ft5726] %s: after reset IC, get 0x0d = 0x%02X\n", __func__, state);
	printk("[ft5726] %s: reset pin test %s\n", __func__, (state == 0x00 ? "Pass" : "Fail" ));

	mutex_unlock(&fts_input_dev->mutex);

	return sprintf(buf, "%s\n", (state == 0x00 ? "Pass" : "Fail" ));
}
static ssize_t ft5726_rstpin_test_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int rst_pin = 0;
	sscanf(buf, "%d\n", &rst_pin);

	return count;
}
#if 0
static ssize_t ft5726_power_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", gpio_get_value(private_ts->pdata->power_gpio));
}

static ssize_t ft5726_power_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int pwr_pin = 0;
	sscanf(buf, "%d\n", &pwr_pin);

	gpio_set_value(private_ts->pdata->power_gpio, (pwr_pin > 0 ? 1 : 0));
	printk("[ft5726] %s: set power_gpio = %s\n", __func__, (pwr_pin > 0 ? "High" : "Low"));
	printk("[ft5726] %s: power_gpio value= %d\n", __func__, gpio_get_value(private_ts->pdata->power_gpio));
        return count;
}

static ssize_t ft5726_tpid_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", FT5726_TP_ID);
}

static ssize_t ft5726_tpid_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return count;
}

static ssize_t ft5726_lcmid_pin_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", FT5726_LCM_ID);
}

static ssize_t ft5726_lcmid_pin_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}

static ssize_t ft5726_chip_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 chip_id = 0x00;
	int ret = 0;

	ret = fts_read_reg(fts_i2c_client, 0xA3, &chip_id);
	if (ret<0)
	{
		printk("[ft5726] %s: read value fail", __func__);
	}
	printk("[ft5726] %s: get chip_id = 0x%02X\n", __func__, chip_id);

	return sprintf(buf, "0x%02X\n", chip_id);
}
static ssize_t ft5726_chip_id_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	return count;
}
#endif
static ssize_t ft5726_fw_vendor_id_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	fts_update_fw_vendor_id(private_ts);

	return sprintf(buf, "0x%02X\n", private_ts->fw_vendor_id);
}

static ssize_t ft5726_fw_vendor_id_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
        return count;
}

#define FT5X0X_CFG_FILEPATH "/system/etc/firmware/"
#define FT5726_RESULT_FILEPATH "/data/data/"

static int ft5x0x_GetInISize(char *config_name)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize = 0;
	char filepath[128];

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_CFG_FILEPATH, config_name);

	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[Focal][TOUCH_ERR] %s : error occured while opening file %s. \n", __func__, filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	filp_close(pfile, NULL);
	return fsize;
}

static int ft5x0x_ReadInIData(char *config_name, char *config_buf)
{
	struct file *pfile = NULL;
	struct inode *inode;
	unsigned long magic;
	off_t fsize;
	char filepath[128];
	loff_t pos;
	mm_segment_t old_fs;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5X0X_CFG_FILEPATH, config_name);
	if (NULL == pfile)
		pfile = filp_open(filepath, O_RDONLY, 0);
	if (IS_ERR(pfile)) {
		pr_err("[Focal][TOUCH_ERR] %s : error occured while opening file %s. \n", __func__, filepath);
		return -EIO;
	}
	inode = pfile->f_dentry->d_inode;
	magic = inode->i_sb->s_magic;
	fsize = inode->i_size;
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	pos = 0;
	vfs_read(pfile, config_buf, fsize, &pos);
	filp_close(pfile, NULL);
	set_fs(old_fs);

	config_buf[fsize] = '\0';
	return 0;
}

static int ft5x0x_get_testparam_from_ini(char *config_name)
{
	char *config_data = NULL;
	int file_size;

	file_size = ft5x0x_GetInISize(config_name);

	pr_err("[Focal][Touch] %s : inisize = %d \n ", __func__, file_size);
	if (file_size <= 0) {
		pr_err("[Focal][TOUCH_ERR] %s : ERROR : Get firmware size failed \n", __func__);
		return -EIO;
	}

	config_data = kmalloc(file_size + 1, GFP_KERNEL);
	if(!config_data)
	{
		pr_err("Focal][TOUCH_ERR] %s : allocate config_data memory fail !\n", __func__);
		return -EIO;
	}

	if (ft5x0x_ReadInIData(config_name, config_data)) {
		pr_err("[Focal][TOUCH_ERR] %s() - ERROR: request_firmware failed \n", __func__);
		kfree(config_data);
		return -EIO;
	}
	else {
		pr_info("[Focal][Touch] %s : ft5x0x_ReadInIData successful \n", __func__);
	}

	set_param_data(config_data);

	return 0;
}

static mm_segment_t oldfs;

static struct file *fts_selftest_file_open(void)
{

	struct file* filp = NULL;
	char filepath[128];
	int err = 0;

	memset(filepath, 0, sizeof(filepath));
	sprintf(filepath, "%s%s", FT5726_RESULT_FILEPATH, "VK_selftest.csv");

	oldfs = get_fs();
	set_fs(get_ds());

	filp = filp_open(filepath, O_WRONLY|O_CREAT, 0777);
	if(IS_ERR(filp)) {
		err = PTR_ERR(filp);
		return NULL;
	}

	return filp;
}

int fts_selftest_file_write(struct file* file, unsigned char *data, int len)
{

	int ret;

	ret = file->f_op->write(file, data, len, &file->f_pos);

	return ret;
}

void fts_selftest_file_close(struct file* file)
{
	set_fs(oldfs);
	filp_close(file, NULL);
}

int self_test_result = 1;

static ssize_t ft5726_test_tp_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if (self_test_result) {
		printk("[Focal][VK] %s : VK test failure \n", __func__);
	}
	else {
		printk("[Focal][VK] %s : VK test pass \n", __func__);
	}
	
	//printk("[Focal][Touch] %s : rawdata_test_result = %d uniformity_test_result = %d, scap_cb_test_result = %d, scap_rawdata_test_result = %d", __func__,
		//rawdata_test_result, uniformity_test_result, scap_cb_test_result, scap_rawdata_test_result);

	return sprintf(buf, "VKselftest : %s\n", self_test_result ? "FAIL" : "PASS");
}

static ssize_t ft5726_test_tp_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	struct i2c_client *client = container_of(dev, struct i2c_client, dev);
	char config_file[128], result_buf[64];
	char *w_buf;
	struct file *w_file;
	int w_len, i = 0;

	mutex_lock(&fts_input_dev->mutex);
	disable_irq(client->irq);
	//ft5726_irq_disable();

	rawdata_test_result = -1;
	uniformity_test_result = -1;
	scap_cb_test_result = -1;
	scap_rawdata_test_result = -1;

	memset(config_file, 0, sizeof(config_file));
	sprintf(config_file, "%s", buf);
	config_file[count-1] = '\0';

	if (ft5x0x_get_testparam_from_ini(config_file) < 0) {
		printk("[Focal][VK_ERR] %s : get testparam from ini failure \n", __func__);
	}
	else {
		printk("[Focal] %s : VK test Start... \n", __func__);

		if (start_test_tp()) {
			printk("[Focal] VKselftest : pass !!!\n");
			self_test_result = 0;
		}
		else {
			printk("[Focal] VKselftest : fail !!!\n");
			self_test_result = 1;
		}

		for (i = 0; i < 3; i++) {
			if (fts_write_reg(client, 0x00, 0x00) >= 0)
				break;
			else
				msleep(200);
		}

		w_file = fts_selftest_file_open();
		if(!w_file)
			printk("%s : Open log file fail !\n", __func__);
		else{
			w_buf = kmalloc(80*1024, GFP_KERNEL);
			if(!w_buf)
				printk("%s : allocate memory fail !\n", __func__);
			else{
				w_len = get_test_data(w_buf);
				fts_selftest_file_write(w_file, w_buf, w_len);
				w_len = sprintf(result_buf, "VKselftest : %s\n", self_test_result ? "FAIL" : "PASS");
				//printk("[FOCAL] VKselftest : %s\n", self_test_result ? "FAIL" : "PASS");
				fts_selftest_file_write(w_file, result_buf, w_len);
				kfree(w_buf);
			}
			fts_selftest_file_close(w_file);
			FreeStoreAllData();
		}

		free_test_param_data();
	}
	//focal3267_reset_ic();
	fts_sw_reset(client);
	enable_irq(client->irq);
	//ft5726_irq_enable();
	mutex_unlock(&fts_input_dev->mutex);
        return count;
}
// add by leo --

static ssize_t show_attrs_handler(struct device *dev,
	struct device_attribute *devattr, char *buf) {
	int ret = -1;

	printk("[Focal][Touch] led show_attrs_handler\n");
	mutex_lock(&fts_input_dev->mutex);
	ret = ledstatus;
	mutex_unlock(&fts_input_dev->mutex);
	return snprintf(buf, 8, "0x%X\n", ret);
	return 0;
}

static ssize_t store_attrs_handler(struct device *dev,
	struct device_attribute *devattr, const char *buf, size_t count) {

	unsigned long value;

	if (kstrtoul(buf, 16, &value)) return -EINVAL;
	printk("[Focal][Touch] led store_attrs_handler: value= 0x%lX\n", value);

	mutex_lock(&fts_input_dev->mutex);
	led_configure(value);
	mutex_unlock(&fts_input_dev->mutex);

	return strnlen(buf, count);
}

static ssize_t switch_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk("[Focal] switch_mode = %d\n", switch_mode);
	return 0;
}

static ssize_t switch_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;

	if (kstrtoul(buf, 16, &value)) return -EINVAL;

	mutex_lock(&fts_input_dev->mutex);
	if (value == 0){
		printk("[Focal] switch_mode = 0x%lX\n", value);
		switch_mode = 0;
	}
	else if(value == 1){
		printk("[Focal] switch_mode = 0x%lX\n", value);
		switch_mode = 1;
	}
	else
		printk("ERROR, please enter 0 or 1\n");
	mutex_unlock(&fts_input_dev->mutex);

	return strnlen(buf, count);
}

static ssize_t gloved_mode_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	printk("[Focal] gloved_mode = %d\n", gloved_mode);
	return 0;
}

static ssize_t gloved_mode_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long value;
	char txbuf[2];
	int err;
	struct fts_ts_data *data = dev_get_drvdata(dev);

	if (kstrtoul(buf, 16, &value)) return -EINVAL;

	mutex_lock(&fts_input_dev->mutex);
	if (value == 0){
		printk("[Focal] gloved_mode = 0x%lX\n", value);
		gloved_mode = 0;
		txbuf[0] = 0xC0;
		txbuf[1] = 0;
		err = fts_i2c_write(data->client, txbuf, sizeof(txbuf));
		msleep(20);
		if(err<0)
			printk("[Focal] %s : write C0 00 fail\n", __func__);
		else
			printk("[Focal] %s : write C0 00 success\n", __func__);
	}
	else if(value == 1){
		printk("[Focal] gloved_mode = 0x%lX\n", value);
		gloved_mode = 1;
		txbuf[0] = 0xC0;
		txbuf[1] = 1;
		err = fts_i2c_write(data->client, txbuf, sizeof(txbuf));
		msleep(20);
		if(err < 0)
			printk("[Focal] %s : write C0 01 fail\n", __func__);
		else
			printk("[Focal] %s : write C0 01 success\n", __func__);
	}
	else
		printk("ERROR, please enter 0 or 1\n");
	mutex_unlock(&fts_input_dev->mutex);

	return strnlen(buf, count);
}

/****************************************/
/* sysfs */
/*get the fw version
*example:cat ftstpfwver
*/
static DEVICE_ATTR(ftstpfwver, S_IRUGO|S_IWUSR, fts_tpfwver_show, fts_tpfwver_store);

static DEVICE_ATTR(ftstpdriverver, S_IRUGO|S_IWUSR, fts_tpdriver_version_show, fts_tpdriver_version_store);
/*upgrade from *.i
*example: echo 1 > ftsfwupdate
*/
static DEVICE_ATTR(ftsfwupdate, S_IRUGO|S_IWUSR, fts_fwupdate_show, fts_fwupdate_store);
/*read and write register
*read example: echo 88 > ftstprwreg ---read register 0x88
*write example:echo 8807 > ftstprwreg ---write 0x07 into register 0x88
*
*note:the number of input must be 2 or 4.if it not enough,please fill in the 0.
*/
static DEVICE_ATTR(ftstprwreg, S_IRUGO|S_IWUSR, fts_tprwreg_show, fts_tprwreg_store);
/*upgrade from app.bin
*example:echo "*_app.bin" > ftsfwupgradeapp
*/
static DEVICE_ATTR(ftsfwupgradeapp, S_IRUGO|S_IWUSR, fts_fwupgradeapp_show, fts_fwupgradeapp_store);
static DEVICE_ATTR(ftsgetprojectcode, S_IRUGO|S_IWUSR, fts_getprojectcode_show, fts_getprojectcode_store);

// add by leo ++
#if 0
static DEVICE_ATTR(ftstpfwdownload, S_IRUGO|S_IWUSR, fts_tpfwdownload_show, fts_tpfwdownload_store);
static DEVICE_ATTR(touch_status, S_IRUGO|S_IWUSR, ft5726_touch_status_show, ft5726_touch_status_store);
#endif
static DEVICE_ATTR(disable_irq, S_IWUSR | S_IRUGO, ft5726_disable_irq_show, ft5726_disable_irq_store);
static DEVICE_ATTR(enable_irq, S_IWUSR | S_IRUGO, ft5726_enable_irq_show, ft5726_enable_irq_store);
static DEVICE_ATTR(reset_pin, S_IWUSR | S_IRUGO, ft5726_reset_pin_show, ft5726_reset_pin_store);
static DEVICE_ATTR(rstpin_test, S_IWUSR | S_IRUGO, ft5726_rstpin_test_show, ft5726_rstpin_test_store);
#if 0
static DEVICE_ATTR(power_pin, S_IWUSR | S_IRUGO, ft5726_power_pin_show, ft5726_power_pin_store);
static DEVICE_ATTR(tp_id, S_IWUSR | S_IRUGO, ft5726_tpid_pin_show, ft5726_tpid_pin_store);
static DEVICE_ATTR(lcm_id, S_IWUSR | S_IRUGO, ft5726_lcmid_pin_show, ft5726_lcmid_pin_store);
static DEVICE_ATTR(chip_id, S_IWUSR | S_IRUGO, ft5726_chip_id_show, ft5726_chip_id_store);
#endif
static DEVICE_ATTR(fw_vendor_id, S_IWUSR | S_IRUGO, ft5726_fw_vendor_id_show, ft5726_fw_vendor_id_store);
static DEVICE_ATTR(test_tp, S_IRUGO|S_IWUSR, ft5726_test_tp_show, ft5726_test_tp_store);
// add by leo --
static DEVICE_ATTR(led_control, S_IWUSR | S_IRUGO, show_attrs_handler, store_attrs_handler);
static DEVICE_ATTR(switch_mode, S_IWUSR | S_IRUGO, switch_mode_show, switch_mode_store);
static DEVICE_ATTR(gloved_mode, S_IWUSR | S_IRUGO, gloved_mode_show, gloved_mode_store);

/*add your attr in here*/
static struct attribute *fts_attributes[] = {
	&dev_attr_ftstpfwver.attr,
	&dev_attr_ftstpdriverver.attr,
	&dev_attr_ftsfwupdate.attr,
	&dev_attr_ftstprwreg.attr,
	&dev_attr_ftsfwupgradeapp.attr,
	&dev_attr_ftsgetprojectcode.attr,
	// add by leo ++
	#if 0
	&dev_attr_ftstpfwdownload.attr,
	&dev_attr_touch_status.attr,
	#endif
	&dev_attr_disable_irq.attr,
	&dev_attr_enable_irq.attr,
	&dev_attr_reset_pin.attr,
	&dev_attr_rstpin_test.attr,
	#if 0
	&dev_attr_power_pin.attr,
	&dev_attr_tp_id.attr,
	&dev_attr_lcm_id.attr,
	&dev_attr_chip_id.attr,
	#endif
	&dev_attr_fw_vendor_id.attr,
	&dev_attr_test_tp.attr,
	&dev_attr_led_control.attr,
	&dev_attr_switch_mode.attr,
	&dev_attr_gloved_mode.attr,
	// add by leo --
	NULL
};

static struct attribute_group fts_attribute_group = {
	.attrs = fts_attributes
};

/************************************************************************
* Name: fts_create_sysfs
* Brief:  create sysfs for debug
* Input: i2c info
* Output: no
* Return: success =0
***********************************************************************/
int fts_create_sysfs(struct i2c_client * client)
{
	int err;
	
	err = sysfs_create_group(&client->dev.kobj, &fts_attribute_group);
	if (0 != err) 
	{
		dev_err(&client->dev, "%s() - ERROR: sysfs_create_group() failed.\n", __func__);
		sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
		return -EIO;
	} 
	else 
	{
		pr_info("fts:%s() - sysfs_create_group() succeeded.\n",__func__);
	}
	return err;
}
/************************************************************************
* Name: fts_remove_sysfs
* Brief:  remove sys
* Input: i2c info
* Output: no
* Return: no
***********************************************************************/
int fts_remove_sysfs(struct i2c_client * client)
{
	sysfs_remove_group(&client->dev.kobj, &fts_attribute_group);
	return 0;
}
