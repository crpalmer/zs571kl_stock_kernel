#include <linux/module.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/version.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/kernel.h>

#include <linux/io.h>
#include <linux/ioctl.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/syscalls.h>
#include <linux/err.h>
#include <linux/completion.h>
#include <linux/cdev.h>
#include <linux/types.h>
#include <linux/wakelock.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/pm.h>

#include <linux/namei.h>
#include <linux/mount.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include "efsa120s.h"
#include "asus_fp_id.h"
#include "asus_navigation.h"

#define VERSION_LOG	"ELAN FINGER PRINT V1.4.3.1"
#ifdef INIT_COMPLETION
#define reinit_completion(x) INIT_COMPLETION(*(x))
#endif
#define SPI_MAX_SPEED		5000000 // chagne by customer
#define _SIGNAL_MODE_		// define = Signal Event, no def = Key mode
#define KEY_FP_INT			KEY_POWER //KEY_WAKEUP // change by customer & framework support
#define KEY_FP_INT2			KEY_1 //KEY_WAKEUP // change by customer & framework support
static int sig; // added v1.43
static pid_t pid; // added v1.43
static unsigned char bsig_pid = 0; // added v1.43
static unsigned char bState = 1; // 1=on, 0=off
#define ELAN_CALI_TIMEOUT_MSEC	10000
struct completion cmd_done;
struct completion cmd_done_irq;
//#define CONFIG_PM_SLEEP
#define _ELAN_DEBUG_
#ifdef _ELAN_DEBUG_
#define ELAN_DEBUG(format, args ...) 		\
	pr_debug("[ELAN]: " format, ##args)
#else
#define ELAN_DEBUG(format, args ...)
#endif

#define ELAN_INFO(format, args ...) 		\
	pr_info("[ELAN]: " format, ##args)
#define ELAN_ERR(format, args ...) 		\
	pr_err("[ELAN]: " format, ##args)

#define INT_NORMAL_HIGH		0x40
//#define ALIG_4_BYTE // change by customer

#define FP_SLEEP 12
#define FP_3V3_EN 16

struct efsa120s_data  {
	struct platform_device	*pdev;
	struct input_dev 	*input_dev;
	struct mutex		fp_mutex;
	struct cdev 		fp_cdev;
	struct class		*fp_class;
	struct device		*platform_device;


	/* File I/O for user-space */
	struct mutex 		sysfs_mutex;
	struct miscdevice 	efsa120_dev;	/* char device for ioctl */

	int 			intr_gpio;
	int	isr;
	int 			rst_gpio;
	int osvcc_pin;
	int vcc3v3;
	struct wake_lock	wake_lock;
	struct wake_lock    elan_fp_wakelock;
	struct regulator *sovcc;
	struct regulator *vcc;
	struct work_struct  work;
	unsigned char bImageReady;
	spinlock_t irq_lock;
	int irq_is_disable;
	//	struct early_suspend early_suspend;
	wait_queue_head_t efsa_wait;
};



static int major_number = 0;
static int minor_number = 0;

static unsigned char status[5] = {0}; // added by samuel

static unsigned char IOIRQ_STATUS = 0;

static int elan_work_flag = 0;
static DECLARE_WAIT_QUEUE_HEAD(elan_poll_wq);

//static unsigned char image_data_offset = 16;

static struct workqueue_struct *efsa120s_wq;
/*
#ifdef CONFIG_HAS_EARLYSUSPEND
static void efsa120s_early_suspend(struct early_suspend *h);
static void efsa120s_late_resume(struct early_suspend *h);

#endif
*/

static int efsa120s_send_signal_to_pid(void)
{
	struct siginfo info;

	if(bsig_pid == 0)
		return -1;
	if(bState == 0) // Display off
		info.si_signo = sig + 1; // global para
	else // Display on
		info.si_signo = sig; // global para
	info.si_errno = 0;
	info.si_code = SI_USER; // send by kill, sigsend, raise ?
	info.si_pid = get_current()->pid; // global para
	info.si_uid = from_kuid_munged(current_user_ns(), current_uid());

	// printk("[ELAN] send signal(0x%x) to pid(0x%x).\n", info.si_signo, pid);
	return kill_proc_info(info.si_signo, &info, pid);
}

static int efsa120s_fingerprint_init(struct platform_device *pdev, unsigned char data[])
{
	struct efsa120s_data *fp = platform_get_drvdata(pdev);

	if(g_ASUS_evbcfgID == IS_EVB) {
	} else {
		gpio_direction_output(fp->rst_gpio, 0);
		mdelay(5);
		gpio_direction_output(fp->rst_gpio, 1);
		mdelay(50);
	}

	ELAN_DEBUG("Fuse Load\n");

	mdelay(1);

	mdelay(1);

	mdelay(1);

	mdelay(1);

	mdelay(1);

	mdelay(1);

	return 0;
}

static ssize_t show_drv_version_value(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%s\n", VERSION_LOG);
}

static DEVICE_ATTR(drv_version, S_IRUGO, show_drv_version_value, NULL);

static ssize_t show_name(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "elfp\n");
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);

static struct attribute *efsa120s_attributes[] = {
	&dev_attr_drv_version.attr,
	&dev_attr_name.attr,
	NULL
};

static struct attribute_group efsa120s_attr_group = {
	.attrs = efsa120s_attributes,
};

static void efsa120s_reset(struct efsa120s_data *fp)
{
	if(g_ASUS_evbcfgID == IS_EVB) {
	} else {
		gpio_direction_output(fp->rst_gpio, 0);
		mdelay(5);
		gpio_direction_output(fp->rst_gpio, 1);
		mdelay(50);
	}
}

static int efsa120s_open(struct inode *inode, struct file *filp)
{
	struct efsa120s_data *fp;

	fp = container_of(inode->i_cdev, struct efsa120s_data, fp_cdev);
	filp->private_data = fp;

	ELAN_DEBUG("%s()\n", __func__);

	return 0;
}

static int efsa120s_close(struct inode *inode, struct file *filp)
{
	//	struct efsa120s_data *fp = filp->private_data;
	//	ELAN_DEBUG("[ELAN] %s()\n", __func__);
	return 0;
}

/*******************************************************
Function:
Disable irq function
Input:
file: file
Output:
None.

 *********************************************************/
void efsa120s_irq_disable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags;
	ELAN_DEBUG("IRQ Disable = %d.\n", fp->isr);

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (!fp->irq_is_disable)
	{
		fp->irq_is_disable = 1;
		disable_irq_nosync(fp->isr);
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

/*******************************************************
Function:
Enable irq function
Input:
file: file
Output:
None.
 *********************************************************/

void efsa120s_irq_enable(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	unsigned long irqflags = 0;
	ELAN_DEBUG("IRQ Enable = %d.\n", fp->isr);

	spin_lock_irqsave(&fp->irq_lock, irqflags);
	if (fp->irq_is_disable)
	{
		enable_irq(fp->isr);
		fp->irq_is_disable = 0;
	}
	spin_unlock_irqrestore(&fp->irq_lock, irqflags);
}

static ssize_t efsa120s_write(struct file *filp, const char __user *user_buf, size_t len, loff_t *off)
{
	struct efsa120s_data *fp = filp->private_data;
	int ret = 0;
	unsigned char buf[130];
	int cmd = user_buf[0];

	/* for test */
	if(cmd == 0) // show function list
	{
		ELAN_INFO("*******************************************************\n");
		ELAN_INFO("0: show function list. %s\n", VERSION_LOG);
		ELAN_INFO("1: efsa120s initialize ...\n");
		ELAN_INFO("2: start scan.\n");
		ELAN_INFO("3: show image min & max.\n");
		ELAN_INFO("4: show image data.\n");
		ELAN_INFO("5/0: spi speed now\n");
		ELAN_INFO("5/1/data: spi speed update. data *100k\n");
		ELAN_INFO("6: hardware reset.\n");
		ELAN_INFO("7/0: interupt disable.\n");
		ELAN_INFO("7/1: interupt enable.\n");
		ELAN_INFO("8/data: write data. head, dummy ... you should set by yourself!!!\n");
		ELAN_INFO("9/cmd/len: read data. head, dummy ... you should count to len!!!\n");
		ELAN_INFO("A/data: IOIRQ_Status update in driver\n");
		ELAN_INFO("C/sig[4]/pid[4]:signal to pid.\n");
		ELAN_INFO("D/State: send power_key, State 0=up, 1=down.\n");
		ELAN_INFO("E/TOUCH/State:State 0=up, 1=down.\n");
		ELAN_INFO("*******************************************************\n");
	} else if(cmd == 1) {
		IOIRQ_STATUS = 0X09;
		efsa120s_fingerprint_init(fp->pdev, buf); // buf do not need
		ELAN_INFO("[ELAN] %s init ...\n", VERSION_LOG);
	}
	else if(cmd == 6) // do reset
	{
		efsa120s_reset(fp);
		ELAN_INFO("efsa120s_reset\n");
	}
	else if(cmd == 7) // enable interrupt or disable
	{
		if(len == 3) {
			if(user_buf[1] == 1){
				ELAN_INFO("Interrupt enable.\n");
				efsa120s_irq_enable(fp);
			} else if(user_buf[1] == 0) {
				ELAN_INFO("Interrupt enable.\n");
				efsa120s_irq_disable(fp);
			} else
				ELAN_INFO("Interrupt enable/disable data error.\n");
		} else
			ELAN_INFO("Interrupt enable/disable length error.\n");
	}
	else if(cmd == 10) // update ioirq
	{
		ELAN_INFO("IOIRQ = 0x%x -> 0x%x.\n", IOIRQ_STATUS, user_buf[1]);
		IOIRQ_STATUS = user_buf[1];
	}
	else if(cmd == 12)// send signal to pid
	{
		if(len == 10) // cmd + 4 + 4 + na
		{
			if(copy_from_user(&sig, &user_buf[1], 4))
				return -1;
			if(copy_from_user(&pid, &user_buf[5], 4))
				return -1;
			bsig_pid = 1;
			efsa120s_send_signal_to_pid();
			ELAN_INFO("send signal sig=%d, pid=%d.\n", sig, (int) pid);
		}
		else
		{
			ELAN_INFO("send signal len error. len = %d. require 10.\n", (int) len);
		}

	}
	else if(cmd == 13) // report power key event, if you need report any key -> input_set_capability @ probe
	{
		if(!(user_buf[1] == 0 || user_buf[1] == 1))
			return -1;

		ELAN_INFO("KEY_#0x%x = %x.\n", KEY_FP_INT, user_buf[1]);
		input_report_key(fp->input_dev, KEY_FP_INT, user_buf[1]); // Added for KEY Event
		input_sync(fp->input_dev);
		mdelay(1);
	}
	else if(cmd == 14) // report touch event
	{
		ELAN_INFO("KEY_#0x%x = %x. It not work now.\n", user_buf[1], user_buf[2]);
	}

	return (ret==0) ? len : ret;
}



static long efsa120s_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct efsa120s_data *fp = filp->private_data;
	unsigned char buf[16];
	unsigned char *pUserBuf;
	int keycode;
	//	int value = 0;

	ELAN_DEBUG("%s() : cmd = [%04X]\n", __func__, cmd);

	switch(cmd)
	{
	case ID_IOCTL_INIT:
		if(copy_from_user(buf, (int *)arg, sizeof(unsigned char)*16))
			return -1;
		efsa120s_fingerprint_init(fp->pdev, buf);
		if(copy_to_user((int *)arg, buf, sizeof(unsigned char)*16))
			return -1;
		ELAN_DEBUG("%s() : ID_IOCTL_INIT\n", __func__);
		break;
	case ID_IOCTL_RESET:
		efsa120s_reset(fp);
		ELAN_DEBUG("%s() : ID_IOCTL_RESET\n", __func__);
		break;
	case ID_IOCTL_IOIRQ_STATUS:
		pUserBuf = (unsigned char *)arg;

		if(copy_from_user(&IOIRQ_STATUS, pUserBuf, 1))
			return -1;
		if((IOIRQ_STATUS & INT_NORMAL_HIGH) == (status[0] & INT_NORMAL_HIGH)) // INT Normal status check
		{
			ELAN_DEBUG("%s() : ID_IOCTL_IOIRQ_STATUS: IOIRQ = %x.\n", __func__, IOIRQ_STATUS);
		} else {
			ELAN_DEBUG("%s() : ID_IOCTL_IOIRQ_STATUS: INT Normal status setting error. IOIRQ = %x, Driver = %x.\n", __func__, (IOIRQ_STATUS & INT_NORMAL_HIGH), (status[0] & INT_NORMAL_HIGH));
			IOIRQ_STATUS = 0; // clear
			return -1;
		}

		// if((IOIRQ_STATUS & 0xA0) || (IOIRQ_STATUS & 0X08))
		// 	efsa120s_irq_enable(fp);

		break;
	case ID_IOCTL_SIG_PID:
		pUserBuf = (unsigned char *)arg;

		if(pUserBuf[0] & 0x80) // Update SIG PID
		{
			if(copy_from_user(&sig, &pUserBuf[1], 4))
				return -1;
			if(copy_from_user(&pid, &pUserBuf[5], 4))
				return -1;
			bsig_pid = 1;
			ELAN_DEBUG("%s() : ID_IOCTL_SIG_PID: Update Event: sig=%d, pid=%d.\n", __func__, sig, (int) pid);
		} else {
			if(copy_to_user(&pUserBuf[1], &sig, 4))
				return -1;
			if(copy_to_user(&pUserBuf[5], &pid, 4))
				return -1;
			ELAN_DEBUG("%s() : ID_IOCTL_SIG_PID: Read Event: sig=%d, pid=%d.\n", __func__, sig, (int) pid);
		}
	case ID_IOCTL_POLL_INIT:
		elan_work_flag = 0;
		ELAN_DEBUG("[IOCTL] POLL INIT\n");
		break;
	case ID_IOCTL_INPUT_KEYCODE: //add input keycode by herman
		keycode = asus_translate_keycode((int __user)arg);
		ELAN_DEBUG("[james] check keycode = %d \n", keycode);

		if (!keycode) {
			ELAN_ERR("Keycode %d not defined, ignored.\n", (int __user)arg);
			break;
		}
		if (keycode != 750) // only report Long press
			break;
		input_report_key(fp->input_dev, keycode, 1); // Added for KEY Event
		input_sync(fp->input_dev);
		input_report_key(fp->input_dev, keycode, 0); // Added for KEY Event
		input_sync(fp->input_dev);
		break;
	case ID_IOCTL_NAV_WOE:
		ELAN_DEBUG("[james] navigation trigger woe flag...\n");
		elan_work_flag = 1;
		wake_up(&elan_poll_wq);
		break;
	case ID_IOCTL_WAKE_LOCK:		 //50
		ELAN_DEBUG("wake lock...\n");
		wake_lock(&fp->elan_fp_wakelock);
		break;
	case ID_IOCTL_WAKE_UNLOCK:			  //51
		ELAN_DEBUG("wake unlock...\n");
		wake_unlock(&fp->elan_fp_wakelock);
		break;
	default:
		ELAN_DEBUG("%s() : Unknown cmd\n", __func__);
		break;

	}
	//mutex_unlock(&fp->fp_mutex);
	return 0;
}

static unsigned int efsa120s_poll(struct file *file, poll_table *wait)
{
	struct efsa120s_data *fp = file->private_data;
	int mask=0;
	ELAN_INFO("%s() IRQ status = %d\n", __func__, gpio_get_value(fp->intr_gpio));
	wait_event_interruptible(elan_poll_wq, elan_work_flag > 0);
	if(elan_work_flag > 0)
		mask = elan_work_flag;

	elan_work_flag = 0;
	ELAN_INFO("%s() mask = %d\n", __func__, mask);
	return mask;
}

static const struct file_operations efsa120s_fops = {
	.owner 			= THIS_MODULE,
	.open 			= efsa120s_open,
	.read 			= NULL,
	.write 			= efsa120s_write,
	.unlocked_ioctl		= efsa120s_ioctl,
	.poll			= efsa120s_poll,
#ifdef _MTK_PLATFORM_
	.compat_ioctl 	= efsa120s_compat_ioctl,
#endif
	.release 		= efsa120s_close,
};

#ifdef _MTK_PLATFORM_
static long efsa120s_compat_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	return efsa120s_ioctl(filp, cmd, (unsigned long)compat_ptr(arg));
}
#endif

/*******************************************************
Function:
efsa120s fingerprint work function
Input:
work: work
Output:
None.
 *********************************************************/
static void efsa120s_fp_work_func(struct work_struct *work)
{
	struct efsa120s_data *fp;

	fp = container_of(work, struct efsa120s_data, work);

	ELAN_DEBUG("%s() IOIRQ=%x.\n", __func__, IOIRQ_STATUS);

	status[0] |= 1; // added by samuel
	elan_work_flag = 1;
	wake_up(&elan_poll_wq);
	if(IOIRQ_STATUS & 0xA0) // WOE Interrupt Enable
	{
		//#ifdef _SIGNAL_MODE_
		efsa120s_send_signal_to_pid();
		//#else
		if(bState == 0) // Display off
		{
			/*input_report_key(fp->input_dev,KEY_FP_INT, 1); // Added for KEY Event
			  input_sync(fp->input_dev);
			  mdelay(300);
			  input_report_key(fp->input_dev,KEY_FP_INT, 0); // Added for KEY Event
			  input_sync(fp->input_dev);
			  mdelay(300);
			  input_report_key(fp->input_dev,KEY_FP_INT, 1); // Added for KEY Event
			  input_sync(fp->input_dev);
			  mdelay(300);
			  input_report_key(fp->input_dev,KEY_FP_INT, 0); // Added for KEY Event
			  input_sync(fp->input_dev);
			  mdelay(300);
			  ELAN_DEBUG(&fp->pdev->dev, "%s() RESERVED=0x%X\n", __func__, KEY_FP_INT);*/
		} else {
			input_report_key(fp->input_dev,KEY_FP_INT2, 1); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			input_report_key(fp->input_dev,KEY_FP_INT2, 0); // Added for KEY Event
			input_sync(fp->input_dev);
			mdelay(1);
			ELAN_DEBUG("%s() RESERVED=0x%X\n", __func__, KEY_FP_INT2);
		}
		//#endif
		return;
	}

	//efsa120s_irq_enable(fp);
}

static irqreturn_t efsa120s_irq_handler(int irq, void *_fp)
{
	struct efsa120s_data *fp = _fp;
	wake_lock_timeout(&fp->wake_lock, msecs_to_jiffies(500));
	elan_work_flag = 1;
	wake_up(&elan_poll_wq);
	ELAN_INFO("%s() IRQ status = %d\n", __func__, gpio_get_value(fp->intr_gpio));

	//efsa120s_irq_disable(fp);

	//queue_work(efsa120s_wq, &fp->work);

	return IRQ_HANDLED;
}


static int efsa120s_setup_cdev(struct efsa120s_data *finger)
{
	dev_t devNum;
	int err = 0;

	ELAN_INFO("%s()\n", __func__);

	err = alloc_chrdev_region(&devNum, 0, 1, "fingerprint");
	if(err < 0) {
		ELAN_ERR("Alloc char dev region fail.\n");
		goto alloc_chrdev_region_fail;
	}

	major_number = MAJOR(devNum);
	minor_number = MINOR(devNum);

	// Create class under /sysfs
	finger->fp_class = class_create(THIS_MODULE, "fingerprint_class");
	if(IS_ERR(finger->fp_class))
	{
		err = -1;
		ELAN_ERR("class create failed\n");
		goto class_create_fail;
	}
	// Create device under /dev
	finger->platform_device = device_create(finger->fp_class, NULL, devNum,
			"%s", "fingerprint");
	if(IS_ERR(finger->platform_device)) {
		err = -1;
		ELAN_ERR("device create failed\n");
		goto device_create_fail;
	}

	// Init Char Dev
	cdev_init(&finger->fp_cdev, &efsa120s_fops);
	finger->fp_cdev.owner = THIS_MODULE;
	finger->fp_cdev.ops = &efsa120s_fops;

	// Region Chae dev under /proc/dev
	err = cdev_add(&finger->fp_cdev, devNum, 1);
	if (err < 0) {
		ELAN_ERR("add chr dev failed\n");
		goto cdev_add_fail;
	}

	return 0;

cdev_add_fail:
	device_destroy(finger->fp_class, devNum);
device_create_fail:
	class_destroy(finger->fp_class);
class_create_fail:
	unregister_chrdev_region(devNum,1);
alloc_chrdev_region_fail:

	return err;
}

static int efsa120s_sysfs_create(struct efsa120s_data *prv_data)
{
	struct efsa120s_data *fp = platform_get_drvdata(prv_data->pdev);
	int error = 0;

	mutex_init(&fp->sysfs_mutex);

	/* Register sysfs */
	error = sysfs_create_group(&fp->pdev->dev.kobj, &efsa120s_attr_group);
	if (error) {
		dev_err(&fp->pdev->dev, "[ELAN] Failed to create sysfs attributes, err: %d\n", error);
		goto fail_un;
	}
	return error;
fail_un:
	/* Remove sysfs */
	sysfs_remove_group(&fp->pdev->dev.kobj, &efsa120s_attr_group);

	/* Release Mutex */
	if(&fp->sysfs_mutex)
		mutex_destroy(&fp->sysfs_mutex);

	return error;
}

/* asus pars dt data*/
static int elan_parse_dt(struct device *dev, struct efsa120s_data *pdata)
{

	struct device_node *np = dev->of_node;

	/* +++reset, irq gpio info+++ */
	if(g_ASUS_evbcfgID == IS_EVB)
		;
	else {
		pdata->rst_gpio = of_get_named_gpio_flags(np, "sleep-gpio",
				0, NULL);
		if (pdata->rst_gpio < 0)
			return pdata->rst_gpio;
	}

	pdata->intr_gpio = of_get_named_gpio_flags(np, "irq-gpio",
			0, NULL);
	if (pdata->intr_gpio < 0)
		return pdata->intr_gpio;
	/* ---reset, irq gpio info--- */

	/* ==optional== */
	if(g_ASUS_evbcfgID == IS_EVB)
			;
	else {
		pdata->osvcc_pin = of_get_named_gpio_flags(np, "osvcc-gpio",
				0, NULL);

		pdata->vcc3v3 = of_get_named_gpio_flags(np, "vcc3v3-gpio",
				0, NULL);
	}

	ELAN_INFO("[ELAN] drdy_pin = %d\n", pdata->intr_gpio);

	return 0;
}
/* elan spidri parse dt data*/

static char efsa120s_gpio_config(void *_fp)
{
	struct efsa120s_data *fp = _fp;
	int ret;

	// Configure INT GPIO (Input)
	ret = gpio_request(fp->intr_gpio, "efsa120-irq");
	if (ret < 0)
	{
		ELAN_ERR("[ELAN] %s() IRQ%d request fail, err=0x%x.\n", __func__, fp->intr_gpio, ret);
		ret = -ENODEV;
	} else {
		gpio_direction_input(fp->intr_gpio);
		fp->isr = gpio_to_irq(fp->intr_gpio);
		ELAN_INFO("[ELAN] %s() IRQ%d=%d request success, err=0x%x.\n", __func__,
				fp->intr_gpio, fp->isr, ret);
	}

	// Configure RST GPIO (Output)
	if(g_ASUS_evbcfgID == IS_EVB) {
	} else {
		ret =  gpio_request(fp->rst_gpio, "efsa120-reset");
		if (ret < 0) {
			gpio_free(fp->intr_gpio);
			free_irq(fp->isr, fp);
			ELAN_ERR("[ELAN] %s() RST%d request fail, err=0x%x.\n", __func__, fp->intr_gpio, ret);
			ret = -ENODEV;
		} else {
			ELAN_INFO("[ELAN] %s() RST%d request success, err=0x%x.\n", __func__, fp->rst_gpio, ret);
			gpio_direction_output(fp->rst_gpio, 0);
			mdelay(20);
			gpio_direction_output(fp->rst_gpio, 1);
			mdelay(20);
			ELAN_INFO("[ELAN] %s() Reset ...\n", __func__);
		}
	}

	return ret;
}

#if 0
/* elan spidri power init*/
static int elan_power_on(struct efsa120s_data *pdata, bool on)
{
	int rc;

	if (!on)
		goto power_off;

	rc = regulator_enable(pdata->sovcc);
	if (rc) {
		printk("[%s]Regulator sovcc enable failed rc=%d\n", __func__, rc);
		return rc;
	}

	rc = regulator_enable(pdata->vcc);
	if (rc) {
		printk("[%s]Regulator vcc enable failed rc=%d\n", __func__, rc);
		regulator_disable(pdata->sovcc);
	}

	return rc;

power_off:
	rc = regulator_disable(pdata->sovcc);
	if (rc) {
		printk("[%s]Regulator sovcc disable failed rc=%d\n", __func__, rc);
		return rc;
	}

	rc = regulator_disable(pdata->vcc);
	if (rc) {
		printk("[%s]Regulator vcc disable failed rc=%d\n", __func__, rc);
		rc = regulator_enable(pdata->sovcc);
		if (rc) {
			printk("[%s]Regulator sovcc disable failed rc=%d\n", __func__, rc);
		}
	}

	return rc;
}

static int elan_power_init(struct device *dev,
		struct efsa120s_data *pdata, bool on)
{
	int rc;

	if (!on)
		goto pwr_deinit;
	/* +++ pars regulator+++ */
	pdata->sovcc = devm_regulator_get(dev, "sovcc");
	if (IS_ERR( pdata->sovcc)) {
		rc = PTR_ERR(pdata->sovcc);
		printk("Regulator get elan sovcc  failed rc=%d\n", rc);
		return rc;
	}

	if (regulator_count_voltages(pdata->sovcc) > 0) {
		rc = regulator_set_voltage(pdata->sovcc, 3300000,
				3300000);
		if (rc) {
			printk("Regulator set sovcc failed vdd rc=%d\n", rc);
			goto reg_sovcc_put;
		}
	}

	pdata->vcc = devm_regulator_get(dev, "vcc");
	if (IS_ERR( pdata->vcc)) {
		rc = PTR_ERR(pdata->vcc);
		printk("Regulator get vcc failed rc=%d\n", rc);
		goto reg_vdd_set_vtg;
	}

	if (regulator_count_voltages(pdata->vcc) > 0) {
		rc = regulator_set_voltage(pdata->vcc, 1800000,
				1800000);
		if (rc) {
			printk("Regulator set_vcc failed vdd rc=%d\n", rc);
			goto reg_vcc_i2c_put;
		}
	}
	/* +++ pars regulator+++ */
	return 0;
reg_vcc_i2c_put:
	regulator_put(pdata->vcc);
reg_vdd_set_vtg:
	if (regulator_count_voltages(pdata->sovcc) > 0)
		regulator_set_voltage(pdata->sovcc, 0, 3300000);
reg_sovcc_put:
	regulator_put(pdata->sovcc);
	return rc;

pwr_deinit:
	if (regulator_count_voltages(pdata->sovcc) > 0)
		regulator_set_voltage(pdata->sovcc, 0, 3300000);

	regulator_put(pdata->sovcc);

	if (regulator_count_voltages(pdata->vcc) > 0)
		regulator_set_voltage(pdata->vcc, 0, 1800000);

	regulator_put(pdata->vcc);
	return 0;
}
/* elan spidri power init*/
#endif

static int efsa120s_probe(struct platform_device *pdev)
{
	struct efsa120s_data *fp;
	struct input_dev *input_dev = NULL;
	int err = 0;
	int i;

	ELAN_INFO("%s() %s\n", __func__, VERSION_LOG);
	init_completion(&cmd_done);
	init_completion(&cmd_done_irq);
	if(g_ASUS_evbcfgID == IS_EVB)
		return -ENODEV;
	else {
		if (!asus_match_hw_id(pdev->dev.of_node, 1)) {
			ELAN_ERR("probe failed\n");
			return -ENODEV;
		}
	}

	/* Allocate Device Data */
	fp = devm_kzalloc(&pdev->dev, sizeof(struct efsa120s_data),
			GFP_KERNEL);
	if(!fp) {
		ELAN_ERR("[ELAN] alloc efsa120s data fail.\n");
		goto alloc_mem_fail;
	}

	err = elan_parse_dt(&pdev->dev, fp);

	/* Init Mutex */
	mutex_init(&fp->fp_mutex);
	init_waitqueue_head(&fp->efsa_wait);

	/* Init Input Device */

	/* Init Input Device */
	input_dev = input_allocate_device();
	if (!input_dev) {
		ELAN_ERR("[ELAN] alloc input_dev fail.\r\n");
		err = -19;
		goto input_allocate_device_fail;
	}

	input_dev->name = "efsa120s";
	input_dev->id.bustype = BUS_SPI;
	input_dev->dev.parent = &pdev->dev;
	input_set_drvdata(input_dev, fp);

	input_dev->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY);
	for (i = 0; i < ARRAY_SIZE(FP_KEYCODES); ++i) {
		int report_keycode = FP_KEYCODES[i].keycode;

		input_set_capability(input_dev, EV_KEY, report_keycode);
		__set_bit(report_keycode, input_dev->keybit);
	}

	/* Register Input Device */
	err = input_register_device(input_dev);
	if(err) {
		ELAN_ERR("[FP] Unable to register input device, error: %d!\r\n",
					err);
		err = -20;
		goto input_dev_creat_fail;
	}
	fp->input_dev = input_dev;
	fp->pdev = pdev;

	platform_set_drvdata(pdev, fp);

	if(g_ASUS_evbcfgID == IS_EVB) {
	} else {
		if (fp->vcc3v3 > 0) {
			err = gpio_request_one(fp->vcc3v3, GPIOF_OUT_INIT_HIGH,
					"elan_vcc_3_3");
			if (err)
				ELAN_ERR("[ELAN] open 3v3 fail ! \n");
		}
		msleep(1);

		if (fp->osvcc_pin > 0) {
			err = gpio_request_one(fp->osvcc_pin, GPIOF_OUT_INIT_HIGH,
					"elan_osvcc");
			if (err)
				ELAN_ERR("[ELAN] open 1v8 fail ! \n");
		}
		msleep(1);
	}

	/*
	   if (elan_power_init(&spi->dev, fp, true) < 0)
	   printk("[ELAN] opps elan_power_init fail ! \n");

	   if (elan_power_on(fp, true) < 0)
	   printk("[ELAN] opps elan_power_on fail ! \n");

	   err = gpio_direction_output(fp->osvcc_pin, 1);
	   if (err < 0) {
	   ELAN_ERR("gpio_direction_output osvcc_pin failed\n");
	   err = -EBUSY;
	   }
	   */

	/* Init Sysfs */
	err = efsa120s_sysfs_create(fp);
	if(err < 0) {
		ELAN_ERR("[ELAN] efsa120s sysfs fail.\n");
		goto sysfs_create_fail;
	}

	/* Init Char Device */
	err = efsa120s_setup_cdev(fp);
	if(err < 0) {
		ELAN_ERR("[ELAN] efsa120s setup device fail.\n");
		goto cdev_add_fail;
	}

	/* Init EFSA120S GPIO */
	err = efsa120s_gpio_config(fp);
	if(err < 0) {
		ELAN_ERR("[ELAN] GPIO request fail (%d).\n", err);
		goto gpio_config_fail;
	}

	efsa120s_wq = create_singlethread_workqueue("efsa120s_wq");
	if (!efsa120s_wq)
	{
		ELAN_ERR("[ELAN] Work Create error! \n");
		goto  request_irq_fail;
	}

	INIT_WORK(&fp->work, efsa120s_fp_work_func);

	wake_lock_init(&fp->wake_lock, WAKE_LOCK_SUSPEND, "elan_fp_wakelock_timeout");
	wake_lock_init(&fp->elan_fp_wakelock, WAKE_LOCK_SUSPEND, "elan_fp_wakelock");

	spin_lock_init(&fp->irq_lock); // Added for ISR 2.6.39 later

	/* Init IRQ FUNC */
	err = request_irq(fp->isr, efsa120s_irq_handler,
			IRQF_NO_SUSPEND | IRQF_TRIGGER_RISING, pdev->dev.driver->name, fp);
	//status[0] |= INT_NORMAL_HIGH; // For INT Normal High(Trigger Low), Mark it when Normal low.
	if(err) {
		ELAN_ERR("[ELAN] Failed to request IRQ %d.\n", err);
		goto  request_irq_fail;
	}

	irq_set_irq_wake(fp->isr, 1);
	return 0;

request_irq_fail:

gpio_config_fail:
	// memory have been kfree in efsa120s_kmalloc_image function.

input_dev_creat_fail:
	platform_set_drvdata(pdev, NULL);
	input_free_device(input_dev);
	input_dev = NULL;

cdev_add_fail:
	cdev_del(&fp->fp_cdev);

sysfs_create_fail:

input_allocate_device_fail:

alloc_mem_fail:
	kfree(fp);
	return -ENOMEM;
}

static int  efsa120s_remove(struct platform_device *pdev)
{
	struct efsa120s_data *fp = platform_get_drvdata(pdev);

	if (fp->intr_gpio)
		free_irq(fp->intr_gpio, fp);

	gpio_free(fp->intr_gpio);
	if(g_ASUS_evbcfgID == IS_EVB)
		;
	else {
		gpio_free(fp->rst_gpio);
		if (fp->osvcc_pin > 0)
			gpio_free(fp->osvcc_pin);
		gpio_free(fp->vcc3v3);
	}

	cdev_del(&fp->fp_cdev);
	device_destroy(fp->fp_class, MKDEV(major_number, minor_number));	//delete device node under /dev
	class_destroy(fp->fp_class);
	unregister_chrdev_region(MKDEV(major_number, minor_number), 1);		//delecte class create by bus
	input_free_device(fp->input_dev);

	if(&fp->fp_mutex)
		mutex_destroy(&fp->fp_mutex);

	platform_set_drvdata(pdev, NULL);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int efsa120s_suspend(struct device *dev)
{
	ELAN_INFO("[ELAN] efsa120s suspend!\n");
	return 0;
}

static int efsa120s_resume(struct device *dev)
{
	ELAN_INFO("[ELAN] efsa120s resume!\n");
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(efsa120s_pm_ops, efsa120s_suspend, efsa120s_resume);

#ifdef CONFIG_OF

static struct of_device_id efsa120s_metallica_table[] = {
	{ .compatible = "elan,efsa120s",},
	{ },
};
#else
#define efsa120s_metallica_table NULL
#endif

static struct platform_driver efsa120s_driver = {
	.driver = {
		.name = "efsa120s",
		.owner = THIS_MODULE,
		.of_match_table = efsa120s_metallica_table,
	},
	.probe 	= efsa120s_probe,
	.remove = efsa120s_remove,
};

static int __init efsa120s_init(void)
{
	int status = 0;
	ELAN_INFO("[FP][ELAN] %s \n", __func__);

	status = platform_driver_register(&efsa120s_driver);

	if (status < 0)
		ELAN_ERR("[FP][ELAN] %s FAIL !\n", __func__);

	return status;
}

static void __exit efsa120s_exist(void)
{

	platform_driver_unregister(&efsa120s_driver);
	if(efsa120s_wq)
		destroy_workqueue(efsa120s_wq);
}

module_init(efsa120s_init);
module_exit(efsa120s_exist);

MODULE_AUTHOR("KennyKang <kenny.kang@emc.com.tw>");
MODULE_DESCRIPTION("ELAN SPI FingerPrint eFSA120S driver");
MODULE_VERSION(VERSION_LOG);
MODULE_LICENSE("GPL");
