/*
 *  i2c driver for Microchip cap12LF1552 cap sensor
 */
#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif

#define CAP_DEBUG(fmt, arg...)  \
        printk("CAP12LF1552: [%s] " fmt , __func__ , ##arg)

enum {
    SENSOR_STATUS     = 0x00,
    SLEEP_CONTROL     = 0x01,
    SENSOR1_THRESHOLD = 0x02,
    SENSOR2_THRESHOLD = 0x03,
    SENSOR3_THRESHOLD = 0x04,
    STANDBY_CHANNEL   = 0x05,
    MAX_DURATION      = 0x0b,
    REVISION          = 0x22
};

//sensor status values
#define BACK 2
#define APP_SWITCH 4

//key code for app switch
#define KEY_APP_SWITCH 580

//touch detection threshold for sensors
#define THRESHOLD 0x96

//duration for leds on
#define BRIGHT_DURATION 2

struct cap12LF1552_data {
    struct i2c_client *client;
    struct attribute_group attrs;
    struct workqueue_struct *cap_wq;
    struct delayed_work work;
    struct input_dev *input_back;
    struct input_dev *input_app_switch;
    int enable;
    int det_gpio;
    int power_gpio;
    unsigned char num_reg;
    char *det_gpio_name;
#ifdef CONFIG_FB
    struct notifier_block fb_notifier;
#endif
};

int prev_val;
int irq_enabled;
struct hrtimer vk_timer;
static DEFINE_MUTEX(cap_mtx);

// function declarations
static s32 cap12LF1552_read_reg(struct i2c_client *client, u8 command);
static s32 cap12LF1552_write_reg(struct i2c_client *client, u8 command, u8 value);
static int select_interrupt_pin_status(struct device *dev);
static int cap12LF1552_init_sensor(struct i2c_client *client);
static irqreturn_t cap12LF1552_interrupt_handler(int irq, void *dev);
static int cap12LF1552_config_irq(struct i2c_client *client);
enum hrtimer_restart cap12LF1552_brightness(struct hrtimer *);
static void cap12LF1552_work_function(struct work_struct *work);
static ssize_t show_attrs_handler(struct device *dev,
    struct device_attribute *devattr, char *buf);
static ssize_t store_attrs_handler(struct device *dev,
    struct device_attribute *devattr, const char *buf, size_t count);
static int cap12LF1552_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int cap12LF1552_suspend(struct device *);
static int cap12LF1552_resume(struct device *);

static const struct dev_pm_ops cap1106_pm_ops = {
    .suspend = cap12LF1552_suspend,
    .resume  = cap12LF1552_resume,
};
