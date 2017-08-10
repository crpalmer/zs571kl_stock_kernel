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

#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/skbuff.h>
#include <linux/time.h>
#include <linux/delay.h>

#include <linux/rwlock_types.h>
#include <linux/completion.h>
#include "anx7688_private_interface.h"
#include "anx7688_public_interface.h"

#include <linux/power_supply.h>
#include <linux/completion.h>
#include <linux/delay.h>

#define LOG_TAG "Anx7688"
extern unsigned char device_addr;
extern struct power_supply *usb_psy;
extern struct completion prswap_completion;
extern int asus_otg_boost_enable(int, bool);
void anx7688_redriver_enable(int);
#ifdef PD_CHARGING_DRIVER_SUPPORT
extern bool asus_is_chg_high_temp(void);
extern bool asus_is_low_battery(void);
extern void asus_allow_5v9v(void);
#endif

/* init setting for TYPE_PWR_SRC_CAP */
static u32 init_src_caps[1] = {
    /*5V, 0.5A, Fixed */
    PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_500MA, PDO_FIXED_FLAGS)
};

/* init setting for TYPE_PWR_SNK_CAP */
static u32 init_snk_cap[3] = {
    /*5V, 2A, Fixed */
    PDO_FIXED(PD_VOLTAGE_5V, PD_CURRENT_2000MA, PDO_FIXED_FLAGS),
    /*9V, 2A, Fixed */
    PDO_FIXED(PD_VOLTAGE_9V, PD_CURRENT_2000MA, PDO_FIXED_FLAGS)
};
/* init setting for TYPE_SVID */
static u8 init_svid[4] = { 0x00, 0x00, 0x01, 0xff };
/* init setting for TYPE_DP_SNK_IDENTITY */
static u8 init_snk_ident[16] = { 0x00, 0x00, 0x00, 0xec,	/*snk_id_hdr */
                                 0x00, 0x00, 0x00, 0x00,	/*snk_cert */
                                 0x00, 0x00, 0x00, 0x00,	/*snk_prd*/
                                 0x39, 0x00, 0x00, 0x51		/*5snk_ama*/
                               };

u8 pd_src_pdo_cnt = 2;
u8 pd_src_pdo[VDO_SIZE] = {
    /*5V 0.9A , 5V 1.5 */
    0x5A, 0x90, 0x01, 0x2A, 0x96, 0x90, 0x01, 0x2A
};

u8 pd_snk_pdo_cnt = 3;
u8 pd_snk_pdo[VDO_SIZE];
u8 pd_rdo[PD_ONE_DATA_OBJECT_SIZE];
u8 DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 configure_DP_caps[PD_ONE_DATA_OBJECT_SIZE];
u8 src_dp_caps[PD_ONE_DATA_OBJECT_SIZE];
unsigned char downstream_pd_cap = 0;

#define INTR_MASK_SETTING 0xC6
/* circular buffer driver */
#define MAX_SEND_BUF_SIZE 8
#define MAX_RECV_BUF_SIZE 8

unsigned char pbuf_rx_front = 0;
unsigned char pbuf_tx_rear = 0;

#define TX_BUF_FRONT 0x11
#define TX_BUF_REAR   0x12
#define TX_BUF_START 0x18	/* 0x18-0x1f */

#define RX_BUF_FRONT 0x13
#define RX_BUF_REAR   0x14
#define RX_BUF_START  0x20	/* 0x20-0x27 */

#define RCVDER_ACK_STATUS 0x15
#define SENDER_ACK_STATUS 0x16


#define tx_buf_rear() pbuf_tx_rear
#define rx_buf_front() pbuf_rx_front

#define rx_buf_rear() ReadReg(RX_BUF_REAR)
#define tx_buf_front() ReadReg(TX_BUF_FRONT)

#define receiver_set_ack_status(val) WriteReg(RCVDER_ACK_STATUS, val)
#define receiver_get_ack_status() ReadReg(RCVDER_ACK_STATUS)

#define sender_get_ack_status() ((ReadReg(SENDER_ACK_STATUS)) & 0x7F)
#define sender_set_ack_status(val) WriteReg(SENDER_ACK_STATUS, val)

#define TRY_ROLE_TIMEOUT  600
#define WAIT_VBUS_TIME  1200
/**
 * @desc:   Interface that change anx7688 as source to charge downstream device
 *
 *
 * @return:  0: success.  1: fail
 *
 */
u8 try_source(void)
{
    unsigned long expire = 0;
    pr_info("Try source start.\n");
    if(!(ReadReg(0x40)&0x08)) {
        pr_info("Current role is DFP, no need Try source\n");
        return 1;
    }

    if(downstream_pd_cap) {
        return interface_pr_swap();
    } else {
        WriteReg(0x05,0x10);  	 // OCM reset
        WriteReg(0x4a,ReadReg(0x4a)|0x01); //cc_soft_en
        //WriteReg(0x47,0x8d);  	 // pull up to 4.7K
        WriteReg(0x6e,0x01);  	 // skip check vbus disable

        WriteReg(0x40, ReadReg(0x40)|0x02); //try DFP
        expire = msecs_to_jiffies(TRY_ROLE_TIMEOUT) + jiffies;

        while(!(ReadReg(0x48)&0x0f))	 {
            if (time_before(expire, jiffies)) {
                pr_info("Try source timeout!0x48 = %x\n", ReadReg(0x48));
                goto try_src_fail;
            }
        }
	asus_otg_boost_enable(1, NULL);
#ifdef SUP_VBUS_CTL
        anx7688_vbus_control(1);
#endif
        WriteReg(0x05,0x0);  //release ocm
        mdelay(50);
        WriteReg(0x6e,0x0);  	 // skip check vbus enable
        if(ReadReg(0x40)&0x08) {
            pr_info("try source swap fail! \n");
            return 1;		      //role swap ng
        } else {
            pr_info("try source swap success! \n");
            return 0; 			// role swap ok
        }
    }
try_src_fail:
    pr_info("try source fail!\n");
    WriteReg(0x40, ReadReg(0x40)&0xfd);  //recover to UFP
    WriteReg(0x05,0x00);  //release ocm
    mdelay(1);
    WriteReg(0x69,0x19);  //vbus off delay time 100ms
    WriteReg(0x6e,0x0);  	 // skip check vbus enable
    return 1;


}

/**
 * @desc:   Interface that change anx7688 as sink  for charging  from downstream device
 *
 *
 * @return:  0: success.  1: fail
 *
 */
u8 try_sink(void)
{
    unsigned long expire = 0;
    pr_info("Try sink start.\n");
    if(ReadReg(0x40)&0x08) {
        pr_info("Current role is UFP, no need Try sink\n");
        return 1;
    }

    if(downstream_pd_cap) {
        return interface_pr_swap();
    } else {
        WriteReg(0x05,0x10);  	 // OCM reset
        WriteReg(0x4a,ReadReg(0x4a) | 0x01); //cc_soft_en
	asus_otg_boost_enable(0, NULL);
#ifdef SUP_VBUS_CTL
        anx7688_vbus_control(0);	//VBUS off
#endif
        WriteReg(0x3f,ReadReg(0x3f) | 0xdf); //VBUS off
        WriteReg(0x36,ReadReg(0x36) | 0x80);//discharge en
        //WriteReg(0x33,ReadReg(0x33) & 0x0f);//Vconn colse

        WriteReg(0x40, ReadReg(0x40) & 0xfd);  //try UFP

        expire = msecs_to_jiffies(TRY_ROLE_TIMEOUT) + jiffies;

        while(!(ReadReg(0x0d) & 0xfc))	 {
            if (time_before(expire, jiffies)) {
                pr_info("Try sink timeout!0x0d = %x\n", ReadReg(0x0d));
                goto try_sink_fail;
            }
        }

        mdelay(650);
        expire = msecs_to_jiffies(WAIT_VBUS_TIME) + jiffies;

        while(!(ReadReg(0x40) & 0x10)) {
            if (time_before(expire, jiffies)) {
                pr_info("wait vbus timeout!0x40 = %x\n", ReadReg(0x40));
                goto try_sink_fail;
            }
        }

        WriteReg(0x36,ReadReg(0x36) & 0x7f); //discharge disable
        WriteReg(0x05,0x00);  //release FW
        if(ReadReg(0x40)&0x08) {
            pr_info("try sink swap success! \n");
            return 0;		      //role swap ok
        } else {
            pr_info("try sink  swap fail! \n");
            return 1; 			  // role swap ng
        }
    }
try_sink_fail:
    pr_info("try sink fail!\n");
    WriteReg(0x40, ReadReg(0x40)|0x02);  //recover to DFP
    WriteReg(0x36,ReadReg(0x36) & 0x7f); //discharge disable
    WriteReg(0x05,0x00);
    return 1;

}

/**
 * @desc:   Interface AP fetch OTP word 1 byte 4,
 *
 *
 * @return:
 *
 */
u8 get_otp_indicator_byte(void)
{
    u8 temp;

    WriteReg(0xe5, 0xa0); //  disable ocm access otp & wake up otp
    WriteReg(0xef, 0x7a); // OTP access key
    WriteReg(0xd0, 0x00);   //high address
    WriteReg(0xd1, 0x01); 		//low address
    WriteReg(0xe5, 0xa1); // otp read start
    while(ReadReg(0xed) & 0x30); // wait for read done
    temp = ReadReg(0xe0);
    WriteReg(0xef, 0x0); // OTP access key clear
    WriteReg(0xe5, 0x0); //enable ocm
    return temp;
}

/**
 * @desc:   The interface AP will get the anx7688's data role
 *
 * @param:  none
 *
 * @return:  data role , dfp 1 , ufp 0, other error: -1, not ready
 *
 */
s8 get_data_role(void)
{
    u8 status;

    /*fetch the data role */
    status = ReadReg(SYSTEM_STSTUS);

    return ((status & DATA_ROLE) != 0);

}

/**
 * @desc:   The interface AP will get the anx7688's power role
 *
 * @param:  none
 *
 * @return:  data role , source 1 , sink 0, other error, -1, not ready
 *
 */
s8 get_power_role(void)
{
    u8 status ;

    /*fetch the power role */
    status = ReadReg(0x40);

    if (status < 0)
	return -1;

    return ((status & 0x08) == 0);
}

/**
 * @desc:   Interface AP fetch the source capability from anx7688
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability in anx7688
 *          src_caps_size: source capability's size
 *
 * @return:  0: success 1: fail
 *
 */
u8 get_src_cap(const u8 *src_caps, u8 src_caps_size)
{

    return 1;
}

/**
 * @desc:   Interface that AP fetch the sink capability from anx7688's downstream device
 *
 * @param:  sink_caps: PDO buffer pointer of sink capability
 *            which will be responsed by anx7688's SINK Capablity Message
 *
 *          snk_caps_len: sink capability max length of the array
 *
 * @return:  sink capability array length>0: success. 1: fail
 *
 */
u8 get_snk_cap(u8 *snk_caps, u8 snk_caps_len)
{

    return 1;
}
/**
 * @desc:   The Interface AP set the source capability to anx7688
 *
 * @param:  pdo_buf: PDO buffer pointer of source capability,
 *                              which can be packed by PDO_FIXED_XXX macro
 *                eg: default5Vsafe src_cap(5V, 0.9A fixed) -->
 *			PDO_FIXED(5000,900, PDO_FIXED_FLAGS)
 *
 *                src_caps_size: source capability's size
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_src_cap(const u8 *src_caps, u8 src_caps_size)
{
    if (NULL == src_caps)
        return CMD_FAIL;
    if ((src_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
        (src_caps_size / PD_ONE_DATA_OBJECT_SIZE) >
        PD_MAX_DATA_OBJECT_NUM) {
        return CMD_FAIL;
    }
    memcpy(pd_src_pdo, src_caps, src_caps_size);
    pd_src_pdo_cnt = src_caps_size / PD_ONE_DATA_OBJECT_SIZE;

    /*send source capabilities message to anx7688 really */
    return interface_send_msg_timeout(TYPE_PWR_SRC_CAP, pd_src_pdo,
                                      pd_src_pdo_cnt *
                                      PD_ONE_DATA_OBJECT_SIZE,
                                      INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to anx7688's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_snk_cap(const u8 *snk_caps, u8 snk_caps_size)
{
    memcpy(pd_snk_pdo, snk_caps, snk_caps_size);
    pd_snk_pdo_cnt = snk_caps_size / PD_ONE_DATA_OBJECT_SIZE;

    /*configure sink cap */
    return interface_send_msg_timeout(TYPE_PWR_SNK_CAP, pd_snk_pdo,
                                      pd_snk_pdo_cnt * 4,
                                      INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the DP's sink capability to anx7688's downstream device
 *
 * @param:  dp_snk_caps: PDO buffer pointer of DP sink capability
 *
 *                dp_snk_caps_size: DP sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_dp_snk_cfg(const u8 *dp_snk_caps, u8 dp_snk_caps_size)
{
    memcpy(configure_DP_caps, dp_snk_caps, dp_snk_caps_size);
    /*configure sink cap */
    return interface_send_msg_timeout(TYPE_DP_SNK_CFG, configure_DP_caps,
                                      4, INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP initialze the DP's capability of anx7688, as source device
 *
 * @param:  dp_caps: DP's capability  pointer of source
 *
 *                dp_caps_size: source DP capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_src_dp_cap(const u8 *dp_caps, u8 dp_caps_size)
{
    if (NULL == dp_caps)
        return CMD_FAIL;
    if ((dp_caps_size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
        (dp_caps_size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
        return CMD_FAIL;
    }

    memcpy(src_dp_caps, dp_caps, dp_caps_size);

    /*configure source DP cap */
    return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
                                      src_dp_caps, dp_caps_size,
                                      INTERFACE_TIMEOUT);
}


/**
 * @desc:   Interface that AP initialze the DP's sink identity of anx7688, as sink device
 *
 * @param:  snk_ident: DP's sink identity
 *
 *                snk_ident_size: DP's sink identity length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_dp_snk_identity(const u8 *snk_ident, u8 snk_ident_size)
{
    return interface_send_msg_timeout(TYPE_DP_SNK_IDENTITY,
                                      (u8 *) snk_ident, snk_ident_size,
                                      INTERFACE_TIMEOUT);
}

/**
 * @desc:   The Interface AP set the VDM packet to anx7688
 *
 * @param:  vdm:  object buffer pointer of VDM
 *
 *                size: vdm packet size
 *
 *@return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_vdm(const u8 *vdm, u8 size)
{
    u8 tmp[32] = { 0 };
    if (NULL == vdm)
        return CMD_FAIL;
    if (size > 3 && size < 32) {
        memcpy(tmp, vdm, size);
        if (tmp[2] == 0x01 && tmp[3] == 0x00) {
            tmp[3] = 0x40;
            return interface_send_msg_timeout(TYPE_VDM, tmp, size,
                                              INTERFACE_TIMEOUT);
        }
    }
    return 1;
}

/**
 * @desc:   The Interface AP set the SVID packet to anx7688
 *
 * @param:  svid:  object buffer pointer of svid
 *
 *                size: svid packet size
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_svid(const u8 *svid, u8 size)
{
    u8 tmp[4] = {
        0
    };
    if (NULL == svid || size != 4)
        return CMD_FAIL;
    memcpy(tmp, svid, size);
    return interface_send_msg_timeout(TYPE_SVID, tmp, size,
                                      INTERFACE_TIMEOUT);
}

/**
 * @desc:   Interface that AP send(configure) the sink capability to anx7688's downstream device
 *
 * @param:  snk_caps: PDO buffer pointer of sink capability
 *
 *                snk_caps_size: sink capability length
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 send_rdo(const u8 *rdo, u8 size)
{
    u8 i;
    if (NULL == rdo)
        return CMD_FAIL;
    if ((size % PD_ONE_DATA_OBJECT_SIZE) != 0 ||
        (size / PD_ONE_DATA_OBJECT_SIZE) > PD_MAX_DATA_OBJECT_NUM) {
        return CMD_FAIL;
    }
    for (i = 0; i < size; i++)
        pd_rdo[i] = *rdo++;

    return interface_send_msg_timeout(TYPE_PWR_OBJ_REQ, pd_rdo, size,
                                      INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send  PR_Swap command to anx7688
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_power_swap(void)
{
    return interface_pr_swap();
}

/**
 * @desc:   The interface AP will send DR_Swap command to anx7688
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_data_swap(void)
{
    return interface_dr_swap();
}

/**
 * @desc:   The interface AP will send accpet command to anx7688
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_accept(void)
{
    return interface_send_msg_timeout(TYPE_ACCEPT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send reject command to anx7688
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_reject(void)
{
    return interface_send_msg_timeout(TYPE_REJECT, 0, 0, INTERFACE_TIMEOUT);
}

/**
 * @desc:   The interface AP will send soft reset command to anx7688
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_soft_reset(void)
{
    return interface_send_soft_rst();
}

/**
 * @desc:   The interface AP will send hard reset command to anx7688
 *
 * @param:  none
 *
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 *
 */
u8 send_hard_reset(void)
{
    return interface_send_hard_rst();
}

char *interface_to_str(unsigned char header_type)
{
    return (header_type == TYPE_PWR_SRC_CAP) ? "src cap" :
           (header_type == TYPE_PWR_SNK_CAP) ? "snk cap" :
           (header_type == TYPE_PWR_OBJ_REQ) ? "RDO" :
           (header_type == TYPE_DP_SNK_IDENTITY) ? "snk identity" :
           (header_type == TYPE_SVID) ? "svid" :
           (header_type == TYPE_PSWAP_REQ) ? "PR_SWAP" :
           (header_type == TYPE_DSWAP_REQ) ? "DR_SWAP" :
           (header_type == TYPE_GOTO_MIN_REQ) ? "GOTO_MIN" :
           (header_type == TYPE_DP_ALT_ENTER) ? "DPALT_ENTER" :
           (header_type == TYPE_DP_ALT_EXIT) ? "DPALT_EXIT" :
           (header_type == TYPE_VCONN_SWAP_REQ) ? "VCONN_SWAP" :
           (header_type == TYPE_GET_DP_SNK_CAP) ? "GET_SINK_DP_CAP" :
           (header_type == TYPE_DP_SNK_CFG) ? "dp cap" :
           (header_type == TYPE_SOFT_RST) ? "Soft Reset" :
           (header_type == TYPE_HARD_RST) ? "Hard Reset" :
           (header_type == TYPE_RESTART) ? "Restart" :
           (header_type == TYPE_PD_STATUS_REQ) ? "PD Status" :
           (header_type == TYPE_ACCEPT) ? "ACCEPT" :
           (header_type == TYPE_REJECT) ? "REJECT" :
           (header_type == TYPE_VDM) ? "VDM" :
           (header_type ==
            TYPE_RESPONSE_TO_REQ) ? "Response to Request" : "Unknown";
}

inline unsigned char cac_checksum(unsigned char *pSendBuf, unsigned char len)
{
    unsigned char i;
    unsigned char checksum;
    checksum = 0;
    for (i = 0; i < len; i++)
        checksum += *(pSendBuf + i);

    return (u8) (0 - checksum);
}

void printb(const char *buf, size_t size)
{
#ifdef OCM_DEBUG
    while (size--)
        printk("%0x ", *buf++);
    printk("\n");
#endif
}

void interface_init(void)
{
    pbuf_rx_front = 0;
    pbuf_tx_rear = 0;
    downstream_pd_cap = 0;
}

void send_initialized_setting(void)
{
    unsigned char send_init_setting_state,c;

    send_init_setting_state=1;

    do {

        switch(send_init_setting_state) {
        default:
        case 0:
            break;
        case 1:
            /* send TYPE_PWR_SRC_CAP init setting */
            send_pd_msg(TYPE_PWR_SRC_CAP, (const char *)init_src_caps,  sizeof(init_src_caps));
            send_init_setting_state++;
            break;
        case 2:
            device_addr= OCM_SLAVE_I2C_ADDR2;
            c= ReadReg( InterfaceSendBuf_Addr);
	    device_addr= OCM_SLAVE_I2C_ADDR1;
            if (c==0) {
                send_init_setting_state++;
            } else
                break;
        case 3:
            /* send TYPE_PWR_SNK_CAP init setting */
            send_pd_msg(TYPE_PWR_SNK_CAP, (const char *)init_snk_cap,
                        sizeof(init_snk_cap));
            send_init_setting_state++;
            break;
        case 4:
            device_addr= OCM_SLAVE_I2C_ADDR2;
            c= ReadReg( InterfaceSendBuf_Addr);
	    device_addr= OCM_SLAVE_I2C_ADDR1;
            if (c==0) {
                send_init_setting_state++;
            } else
                break;
        case 5:
            /* send TYPE_DP_SNK_IDENTITY init setting */
            send_pd_msg(TYPE_DP_SNK_IDENTITY, init_snk_ident,
                        sizeof(init_snk_ident));
            send_init_setting_state++;
            break;
        case 6:
            device_addr= OCM_SLAVE_I2C_ADDR2;
            c= ReadReg( InterfaceSendBuf_Addr);
	    device_addr= OCM_SLAVE_I2C_ADDR1;
            if (c==0) {
                send_init_setting_state++;
            } else
                break;
        case 7:
            /* send TYPE_SVID init setting */
            send_pd_msg(TYPE_SVID, init_svid, sizeof(init_svid));
            send_init_setting_state++;
            break;
        case 8:
        case 9:
            send_init_setting_state = 0;
            break;
        }
    } while(send_init_setting_state!=0);

}

void chip_register_init(void)
{
#ifdef PD_CTS_TEST
    /* modify the below registers for CTS test as specific platform */

    /* tVbusOFF timer :Delay time for pulling down Cable Det pin when CC disconnected
      Timer = (this value * 4)ms */
    //WriteReg(VBUS_DELAY_TIME, 0x7d);

    /*HARD_RESET Delay time for OCM
      Real value: 30ms - (T_TIME_1 & 0x0F)*/
    //WriteReg(T_TIME_1, 0x07);
#else
    /*OHO-439, in AP side, for the interoperability,
     set the try.UFP period to 0x96*2 = 300ms. */
    WriteReg(TRY_UFP_TIMER, 0x96);
#endif

#ifdef SUP_TRY_SRC_SINK
    WriteReg(VBUS_DELAY_TIME, 0x19);
#endif

#ifdef SUP_INT_VECTOR
    /* set interrupt vector mask bit as platform needed  0: eable 1: disable*/
    WriteReg(INTERFACE_INTR_MASK, INTR_MASK_SETTING);
#else
    WriteReg(INTERFACE_INTR_MASK, 0xff);
#endif

#ifdef AUTO_RDO_ENABLE
    WriteReg(AUTO_PD_MODE, ReadReg(AUTO_PD_MODE) | AUTO_PD_ENABLE);
    if (asus_is_chg_high_temp()) {
	//VBUS_5V;
	/*Maximum Voltage in 100mV units*/
	WriteReg(MAX_VOLTAGE_SETTING, 0x32);  /* 5V */
	/*Maximum Power in 500mW units*/ 
	WriteReg(MAX_POWER_SETTING, 0x0a);	 /* 5W */
	/*Minimum Power in 500mW units*/
	WriteReg(MIN_POWER_SETTING, 0x002);  /* 2.5W */
    } else if (asus_is_low_battery()) {
	//VBUS_5V;
	WriteReg(MAX_VOLTAGE_SETTING, 0x32);  /* 5V */
	/*Maximum Power in 500mW units*/ 
	WriteReg(MAX_POWER_SETTING, 0x1e);	 /* 15W */
	/*Minimum Power in 500mW units*/
	WriteReg(MIN_POWER_SETTING, 0x005);  /* 2.5W */
    } else {
	//VBUS_9V;
	/*Maximum Voltage in 100mV units*/
	WriteReg(MAX_VOLTAGE_SETTING, 0x5a);  /* 9V */
	/*Maximum Power in 500mW units*/ 
	WriteReg(MAX_POWER_SETTING, 0x24);	 /* 18W */
	/*Minimum Power in 500mW units*/
	WriteReg(MIN_POWER_SETTING, 0x002);  /* 2.5W */
    }
#endif
}

inline void reciever_reset_queue(void)
{
    rx_buf_front() = rx_buf_rear();
    WriteReg(RX_BUF_FRONT, rx_buf_front());
}

#define STS_DATA_ROLE_CHANGE (((sys_status&DATA_ROLE)!=(sys_sta_bak&DATA_ROLE)) ? DATA_ROLE_CHANGE:0)
#define STS_VCONN_CHANGE (((sys_status&VCONN_STATUS)!=(sys_sta_bak&VCONN_STATUS)) ? VCONN_CHANGE:0)
#define STS_VBUS_CHANGE (((sys_status&VBUS_STATUS)!=(sys_sta_bak&VBUS_STATUS)) ? VBUS_CHANGE:0)
void handle_intr_vector(void)
{
    static unsigned char sys_sta_bak = 0;
    unsigned char sys_status;
    u8 intr_vector = ReadReg(INTERFACE_CHANGE_INT);
    u8 status;

#ifdef OCM_DEBUG_PD
    pr_info(" intr vector = %x\n",  intr_vector);
#endif
    WriteReg(INTERFACE_CHANGE_INT, intr_vector & (~intr_vector));
    if ((~INTR_MASK_SETTING) & intr_vector & RECEIVED_MSG)
        polling_interface_msg(INTERACE_TIMEOUT_MS);
    if ((~INTR_MASK_SETTING) & intr_vector & CC_STATUS_CHANGE) {
        status = ReadReg(NEW_CC_STATUS);
        pd_cc_status_default_func(status);
    }
    sys_status = ReadReg(SYSTEM_STSTUS);

    if ((~INTR_MASK_SETTING) & ((intr_vector & VBUS_CHANGE) | STS_VBUS_CHANGE)) {
        status = ReadReg(SYSTEM_STSTUS);
        pd_vbus_control_default_func(status & VBUS_STATUS);
    }
/*
    if ((~INTR_MASK_SETTING) & ((intr_vector & VCONN_CHANGE) | STS_VCONN_CHANGE)) {
        status = ReadReg(SYSTEM_STSTUS);
        pd_vconn_control_default_func(status & VCONN_STATUS);
    }
*/
    if ((~INTR_MASK_SETTING) & ((intr_vector & DATA_ROLE_CHANGE) | STS_DATA_ROLE_CHANGE)) {
        status = ReadReg(SYSTEM_STSTUS);
        pd_drole_change_default_func(status & DATA_ROLE);
    }
#ifdef AUTO_RDO_ENABLE
    if ((~INTR_MASK_SETTING) & intr_vector & PD_GOT_POWER)
	pd_got_rdo_max_value();
#endif
    sys_sta_bak = sys_status;

    clear_soft_interrupt();


}

/* in Tx's side routine check timeout & cable down */
#define TX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		pr_info("TX Timeout %d\n", msg_total_len);\
		return CMD_FAIL;\
	}	\
} while (0)
#define RX_ROUTINE_CHECK() do {\
	if (time_before(expire, jiffies)) {\
		goto err_timeout;\
	}	\
} while (0)




/* Desc: send interface message to ocm
 * Args: timeout,  block timeout time
@return:  0: success, 1: reject, 2: fail, 3: busy
 */
u8 interface_send_msg_timeout(u8 type, u8 *pbuf, u8 len, int timeout_ms)
{
    unsigned char c,sending_len;
    unsigned char WriteDataBuf[32];


    /* full, return 0 */
    WriteDataBuf[0] = len + 1;	/* cmd */
    WriteDataBuf[1] = type;
    memcpy(WriteDataBuf + 2, pbuf, len);
    /* cmd + checksum */
    WriteDataBuf[len + 2] = cac_checksum(WriteDataBuf, len + 1 + 1);



    sending_len = WriteDataBuf[0] + 2;
    device_addr= OCM_SLAVE_I2C_ADDR2;


    c=ReadReg( InterfaceSendBuf_Addr);
    if (c==0) {
        WriteBlockReg( InterfaceSendBuf_Addr , sending_len, &WriteDataBuf[0]);
    } else {
        pr_info("Tx Buf Full\n");
	return CMD_FAIL;
    }
    device_addr= OCM_SLAVE_I2C_ADDR1;

    return 0;

}

/* Desc: polling private interface interrupt request message
 * Args: timeout,  block timeout time
 * @return:  0: success, 1: reject, 2: fail, 3: busy
 * Interface's Format:
 *	1Byte Len + 1Byte Type + Len Bytes Data + 1Byte checksum   */
u8 polling_interface_msg(int timeout_ms)
{
    unsigned char ReadDataBuf[32];
    unsigned char global_i,checksum ;
    extern unsigned char device_addr;

    device_addr= OCM_SLAVE_I2C_ADDR2;

    ReadBlockReg( InterfaceRecvBuf_Addr, 32, (unsigned char *)ReadDataBuf);

    if (ReadDataBuf[0]!=0) {

        WriteReg(InterfaceRecvBuf_Addr,0);

        device_addr= OCM_SLAVE_I2C_ADDR1;

        checksum = 0;
        for(global_i = 0; global_i < ReadDataBuf[0] + 2; global_i++) {
            checksum += ReadDataBuf[global_i];
        }
        if(checksum == 0) {
#ifdef OCM_DEBUG_PD
            pr_info("\n>>%s\n", interface_to_str(ReadDataBuf[1]));
#endif
            dispatch_rcvd_pd_msg((PD_MSG_TYPE) ReadDataBuf[1], &(ReadDataBuf[2]), ReadDataBuf[0] - 1);
            return CMD_SUCCESS;


        } else {
            pr_info("checksum error! n");
            return CMD_FAIL;
        }

    } else {
        device_addr= OCM_SLAVE_I2C_ADDR1;
    }

    return CMD_FAIL;

}

/* define max request current 3A and voltage 5V */
#define MAX_REQUEST_VOLTAGE 5000
#define MAX_REQUEST_CURRENT 900
#define set_rdo_value(v0, v1, v2, v3)	\
	do {				\
		pd_rdo[0] = (v0);	\
		pd_rdo[1] = (v1);	\
		pd_rdo[2] = (v2);	\
		pd_rdo[3] = (v3);	\
	} while (0)

u8 sel_voltage_pdo_index = 0x02;
#ifdef PD_CHARGING_DRIVER_SUPPORT
#define MAX_POWER 18000000
extern struct workqueue_struct *pd_workqueue;
extern int asus_typec_DFP_type(enum typec_power_supply_type, int, int);
extern void asus_smbchg_usbin_suspend(bool);
extern struct work_struct pdwork;
enum vbus_jump { VBUS_5V, VBUS_9V};
u8 pd_rdo_5v[PD_ONE_DATA_OBJECT_SIZE];
u8 pdo_response;
void usbpd_jump_voltage(void);
extern struct completion rdo_completion;
struct power_data_object *pdo_cal;
struct power_list {
	u32 voltmv;
	u32 currma;
	u8  index;
	u16 pdo_h;
};
struct power_list p_list[2];

struct power_data_object {
	u32 voltage;
	u16 icurrent;
	u32 watt;
	u16 pdo_h;
	u16 pdo_l;
	u8 index;
};

void pd_send_rdo_resutl(void) {

	enum typec_power_supply_type chg_type;
	enum vbus_jump mode;

	if (asus_is_low_battery())
		mode = VBUS_5V;
	else
		mode = VBUS_9V;

	if(asus_is_chg_high_temp())
		mode = VBUS_5V;

	if (pdo_response == CMD_SUCCESS) {
		if(p_list[mode].voltmv > 5000) {
			/*jump voltage call charger usb suspend*/
			asus_smbchg_usbin_suspend(true);
			chg_type = TYPEC_PD_9V_CHARGER;
		} else {
			if(asus_is_chg_high_temp()) {
				/*In high tempture status keep type at PD 9V and 5V/1A*/
				pr_info("PD request success 5V1A\n");
				asus_typec_DFP_type(TYPEC_PD_9V_CHARGER, 5000, 1000);
				return;
			}

			chg_type = TYPEC_PD_5V_CHARGER;
		}

		pr_info("PD request success\n");
		if (p_list[mode].currma == 0)
			asus_typec_DFP_type(chg_type, 5000, 100);//default 5v/0.1A
		else {
			asus_allow_5v9v();
			pr_info("PD votagle = %d, current = %d\n", p_list[mode].voltmv, p_list[mode].currma);
			asus_typec_DFP_type(chg_type, p_list[mode].voltmv, p_list[mode].currma);
		}
	} else
		pr_err("PD request fail\n");

	return;
}

u8 fit_rdo_from_source_caps(u8 obj_cnt, u8 *buf, enum vbus_jump mode)
{
	u8 i = 0;
	u16 pdo_h_tmp, pdo_l_tmp;
	u16 max_request_ma;
	u32 pdo_max, pdo_max_tmp;
	u32 pdo_watt_max, pdo_watt_max_tmp;
	u32 pdo_vol_min_tmp, pdo_vol_min;
	u32 lim_current;;

	pdo_max = 0;
	obj_cnt &= 0x07;
	pdo_watt_max = 0;

	if (mode == VBUS_9V)
		pdo_vol_min = 9000;
	else if (mode == VBUS_5V)
		pdo_vol_min = 5000;
	else
		return 1;

	pdo_cal = (struct power_data_object *) kmalloc(sizeof(struct power_data_object) * obj_cnt, GFP_KERNEL);
	if (!pdo_cal) {
		pr_debug("Failed to malloc memery for pdo\n");
		return 1;
	}
	/* find the max voltage pdo */
	for (i = 0; i < obj_cnt; i++) {
		pdo_l_tmp = buf[i * 4 + 0];
		pdo_l_tmp |= (u16) buf[i * 4 + 1] << 8;
		pdo_h_tmp = buf[i * 4 + 2];
		pdo_h_tmp |= (u16) buf[i * 4 + 3] << 8;

		/* get max voltage now */
		pdo_max_tmp =
		    (u16) (((((pdo_h_tmp & 0xf) << 6) | (pdo_l_tmp >> 10)) &
			    0x3ff) * 50);
		max_request_ma = (u16) ((pdo_l_tmp & 0x3ff) * 10);

		pdo_cal[i].voltage = pdo_max_tmp;
#ifdef OCM_DEBUG_PD
		pr_info("    index=%d Vol=%d, Ia=%d Watt=%d\n", i+1, pdo_max_tmp, max_request_ma, pdo_max_tmp * max_request_ma);
#endif

		if ((pdo_h_tmp & 0xc000) == 0) {
			/*limit the current*/
			if (pdo_max_tmp <= pdo_vol_min) {
				if (pdo_max_tmp <= 6000)
					lim_current = 3000;
				else if (pdo_max_tmp <= 9000)
					lim_current = 2000;
			} else
				lim_current = 0;
		} else
			lim_current = 0;/*pdo type is not fixed supply*/

		max_request_ma = (max_request_ma > lim_current) ? lim_current : max_request_ma;
		pdo_cal[i].pdo_h = pdo_h_tmp;
		pdo_cal[i].pdo_l = pdo_l_tmp;
		pdo_cal[i].icurrent = max_request_ma;
		pdo_cal[i].watt = pdo_max_tmp * max_request_ma;
		pdo_cal[i].index = i;
#ifdef OCM_DEBUG_PD
		pr_info("fit index=%d Vol=%d, Ia=%d Watt=%d\n", i+1, pdo_cal[i].voltage, pdo_cal[i].icurrent, pdo_cal[i].watt);
#endif
	}

	for (i = 0; i < obj_cnt; i++) {
		pdo_watt_max_tmp = pdo_cal[i].watt;
		if (pdo_watt_max_tmp > pdo_watt_max)
			pdo_watt_max = pdo_cal[i].watt;
	}
#ifdef OCM_DEBUG_PD
	pr_info("Watt MAX=%d\n", pdo_watt_max);
#endif
	for (i = 0; i < obj_cnt; i++) {
		if (pdo_watt_max == pdo_cal[i].watt) {
			pdo_vol_min_tmp = pdo_cal[i].voltage;
			if (pdo_vol_min_tmp <= pdo_vol_min) {
				pdo_vol_min = pdo_vol_min_tmp;
				p_list[mode].index = pdo_cal[i].index;
			}
		}
	}
	p_list[mode].voltmv = pdo_cal[p_list[mode].index].voltage;
	p_list[mode].currma = pdo_cal[p_list[mode].index].icurrent;
	p_list[mode].pdo_h = pdo_cal[p_list[mode].index].pdo_h;
#ifdef OCM_DEBUG_PD
	pr_info("chose profile %d VOL=%d MA=%d WAT=%d with mode = %s\n", p_list[mode].index + 1, pdo_cal[p_list[mode].index].voltage, pdo_cal[p_list[mode].index].icurrent, pdo_cal[p_list[mode].index].watt, mode ? "VBUS_9V" : "VBUS_5V");
#endif
	kfree(pdo_cal);
	pdo_cal = NULL;

	return 0;
}

void fit_rdo_value(void){

	enum vbus_jump mode;
	u32 pdo_max;

	if (asus_is_low_battery())
		mode = VBUS_5V;
	else
		mode = VBUS_9V;

	if(asus_is_chg_high_temp())
		mode = VBUS_5V;

	if (p_list[mode].currma != 0) {

		pdo_max =RDO_FIXED(p_list[mode].index + 1, p_list[mode].currma, p_list[mode].currma, 0);
		set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
			      (pdo_max >> 16) & 0xff,
			      (pdo_max >> 24) & 0xff);
	} else {

		pr_info("RDO Mismatch !!!\n");
		set_rdo_value(0x0A, 0x28, 0x00, 0x10);
	}
}

#else
/* default request max RDO */
u8 build_rdo_from_source_caps(u8 obj_cnt, u8 *buf)
{
    u8 i = 0;
    u16 pdo_h, pdo_l, pdo_h_tmp, pdo_l_tmp;
    u16 max_request_ma;
    u32 pdo_max, pdo_max_tmp;

    pdo_max = 0;
    obj_cnt &= 0x07;

    /* find the max voltage pdo */
    for (i = 0; i < obj_cnt; i++) {
        pdo_l_tmp = buf[i * 4 + 0];
        pdo_l_tmp |= (u16) buf[i * 4 + 1] << 8;
        pdo_h_tmp = buf[i * 4 + 2];
        pdo_h_tmp |= (u16) buf[i * 4 + 3] << 8;

        /* get max voltage now */
        pdo_max_tmp =
            (u16) (((((pdo_h_tmp & 0xf) << 6) | (pdo_l_tmp >> 10)) &
                    0x3ff) * 50);
        if (pdo_max_tmp > pdo_max) {
            pdo_max = pdo_max_tmp;
            pdo_l = pdo_l_tmp;
            pdo_h = pdo_h_tmp;
            sel_voltage_pdo_index = i;
        }
    }
#ifdef OCM_DEBUG
    pr_info("maxV=%d, cnt %d index %d\n", pdo_max_tmp, obj_cnt,
            sel_voltage_pdo_index);
#endif
    if ((pdo_h & (3 << 14)) != (PDO_TYPE_BATTERY >> 16)) {
        max_request_ma = (u16) ((pdo_l & 0x3ff) * 10);
#ifdef OCM_DEBUG
        pr_info("maxMa %d\n", max_request_ma);
#endif
        /* less than 900mA */
        if (max_request_ma < MAX_REQUEST_CURRENT) {
            pdo_max =
                RDO_FIXED(sel_voltage_pdo_index + 1, max_request_ma,
                          max_request_ma, 0);
            pdo_max |= RDO_CAP_MISMATCH;
            set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
                          (pdo_max >> 16) & 0xff,
                          (pdo_max >> 24) & 0xff);
            return 1;
        } else {
            pdo_max =
                RDO_FIXED(sel_voltage_pdo_index + 1,
                          MAX_REQUEST_CURRENT, MAX_REQUEST_CURRENT,
                          0);
            set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
                          (pdo_max >> 16) & 0xff,
                          (pdo_max >> 24) & 0xff);

            return 1;
        }
    } else {
        pdo_max =
            RDO_FIXED(sel_voltage_pdo_index + 1, MAX_REQUEST_CURRENT,
                      MAX_REQUEST_CURRENT, 0);
        set_rdo_value(pdo_max & 0xff, (pdo_max >> 8) & 0xff,
                      (pdo_max >> 16) & 0xff, (pdo_max >> 24) & 0xff);
        return 1;
    }

    pr_info("RDO Mismatch !!!\n");
    set_rdo_value(0x0A, 0x28, 0x00, 0x10);

    return 0;
}
#endif
u32 change_bit_order(u8 *pbuf)
{
    return ((u32)pbuf[3] << 24) | ((u32)pbuf[2] << 16)
           | ((u32)pbuf[1] << 8) | pbuf[0];
}
u8 pd_check_requested_voltage(u32 rdo)
{
    int max_ma = rdo & 0x3FF;
    int op_ma = (rdo >> 10) & 0x3FF;
    int idx = rdo >> 28;

    u32 pdo;
    u32 pdo_max;

    if (!idx || idx > pd_src_pdo_cnt) {
        pr_info("rdo = %x, Requested RDO is %d, Provided RDO number is %d\n", rdo, (unsigned int)idx, (unsigned int)pd_src_pdo_cnt);
        return 0; /* Invalid index */
    }
    //Update to pass TD.PD.SRC.E12 Reject Request
    pdo = change_bit_order(pd_src_pdo + ((idx - 1) * 4));
    pdo_max = (pdo & 0x3ff);
#ifdef OCM_DEBUG
    pr_info("pdo_max = %x\n", pdo_max);
#endif
    //TRACE3("Requested  %d/~%d mA, idx %d\n",      (u16)op_ma * 10, (u16)max_ma *10, (u16)idx);
    /* check current ... */
    if (op_ma > pdo_max)//Update to pass TD.PD.SRC.E12 Reject Request
        return 0; /* too much op current */
    if (max_ma > pdo_max)//Update to pass TD.PD.SRC.E12 Reject Request
        return 0; /* too much max current */



    return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_source_caps_default_callback(void *para, u8 para_len)
{
    u8 *pdo = 0;
    u8 ret = 0;
    pdo = (u8 *) para;
    if (para_len % 4 != 0)
        return ret;
#ifdef PD_CHARGING_DRIVER_SUPPORT
	ret =+ fit_rdo_from_source_caps(para_len / 4, para, VBUS_5V);
	ret =+ fit_rdo_from_source_caps(para_len / 4, para, VBUS_9V);
	fit_rdo_value();
	ret =+ interface_send_request();
	if (!ret) {
		reinit_completion(&rdo_completion);
		queue_work(pd_workqueue, &pdwork);
	} else
		pr_err("PD send request failed\n");
#else
    if (build_rdo_from_source_caps(para_len / 4, para)) {
        ret = interface_send_request();
        pr_info("Snd RDO %x %x %x %x succ\n", pd_rdo[0], pd_rdo[1],
                pd_rdo[2], pd_rdo[3]);
    }
#endif
    return ret;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
   * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_sink_caps_default_callback(void *para, u8 para_len)
{
    u8 *pdo = 0;

    pdo = (u8 *) para;
    if (para_len % 4 != 0)
        return 0;
    if (para_len > VDO_SIZE)
        return 0;
    /*do what you want to do */
    return 1;
}

/* Recieve Power Delivery Source Capability message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_pwr_object_req_default_callback(void *para, u8 para_len)
{
    u8 *pdo = (u8 *) para;
    u8 ret = 1;
    u32 rdo = 0;
    if (para_len != 4)
        return ret;

    rdo = pdo[0] | (pdo[1] << 8) | (pdo[2] << 16) | (pdo[3] << 24);
    if (pd_check_requested_voltage(rdo))
        ret = send_accept();
    else
        ret = interface_send_reject();

    return ret;
}

/* Recieve accept message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_accept_default_callback(void *para, u8 para_len)
{

    return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_reject_default_callback(void *para, u8 para_len)
{

    return 1;
}

/* Recieve reject message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through register_default_pd_message_callbacku_func
  *  void *para : should be null
  *   para_len : 0
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_goto_min_default_callback(void *para, u8 para_len)
{

    return 1;
}

void pd_vbus_control_default_func(bool on)
{
    asus_otg_boost_enable((int) on, NULL);
#ifdef OCM_DEBUG_PD
    pr_info("%s : vbus control %d\n", LOG_TAG, (int)on);
#endif
#ifdef SUP_VBUS_CTL
    anx7688_vbus_control(on);
#endif

}

void pd_vconn_control_default_func(bool on)
{
    /* to enable or disable VConn */

}

void pd_cc_status_default_func(u8 cc_status)
{
#ifdef PD_CHARGING_DRIVER_SUPPORT
	switch (cc_status) {
	    case 0xc0:
	    case 0x0c:
		asus_typec_DFP_type(TYPEC_3A_CHARGER, 0, 0);
		break;
	    case 0x80:
	    case 0x08:
		asus_typec_DFP_type(TYPEC_1_5A_CHARGER, 0, 0);
		break;
	    case 0x40:
	    case 0x04:
		asus_typec_DFP_type(TYPEC_UNKNOWN_CHARGER, 0, 0);
		break;
	    case 0x12:
	    case 0x21:
		anx7688_redriver_enable(1);
		break;
	    default:
		break;
	}
#endif
    /* cc status */
#ifdef OCM_DEBUG_PD
    pr_info("cc status %x\n", cc_status);
#endif
}

void pd_drole_change_default_func(bool on)
{
    /* data role changed */
    if (!usb_psy) {
	pr_err("USB power_supply not found\n");
	return;
    } else {
	power_supply_set_usb_otg(usb_psy, on);
    }

#ifdef OCM_DEBUG_PD
    pr_info("%s : ID control %d\n", LOG_TAG, (int)on);
#endif


}

void pd_got_rdo_max_value()
{
	u32 maxV, maxW, maxA;

	maxV = 100 * ReadReg(0x1e);
	maxW = 500 * ReadReg(0x1f);
	maxA = (maxW * 1000 ) / maxV;

	if (maxV > 6000 && maxA > 2000)
		maxA = 2000;
	else if (maxV <= 6000 && maxA > 3000)
		maxA = 3000;

	if (maxV > 5000) {
		asus_typec_DFP_type(TYPEC_PD_9V_CHARGER, maxV, maxA);
	} else {
		asus_typec_DFP_type(TYPEC_PD_5V_CHARGER, maxV, maxA);
	}

	pr_info("anx7688 maxV = %d\n", maxV);
	pr_info("anx7688 maxW = %d\n", maxW);
}

/* Recieve comand response message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  *  void *para :
  *   para_len :
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_cmd_rsp_default_callback(void *para, u8 para_len)
{
    u8 pd_cmd, pd_response;
    pd_cmd =  *(u8 *)para;
    pd_response = *((u8 *)para + 1);


    switch (pd_cmd) {
    case TYPE_PWR_OBJ_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd RDO request result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd RDO reques result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd RDO reques result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd RDO reques result is fail\n");
        else
            pr_info("pd_cmd RDO reques result is unknown\n");

#ifndef AUTO_RDO_ENABLE
#ifdef PD_CHARGING_DRIVER_SUPPORT
	pdo_response = pd_response;
	complete(&rdo_completion);
#endif
#endif
        break;
    case TYPE_VCONN_SWAP_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd VCONN Swap result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd VCONN Swap result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd VCONN Swap result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd VCONN Swap result is fail\n");
        else
            pr_info("pd_cmd VCONN Swap result is unknown\n");
        break;
    case TYPE_DSWAP_REQ:

        if (pd_response == CMD_SUCCESS)
            pr_info("pd_cmd DRSwap result is successful\n");
        else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd DRSwap result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd DRSwap result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd DRSwap result is fail\n");
        else
            pr_info("pd_cmd DRSwap result is unknown\n");
        break;
    case TYPE_PSWAP_REQ:

        if (pd_response == CMD_SUCCESS) {
            pr_info("pd_cmd PRSwap result is successful\n");
	    complete(&prswap_completion);
	} else if (pd_response == CMD_REJECT)
            pr_info("pd_cmd PRSwap result is rejected\n");
        else if (pd_response == CMD_BUSY)
            pr_info("pd_cmd PRSwap result is busy\n");
        else if (pd_response == CMD_FAIL)
            pr_info("pd_cmd PRSwap result is fail\n");
        else
            pr_info("pd_cmd PRSwap result is unknown\n");
        break;
    default:
        break;
    }

    return CMD_SUCCESS;
}

u8 recv_pd_hard_rst_default_callback(void *para, u8 para_len)
{
#ifdef OCM_DEBUG
    pr_info("recv pd hard reset\n");
#endif

    return CMD_SUCCESS;
}

/* Recieve Data Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_dswap_default_callback(void *para, u8 para_len)
{
    /* dswap just notice AP, do nothing */
    return 1;
}

/* Recieve Power Role Swap message's callback function.
  * it can be rewritten by customer just reimmplement this function,
  * through init_pd_msg_callback, it it pd_callback is not 0, using the default
  *  void *para : in this function it means PDO pointer
  *   para_len : means PDO length
 * @return:  0: success, 1: reject, 2: fail, 3: busy
  */
u8 recv_pd_pswap_default_callback(void *para, u8 para_len)
{
    /* pswap just notice AP, do nothing */
    return 1;
}
static pd_callback_t pd_callback_array[256] = { 0 };

pd_callback_t get_pd_callback_fnc(PD_MSG_TYPE type)
{
    pd_callback_t fnc = 0;
    if (type < 256)
        fnc = pd_callback_array[type];
    return fnc;
}

void set_pd_callback_fnc(PD_MSG_TYPE type, pd_callback_t fnc)
{
    pd_callback_array[type] = fnc;
}

void init_pd_msg_callback(void)
{
    u8 i = 0;
    for (i = 0; i < 256; i++)
        pd_callback_array[i] = 0x0;
}

#ifdef PD_CHARGING_DRIVER_SUPPORT
/*do jump*/
void usbpd_jump_voltage(void) {

#ifdef AUTO_RDO_ENABLE
	anx7688_power_standby();
	mdelay(1);
	anx7688_hardware_poweron();
#else
	u8 ret;
	fit_rdo_value();
	ret = interface_send_request();
	if(!ret)
		queue_work(pd_workqueue, &pdwork);
	else
		pr_err("pd send request failed\n");
#endif
	pr_info("anx7688 usbpd jump voltage\n");
}
EXPORT_SYMBOL(usbpd_jump_voltage);
#endif
