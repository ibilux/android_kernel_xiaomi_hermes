/*
 * Gas_Gauge driver for CW2015/2013
 * Copyright (C) 2012, CellWise
 * Copyright (C) 2018 XiaoMi, Inc.
 *
 * Authors: ChenGang <ben.chen@cellwise-semi.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.And this driver depends on
 * I2C and uses IIC bus for communication with the host.
 *
 */

#include <asm/fcntl.h>
#include <asm/uaccess.h>
#include <asm/unistd.h>

#include <cust_charging.h>

#include <linux/delay.h>
#include <linux/fcntl.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/ioctl.h>
#include <linux/irq.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/syscalls.h>
#include <linux/time.h>
#include <linux/unistd.h>
#include <linux/workqueue.h>

#include <mach/board.h>
#include <mach/charging.h>
#include <mach/cw2015_battery.h>
#include <mach/mt_gpio.h>

#define REG_VCELL 0x2
#define REG_SOC 0x4
#define REG_CONFIG 0x8
#define REG_MODE 0xA
#define REG_BATINFO 0x10
#define MODE_SLEEP_MASK (3 << 6)
#define MODE_SLEEP (3 << 6)
#define MODE_NORMAL (0 << 6)
#define MODE_QUICK_START (3 << 4)
#define MODE_RESTART 0xf

#define CONFIG_UPDATE_FLG (1 << 1)
#define ATHD (0 << 3)

#define BATTERY_UP_MAX_CHANGE 420
#define BATTERY_DOWN_CHANGE 60
#define BATTERY_DOWN_MIN_CHANGE_RUN 30
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800

#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE 1800
#define DEVICE_RUN_TIME_FIX_VALUE 40

#define USB_CHARGER_MODE 1
#define AC_CHARGER_MODE	2

#define FILE_PATH "/data/lastsoc"

static struct i2c_client *cw2015_i2c_client;

#define CW2015_DEBUG 1
#define CW2015_TAG "[CW2015]"
#ifdef CW2015_DEBUG
#define CW2015_FUN(f) printk(KERN_INFO CW2015_TAG "%s\n", __FUNCTION__)
#define CW2015_ERR(fmt, args...) printk(KERN_INFO CW2015_TAG "%s" fmt "\n", __FUNCTION__, ##args)
#define CW2015_LOG(fmt, args...) printk(KERN_INFO CW2015_TAG "%s" fmt "\n", __FUNCTION__, ##args)
#endif

#define CW2015_DEV_NAME "CW2015"

static const struct i2c_device_id FG_CW2015_i2c_id[] = {{CW2015_DEV_NAME, 0}, {}};
static struct i2c_board_info __initdata i2c_FG_CW2015 = {I2C_BOARD_INFO(CW2015_DEV_NAME, 0x62)};

int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;
int g_mtk_init_vol = -10;

extern int FG_charging_type;
extern int FG_charging_status;

static u8 config_info_cos[SIZE_BATINFO] = {
    0x17, 0xF3, 0x63, 0x6A, 0x6A, 0x68, 0x68, 0x65, 0x63, 0x60, 0x5B, 0x59, 0x65, 0x5B, 0x46, 0x41,
    0x36, 0x31, 0x28, 0x27, 0x31, 0x35, 0x43, 0x51, 0x1C, 0x3B, 0x0B, 0x85, 0x22, 0x42, 0x5B, 0x82,
    0x99, 0x92, 0x98, 0x96, 0x3D, 0x1A, 0x66, 0x45, 0x0B, 0x29, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52,
    0x82, 0x8C, 0x92, 0x96, 0x54, 0xC2, 0xBA, 0xCB, 0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0xA5, 0x49
};

static u8 config_info_des[SIZE_BATINFO] = {
    0x17, 0xF9, 0x6D, 0x6D, 0x6B, 0x67, 0x65, 0x64, 0x58, 0x6D, 0x6D, 0x48, 0x57, 0x5D, 0x4A, 0x43,
    0x37, 0x31, 0x2B, 0x20, 0x24, 0x35, 0x44, 0x55, 0x20, 0x37, 0x0B, 0x85, 0x2A, 0x4A, 0x56, 0x68,
    0x74, 0x6B, 0x6D, 0x6E, 0x3C, 0x1A, 0x5C, 0x45, 0x0B, 0x30, 0x52, 0x87, 0x8F, 0x91, 0x94, 0x52,
    0x82, 0x8C, 0x92, 0x96, 0x64, 0xB4, 0xDB, 0xCB, 0x2F, 0x7D, 0x72, 0xA5, 0xB5, 0xC1, 0xA5, 0x42
};

int hmi_battery_version = 0;
void hmi_get_battery_version(void)
{
    char *ptr;
    ptr = strstr(saved_command_line, "batversion=");
    ptr += strlen("batversion=");
    hmi_battery_version = simple_strtol(ptr, NULL, 10);
}

struct cw_battery {
    struct i2c_client *client;
    struct workqueue_struct *battery_workqueue;
    struct delayed_work battery_delay_work;

    long sleep_time_capacity_change;
    long run_time_capacity_change;

    long sleep_time_charge_start;
    long run_time_charge_start;

    int usb_online;
    int charger_mode;
    int capacity;
    int voltage;
};

struct cw_store {
    long bts;
    int OldSOC;
    int DetSOC;
    int AlRunFlag;
};

static int PowerResetFlag = -1;

static unsigned int cw_convertData(struct cw_battery *cw_bat, unsigned int ts)
{
    unsigned int i = ts % 4096, n = ts / 4096;
    unsigned int ret = 65536;

    if (i >= 1700) {
        i -= 1700;
        ret = (ret * 3) / 4;
    }

    if (i >= 1700) {
        i -= 1700;
        ret = (ret * 3) / 4;
    }

    if (i >= 789) {
        i -= 789;
        ret = (ret * 7) / 8;
    }

    if (i >= 381) {
        i -= 381;
        ret = (ret * 15) / 16;
    }

    if (i >= 188) {
        i -= 188;
        ret = (ret * 31) / 32;
    }

    if (i >= 188) {
        i -= 188;
        ret = (ret * 31) / 32;
    }

    if (i >= 93) {
        i -= 93;
        ret = (ret * 61) / 64;
    }

    if (i >= 46) {
        i -= 46;
        ret = (ret * 127) / 128;
    }

    if (i >= 23) {
        i -= 23;
        ret = (ret * 255) / 256;
    }

    if (i >= 11) {
        i -= 11;
        ret = (ret * 511) / 512;
    }

    if (i >= 6) {
        i -= 6;
        ret = (ret * 1023) / 1024;
    }

    if (i >= 3) {
        i -= 3;
        ret = (ret * 2047) / 2048;
    }

    if (i >= 3) {
        i -= 3;
        ret = (ret * 2047) / 2048;
    }

    return ret >> n;
}

static int AlgNeed(struct cw_battery *cw_bat, int newSoc, int oldSoc)
{
    if (newSoc - oldSoc > -20 && newSoc - oldSoc < 20)
        return 2; // this is old battery
    else
        return 1; // this is new battery
}

static int cw_algorithm(struct cw_battery *cw_bat,int real_capacity)
{
    struct file *file = NULL;
    struct cw_store st;
    struct inode *inode;
    mm_segment_t old_fs;
    int vmSOC;
    unsigned int utemp,utemp1;
    long timeNow;

    /*0728 Chaman start*/
    static unsigned long timeNow_last  = -1;
    long timeChanged = 0;
    long timeChanged_rtc = 0;
    struct timespec ts;
    /*0728 Chaman end*/

    static int count_fail=0;
    static int SOC_Dvalue = 0;
    static int Time_real3 = 0;
    static int Time_Dvalue = 0;
    static int Join_Fast_Close_SOC = 0;
    struct timespec ktime_ts;
    long suspend_time = 0;
    static unsigned long suspend_time_last  = 0;
    long suspend_time_changed = 0;
    static int mtk_init_vol = -10;


#define USE_MTK_INIT_VOL
//#undef USE_MTK_INIT_VOL

#ifdef USE_MTK_INIT_VOL
    if (mtk_init_vol == -10) {
        printk("Chaman check mtk init soc is not be saved! why??\n");
        return real_capacity;
    }
    printk("Chaman %s %d mtk_init_vol = %d !\n", __FILE__, __LINE__, mtk_init_vol);
#endif

    //CW2015_ERR("cw2015_file_test sizeof(long) = %d sizeof(int) = %d sizeof(st) = %d\n",sizeof(long),sizeof(int),sizeof(st));
    timeNow = get_seconds();
    vmSOC = real_capacity;

    file = filp_open(FILE_PATH,O_RDWR|O_CREAT,0644);

    if (IS_ERR(file)) {
        CW2015_ERR(" error occured while opening file %s,exiting...\n",FILE_PATH);
        return real_capacity;
    }

    old_fs = get_fs();
    set_fs(KERNEL_DS);
    inode = file->f_dentry->d_inode;

    if ((long)(inode->i_size)<(long)sizeof(st)) {
        if (count_fail < 2)
        {
            count_fail++;
            filp_close(file,NULL);
            return real_capacity;
        }

        st.bts = timeNow;
        st.OldSOC = real_capacity;
        st.DetSOC = 0;
        st.AlRunFlag = -1;
        //file->f_pos = 0;
        //vfs_write(file,(char*)&st,sizeof(st),&file->f_pos);
        CW2015_ERR("cw2015_file_test  file size error!\n");
    }
    else
    {
        count_fail=0;
        file->f_pos = 0;
        vfs_read(file,(char*)&st,sizeof(st),&file->f_pos);

        CW2015_ERR(" success opening file, file_path=%s \n", FILE_PATH);
    }

    /*0909 start*/
    get_monotonic_boottime(&ts);
    ktime_get_ts(&ktime_ts);
    suspend_time = ts.tv_sec - ktime_ts.tv_sec;
    if (timeNow_last != -1 && ts.tv_sec > DEVICE_RUN_TIME_FIX_VALUE) {
        suspend_time_changed = suspend_time - suspend_time_last;
        timeChanged_rtc = timeNow - timeNow_last;
        timeChanged = timeNow - timeNow_last -suspend_time_changed;
        printk(KERN_INFO "[FW_2015]suspend_time_changed = \t%ld,timeChanged_rtc = \t%ld, timeChanged = \t%ld\n",
               suspend_time_changed, timeChanged_rtc, timeChanged);
        if (timeChanged < -60 || timeChanged > 60) {
            st.bts = st.bts + timeChanged_rtc;
            CW2015_ERR(" 1 st.bts = \t%ld\n", st.bts);
        }
    }
    timeNow_last = timeNow;
    suspend_time_last = suspend_time;


    if (((st.bts) < 0) || (st.OldSOC > 100) || (st.OldSOC < 0) || (st.DetSOC < -1))
        /*0728 Chaman end*/
    {
        CW2015_ERR("cw2015_file_test  reading file error!\n");
        CW2015_ERR("cw2015_file_test  st.bts = %ld st.OldSOC = %d st.DetSOC = %d st.AlRunFlag = %d  vmSOC = %d  2015SOC=%d\n",st.bts,st.OldSOC,st.DetSOC,st.AlRunFlag,vmSOC,real_capacity);

        st.bts = timeNow;
        st.OldSOC = real_capacity;
        st.DetSOC = 0;
        st.AlRunFlag = -1;
        //CW2015_ERR("cw2015_file_test  reading file error!\n");
    }

    /*Chaman 0729 need check start*/
    if (PowerResetFlag == 1) {
        PowerResetFlag = -1;
#ifdef USE_MTK_INIT_VOL
        if (mtk_init_vol > 4372) {
            st.DetSOC = 100 - real_capacity;
        } else if (mtk_init_vol > 4349) {
            st.DetSOC = 98 - real_capacity;
        } else if (mtk_init_vol > 4325) {
            st.DetSOC = 96 - real_capacity;
        } else if (mtk_init_vol > 4302) {
            st.DetSOC = 94 - real_capacity;
        } else if (mtk_init_vol > 4278) {
            st.DetSOC = 92 - real_capacity;
        } else if (mtk_init_vol > 4255) {
            st.DetSOC = 90 - real_capacity;
        } else if (mtk_init_vol > 4233) {
            st.DetSOC = 88 - real_capacity;
        } else if (mtk_init_vol > 4211) {
            st.DetSOC = 86 - real_capacity;
        } else if (mtk_init_vol > 4189) {
            st.DetSOC = 84 - real_capacity;
        } else if (mtk_init_vol > 4168) {
            st.DetSOC = 82 - real_capacity;
        } else if (mtk_init_vol > 4147) {
            st.DetSOC = 80 - real_capacity;
        } else if (mtk_init_vol > 4126) {
            st.DetSOC = 78 - real_capacity;
        } else if (mtk_init_vol > 4106) {
            st.DetSOC = 76 - real_capacity;
        } else if (mtk_init_vol > 4089) {
            st.DetSOC = 74 - real_capacity;
        } else if (mtk_init_vol > 4071) {
            st.DetSOC = 72 - real_capacity;
        } else if (mtk_init_vol > 4048) {
            st.DetSOC = 70 - real_capacity;
        } else if (mtk_init_vol > 4024) {
            st.DetSOC = 68 - real_capacity;
        } else if (mtk_init_vol > 4001) {
            st.DetSOC = 66 - real_capacity;
        } else if (mtk_init_vol > 3977) {
            st.DetSOC = 64 - real_capacity;
        } else if (mtk_init_vol > 3965) {
            st.DetSOC = 62 - real_capacity;
        } else if (mtk_init_vol > 3953) {
            st.DetSOC = 60 - real_capacity;
        } else if (mtk_init_vol > 3936) {
            st.DetSOC = 58 - real_capacity;
        } else if (mtk_init_vol > 3919) {
            st.DetSOC = 56 - real_capacity;
        } else if (mtk_init_vol > 3901) {
            st.DetSOC = 54 - real_capacity;
        } else if (mtk_init_vol > 3882) {
            st.DetSOC = 52 - real_capacity;
        } else if (mtk_init_vol > 3869) {
            st.DetSOC = 50 - real_capacity;
        } else if (mtk_init_vol > 3857) {
            st.DetSOC = 48 - real_capacity;
        } else if (mtk_init_vol > 3846) {
            st.DetSOC = 46 - real_capacity;
        } else if (mtk_init_vol > 3835) {
            st.DetSOC = 44 - real_capacity;
        } else if (mtk_init_vol > 3827) {
            st.DetSOC = 42 - real_capacity;
        } else if (mtk_init_vol > 3818) {
            st.DetSOC = 40 - real_capacity;
        } else if (mtk_init_vol > 3811) {
            st.DetSOC = 38 - real_capacity;
        } else if (mtk_init_vol > 3804) {
            st.DetSOC = 36 - real_capacity;
        } else if (mtk_init_vol > 3797) {
            st.DetSOC = 34 - real_capacity;
        } else if (mtk_init_vol > 3790) {
            st.DetSOC = 32 - real_capacity;
        } else if (mtk_init_vol > 3786) {
            st.DetSOC = 30 - real_capacity;
        } else if (mtk_init_vol > 3781) {
            st.DetSOC = 28 - real_capacity;
        } else if (mtk_init_vol > 3775) {
            st.DetSOC = 26 - real_capacity;
        } else if (mtk_init_vol > 3770) {
            st.DetSOC = 24 - real_capacity;
        } else if (mtk_init_vol > 3762) {
            st.DetSOC = 22 - real_capacity;
        } else if (mtk_init_vol > 3753) {
            st.DetSOC = 20 - real_capacity;
        } else if (mtk_init_vol > 3742) {
            st.DetSOC = 18 - real_capacity;
        } else if (mtk_init_vol > 3731) {
            st.DetSOC = 16 - real_capacity;
        } else if (mtk_init_vol > 3715) {
            st.DetSOC = 14 - real_capacity;
        } else if (mtk_init_vol > 3699) {
            st.DetSOC = 12 - real_capacity;
        } else if (mtk_init_vol > 3694) {
            st.DetSOC = 10 - real_capacity;
        } else if (mtk_init_vol > 3689) {
            st.DetSOC = 8 - real_capacity;
        } else if (mtk_init_vol > 3681) {
            st.DetSOC = 6 - real_capacity;
        } else if (mtk_init_vol > 3673) {
            st.DetSOC = 4 - real_capacity;
        } else if (mtk_init_vol > 3660) {
            st.DetSOC = 3 - real_capacity;
        } else {
            st.DetSOC = 1;
        }
#else
        if (hmi_battery_version==2) {
            if (real_capacity == 0) {
                st.DetSOC = 1;
            } else if (real_capacity == 1) {
                st.DetSOC = 3;
            } else if (real_capacity == 2) {
                st.DetSOC = 10;
            } else if (real_capacity == 3) {
                st.DetSOC = 19;
            } else if (real_capacity == 4) {
                st.DetSOC = 20;
            } else if (real_capacity == 5) {
                st.DetSOC = 22;
            } else if (real_capacity < 14) {
                st.DetSOC = 23;
            } else if (real_capacity < 21) {
                st.DetSOC = 26;
            } else if (real_capacity < 26) {
                st.DetSOC = 25;
            } else if (real_capacity < 31) {
                st.DetSOC = 23;
            } else if (real_capacity < 36) {
                st.DetSOC = 22;
            } else if (real_capacity < 41) {
                st.DetSOC = 20;
            } else if (real_capacity < 51) {
                st.DetSOC = 16;
            } else if (real_capacity < 61) {
                st.DetSOC = 9;
            } else if (real_capacity < 71) {
                st.DetSOC = 7;
            } else if (real_capacity < 81) {
                st.DetSOC = 8;
            } else if (real_capacity < 88) {
                st.DetSOC = 9;
            } else if (real_capacity < 94) {
                st.DetSOC = 6;
            } else if (real_capacity <= 100) {
                vmSOC = 100;
                st.DetSOC = 100 - real_capacity;
            }
        } else {
            if (real_capacity == 0) {
                st.DetSOC = 1;
            } else if (real_capacity == 1) {
                st.DetSOC = 3;
            } else if (real_capacity == 2) {
                st.DetSOC = 10;
            } else if (real_capacity == 3) {
                st.DetSOC = 20;
            } else if (real_capacity == 4) {
                st.DetSOC = 23;
            } else if (real_capacity == 5) {
                st.DetSOC = 27;
            } else if (real_capacity < 14) {
                st.DetSOC = 28;
            } else if (real_capacity < 21) {
                st.DetSOC = 30;
            } else if (real_capacity < 26) {
                st.DetSOC = 25;
            } else if (real_capacity < 34) {
                st.DetSOC = 20;
            } else if (real_capacity < 39) {
                st.DetSOC = 17;
            } else if (real_capacity < 45) {
                st.DetSOC = 13;
            } else if (real_capacity < 51) {
                st.DetSOC = 12;
            } else if (real_capacity < 61) {
                st.DetSOC = 11;
            } else if (real_capacity < 71) {
                st.DetSOC = 13;
            } else if (real_capacity < 81) {
                st.DetSOC = 11;
            } else if (real_capacity < 90) {
                st.DetSOC = 10;
            } else if (real_capacity <= 100) {
                vmSOC = 100;
                st.DetSOC = 100 - real_capacity;
            }
        }

#endif

        if (AlgNeed(cw_bat, st.DetSOC + real_capacity, st.OldSOC) == 2) {
            st.DetSOC = st.OldSOC - real_capacity + 1;
            CW2015_ERR("st.DetDoc=%d\n", st.DetSOC);
        }

        st.AlRunFlag = 1;
        st.bts = timeNow;
        vmSOC = real_capacity + st.DetSOC;
        CW2015_ERR("cw2015_file_test  PowerResetFlag == 1!\n");
    }

    /*Chaman 0729 need check end*/

    else if (Join_Fast_Close_SOC && (st.AlRunFlag > 0)) {
        if (timeNow >= (Time_real3 + Time_Dvalue)) {
            vmSOC = st.OldSOC - 1;
            Time_real3 = timeNow;
        }
        /*0728 Chaman start*/
        else {
            vmSOC = st.OldSOC;
        }
        /*0728 Chaman end*/
        if (vmSOC == real_capacity)
        {
            st.AlRunFlag = -1;
            CW2015_ERR("cw2015_file_test  algriothm end of decrease acceleration\n");
        }

    }
    else  if (((st.AlRunFlag) >0)&&((st.DetSOC) != 0)) {
        /*caculation */
        /*0728 Chaman start */
        get_monotonic_boottime(&ts);
        if (real_capacity < 1 && cw_bat->charger_mode == 0 && ts.tv_sec > DEVICE_RUN_TIME_FIX_VALUE) { //add 0702 for vmSOC to real_capacity quickly when real_capacity very low
            if (SOC_Dvalue == 0) {
                SOC_Dvalue = st.OldSOC - real_capacity;
                if (SOC_Dvalue == 0)
                {
                    st.AlRunFlag = -1;
                    printk(KERN_INFO "[FG_CW2015]cw2015_file_test  algriothm end of decrease acceleration[2]\n");
                }
                else
                {
                    printk(KERN_INFO "[FG_CW2015]cw2015_file_test  begin of decrease acceleration \n");
                    Time_real3 = timeNow;
                    if ((cw_bat->voltage) < 3480) {
                        Time_Dvalue = 20/(SOC_Dvalue);
                    }
                    else {
                        Time_Dvalue = 90/(SOC_Dvalue);
                    }
                    Join_Fast_Close_SOC = 1;
                    vmSOC = st.OldSOC;
                }
            }
        }/*0728 Chaman end*/
        else
        {
            utemp1 = 32768/(st.DetSOC);
            if ((st.bts)<timeNow)
                utemp = cw_convertData(cw_bat,(timeNow-st.bts));
            else
                utemp = cw_convertData(cw_bat,1);
            CW2015_ERR("cw2015_file_test  convertdata = %d\n",utemp);
            if ((st.DetSOC)<0)
                vmSOC = real_capacity-(int)((((unsigned int)((st.DetSOC)*(-1))*utemp)+utemp1)/65536);
            else
                vmSOC = real_capacity+(int)((((unsigned int)(st.DetSOC)*utemp)+utemp1)/65536);

            if (vmSOC == real_capacity)
            {
                st.AlRunFlag = -1;
                CW2015_ERR("cw2015_file_test  algriothm end\n");
            }
        }
    }
    else
    {
        /*Game over*/
        st.AlRunFlag = -1;
        st.bts = timeNow;
        CW2015_ERR("cw2015_file_test  no algriothm\n");
    }
    //CW2015_ERR("cw2015_file_test  sizeof(st) = %d filesize = %ld time = %d\n ",sizeof(st),(long)(inode->i_size),timeNow);
    //CW2015_ERR("cw2015_file_test  st.bts = %ld st.OldSOC = %d st.DetSOC = %d st.AlRunFlag = %d  vmSOC = %d  2015SOC=%d\n",st.bts,st.OldSOC,st.DetSOC,st.AlRunFlag,vmSOC,real_capacity);
    CW2015_ERR("cw2015_file_test debugdata,\t%ld,\t%d,\t%d,\t%d,\t%d,\t%ld,\t%d,\t%d,\t%d\n",timeNow,cw_bat->capacity,cw_bat->voltage,vmSOC,st.DetSOC,st.bts,st.AlRunFlag,real_capacity,st.OldSOC);
    if (vmSOC>100)
        vmSOC = 100;
    else if (vmSOC<0)
        vmSOC = 0;
    st.OldSOC = vmSOC;
    file->f_pos = 0;
    vfs_write(file,(char*)&st,sizeof(st),&file->f_pos);
    set_fs(old_fs);
    filp_close(file,NULL);
    file = NULL;

    return vmSOC;

}



static int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
    int ret = 0;
    ret = i2c_smbus_read_byte_data(client,reg);
    printk("cw_read buf2 = %d",ret);
    if (ret < 0)
        return ret;
    else {
        buf[0] = ret;
        ret = 0;
    }
    return ret;
}

static int cw_write(struct i2c_client *client, u8 reg, u8 const buf[])
{
    int ret = 0;
    ret =  i2c_smbus_write_byte_data(client,reg,buf[0]);
    return ret;
}

static int cw_read_word(struct i2c_client *client, u8 reg, u8 buf[])
{
    int ret = 0;
    unsigned int data = 0;
    data = i2c_smbus_read_word_data(client, reg);
    buf[0] = data & 0x00FF;
    buf[1] = (data & 0xFF00)>>8;
    return ret;
}

static int cw_update_config_info(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val;
    int i;
    u8 reset_val;

    CW2015_LOG("func: %s-------\n", __func__);
    if (hmi_battery_version == 2)
        CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_des[0]);
    else
        CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_cos[0]);

    ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
    CW2015_LOG("cw_update_config_info reg_val = 0x%x",reg_val);
    if (ret < 0)
        return ret;

    reset_val = reg_val;
    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        CW2015_ERR("Error, device in sleep mode, cannot update battery info\n");
        return -1;
    }

    for (i = 0; i < SIZE_BATINFO; i++) {
        if (hmi_battery_version == 2)
            ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_des[i]);
        else
            ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_cos[i]);

        if (ret < 0) {
            return ret;
        }
    }

    for (i = 0; i < SIZE_BATINFO; i++) {
        ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
        if (hmi_battery_version == 2) {
            if (reg_val != config_info_des[i])
                return -1;
        } else {
            if (reg_val != config_info_cos[i])
                return -1;
        }
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret;

    reg_val |= CONFIG_UPDATE_FLG;
    reg_val &= 0x07;
    reg_val |= ATHD;
    ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret;

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0)
        return ret;

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
        CW2015_LOG("update flag for new battery info have not set..\n");
    }

    if ((reg_val & 0xf8) != ATHD) {
        CW2015_LOG("the new ATHD have not set..\n");
    }

    /* reset */
    reset_val &= ~(MODE_RESTART);
    reg_val = reset_val | MODE_RESTART;
    ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
    if (ret < 0)
        return ret;

    msleep(10);
    ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
    if (ret < 0)
        return ret;
    PowerResetFlag = 1;
    CW2015_ERR("cw2015_file_test  set PowerResetFlag/n ");
    msleep(10);

    return 0;
}

static int cw_init(struct cw_battery *cw_bat)
{
    int ret;
    int i;
    u8 reg_val = MODE_SLEEP;

    hmi_get_battery_version();

    if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
        reg_val = MODE_NORMAL;

        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        if (ret < 0) return ret;
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0) return ret;

    CW2015_LOG("the new ATHD haven't been set, reg_val = 0x%x", reg_val);
    if ((reg_val & 0xf8) != ATHD) {
        CW2015_LOG("the new ATHD haven't been set");
        reg_val &= 0x07;
        reg_val |= ATHD;
        ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
        if (ret < 0) return ret;
    }

    ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
    if (ret < 0) return ret;

    CW2015_LOG("reg_val = %d", reg_val);

    if (!(reg_val & CONFIG_UPDATE_FLG)) {
        CW2015_LOG("update flag for new battery info haven't been set");

        ret = cw_update_config_info(cw_bat);
        if (ret < 0) return ret;
    } else {
        for (i = 0; i < SIZE_BATINFO; i++) {
            ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
            if (ret < 0) return ret;

            if (hmi_battery_version == 2) {
                if (config_info_des[i] != reg_val)
                    break;
            } else {
                if (config_info_cos[i] != reg_val)
                    break;
            }
        }

        if (i != SIZE_BATINFO) {
            CW2015_LOG("update flag for new battery info haven't been set");
            ret = cw_update_config_info(cw_bat);
            if (ret < 0) return ret;
        }
    }

    for (i = 0; i < 30; i++) {
        ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
        if (ret < 0) return ret;
        else if (reg_val <= 0x64) break;

        msleep(100);

        if (i > 25) CW2015_ERR("invalid power error");
    }

    if (i >= 30) {
        reg_val = MODE_SLEEP;
        ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
        CW2015_ERR("invalid power error");
        return -1;
    }

    return 0;
}

static void cw_update_time_member_charge_start(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    cw_bat->run_time_charge_start = new_run_time;
    cw_bat->sleep_time_charge_start = new_sleep_time;
}

static void cw_update_time_member_capacity_change(struct cw_battery *cw_bat)
{
    struct timespec ts;
    int new_run_time;
    int new_sleep_time;

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;

    cw_bat->run_time_capacity_change = new_run_time;
    cw_bat->sleep_time_capacity_change = new_sleep_time;
}

static int cw_get_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity;
    int ret;
    u8 reg_val[2];

    struct timespec ts;
    long new_run_time;
    long new_sleep_time;
    long capacity_or_aconline_time;
    int allow_change;
    int allow_capacity;
    static int if_quickstart = 0;
    static int jump_flag = 0;
    static int reset_loop = 0;
    int charge_time;
    u8 reset_val;

    ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
    if (ret < 0) return ret;

    cw_capacity = reg_val[0];
    if ((cw_capacity < 0) || (cw_capacity > 100)) {
        CW2015_ERR("get cw_capacity error; cw_capacity = %d", cw_capacity);
        reset_loop++;

        if (reset_loop > 5) {
            reset_val = MODE_SLEEP;

            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0) return ret;

            reset_val = MODE_NORMAL;
            msleep(10);

            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0) return ret;

            ret = cw_init(cw_bat);
            if (ret) return ret;

            reset_loop = 0;
        }

        return cw_capacity;
    } else
        reset_loop = 0;

    CW2015_LOG("cw_capacity = %d", cw_capacity);
    cw_capacity = cw_algorithm(cw_bat,cw_capacity);

    ktime_get_ts(&ts);
    new_run_time = ts.tv_sec;

    get_monotonic_boottime(&ts);
    new_sleep_time = ts.tv_sec - new_run_time;
    CW2015_LOG("charger_mode = %d",cw_bat->charger_mode);

    if ((cw_bat->charger_mode == 0) &&
            (cw_capacity > cw_bat->capacity) &&
            (cw_capacity < (cw_bat->capacity + 20))) {
        if (!(cw_capacity == 0 && cw_bat->capacity <= 2))
            cw_capacity = cw_bat->capacity;
    }

    if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {
        capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
        capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
        allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
        if (allow_change > 0) {
            allow_capacity = cw_bat->capacity + allow_change;
            cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
            jump_flag = 1;
        } else if (cw_capacity <= cw_bat->capacity)
            cw_capacity = cw_bat->capacity;
    } else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {
        capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
        capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
        allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
        if (allow_change > 0) {
            allow_capacity = cw_bat->capacity - allow_change;
            if (cw_capacity >= allow_capacity)
                jump_flag = 0;
            else
                cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
        } else if (cw_capacity <= cw_bat->capacity)
            cw_capacity = cw_bat->capacity;
    }

    if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {
        allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
        allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

        allow_capacity = cw_bat->capacity - allow_change;
        cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;

        reset_val = MODE_SLEEP;

        ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
        if (ret < 0) return ret;

        reset_val = MODE_NORMAL;
        msleep(10);

        ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
        if (ret < 0) return ret;

        ret = cw_init(cw_bat);
        if (ret) return ret;
    }

    if ((cw_bat->charger_mode > 0) && (cw_capacity == 0)) {
        charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
        if ((charge_time > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
            reset_val = MODE_SLEEP;

            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret < 0) return ret;

            reset_val = MODE_NORMAL;
            msleep(10);

            ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
            if (ret) return ret;

            ret = cw_init(cw_bat);
            if (ret) return ret;

            if_quickstart = 1;
        }
    } else if ((if_quickstart == 1) && (cw_bat->charger_mode == 0))
        if_quickstart = 0;

    return cw_capacity;
}

static int cw_get_vol(struct cw_battery *cw_bat)
{
    int ret;
    u8 reg_val[2];
    u16 value16, value16_1, value16_2, value16_3;
    int voltage;

    CW2015_FUN();

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if (ret < 0) return ret;
    value16 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if (ret < 0) return ret;
    value16_1 = (reg_val[0] << 8) + reg_val[1];

    ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
    if (ret < 0) return ret;
    value16_2 = (reg_val[0] << 8) + reg_val[1];

    if (value16 > value16_1) {
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    if (value16_1 > value16_2) {
        value16_3 =value16_1;
        value16_1 =value16_2;
        value16_2 =value16_3;
    }

    if (value16 > value16_1) {
        value16_3 = value16;
        value16 = value16_1;
        value16_1 = value16_3;
    }

    voltage = value16_1 * 312 / 1024;
    CW2015_LOG("voltage = %d", voltage);
    return voltage;
}

static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
    int cw_capacity = cw_get_capacity(cw_bat);

    if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
        cw_bat->capacity = cw_capacity;
        cw_update_time_member_capacity_change(cw_bat);

        if (cw_bat->capacity == 0) CW2015_ERR("capacity=0 and will shutdown if it isn't charging");
    }
    CW2015_LOG("cw_capacity = %d", cw_bat->capacity);
}

static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
    int ret = cw_get_vol(cw_bat);
    if ((ret >= 0) && (cw_bat->voltage != ret))
        cw_bat->voltage = ret;
}

static int get_usb_charge_state(struct cw_battery *cw_bat)
{
    int usb_status = 0;

    CW2015_LOG("FG_charging_type = %d", FG_charging_type);
    if (FG_charging_status == 0) {
        usb_status = 0;
        cw_bat->charger_mode = 0;
    } else {
        if (FG_charging_type==STANDARD_HOST) {
            usb_status = 1;
            cw_bat->charger_mode = USB_CHARGER_MODE;
        } else {
            usb_status = 2;
            cw_bat->charger_mode = AC_CHARGER_MODE;
        }
    }
    CW2015_LOG("usb_status = %d, FG_charging_status = %d", usb_status, FG_charging_status);

    return usb_status;
}

static int rk_usb_update_online(struct cw_battery *cw_bat)
{
    int ret = 0;
    int usb_status = 0;

    usb_status = get_usb_charge_state(cw_bat);
    if (usb_status == 2) {
        if (cw_bat->charger_mode != AC_CHARGER_MODE) {
            cw_bat->charger_mode = AC_CHARGER_MODE;
            ret = 1;
        }

        if (cw_bat->usb_online != 1) {
            cw_bat->usb_online = 1;
            cw_update_time_member_charge_start(cw_bat);
        }
    } else if (usb_status == 1) {
        if (cw_bat->charger_mode != USB_CHARGER_MODE) {
            cw_bat->charger_mode = USB_CHARGER_MODE;
            ret = 1;
        }

        if (cw_bat->usb_online != 1) {
            cw_bat->usb_online = 1;
            cw_update_time_member_charge_start(cw_bat);
        }
    } else if (usb_status == 0 && cw_bat->usb_online != 0) {
        cw_bat->charger_mode = 0;
        cw_update_time_member_charge_start(cw_bat);
        cw_bat->usb_online = 0;
        ret = 1;
    }

    return ret;
}

static void cw_bat_work(struct work_struct *work)
{
    struct delayed_work *delay_work;
    struct cw_battery *cw_bat;

    CW2015_FUN();

    delay_work = container_of(work, struct delayed_work, work);
    cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);
    rk_usb_update_online(cw_bat);

    if (cw_bat->usb_online == 1) rk_usb_update_online(cw_bat);

    rk_bat_update_capacity(cw_bat);
    rk_bat_update_vol(cw_bat);
    g_cw2015_capacity = cw_bat->capacity;
    g_cw2015_vol = cw_bat->voltage;

    CW2015_LOG("voltage = %d, capacity = %d", cw_bat->voltage, cw_bat->capacity);

    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(8000));
}

static int cw2015_i2c_detect(struct i2c_client *client, struct i2c_board_info *info)
{
    CW2015_FUN();
    strcpy(info->type, CW2015_DEV_NAME);
    return 0;
}

static int cw2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    struct cw_battery *cw_bat;
    int ret;
    int loop = 0;

    CW2015_FUN();
    mt_set_gpio_mode(GPIO1, 3);
    mt_set_gpio_mode(GPIO2, 3);

    cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
    if (!cw_bat) {
        CW2015_ERR("failed to allocate memory");
        return -ENOMEM;
    }

    i2c_set_clientdata(client, cw_bat);
    cw_bat->client = client;

    ret = cw_init(cw_bat);
    while ((loop++ < 2000) && (ret != 0))
        ret = cw_init(cw_bat);

    if (ret) return ret;

    cw_bat->usb_online = 0;
    cw_bat->charger_mode = 0;
    cw_bat->capacity = 1;
    cw_bat->voltage = 0;

    cw_update_time_member_capacity_change(cw_bat);
    cw_update_time_member_charge_start(cw_bat);

    cw_bat->battery_workqueue = create_singlethread_workqueue("rk_battery");
    INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));

    CW2015_LOG("cw2015/cw2013 driver v1.2 probe sucess");
    return 0;
}

static int  cw2015_i2c_remove(struct i2c_client *client)
{
    struct cw_battery *data = i2c_get_clientdata(client);

    CW2015_FUN();

    cancel_delayed_work(&data->battery_delay_work);
    cw2015_i2c_client = NULL;
    i2c_unregister_device(client);
    kfree(data);

    return 0;
}

static int cw2015_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
    struct cw_battery *cw_bat = i2c_get_clientdata(client);

    CW2015_FUN();
    cancel_delayed_work(&cw_bat->battery_delay_work);

    return 0;
}

static int cw2015_i2c_resume(struct i2c_client *client)
{
    struct cw_battery *cw_bat = i2c_get_clientdata(client);

    CW2015_FUN();
    queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(100));

    return 0;
}

static struct i2c_driver cw2015_i2c_driver = {
    .probe = cw2015_i2c_probe,
    .remove = cw2015_i2c_remove,
    .detect = cw2015_i2c_detect,
    .suspend = cw2015_i2c_suspend,
    .resume = cw2015_i2c_resume,
    .id_table = FG_CW2015_i2c_id,
    .driver = {
        .name = CW2015_DEV_NAME
    },
};

static int __init cw_bat_init(void)
{
    CW2015_FUN();
    i2c_register_board_info(4, &i2c_FG_CW2015, 1);
    if (i2c_add_driver(&cw2015_i2c_driver)) {
        CW2015_ERR("init driver error");
        return -1;
    }

    return 0;
}

static void __exit cw_bat_exit(void)
{
    CW2015_FUN();
    i2c_del_driver(&cw2015_i2c_driver);
}

module_init(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("xhc<xhc@rock-chips.com>");
MODULE_DESCRIPTION("cw2015/cw2013 battery driver");
MODULE_LICENSE("GPL");
