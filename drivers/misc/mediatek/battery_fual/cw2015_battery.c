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
#define BAT_CHANGE_ALGORITHM

#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/kernel.h>
#include <linux/i2c.h>
#include <mach/mt_gpio.h>
#include <linux/delay.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <mach/board.h>
#include <mach/cw2015_battery.h>

#ifdef BAT_CHANGE_ALGORITHM
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/syscalls.h>
#include <asm/unistd.h>
#include <asm/uaccess.h>
#include <asm/fcntl.h>
#define FILE_PATH "/data/lastsoc"
#define CPSOC  90
#endif
#include <linux/dev_info.h>//add liuchao
#include <linux/module.h>
#include <linux/unistd.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/ioctl.h>
#include <linux/fcntl.h>
#include <cust_charging.h>
#include <mach/charging.h>

#define REG_VERSION             0x0
#define REG_VCELL               0x2
#define REG_SOC                 0x4
#define REG_RRT_ALERT           0x6
#define REG_CONFIG              0x8
#define REG_MODE                0xA
#define REG_BATINFO             0x10
#define MODE_SLEEP_MASK         (0x3<<6)
#define MODE_SLEEP              (0x3<<6)
#define MODE_NORMAL             (0x0<<6)
#define MODE_QUICK_START        (0x3<<4)
#define MODE_RESTART            (0xf<<0)

#define CONFIG_UPDATE_FLG       (0x1<<1)
#define ATHD                    (0x0<<3)        //ATHD = 0%

#define CW_I2C_SPEED            100000          // default i2c speed set 100khz
#define BATTERY_UP_MAX_CHANGE   420             // the max time allow battery change quantity
#define BATTERY_DOWN_CHANGE   60                // the max time allow battery change quantity
#define BATTERY_DOWN_MIN_CHANGE_RUN 30          // the min time allow battery change quantity when run
#define BATTERY_DOWN_MIN_CHANGE_SLEEP 1800      // the min time allow battery change quantity when run 30min

#define BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE 1800
#define DEVICE_RUN_TIME_FIX_VALUE 40

#define NO_STANDARD_AC_BIG_CHARGE_MODE 1
// #define SYSTEM_SHUTDOWN_VOLTAGE  3400000        //set system shutdown voltage related in battery info.
#define BAT_LOW_INTERRUPT    1

#define USB_CHARGER_MODE        1
#define AC_CHARGER_MODE         2

#define USE_MTK_INIT_VOL

static struct i2c_client *cw2015_i2c_client; /* global i2c_client to support ioctl */
static struct workqueue_struct *cw2015_workqueue;

#define FG_CW2015_DEBUG                0
#define FG_CW2015_TAG                  "[FG_CW2015]"
#if FG_CW2015_DEBUG
#define FG_CW2015_FUN(f)               printk(KERN_ERR FG_CW2015_TAG"%s\n", __FUNCTION__)
#define FG_CW2015_ERR(fmt, args...)    printk(KERN_ERR FG_CW2015_TAG"%s %d : "fmt, __FUNCTION__, __LINE__, ##args)
#define FG_CW2015_LOG(fmt, args...)    printk(KERN_ERR FG_CW2015_TAG fmt, ##args)
#endif
#define CW2015_DEV_NAME     "CW2015"
static const struct i2c_device_id FG_CW2015_i2c_id[] = {{CW2015_DEV_NAME,0},{}};
static struct i2c_board_info __initdata i2c_FG_CW2015={ I2C_BOARD_INFO("CW2015", 0x62)};
static struct i2c_driver FG_CW2015_i2c_driver;
int g_cw2015_capacity = 0;
int g_cw2015_vol = 0;
int g_mtk_init_vol = -10;
extern int FG_charging_type ;
extern int FG_charging_status ;
int CW2015_test_init=0;

extern int Charger_enable_Flag; //add by longcheer_liml_2015_10_12

#define queue_delayed_work_time  8000

/*Chaman add for create sysfs start*/
static int file_sys_state = 1;
/*Chaman add for create sysfs end*/

#ifdef CONFIG_CM865_MAINBOARD
static u8 config_info_cos[SIZE_BATINFO] = { //guangyu coslight 180mv
0x17,0xF6,0x6A,0x6A,0x6D,0x66,0x67,0x63,0x5E,0x63,0x60,0x54,0x5B,0x5A,0x49,0x41,
0x36,0x2E,0x2B,0x20,0x21,0x2E,0x41,0x4E,0x34,0x1D,0x0C,0xCD,0x2C,0x4C,0x4E,0x5D,
0x69,0x65,0x67,0x68,0x3D,0x1A,0x6B,0x40,0x03,0x2B,0x38,0x71,0x84,0x95,0x9F,0x09,
0x36,0x6D,0x96,0xA2,0x5E,0xB3,0xE0,0x70,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x46,0xAE
};
static u8 config_info_sun[SIZE_BATINFO] = {//xinwangda Sunwoda 600mv
0x17,0xEC,0x62,0x6B,0x6A,0x6B,0x67,0x64,0x60,0x63,0x60,0x56,0x5A,0x54,0x49,0x43,
0x36,0x31,0x2B,0x27,0x24,0x2E,0x43,0x4A,0x35,0x20,0x0C,0xCD,0x3C,0x5C,0x56,0x64,
0x6C,0x65,0x66,0x66,0x3E,0x1A,0x64,0x3D,0x04,0x2B,0x2D,0x52,0x83,0x96,0x98,0x13,
0x5F,0x8A,0x92,0xBF,0x46,0xA9,0xD9,0x70,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x46,0xAE
};
static u8 config_info_scud[SIZE_BATINFO] = {//feimaotui Scud   900mV
0x17,0xF0,0x60,0x68,0x6C,0x6A,0x66,0x63,0x60,0x62,0x69,0x50,0x59,0x5B,0x4B,0x42,
0x3B,0x31,0x2B,0x24,0x20,0x32,0x49,0x59,0x17,0x17,0x0C,0xCD,0x2D,0x4D,0x53,0x62,
0x6D,0x60,0x5F,0x61,0x3C,0x1B,0x8E,0x2E,0x02,0x42,0x41,0x4F,0x84,0x96,0x96,0x2C,
0x4E,0x71,0x96,0xC1,0x4C,0xAC,0xE3,0xCB,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0x46,0xAE
};
#else
static u8 config_info_cos[SIZE_BATINFO] = {
0x17,0xF3,0x63,0x6A,0x6A,0x68,0x68,0x65,0x63,0x60,0x5B,0x59,0x65,0x5B,0x46,0x41,
0x36,0x31,0x28,0x27,0x31,0x35,0x43,0x51,0x1C,0x3B,0x0B,0x85,0x22,0x42,0x5B,0x82,
0x99,0x92,0x98,0x96,0x3D,0x1A,0x66,0x45,0x0B,0x29,0x52,0x87,0x8F,0x91,0x94,0x52,
0x82,0x8C,0x92,0x96,0x54,0xC2,0xBA,0xCB,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0xA5,0x49
};
static u8 config_info_des[SIZE_BATINFO] = { //desay
0x17,0xF9,0x6D,0x6D,0x6B,0x67,0x65,0x64,0x58,0x6D,0x6D,0x48,0x57,0x5D,0x4A,0x43,
0x37,0x31,0x2B,0x20,0x24,0x35,0x44,0x55,0x20,0x37,0x0B,0x85,0x2A,0x4A,0x56,0x68,
0x74,0x6B,0x6D,0x6E,0x3C,0x1A,0x5C,0x45,0x0B,0x30,0x52,0x87,0x8F,0x91,0x94,0x52,
0x82,0x8C,0x92,0x96,0x64,0xB4,0xDB,0xCB,0x2F,0x7D,0x72,0xA5,0xB5,0xC1,0xA5,0x42
};
#endif

#ifdef CONFIG_CM865_MAINBOARD
extern int PMIC_IMM_GetOneChannelValue(int dwChannel,int deCount,int trimd);
int hmi_battery_version=0;
void hmi_get_battery_version(void)
{
	int id_volt=0;
	id_volt=PMIC_IMM_GetOneChannelValue(BATTERY_ID_CHANNEL_NUM_PMIC,5,0);	
	printk("[fgauge_get_profile_id]id_vol id_volt= %d\n",id_volt);
	
	if(id_volt !=0)
	{
		if(id_volt < BATTERY_ID_VOLTAGE)
			hmi_battery_version =1;
		else if(id_volt < BATTERY_ID_VOLTAGE_2) 
			hmi_battery_version =2;
		else
			hmi_battery_version =3;
	}else{
		hmi_battery_version =0;
	}
	printk("~~liml_test_hmi_battery_version=%d\n",hmi_battery_version);
}
#else//def BATTERY_SWICTH
int hmi_battery_version=0;
void hmi_get_battery_version(void)
{
	char *ptr;
	ptr =strstr(saved_command_line,"batversion=");
	ptr +=strlen("batversion=");
	hmi_battery_version=simple_strtol(ptr,NULL,10);
	printk("liuchao_test_hmi_battery_version=%d\n",hmi_battery_version);
}
#endif

static struct cw_bat_platform_data cw_bat_platdata = {
	.dc_det_pin = 0,
	.dc_det_level = 0,

	.bat_low_pin = 0,
	.bat_low_level = 0,   
	.chg_ok_pin = 0,
	.chg_ok_level = 0,

	.is_usb_charge = 0,
	.chg_mode_sel_pin = 0,
	.chg_mode_sel_level = 0,

	.cw_bat_config_info = config_info_cos,
};

struct cw_battery {
	struct i2c_client *client;
	struct workqueue_struct *battery_workqueue;
	struct delayed_work battery_delay_work;
	struct delayed_work dc_wakeup_work;
	struct delayed_work bat_low_wakeup_work;
	struct cw_bat_platform_data *plat_data;

	struct power_supply rk_bat;
	struct power_supply rk_ac;
	struct power_supply rk_usb;

	long sleep_time_capacity_change;      // the sleep time from capacity change to present, it will set 0 when capacity change 
	long run_time_capacity_change;

	long sleep_time_charge_start;      // the sleep time from insert ac to present, it will set 0 when insert ac
	long run_time_charge_start;

	int dc_online;
	int usb_online;
	int charger_mode;
	int charger_init_mode;
	int capacity;
	int voltage;
	int status;
	int time_to_empty;
	int alt;

	int bat_change;
};

#ifdef BAT_CHANGE_ALGORITHM
struct cw_store{
	long bts; 
	int OldSOC;
	int DetSOC;
	int AlRunFlag;   
};
#endif

struct cw_battery *CW2015_obj = NULL;
static struct cw_battery *g_CW2015_ptr = NULL;

#ifdef BAT_CHANGE_ALGORITHM
static int PowerResetFlag = -1;
static int alg_run_flag = -1;
#endif

#ifdef BAT_CHANGE_ALGORITHM
static unsigned int cw_convertData(struct cw_battery *cw_bat,unsigned int ts)
{
	unsigned int i = ts%4096,n = ts/4096;
	unsigned int ret = 65536;

	if(i>=1700){i-=1700;ret=(ret*3)/4;}else{}
	if(i>=1700){i-=1700;ret=(ret*3)/4;}else{}
	if(i>=789){i-=789;ret=(ret*7)/8;}else{}
	if(i>=381){i-=381;ret=(ret*15)/16;}else{}  
	if(i>=188){i-=188;ret=(ret*31)/32;}else{}
	if(i>=188){i-=188;ret=(ret*31)/32;}else{}
	if(i>=93){i-=93;ret=(ret*61)/64;}else{}
	if(i>=46){i-=46;ret=(ret*127)/128;}else{}
	if(i>=23){i-=23;ret=(ret*255)/256;}else{}
	if(i>=11){i-=11;ret=(ret*511)/512;}else{}
	if(i>=6){i-=6;ret=(ret*1023)/1024;}else{}
	if(i>=3){i-=3;ret=(ret*2047)/2048;}else{} 
	if(i>=3){i-=3;ret=(ret*2047)/2048;}else{} 
		
	return ret>>n; 
}

static int AlgNeed(struct cw_battery *cw_bat, int SOC_NEW, int SOC_OLD)
{
	printk("Chaman num = %d SOC_NEW = %d   SOC_OLD = %d \n", __LINE__ ,SOC_NEW, SOC_OLD);
	if(SOC_NEW - SOC_OLD > -20 && SOC_NEW - SOC_OLD < 20){
		return 2; // this is old battery
	}else{
		return 1; // this is new battery
	}
}

static int cw_algorithm(struct cw_battery *cw_bat,int real_capacity)
{
	struct file *file = NULL;
	struct cw_store st;
	struct inode *inode;
	mm_segment_t old_fs;
	int fileresult;
	int vmSOC; 
	unsigned int utemp,utemp1;
	long timeNow;
	int count = 0;
	static unsigned long timeNow_last  = -1;
	long timeChanged = 0;
	long timeChanged_rtc = 0;
	struct timespec ts;
	
	static int count_fail=0;
	static int SOC_Dvalue = 0;
	static int Time_real3 = 0;
	static int Time_Dvalue = 0;
	static int Join_Fast_Close_SOC = 0;
	struct timespec ktime_ts;
	long suspend_time = 0;
	static unsigned long suspend_time_last  = 0;
	long suspend_time_changed = 0;
	static int count_time=0;
	static int mtk_init_vol = -10;
	static int return_vmSOC = 0;
	
#ifdef USE_MTK_INIT_VOL
	if(mtk_init_vol == -10 && g_mtk_init_vol != -10){
		mtk_init_vol = g_mtk_init_vol;
		mtk_init_vol = mtk_init_vol - 65;
		printk("Chaman %s %d mtk_init_vol = %d !\n", __FILE__, __LINE__, mtk_init_vol);
	}
	if(mtk_init_vol == -10){
		printk("Chaman check mtk init soc is not be saved! why??\n");
		return real_capacity;
	}
	printk("Chaman %s %d mtk_init_vol = %d !\n", __FILE__, __LINE__, mtk_init_vol);
#endif

	timeNow = get_seconds();
	vmSOC = real_capacity;

	file = filp_open(FILE_PATH,O_RDWR|O_CREAT,0644);
	if(IS_ERR(file))
	{
#if FG_CW2015_DEBUG
		FG_CW2015_ERR(" error occured while opening file %s,exiting...\n",FILE_PATH);
#endif
		return real_capacity;
	}

	old_fs = get_fs();
	set_fs(KERNEL_DS); 
	inode = file->f_dentry->d_inode;

	if((long)(inode->i_size)<(long)sizeof(st))
	{
		if(count_fail < 2)
		{
			count_fail++;
			filp_close(file,NULL);
			return real_capacity;
		}
		st.bts = timeNow;
		st.OldSOC = real_capacity;
		st.DetSOC = 0;
		st.AlRunFlag = -1; 
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("cw2015_file_test  file size error!\n");
#endif
	}
	else
	{
		count_fail=0;
		file->f_pos = 0;
		vfs_read(file,(char*)&st,sizeof(st),&file->f_pos);

#if FG_CW2015_DEBUG
		FG_CW2015_ERR(" success opening file, file_path=%s \n", FILE_PATH);
#endif
	}

	get_monotonic_boottime(&ts);
	ktime_get_ts(&ktime_ts);
	suspend_time = ts.tv_sec - ktime_ts.tv_sec;
	if(timeNow_last != -1 && ts.tv_sec > DEVICE_RUN_TIME_FIX_VALUE){
		suspend_time_changed = suspend_time - suspend_time_last;
		timeChanged_rtc = timeNow - timeNow_last;
		timeChanged = timeNow - timeNow_last -suspend_time_changed;
		printk(KERN_INFO "[FW_2015]suspend_time_changed = \t%ld,timeChanged_rtc = \t%ld, timeChanged = \t%ld\n",
		suspend_time_changed, timeChanged_rtc, timeChanged);
		if(timeChanged < -60 || timeChanged > 60){
			st.bts = st.bts + timeChanged_rtc;
#if FG_CW2015_DEBUG
			FG_CW2015_ERR(" 1 st.bts = \t%ld\n", st.bts);
#endif
		}
	}
	timeNow_last = timeNow;
	suspend_time_last = suspend_time;

	if(((st.bts) < 0) || (st.OldSOC > 100) || (st.OldSOC < 0) || (st.DetSOC < -1)) 
	{
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("cw2015_file_test  reading file error!\n"); 
		FG_CW2015_ERR("cw2015_file_test  st.bts = %ld st.OldSOC = %d st.DetSOC = %d st.AlRunFlag = %d  vmSOC = %d  2015SOC=%d\n",st.bts,st.OldSOC,st.DetSOC,st.AlRunFlag,vmSOC,real_capacity); 
#endif
		st.bts = timeNow;
		st.OldSOC = real_capacity;
		st.DetSOC = 0;
		st.AlRunFlag = -1; 
	}

	if(PowerResetFlag == 1)
	{
		PowerResetFlag = -1;
#ifdef USE_MTK_INIT_VOL
		if(mtk_init_vol > 4372){
			st.DetSOC = 100 - real_capacity;
		}else if(mtk_init_vol > 4349){
			st.DetSOC = 98 - real_capacity;
		}else if(mtk_init_vol > 4325){
			st.DetSOC = 96 - real_capacity;
		}else if(mtk_init_vol > 4302){
			st.DetSOC = 94 - real_capacity;
		}else if(mtk_init_vol > 4278){
			st.DetSOC = 92 - real_capacity;
		}else if(mtk_init_vol > 4255){
			st.DetSOC = 90 - real_capacity;
		}else if(mtk_init_vol > 4233){
			st.DetSOC = 88 - real_capacity;
		}else if(mtk_init_vol > 4211){
			st.DetSOC = 86 - real_capacity;
		}else if(mtk_init_vol > 4189){
			st.DetSOC = 84 - real_capacity;
		}else if(mtk_init_vol > 4168){
			st.DetSOC = 82 - real_capacity;
		}else if(mtk_init_vol > 4147){
			st.DetSOC = 80 - real_capacity;
		}else if(mtk_init_vol > 4126){
			st.DetSOC = 78 - real_capacity;
		}else if(mtk_init_vol > 4106){
			st.DetSOC = 76 - real_capacity;
		}else if(mtk_init_vol > 4089){
			st.DetSOC = 74 - real_capacity;
		}else if(mtk_init_vol > 4071){
			st.DetSOC = 72 - real_capacity;
		}else if(mtk_init_vol > 4048){
			st.DetSOC = 70 - real_capacity;
		}else if(mtk_init_vol > 4024){
			st.DetSOC = 68 - real_capacity;
		}else if(mtk_init_vol > 4001){
			st.DetSOC = 66 - real_capacity;
		}else if(mtk_init_vol > 3977){
			st.DetSOC = 64 - real_capacity;
		}else if(mtk_init_vol > 3965){
			st.DetSOC = 62 - real_capacity;
		}else if(mtk_init_vol > 3953){
			st.DetSOC = 60 - real_capacity;
		}else if(mtk_init_vol > 3936){
			st.DetSOC = 58 - real_capacity;
		}else if(mtk_init_vol > 3919){
			st.DetSOC = 56 - real_capacity;
		}else if(mtk_init_vol > 3901){
			st.DetSOC = 54 - real_capacity;
		}else if(mtk_init_vol > 3882){
			st.DetSOC = 52 - real_capacity;
		}else if(mtk_init_vol > 3869){
			st.DetSOC = 50 - real_capacity;
		}else if(mtk_init_vol > 3857){
			st.DetSOC = 48 - real_capacity;
		}else if(mtk_init_vol > 3846){
			st.DetSOC = 46 - real_capacity;
		}else if(mtk_init_vol > 3835){
			st.DetSOC = 44 - real_capacity;
		}else if(mtk_init_vol > 3827){
			st.DetSOC = 42 - real_capacity;
		}else if(mtk_init_vol > 3818){
			st.DetSOC = 40 - real_capacity;
		}else if(mtk_init_vol > 3811){
			st.DetSOC = 38 - real_capacity;
		}else if(mtk_init_vol > 3804){
			st.DetSOC = 36 - real_capacity;
		}else if(mtk_init_vol > 3797){
			st.DetSOC = 34 - real_capacity;
		}else if(mtk_init_vol > 3790){
			st.DetSOC = 32 - real_capacity;
		}else if(mtk_init_vol > 3786){
			st.DetSOC = 30 - real_capacity;
		}else if(mtk_init_vol > 3781){
			st.DetSOC = 28 - real_capacity;
		}else if(mtk_init_vol > 3775){
			st.DetSOC = 26 - real_capacity;
		}else if(mtk_init_vol > 3770){
			st.DetSOC = 24 - real_capacity;
		}else if(mtk_init_vol > 3762){
			st.DetSOC = 22 - real_capacity;
		}else if(mtk_init_vol > 3753){
			st.DetSOC = 20 - real_capacity;
		}else if(mtk_init_vol > 3742){
			st.DetSOC = 18 - real_capacity;
		}else if(mtk_init_vol > 3731){
			st.DetSOC = 16 - real_capacity;
		}else if(mtk_init_vol > 3715){
			st.DetSOC = 14 - real_capacity;
		}else if(mtk_init_vol > 3699){
			st.DetSOC = 12 - real_capacity;
		}else if(mtk_init_vol > 3694){
			st.DetSOC = 10 - real_capacity;
		}else if(mtk_init_vol > 3689){
			st.DetSOC = 8 - real_capacity;
		}else if(mtk_init_vol > 3681){
			st.DetSOC = 6 - real_capacity;
		}else if(mtk_init_vol > 3673){
			st.DetSOC = 4 - real_capacity;
		}else if(mtk_init_vol > 3660){
			st.DetSOC = 3 - real_capacity;
		}else{
			st.DetSOC = 1;
		}
#else 
		if(hmi_battery_version==2){
			if(real_capacity == 0){
				st.DetSOC = 1;
			}else if(real_capacity == 1){
				st.DetSOC = 3;
			}else if(real_capacity == 2){
				st.DetSOC = 10;
			}else if(real_capacity == 3){
				st.DetSOC = 19;
			}else if(real_capacity == 4){
				st.DetSOC = 20;
			}else if(real_capacity == 5){
				st.DetSOC = 22;
			}else if(real_capacity < 14){
				st.DetSOC = 23;
			}else if(real_capacity < 21){
				st.DetSOC = 26;
			}else if(real_capacity < 26){
				st.DetSOC = 25;
			}else if(real_capacity < 31){
				st.DetSOC = 23;
			}else if(real_capacity < 36){
				st.DetSOC = 22;
			}else if(real_capacity < 41){
				st.DetSOC = 20;
			}else if(real_capacity < 51){
				st.DetSOC = 16;
			}else if(real_capacity < 61){
				st.DetSOC = 9;
			}else if(real_capacity < 71){
				st.DetSOC = 7;
			}else if(real_capacity < 81){
				st.DetSOC = 8;
			}else if(real_capacity < 88){
				st.DetSOC = 9;
			}else if(real_capacity < 94){
				st.DetSOC = 6;
			}else if(real_capacity <= 100){
				vmSOC = 100;
				st.DetSOC = 100 - real_capacity;
			}
		}else{
			if(real_capacity == 0){
				st.DetSOC = 1;
			}else if(real_capacity == 1){
				st.DetSOC = 3;
			}else if(real_capacity == 2){
				st.DetSOC = 10;
			}else if(real_capacity == 3){
				st.DetSOC = 20;
			}else if(real_capacity == 4){
				st.DetSOC = 23;
			}else if(real_capacity == 5){
				st.DetSOC = 27;
			}else if(real_capacity < 14){
				st.DetSOC = 28;
			}else if(real_capacity < 21){
				st.DetSOC = 30;
			}else if(real_capacity < 26){
				st.DetSOC = 25;
			}else if(real_capacity < 34){
				st.DetSOC = 20;
			}else if(real_capacity < 39){
				st.DetSOC = 17;
			}else if(real_capacity < 45){
				st.DetSOC = 13;
			}else if(real_capacity < 51){
				st.DetSOC = 12;
			}else if(real_capacity < 61){
				st.DetSOC = 11;
			}else if(real_capacity < 71){
				st.DetSOC = 13;
			}else if(real_capacity < 81){
				st.DetSOC = 11;
			}else if(real_capacity < 90){
				st.DetSOC = 10;
			}else if(real_capacity <= 100){
				vmSOC = 100;
				st.DetSOC = 100 - real_capacity;
			}
		}
#endif

		if(AlgNeed(cw_bat, st.DetSOC + real_capacity, st.OldSOC) == 2){
			st.DetSOC = st.OldSOC - real_capacity + 1;
#if FG_CW2015_DEBUG
			FG_CW2015_ERR("st.DetDoc=%d\n", st.DetSOC);
#endif
		}

		st.AlRunFlag = 1;
		st.bts = timeNow;
		vmSOC = real_capacity + st.DetSOC;
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("cw2015_file_test  PowerResetFlag == 1!\n");
#endif
	}

	else if(Join_Fast_Close_SOC && (st.AlRunFlag > 0)){
		if(timeNow >= (Time_real3 + Time_Dvalue)){
			vmSOC = st.OldSOC - 1;
			Time_real3 = timeNow;
		}
		else{
			vmSOC = st.OldSOC;
		}
		if (vmSOC == real_capacity)
		{
			st.AlRunFlag = -1;
#if FG_CW2015_DEBUG
			FG_CW2015_ERR("cw2015_file_test  algriothm end of decrease acceleration\n");
#endif
		}
	}
	else if(((st.AlRunFlag) >0)&&((st.DetSOC) != 0))
	{
		get_monotonic_boottime(&ts);
		if(real_capacity < 1 && cw_bat->charger_mode == 0 && ts.tv_sec > DEVICE_RUN_TIME_FIX_VALUE){
			if (SOC_Dvalue == 0){
				SOC_Dvalue = st.OldSOC - real_capacity;
				if(SOC_Dvalue == 0)
				{
					st.AlRunFlag = -1;
					printk(KERN_INFO "[FG_CW2015]cw2015_file_test  algriothm end of decrease acceleration[2]\n");
				}
				else
				{
					printk(KERN_INFO "[FG_CW2015]cw2015_file_test  begin of decrease acceleration \n");
					Time_real3 = timeNow;
				 	if((cw_bat->voltage) < 3480){
						Time_Dvalue = 20/(SOC_Dvalue);
				 	}
				 	else{
						Time_Dvalue = 90/(SOC_Dvalue);
					}
					Join_Fast_Close_SOC = 1;
					vmSOC = st.OldSOC;
				}
			}		
		}
		else
		{
			utemp1 = 32768/(st.DetSOC);
			if((st.bts)<timeNow)
				utemp = cw_convertData(cw_bat,(timeNow-st.bts));
			else
				utemp = cw_convertData(cw_bat,1);
#if FG_CW2015_DEBUG
			FG_CW2015_ERR("cw2015_file_test  convertdata = %d\n",utemp);
#endif
			if((st.DetSOC)<0)
				vmSOC = real_capacity-(int)((((unsigned int)((st.DetSOC)*(-1))*utemp)+utemp1)/65536);
			else
				vmSOC = real_capacity+(int)((((unsigned int)(st.DetSOC)*utemp)+utemp1)/65536);

			if (vmSOC == real_capacity)
			{
				st.AlRunFlag = -1;
#if FG_CW2015_DEBUG
				FG_CW2015_ERR("cw2015_file_test  algriothm end\n");
#endif
			}
		}
	}
	else
	{
		st.AlRunFlag = -1;
		st.bts = timeNow;
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("cw2015_file_test  no algriothm\n");
#endif
	}
#if FG_CW2015_DEBUG
	FG_CW2015_ERR("cw2015_file_test debugdata,\t%ld,\t%d,\t%d,\t%d,\t%d,\t%ld,\t%d,\t%d,\t%d\n",timeNow,cw_bat->capacity,cw_bat->voltage,vmSOC,st.DetSOC,st.bts,st.AlRunFlag,real_capacity,st.OldSOC);
#endif
	alg_run_flag = st.AlRunFlag;

	if(vmSOC>100)
		vmSOC = 100;
	else if(vmSOC<0)    
		vmSOC = 0;
	st.OldSOC = vmSOC;
	file->f_pos = 0;
	vfs_write(file,(char*)&st,sizeof(st),&file->f_pos);
	set_fs(old_fs);
	filp_close(file,NULL);
	file = NULL;

	if(return_vmSOC < 5){
		if(return_vmSOC > 0){
			file_sys_state = 2;
			printk("cw2015 return_vmsoc=%d, file_sys_state=%d \n", return_vmSOC, file_sys_state);
		}
		return_vmSOC++;
	}
	return vmSOC;

}
#endif

static int cw_read(struct i2c_client *client, u8 reg, u8 buf[])
{
	int ret = 0;

	ret = i2c_smbus_read_byte_data(client,reg);
	printk("cw_read buf2 = %d",ret);
	if (ret < 0)
	{
	return ret;
	}
	else
	{
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

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("func: %s-------\n", __func__);
#endif

#if FG_CW2015_DEBUG
#ifdef CONFIG_CM865_MAINBOARD
	if(hmi_battery_version==2)
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_sun[0]);
	else if(hmi_battery_version==3)
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_scud[0]);
	else
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_cos[0]);      
#else
	if(hmi_battery_version==2)
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_des[0]);//liuchao
	else
		FG_CW2015_LOG("test cw_bat_config_info = 0x%x",config_info_cos[0]);//liuchao
#endif
#endif
	/* make sure no in sleep mode */
	ret = cw_read(cw_bat->client, REG_MODE, &reg_val);
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw_update_config_info reg_val = 0x%x",reg_val);
#endif
	if (ret < 0)
		return ret;

	reset_val = reg_val;
	if((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) {
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("Error, device in sleep mode, cannot update battery info\n");
#endif
		return -1;
	}

	/* update new battery info */
	for (i = 0; i < SIZE_BATINFO; i++) {
#ifdef CONFIG_CM865_MAINBOARD  
		if(hmi_battery_version==2)
			ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_sun[i]);
		else if(hmi_battery_version==3)
			ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_scud[i]);
		else
			ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_cos[i]);
		if (ret < 0)
			return ret;
	}

	/* readback & check */
	for (i = 0; i < SIZE_BATINFO; i++)
	{
		ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
		if(hmi_battery_version==2)
		{
			if (reg_val != config_info_sun[i])
			return -1;
		}
		else if(hmi_battery_version==3)
		{
			if (reg_val != config_info_scud[i])
			return -1;
		}
		else
		{
			if (reg_val != config_info_cos[i])
			return -1;
		}	
	}
#else
	if(hmi_battery_version==2)
		ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_des[i]);
	else
		ret = cw_write(cw_bat->client, REG_BATINFO + i, &config_info_cos[i]);
		
	if (ret < 0)
		return ret;
	}
	/* readback & check */
	for (i = 0; i < SIZE_BATINFO; i++)
	{
		ret = cw_read(cw_bat->client, REG_BATINFO + i, &reg_val);
		if(hmi_battery_version==2)
		{
			if (reg_val != config_info_des[i])
			return -1;
		}
		else
		{
			if (reg_val != config_info_cos[i])
			return -1;
		}
	}
#endif

	/* set cw2015/cw2013 to use new battery info */
	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	reg_val |= CONFIG_UPDATE_FLG;   /* set UPDATE_FLAG */
	reg_val &= 0x07;                /* clear ATHD */
	reg_val |= ATHD;                /* set ATHD */
	ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	/* check 2015/cw2013 for ATHD & update_flag */ 
	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

	if (!(reg_val & CONFIG_UPDATE_FLG)) {
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("update flag for new battery info have not set..\n");
#endif
	}

	if ((reg_val & 0xf8) != ATHD) {
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("the new ATHD have not set..\n");
#endif
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
#ifdef BAT_CHANGE_ALGORITHM
	PowerResetFlag = 1;
#if FG_CW2015_DEBUG
	FG_CW2015_ERR("cw2015_file_test  set PowerResetFlag/n ");
#endif
#endif
	msleep(10);

	return 0;
}

static int cw_init(struct cw_battery *cw_bat)
{
	int ret;
	int i;
	u8 reg_val = MODE_SLEEP;
	hmi_get_battery_version();//liuchao

	struct devinfo_struct *dev = (struct devinfo_struct*)kmalloc(sizeof(struct devinfo_struct), GFP_KERNEL);
	dev->device_type = "Battery";
	dev->device_vendor = DEVINFO_NULL;
	dev->device_ic = DEVINFO_NULL;
	dev->device_version = DEVINFO_NULL;
	if(hmi_battery_version==1)
		dev->device_module = "Cos";
	else if(hmi_battery_version==2)
		dev->device_module = "Des";
	else
		dev->device_module = "ERROR";
	dev->device_info = DEVINFO_NULL;
	dev->device_used = DEVINFO_USED;
	DEVINFO_CHECK_ADD_DEVICE(dev);

     //   printk("cw2015_init_-%d\n",__LINE__);
	if ((reg_val & MODE_SLEEP_MASK) == MODE_SLEEP) 
	{
		reg_val = MODE_NORMAL;

		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
		if (ret < 0)
			return ret;
				 
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		return ret;

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("the new ATHD have not set reg_val = 0x%x\n",reg_val);
#endif
	if ((reg_val & 0xf8) != ATHD) 
	{
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("the new ATHD have not set\n");
#endif
		reg_val &= 0x07;    /* clear ATHD */
		reg_val |= ATHD;    /* set ATHD */
		ret = cw_write(cw_bat->client, REG_CONFIG, &reg_val);
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("cw_init 1111\n");
#endif
		if (ret < 0)
			return ret;
	}

	ret = cw_read(cw_bat->client, REG_CONFIG, &reg_val);
	if (ret < 0)
		 return ret;

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw_init REG_CONFIG = %d\n",reg_val);
#endif

	if (!(reg_val & CONFIG_UPDATE_FLG))
	{
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("update flag for new battery info have not set\n");
#endif

#ifdef CONFIG_CM865_MAINBOARD
		ret = cw_update_config_info(cw_bat);
		if (ret < 0)
			return ret;
	} else {
		for(i = 0; i < SIZE_BATINFO; i++) { 
			ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
			if (ret < 0)
				return ret;
			
			if(hmi_battery_version==2)
			{
				if (config_info_sun[i] != reg_val)
				break;
			}
			else if(hmi_battery_version==3)
			{
				if (config_info_scud[i] != reg_val)
				break;
			}
			else
			{
				if (config_info_cos[i] != reg_val)
				break;
			}
		}
#else
		ret = cw_update_config_info(cw_bat);
		if (ret < 0)
			return ret;
	} else {
		for(i = 0; i < SIZE_BATINFO; i++) 
		{ 
			ret = cw_read(cw_bat->client, (REG_BATINFO + i), &reg_val);
			if (ret < 0)
				return ret;
				 
			if(hmi_battery_version==2)
			{
				if (config_info_des[i] != reg_val)
				break;
			}
			else
			{
				if (config_info_cos[i] != reg_val)
				break;
			}
		}
#endif
		if (i != SIZE_BATINFO) {
#if FG_CW2015_DEBUG
			FG_CW2015_LOG("update flag for new battery info have not set\n"); 
#endif
			ret = cw_update_config_info(cw_bat);
			if (ret < 0)
				return ret; 
		}
	}

	for (i = 0; i < 30; i++) 
	{
		ret = cw_read(cw_bat->client, REG_SOC, &reg_val);
		if (ret < 0)
			return ret;

		else if (reg_val <= 0x64) 
			break;

		msleep(100);
		if (i > 25)
		{
#if FG_CW2015_DEBUG
			FG_CW2015_ERR("cw2015/cw2013 input unvalid power error\n");
#endif
		}

	}
	if (i >=30)
	{
		reg_val = MODE_SLEEP;
		ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("cw2015/cw2013 input unvalid power error_2\n");
#endif
		return -1;
	}
	CW2015_test_init=1;

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

extern int g_platform_boot_mode;
static int cw_quickstart(struct cw_battery *cw_bat)
{
	int ret = 0;
	u8 reg_val = MODE_QUICK_START;

	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if(ret < 0) {
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("Error quick start1\n");
#endif
		return ret;
	}

	reg_val = MODE_NORMAL;

	ret = cw_write(cw_bat->client, REG_MODE, &reg_val);
	if(ret < 0) {
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("Error quick start2\n");
#endif
		return ret;
	}
	return 1;
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
	static int jump_flag =0;
	static int reset_loop =0;
	int charge_time;
	u8 reset_val;
	int loop =0;
	static int count_time=0;
	static int count_time_sum=0;
	static int count_real_capacity=0;
	u8 count_real_sum = 0;

	ret = cw_read_word(cw_bat->client, REG_SOC, reg_val);
	if (ret < 0)
		return ret;

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw_get_capacity cw_capacity_0 = %d,cw_capacity_1 = %d\n",reg_val[0],reg_val[1]);
#endif
	cw_capacity = reg_val[0];
	if ((cw_capacity < 0) || (cw_capacity > 100)) {
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("get cw_capacity error; cw_capacity = %d\n", cw_capacity);
#endif
		reset_loop++;

		if (reset_loop >5) {
			reset_val = MODE_SLEEP;
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
			reset_val = MODE_NORMAL;
			msleep(10);
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;

			ret = cw_init(cw_bat);
			if (ret) 
				return ret;
			reset_loop =0;
		}

		return cw_capacity;
	} else {
		reset_loop =0;
	}

#if FG_CW2015_DEBUG
	if (cw_capacity == 0) 
		FG_CW2015_LOG("the cw201x capacity is 0 !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
	else
		FG_CW2015_LOG("the cw201x capacity is %d, funciton: %s\n", cw_capacity, __func__);
#endif

#ifdef  BAT_CHANGE_ALGORITHM
	if( g_platform_boot_mode == 8 )
	{
		PowerResetFlag = -1; 
		count_real_sum = 26;
	}else{
		count_real_sum = 5;
	}
	cw_capacity = cw_algorithm(cw_bat,cw_capacity);	
#endif

	ktime_get_ts(&ts);
	new_run_time = ts.tv_sec;

	get_monotonic_boottime(&ts);
	new_sleep_time = ts.tv_sec - new_run_time;
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw_get_capacity cw_bat->charger_mode = %d\n",cw_bat->charger_mode);
#endif
	//count_time == 20s  

		if(count_real_capacity <= count_real_sum) {
			count_real_capacity++;
#if FG_CW2015_DEBUG
			FG_CW2015_LOG("count_real_capacity = %d\n",cw_bat->charger_mode);
#endif
		}

#ifdef CONFIG_CM865_MAINBOARD //add by longcheer_liml_2015_10_12
	if(Charger_enable_Flag==0)
	{
		if ((cw_bat->charger_mode == 0) && (cw_capacity > cw_bat->capacity)&&(cw_capacity < (cw_bat->capacity+20))&&(count_real_capacity>= count_real_sum ))
		{             // modify battery level swing
			if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) 
		{
			cw_capacity = cw_bat->capacity;
		}
	}
	}else{
		if (
#ifdef CHARGING_NO_DOWN_CAP //liuchao
		((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity > (cw_bat->capacity - 9)))
		||
#endif
		((cw_bat->charger_mode == 0) && (cw_capacity > cw_bat->capacity)&&(cw_capacity < (cw_bat->capacity+20))&&(count_real_capacity>= count_real_sum ) )) 
		{             // modify battery level swing
			if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) 
		{
			cw_capacity = cw_bat->capacity;
		}
		}
	}

#else
	if (
#ifdef CHARGING_NO_DOWN_CAP //liuchao
		((cw_bat->charger_mode > 0) && (cw_capacity <= (cw_bat->capacity - 1)) && (cw_capacity > (cw_bat->capacity - 9))) || 
#endif
		((cw_bat->charger_mode == 0) && (cw_capacity > cw_bat->capacity)&&(cw_capacity < (cw_bat->capacity+20))&&(count_real_capacity>= count_real_sum ) )) {             // modify battery level swing
		if (!(cw_capacity == 0 && cw_bat->capacity <= 2)) 
		{		
			cw_capacity = cw_bat->capacity;
		}
	} 
#endif

	if ((cw_bat->charger_mode > 0) && (cw_capacity >= 95) && (cw_capacity <= cw_bat->capacity)) {     // avoid no charge full
		capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
		capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
		allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_UP_MAX_CHANGE;
		if (allow_change > 0) {
			allow_capacity = cw_bat->capacity + allow_change; 
			cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			jump_flag =1;
		} else if (cw_capacity <= cw_bat->capacity) {
			cw_capacity = cw_bat->capacity; 
		}

	}		 
	else if ((cw_bat->charger_mode == 0) && (cw_capacity <= cw_bat->capacity ) && (cw_capacity >= 90) && (jump_flag == 1)) {     // avoid battery level jump to CW_BAT
		capacity_or_aconline_time = (cw_bat->sleep_time_capacity_change > cw_bat->sleep_time_charge_start) ? cw_bat->sleep_time_capacity_change : cw_bat->sleep_time_charge_start;
		capacity_or_aconline_time += (cw_bat->run_time_capacity_change > cw_bat->run_time_charge_start) ? cw_bat->run_time_capacity_change : cw_bat->run_time_charge_start;
		allow_change = (new_sleep_time + new_run_time - capacity_or_aconline_time) / BATTERY_DOWN_CHANGE;
		if (allow_change > 0) {
			allow_capacity = cw_bat->capacity - allow_change; 
			if (cw_capacity >= allow_capacity) {
				jump_flag =0;
			}
			else{
				cw_capacity = (allow_capacity <= 100) ? allow_capacity : 100;
			}
		} else if (cw_capacity <= cw_bat->capacity) {
			cw_capacity = cw_bat->capacity;
		}
	}
	
	if ((cw_capacity == 0) && (cw_bat->capacity > 1)) {              // avoid battery level jump to 0% at a moment from more than 2%
		allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
		allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

		allow_capacity = cw_bat->capacity - allow_change;
		cw_capacity = (allow_capacity >= cw_capacity) ? allow_capacity: cw_capacity;
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("report GGIC POR happened\n");
#endif
		reset_val = MODE_SLEEP;               
		ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
		if (ret < 0)
			return ret;
		reset_val = MODE_NORMAL;
		msleep(10);
		ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
		if (ret < 0)
			return ret;
						
		ret = cw_init(cw_bat);
		if (ret) 
			return ret;	 
	}
	
	if((cw_bat->charger_mode > 0) &&(cw_capacity == 0))
	{		  
		charge_time = new_sleep_time + new_run_time - cw_bat->sleep_time_charge_start - cw_bat->run_time_charge_start;
		if ((charge_time > BATTERY_DOWN_MAX_CHANGE_RUN_AC_ONLINE) && (if_quickstart == 0)) {
			reset_val = MODE_SLEEP;               
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
			reset_val = MODE_NORMAL;
			msleep(10);
			ret = cw_write(cw_bat->client, REG_MODE, &reset_val);
			if (ret < 0)
				return ret;
						
			ret = cw_init(cw_bat);
			if (ret) 
				return ret;
#if FG_CW2015_DEBUG
			FG_CW2015_LOG("report battery capacity still 0 if in changing\n");
#endif
			if_quickstart = 1;
		}
	} else if ((if_quickstart == 1)&&(cw_bat->charger_mode == 0)) {
		if_quickstart = 0;
	}

#ifdef SYSTEM_SHUTDOWN_VOLTAGE
	if ((cw_bat->charger_mode == 0) && (cw_capacity <= 20) && (cw_bat->voltage <= SYSTEM_SHUTDOWN_VOLTAGE)) {      	     
		if (if_quickstart == 10) {  
		
			allow_change = ((new_run_time - cw_bat->run_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_RUN);
			allow_change += ((new_sleep_time - cw_bat->sleep_time_capacity_change) / BATTERY_DOWN_MIN_CHANGE_SLEEP);

			allow_capacity = cw_bat->capacity - allow_change;
			cw_capacity = (allow_capacity >= 0) ? allow_capacity: 0;
		
			if (cw_capacity < 1){	     	      	
				cw_quickstart(cw_bat);
				if_quickstart = 12;
				cw_capacity = 0;
			}
		} else if (if_quickstart <= 10)
			if_quickstart =if_quickstart + 2;
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("the cw201x voltage is less than SYSTEM_SHUTDOWN_VOLTAGE !!!!!!!, funciton: %s, line: %d\n", __func__, __LINE__);
#endif
	} else if ((cw_bat->charger_mode > 0)&& (if_quickstart <= 12)) {
		if_quickstart = 0;
	}
#endif
	return cw_capacity;
}

int cw2015_check = 0;
static int cw_get_vol(struct cw_battery *cw_bat)
{
	int ret;
	u8 reg_val[2];
	u16 value16, value16_1, value16_2, value16_3;
	int voltage;
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw_get_vol \n");
#endif

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
	{
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("cw_get_vol 1111\n");
#endif
		return ret;
	}
	value16 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
	{
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("cw_get_vol 2222\n");
#endif
		return ret;
	}
	value16_1 = (reg_val[0] << 8) + reg_val[1];

	ret = cw_read_word(cw_bat->client, REG_VCELL, reg_val);
	if (ret < 0)
	{
#if FG_CW2015_DEBUG
		FG_CW2015_LOG("cw_get_vol 3333\n");
#endif
		return ret;
	}
	value16_2 = (reg_val[0] << 8) + reg_val[1];

	if(value16 > value16_1)
	{	 
		value16_3 = value16;
		value16 = value16_1;
		value16_1 = value16_3;
	}
		
	if(value16_1 > value16_2)
	{
		value16_3 =value16_1;
		value16_1 =value16_2;
		value16_2 =value16_3;
	}
		
	if(value16 >value16_1)
	{	 
		value16_3 =value16;
		value16 =value16_1;
		value16_1 =value16_3;
	}		

	voltage = value16_1 * 312 / 1024;
	//voltage = voltage * 1000;
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw_get_vol 4444 voltage = %d\n",voltage);
#endif
	if(voltage ==0)
		cw2015_check++;
	return voltage;
}

#ifdef BAT_LOW_INTERRUPT
static int cw_get_alt(struct cw_battery *cw_bat)
{
	int ret = 0;
	u8 reg_val;
	u8 value8 = 0;
	int alrt;
	
	ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
	if (ret < 0)
		return ret;
	value8 = reg_val;
	alrt = value8 >> 7;
	
	value8 = value8 & 0x7f;
	reg_val = value8;
	ret = cw_write(cw_bat->client, REG_RRT_ALERT, &reg_val);
	if(ret < 0) {
#if FG_CW2015_DEBUG
		FG_CW2015_ERR( "Error clear ALRT\n");
#endif
		return ret;
	}
	return alrt;
}
#endif

static int cw_get_time_to_empty(struct cw_battery *cw_bat)
{
	int ret;
	u8 reg_val;
	u16 value16;

	ret = cw_read(cw_bat->client, REG_RRT_ALERT, &reg_val);
	if (ret < 0)
		return ret;

	value16 = reg_val;

	ret = cw_read(cw_bat->client, REG_RRT_ALERT + 1, &reg_val);
	if (ret < 0)
		return ret;

	value16 = ((value16 << 8) + reg_val) & 0x1fff;
	return value16;
}

static void rk_bat_update_capacity(struct cw_battery *cw_bat)
{
	int cw_capacity;
#ifdef BAT_CHANGE_ALGORITHM
	cw_capacity = cw_get_capacity(cw_bat);
#if FG_CW2015_DEBUG
	FG_CW2015_ERR("cw2015_file_test userdata,	%ld,	%d,	%d\n",get_seconds(),cw_capacity,cw_bat->voltage);
#endif
#else
	cw_capacity = cw_get_capacity(cw_bat);
#endif
	if ((cw_capacity >= 0) && (cw_capacity <= 100) && (cw_bat->capacity != cw_capacity)) {
		cw_bat->capacity = cw_capacity;
		cw_bat->bat_change = 1;
		cw_update_time_member_capacity_change(cw_bat);

	#if FG_CW2015_DEBUG
		if (cw_bat->capacity == 0)
		FG_CW2015_LOG("report battery capacity 0 and will shutdown if no changing\n");
	#endif
	}
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("rk_bat_update_capacity cw_capacity = %d\n",cw_bat->capacity);
#endif
}

static void rk_bat_update_vol(struct cw_battery *cw_bat)
{
	int ret;

	ret = cw_get_vol(cw_bat);
	if ((ret >= 0) && (cw_bat->voltage != ret)) {
		cw_bat->voltage = ret;
		cw_bat->bat_change = 1;
	}
}

static void rk_bat_update_status(struct cw_battery *cw_bat)
{
	int status;

	if (cw_bat->charger_mode > 0) {
		if (cw_bat->capacity >= 100) 
			status=POWER_SUPPLY_STATUS_FULL;
		else
			status=POWER_SUPPLY_STATUS_CHARGING;
	} else {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
	}

	if (cw_bat->status != status) {
		cw_bat->status = status;
		cw_bat->bat_change = 1;
	} 
}

static void rk_bat_update_time_to_empty(struct cw_battery *cw_bat)
{
	int ret;

	ret = cw_get_time_to_empty(cw_bat);
	if ((ret >= 0) && (cw_bat->time_to_empty != ret)) {
		cw_bat->time_to_empty = ret;
		cw_bat->bat_change = 1;
	}
}

static int get_usb_charge_state(struct cw_battery *cw_bat)
{
	int usb_status = 0;

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("get_usb_charge_state FG_charging_type = %d\n",FG_charging_type);
#endif
	if(FG_charging_status == 0)
	{
		usb_status = 0;
		cw_bat->charger_mode = 0;
	}
	else
	{
		if(FG_charging_type==STANDARD_HOST)
		{
			usb_status = 1;
			cw_bat->charger_mode = USB_CHARGER_MODE;
		}
		else
		{
			usb_status = 2;
			cw_bat->charger_mode = AC_CHARGER_MODE;
		}
	}
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("get_usb_charge_state usb_status = %d,FG_charging_status = %d\n",usb_status,FG_charging_status);
#endif

	return usb_status;
}

static int rk_usb_update_online(struct cw_battery *cw_bat)
{
	int ret = 0;
	int usb_status = 0;

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("rk_usb_update_online FG_charging_status = %d\n", FG_charging_status);
#endif

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
	int ret;
	static int count_real_capacity = 0;

#if FG_CW2015_DEBUG
	FG_CW2015_FUN();
#endif
	printk("cw_bat_work\n");

	delay_work = container_of(work, struct delayed_work, work);
	cw_bat = container_of(delay_work, struct cw_battery, battery_delay_work);
	ret = rk_usb_update_online(cw_bat);

	if (cw_bat->usb_online == 1) 
		ret = rk_usb_update_online(cw_bat);

	rk_bat_update_capacity(cw_bat);
	rk_bat_update_vol(cw_bat);
	g_cw2015_capacity = cw_bat->capacity;
	g_cw2015_vol = cw_bat->voltage;

	printk("cw_bat_work 777 vol = %d,cap = %d\n",cw_bat->voltage,cw_bat->capacity);
	if (cw_bat->bat_change) {
		cw_bat->bat_change = 0;
	}

	if(count_real_capacity < 30 && g_platform_boot_mode == 8) {
		queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(1000));
		count_real_capacity++;
	} else {
		queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(queue_delayed_work_time));
	}
}

/*----------------------------------------------------------------------------*/
static int cw2015_i2c_detect(struct i2c_client *client, struct i2c_board_info *info) 
{
#if FG_CW2015_DEBUG
	FG_CW2015_FUN();
#endif

	strcpy(info->type, CW2015_DEV_NAME);
	return 0;
}

/*----------------------------------------------------------------------------*/
static ssize_t file_state_show(struct device *d, struct device_attribute *a, char *buf)
{
	return sprintf(buf, "%d", file_sys_state);
}
static DEVICE_ATTR(file_state, S_IRUGO, file_state_show, NULL);

static int cw2015_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct cw_battery *cw_bat;
	int ret;
	int irq;
	int irq_flags;
	int loop = 0;

#if FG_CW2015_DEBUG
	FG_CW2015_FUN();
#endif

	mt_set_gpio_mode(GPIO1, 3);
	mt_set_gpio_mode(GPIO2, 3);

	file_sys_state = 1;

	cw_bat = kzalloc(sizeof(struct cw_battery), GFP_KERNEL);
	if (!cw_bat) {
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("fail to allocate memory\n");
#endif
		return -ENOMEM;
	}

	i2c_set_clientdata(client, cw_bat);
	cw_bat->plat_data = client->dev.platform_data;
	cw_bat->client = client;
	cw_bat->plat_data = &cw_bat_platdata;
	ret = cw_init(cw_bat);

	if (ret) 
		return ret;
	cw_bat->dc_online = 0;
	cw_bat->usb_online = 0;
	cw_bat->charger_mode = 0;
	cw_bat->capacity = 1;
	cw_bat->voltage = 0;
	cw_bat->status = 0;
	cw_bat->time_to_empty = 0;
	cw_bat->bat_change = 0;

	cw_update_time_member_capacity_change(cw_bat);
	cw_update_time_member_charge_start(cw_bat);

	device_create_file(&client->dev, &dev_attr_file_state);

	cw_bat->battery_workqueue = create_singlethread_workqueue("rk_battery");
	INIT_DELAYED_WORK(&cw_bat->battery_delay_work, cw_bat_work);
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(10));

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw2015/cw2013 driver v1.2 probe sucess\n");
#endif
	return 0;

rk_usb_register_fail:
	power_supply_unregister(&cw_bat->rk_bat);
rk_ac_register_fail:
	power_supply_unregister(&cw_bat->rk_ac);
rk_bat_register_fail:

#if FG_CW2015_DEBUG
	FG_CW2015_LOG("cw2015/cw2013 driver v1.2 probe error!!!!\n");
#endif
	return ret;
}

static int  cw2015_i2c_remove(struct i2c_client *client)//__devexit
{
	struct cw_battery *data = i2c_get_clientdata(client);

#if FG_CW2015_DEBUG
	FG_CW2015_FUN();
#endif
	cancel_delayed_work(&data->battery_delay_work);
	cw2015_i2c_client = NULL;
	i2c_unregister_device(client);
	kfree(data);

	return 0;
}

static int cw2015_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{

	struct cw_battery *cw_bat = i2c_get_clientdata(client);

#if FG_CW2015_DEBUG
	FG_CW2015_FUN();
#endif
	cancel_delayed_work(&cw_bat->battery_delay_work);

	return 0;
}

static int cw2015_i2c_resume(struct i2c_client *client)
{
	struct cw_battery *cw_bat = i2c_get_clientdata(client);

#if FG_CW2015_DEBUG
	FG_CW2015_FUN();
#endif
	queue_delayed_work(cw_bat->battery_workqueue, &cw_bat->battery_delay_work, msecs_to_jiffies(100));

	return 0;
}

static struct i2c_driver cw2015_i2c_driver = {	
	.probe      = cw2015_i2c_probe,
	.remove     = cw2015_i2c_remove,
	.detect     = cw2015_i2c_detect,
	.suspend    = cw2015_i2c_suspend,
	.resume     = cw2015_i2c_resume,
	.id_table   = FG_CW2015_i2c_id,
	.driver = {
//		.owner          = THIS_MODULE,
		.name           = CW2015_DEV_NAME,
	},
};

static int __init cw_bat_init(void)
{
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("%s: \n", __func__); 
#endif
	printk("cw_bat_init\n");
	i2c_register_board_info(4, &i2c_FG_CW2015, 1);

	if(i2c_add_driver(&cw2015_i2c_driver))
	{
#if FG_CW2015_DEBUG
		FG_CW2015_ERR("add driver error\n");
#endif
		return -1;
	}
	return 0;
}

static void __exit cw_bat_exit(void)
{
#if FG_CW2015_DEBUG
	FG_CW2015_LOG("%s: \n", __func__); 
#endif
	printk("cw_bat_exit\n");
	i2c_del_driver(&i2c_FG_CW2015);
}

module_init(cw_bat_init);
module_exit(cw_bat_exit);

MODULE_AUTHOR("xhc<xhc@rock-chips.com>");
MODULE_DESCRIPTION("cw2015/cw2013 battery driver");
MODULE_LICENSE("GPL");

