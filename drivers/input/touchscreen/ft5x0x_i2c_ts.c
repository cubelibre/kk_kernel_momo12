/* 
 * drivers/input/touchscreen/ft5x0x_ts.c
 *
 * FocalTech ft5x0x TouchScreen driver. 
 *
 * Copyright (c) 2010  Focal tech Ltd.
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
 *
 *	note: only support mulititouch	Wenfs 2010-10-01
 */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <mach/gpio.h>
#include <linux/irq.h>
#include <mach/board.h>
#include <linux/input/mt.h>
#include "ft5x0x_i2c_ts.h"

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif

#define CONFIG_FT5X0X_MULTITOUCH 1

#if 0
	#define DBG(msg...)	printk(msg);
#else
	#define DBG(msg...)
#endif

/*
 * Added by yick @RockChip
 * Compatible with both types of firmware
 * default: point - only pressdown finger num
 * 			event - both down and up event
 */
#define USE_POINT 1
#if USE_POINT
uint16_t down_table	= 0;
uint16_t up_table	= ~0;
#endif

#define PRESS_MAX       255

//#define FT5X0X_NAME	"ft5x0x_ts" 
#define MAX_CONTACTS 10
/*enum ft5x0x_ts_regs {
	FT5X0X_REG_PMODE	= 0xA5,	// Power Consume Mode		
};*/

//FT5X0X_REG_PMODE
#define PMODE_ACTIVE        0x00
#define PMODE_MONITOR       0x01
#define PMODE_STANDBY       0x02
#define PMODE_HIBERNATE     0x03

#ifndef ABS_MT_TOUCH_MAJOR
#define ABS_MT_TOUCH_MAJOR	0x30	/* touching ellipse */
#define ABS_MT_TOUCH_MINOR	0x31	/* (omit if circular) */
#define ABS_MT_WIDTH_MAJOR	0x32	/* approaching ellipse */
#define ABS_MT_WIDTH_MINOR	0x33	/* (omit if circular) */
#define ABS_MT_ORIENTATION	0x34	/* Ellipse orientation */
#define ABS_MT_POSITION_X	  0x35	/* Center X ellipse position */
#define ABS_MT_POSITION_Y	  0x36	/* Center Y ellipse position */
#define ABS_MT_TOOL_TYPE	  0x37	/* Type of touching device */
#define ABS_MT_BLOB_ID		  0x38	/* Group set of pkts as blob */
#endif /* ABS_MT_TOUCH_MAJOR */

struct point_data {
	u8 status;
	u8 id;
	u16 x;
	u16 y;
};

struct ts_event {
  u16  touch_point;
  struct point_data point[MAX_CONTACTS];
};

struct ft5x0x_ts_dev {
  struct i2c_client *client;	
	struct input_dev	*input_dev;
	struct hrtimer timer;
	int    irq;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
	struct early_suspend	early_suspend;
};

static struct ft5x0x_ts_dev *g_dev;
static struct i2c_client *this_client;

static int timer_flag = 0;
static struct workqueue_struct *ft5x0x_wq;
static enum hrtimer_restart ft5x0x_ts_timer(struct hrtimer *timer);

static int ft5x0x_i2c_rxdata(char *rxdata, int length)
{
	int ret;

	struct i2c_msg msgs[] = {
		{
			.addr	= g_dev->client->addr,
			.flags	= 0,
			.len	= 1,
			.buf	= rxdata,
			.scl_rate = FT5X0X_I2C_SPEED,
		},
		{
			.addr	= g_dev->client->addr,
			.flags	= I2C_M_RD,
			.len	= length,
			.buf	= rxdata,
			.scl_rate = FT5X0X_I2C_SPEED,
		},
	};

	ret = i2c_transfer(g_dev->client->adapter, msgs, 2);
	if (ret < 0)
		pr_err("msg %s i2c read error: %d\n", __func__, ret);
	
	return ret;
}

static int ft5x0x_i2c_txdata(char *txdata, int length)
{
	int ret;

	struct i2c_msg msg[] = {
		{
			.addr	= g_dev->client->addr,
			.flags	= 0,
			.len	= length,
			.buf	= txdata,
			.scl_rate = FT5X0X_I2C_SPEED,
		},
	};

	ret = i2c_transfer(g_dev->client->adapter, msg, 1);
	if (ret < 0)
		pr_err("%s i2c write error: %d\n", __func__, ret);

	return ret;
}

static int ft5x0x_set_reg(u8 addr, u8 para)
{
    u8 buf[3];
    int ret = -1;

    buf[0] = addr;
    buf[1] = para;
    ret = ft5x0x_i2c_txdata(buf, 2);
    if (ret < 0) {
        pr_err("write reg failed! %#x ret: %d", buf[0], ret);
        return -1;
    }
    
    return 0;
}


/***********************************************************************
  [function]: 
callback:              read data from ctpm by i2c interface;
[parameters]:
buffer[in]:            data buffer;
length[in]:           the length of the data buffer;
[return]:
FTS_TRUE:            success;
FTS_FALSE:           fail;
 ************************************************************************/
static bool i2c_read_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;

	ret=i2c_master_recv(this_client, pbt_buf, dw_lenth);

	if(ret<=0)
	{
		printk("[TSP]i2c_read_interface error\n");
		return FTS_FALSE;
	}

	return FTS_TRUE;
}



/***********************************************************************
  [function]: 
callback:               write data to ctpm by i2c interface;
[parameters]:
buffer[in]:             data buffer;
length[in]:            the length of the data buffer;
[return]:
FTS_TRUE:            success;
FTS_FALSE:           fail;
 ************************************************************************/
static bool  i2c_write_interface(u8* pbt_buf, int dw_lenth)
{
	int ret;
	ret=i2c_master_send(this_client, pbt_buf, dw_lenth);
	if(ret<=0)
	{
		printk("[TSP]i2c_write_interface error line = %d, ret = %d\n", __LINE__, ret);
		return FTS_FALSE;
	}

	return FTS_TRUE;
}
/***********************************************************************
  [function]: 
callback:         send a command to ctpm.
[parameters]:
btcmd[in]:       command code;
btPara1[in]:     parameter 1;    
btPara2[in]:     parameter 2;    
btPara3[in]:     parameter 3;    
num[in]:         the valid input parameter numbers, 
if only command code needed and no 
parameters followed,then the num is 1;    
[return]:
FTS_TRUE:      success;
FTS_FALSE:     io fail;
 ************************************************************************/
static bool cmd_write(u8 btcmd,u8 btPara1,u8 btPara2,u8 btPara3,u8 num)
{
	u8 write_cmd[4] = {0};

	write_cmd[0] = btcmd;
	write_cmd[1] = btPara1;
	write_cmd[2] = btPara2;
	write_cmd[3] = btPara3;
	return i2c_write_interface(write_cmd, num);
}




/***********************************************************************
  [function]: 
callback:         write a byte data  to ctpm;
[parameters]:
buffer[in]:       write buffer;
length[in]:      the size of write data;    
[return]:
FTS_TRUE:      success;
FTS_FALSE:     io fail;
 ************************************************************************/
static bool byte_write(u8* buffer, int length)
{

	return i2c_write_interface(buffer, length);
}




/***********************************************************************
  [function]: 
callback:         read a byte data  from ctpm;
[parameters]:
buffer[in]:       read buffer;
length[in]:      the size of read data;    
[return]:
FTS_TRUE:      success;
FTS_FALSE:     io fail;
 ************************************************************************/
static bool byte_read(u8* buffer, int length)
{
	return i2c_read_interface(buffer, length);
}

#define    FTS_PACKET_LENGTH        128

static unsigned char CTPM_FW[]=
{
#if defined(CONFIG_BOARD_TYPE_ZM861)
//#include "dpt_N3708A_20120306_app.i"
#else
//#include "ft_app.i"
#endif
};

E_UPGRADE_ERR_TYPE  fts_ctpm_fw_upgrade(FTS_BYTE* pbt_buf, FTS_DWRD dw_lenth)
{
    FTS_BYTE reg_val[2] = {0};
    FTS_DWRD i = 0;

    FTS_DWRD  packet_number;
    FTS_DWRD  j;
    FTS_DWRD  temp;
    FTS_DWRD  lenght;
    FTS_BYTE  packet_buf[FTS_PACKET_LENGTH + 6];
    FTS_BYTE  auc_i2c_write_buf[10];
    FTS_BYTE bt_ecc;
    int      i_ret;

    /*********Step 1:Reset  CTPM *****/
    /*write 0xaa to register 0xfc*/
    ft5x0x_set_reg(0xfc,0xaa);
    mdelay(50);
     /*write 0x55 to register 0xfc*/
    ft5x0x_set_reg(0xfc,0x55);
    printk("[TSP] Step 1: Reset CTPM test\n");
   
    mdelay(30);   


    /*********Step 2:Enter upgrade mode *****/
    auc_i2c_write_buf[0] = 0x55;
    auc_i2c_write_buf[1] = 0xaa;
    do
    {
        i ++;
        i_ret = ft5x0x_i2c_txdata(auc_i2c_write_buf, 2);
        mdelay(5);
    }while(i_ret <= 0 && i < 5 );

    /*********Step 3:check READ-ID***********************/        
    cmd_write(0x90,0x00,0x00,0x00,4);
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }

     /*********Step 4:erase app*******************************/
    cmd_write(0x61,0x00,0x00,0x00,1);  //erase app area
    mdelay(1500); 
    cmd_write(0x63,0x00,0x00,0x00,1);  //erase panel parameter area
    mdelay(100);
    printk("[FTS] Step 4: erase. \n");

    /*********Step 5:write firmware(FW) to ctpm flash*********/
    bt_ecc = 0;
    printk("[TSP] Step 5: start upgrade. \n");
    dw_lenth = dw_lenth - 8;
    packet_number = (dw_lenth) / FTS_PACKET_LENGTH;
    packet_buf[0] = 0xbf;
    packet_buf[1] = 0x00;
    for (j=0;j<packet_number;j++)
    {
        temp = j * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        lenght = FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(lenght>>8);
        packet_buf[5] = (FTS_BYTE)lenght;

        for (i=0;i<FTS_PACKET_LENGTH;i++)
        {
            packet_buf[6+i] = pbt_buf[j*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }
        
        byte_write(&packet_buf[0],FTS_PACKET_LENGTH + 6);
        mdelay(FTS_PACKET_LENGTH/6 + 1);
        if ((j * FTS_PACKET_LENGTH % 1024) == 0)
        {
              printk("[TSP] upgrade the 0x%x th byte.\n", ((unsigned int)j) * FTS_PACKET_LENGTH);
        }
    }

    if ((dw_lenth) % FTS_PACKET_LENGTH > 0)
    {
        temp = packet_number * FTS_PACKET_LENGTH;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;

        temp = (dw_lenth) % FTS_PACKET_LENGTH;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;

        for (i=0;i<temp;i++)
        {
            packet_buf[6+i] = pbt_buf[ packet_number*FTS_PACKET_LENGTH + i]; 
            bt_ecc ^= packet_buf[6+i];
        }

        byte_write(&packet_buf[0],temp+6);    
        mdelay(20);
    }

    //send the last six byte
    for (i = 0; i<6; i++)
    {
        temp = 0x6ffa + i;
        packet_buf[2] = (FTS_BYTE)(temp>>8);
        packet_buf[3] = (FTS_BYTE)temp;
        temp =1;
        packet_buf[4] = (FTS_BYTE)(temp>>8);
        packet_buf[5] = (FTS_BYTE)temp;
        packet_buf[6] = pbt_buf[ dw_lenth + i]; 
        bt_ecc ^= packet_buf[6];

        byte_write(&packet_buf[0],7);  
        mdelay(20);
    }

    /*********Step 6: read out checksum***********************/
    /*send the opration head*/
    cmd_write(0xcc,0x00,0x00,0x00,1);
    byte_read(reg_val,1);
    printk("[TSP] Step 6:  ecc read 0x%x, new firmware 0x%x. \n", reg_val[0], bt_ecc);
    if(reg_val[0] != bt_ecc)
    {
        return ERR_ECC;
    }

    /*********Step 7: reset the new FW***********************/
    cmd_write(0x07,0x00,0x00,0x00,1);

    return ERR_OK;
}


int fts_ctpm_auto_clb(void)
{
    unsigned char uc_temp;
    unsigned char i ;

    printk("[FTS] start auto CLB.\n");
    msleep(200);
    ft5x0x_set_reg(0, 0x40);  
    mdelay(100);   //make sure already enter factory mode
    ft5x0x_set_reg(2, 0x4);  //write command to start calibration
    mdelay(300);
    for(i=0;i<100;i++)
    {
    	i2c_master_reg8_recv(this_client, 0, &uc_temp, 1, FT5X0X_I2C_SPEED);
      //  ft5x0x_read_reg(0,&uc_temp);
      printk("uctemp=%d",uc_temp);
        if ( ((uc_temp&0x70)>>4) == 0x0)  //return to normal mode, calibration finish
        {
            break;
        }
        mdelay(200);
        printk("[FTS] waiting calibration %d\n",i);
        
    }
    printk("[FTS] calibration OK.\n");
    
    msleep(300);
    ft5x0x_set_reg(0, 0x40);  //goto factory mode
    mdelay(100);   //make sure already enter factory mode
    ft5x0x_set_reg(2, 0x5);  //store CLB result
    mdelay(300);
    ft5x0x_set_reg(0, 0x0); //return to normal mode 
    msleep(300);
    printk("[FTS] store CLB result OK.\n");
    return 0;
}

int fts_ctpm_fw_reset(void)
{
   //int i_ret;
   FTS_BYTE reg_val[2] = {0};
	   
    msleep(300);
	
    cmd_write(0x90,0x00,0x00,0x00,4);	
    byte_read(reg_val,2);
    if (reg_val[0] == 0x79 && reg_val[1] == 0x3)
    {
        printk("[TSP] Step 3: CTPM ID,ID1 = 0x%x,ID2 = 0x%x\n",reg_val[0],reg_val[1]);
    }
    else
    {
        return ERR_READID;
        //i_is_new_protocol = 1;
    }
	
    cmd_write(0x07,0x00,0x00,0x00,1);
   msleep(300);
   return 0;
}


int fts_ctpm_fw_upgrade_with_i_file(void)
{
   FTS_BYTE*     pbt_buf = FTS_NULL;
   int i_ret;
    
    //=========FW upgrade========================*/
   pbt_buf = CTPM_FW;
   /*call the upgrade function*/
   i_ret =  fts_ctpm_fw_upgrade(pbt_buf,sizeof(CTPM_FW));
   if (i_ret != 0)
   {
       //error handling ...
       //TBD
   }
   else
   {
        printk("[FTS] upgrade successfully.\n");
        fts_ctpm_auto_clb();  //start auto CLB
   }

   return i_ret;
}

unsigned char fts_ctpm_get_upg_ver(void)
{
    unsigned int ui_sz;
    ui_sz = sizeof(CTPM_FW);
    if (ui_sz > 2)
    {
        return CTPM_FW[ui_sz - 2];
    }
    else
    {
        //TBD, error handling?
        return 0xff; //default value
    }
}

static enum hrtimer_restart ft5x0x_ts_timer(struct hrtimer *timer)
{
	struct ft5x0x_ts_dev *ts = container_of(timer, struct ft5x0x_ts_dev, timer);
	queue_work(ft5x0x_wq, &ts->pen_event_work);
	timer_flag = 0;
//	printk("%s\n", __FUNCTION__);
//	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	return HRTIMER_NORESTART;
}

static int ft5x0x_read_data(void)
{
	struct ft5x0x_ts_dev *data = i2c_get_clientdata(g_dev->client);
	struct ts_event *event = &data->event;

	u8 buf[64]= {0};//set send addr to 0x00 *important*
	int ret = -1;
//	u8 i, touch_reg = 0;
//	u8 point_id = 0;

	ret = ft5x0x_i2c_rxdata(buf, 64);
/*
    	if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}*/
#if 0	// Just for debug
		u8 uc_ecc;
		int i;
		uc_ecc = buf[2];
		for (i=0; i<5; i++)
		{
			uc_ecc ^= buf[3+6*i];
			uc_ecc ^= buf[4+6*i];
			uc_ecc ^= buf[5+6*i];
			uc_ecc ^= buf[6+6*i];
		}
//		if (uc_ecc == buf[1])  break;
//	}


	if (uc_ecc != buf[1])
	{
		printk("ecc check error uc_ecc=0x%x, buf[1]=0x%x.\n",uc_ecc, buf[1]);
		return 1;
	}
#endif

	memset(event, ~0x00, sizeof(struct ts_event));

#if USE_POINT
	event->touch_point = buf[2] & 0x0f;// 0000 1111
#else
	event->touch_point = buf[2] >>4;// 0000 1111
#endif
//	printk("event->touch_point =%d\r\n", event->touch_point);
//	if (event->touch_point)
	if (timer_flag == 1)
		hrtimer_start(&data->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);

#ifdef CONFIG_FT5X0X_MULTITOUCH
    switch (event->touch_point) {
		case 10:
			event->point[9].status = (buf[0x39] & 0xc0)>>6;
			event->point[9].id = (buf[0x3b] & 0xf0)>>4;
			event->point[9].x = ((s16)(buf[0x39] & 0x07)<<8 | (s16)buf[0x3a]);
			event->point[9].y = (s16)(buf[0x3b] & 0x07)<<8 | (s16)buf[0x3c];			
		case 9:
			event->point[8].status = (buf[0x33] & 0xc0)>>6;
			event->point[8].id = (buf[0x35] & 0xf0)>>4;
			event->point[8].x = ((s16)(buf[0x33] & 0x07)<<8 | (s16)buf[0x34]);
			event->point[8].y = (s16)(buf[0x35] & 0x07)<<8 | (s16)buf[0x36];		
		case 8:
			event->point[7].status = (buf[0x2d] & 0xc0)>>6;
			event->point[7].id = (buf[0x2f] & 0xf0)>>4;
			event->point[7].x = ((s16)(buf[0x2d] & 0x07)<<8 | (s16)buf[0x2e]);
			event->point[7].y = (s16)(buf[0x2f] & 0x07)<<8 | (s16)buf[0x30];			
		case 7:
			event->point[6].status = (buf[0x27] & 0xc0)>>6;
			event->point[6].id = (buf[0x29] & 0xf0)>>4;
			event->point[6].x = ((s16)(buf[0x27] & 0x07)<<8 | (s16)buf[0x28]);
			event->point[6].y = (s16)(buf[0x29] & 0x07)<<8 | (s16)buf[0x2a];			
		case 6:
			event->point[5].status = (buf[0x21] & 0xc0)>>6;
			event->point[5].id = (buf[0x23] & 0xf0)>>4;
			event->point[5].x = ((s16)(buf[0x21] & 0x07)<<8 | (s16)buf[0x22]);
			event->point[5].y = (s16)(buf[0x23] & 0x07)<<8 | (s16)buf[0x24];		
		case 5:
			event->point[4].status = (buf[0x1b] & 0xc0)>>6;
			event->point[4].id = (buf[0x1d] & 0xf0)>>4;
			event->point[4].x = ((s16)(buf[0x1b] & 0x07)<<8 | (s16)buf[0x1c]);
			event->point[4].y = (s16)(buf[0x1d] & 0x07)<<8 | (s16)buf[0x1e];
			DBG("point5: x=%d y=%d\r\n",event->point[4].x,event->point[4].y);
		case 4:
			event->point[3].status = (buf[0x15] & 0xc0)>>6;
			event->point[3].id = (buf[0x17] & 0xf0)>>4;
			event->point[3].x = ((s16)(buf[0x15] & 0x07)<<8 | (s16)buf[0x16]);
			event->point[3].y = (s16)(buf[0x17] & 0x07)<<8 | (s16)buf[0x18];
			DBG("point4: x=%d y=%d\r\n",event->point[3].x,event->point[3].y);
		case 3:
			event->point[2].status = (buf[0x0f] & 0xc0)>>6;
			event->point[2].id = (buf[0x11] & 0xf0)>>4;
			event->point[2].x = ((s16)(buf[0x0f] & 0x07)<<8 | (s16)buf[0x10]);
			event->point[2].y = (s16)(buf[0x11] & 0x07)<<8 | (s16)buf[0x12];
			DBG("point3: x=%d y=%d\r\n",event->point[2].x,event->point[2].y);
		case 2:
			event->point[1].status = (buf[0x09] & 0xc0)>>6;
			event->point[1].id = (buf[0x0b] & 0xf0)>>4;
			event->point[1].x = ((s16)(buf[0x09] & 0x07)<<8 | (s16)buf[0x0a]);
			event->point[1].y = (s16)(buf[0x0b] & 0x07)<<8 | (s16)buf[0x0c];
			DBG("point2: x=%d y=%d\r\n",event->point[1].x,event->point[1].y);
		case 1:
			event->point[0].status = (buf[0x03] & 0xc0)>>6;
			event->point[0].id = (buf[0x05] & 0xf0)>>4;
			event->point[0].x = ((s16)(buf[0x03] & 0x0F)<<8 | (s16)buf[0x04]);
			event->point[0].y = (s16)(buf[0x05] & 0x0F)<<8 | (s16)buf[0x06];
			DBG("point1: x=%d y=%d\r\n",event->point[0].x,event->point[0].y);
        default:
		    return 0;
	}
#endif
}

static void ft5x0x_report_value(void)
{
	struct ft5x0x_ts_dev *data = i2c_get_clientdata(g_dev->client);
	struct ts_event *event = &data->event;
	struct ft5x0x_platform_data *pdata = this_client->dev.platform_data;
	u8 i = 0;

#if USE_POINT
	down_table = 0;
//	hrtimer_start(&ssl_priv->timer, ktime_set(0, TimeoutInterupt), HRTIMER_MODE_REL);
//	hrtimer_start(&data->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	if (event->touch_point > 0)
	{
		for(i=0; i<event->touch_point; i++) {
			input_mt_slot(data->input_dev, event->point[i].id);
		
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point[i].id);
			// printk("1111  posx =%d ,posy=%d \n", event->point[i].x, event->point[i].y);
			down_table |= 1 << event->point[i].id;
			 event->point[i].x = event->point[i].x *pdata->screen_max_x/(pdata->touch_max_x); 
			 event->point[i].y = event->point[i].y *pdata->screen_max_y/(pdata->touch_max_y);
			if (pdata->swap_xy)
				swap(event->point[i].x, event->point[i].y);
			if (pdata->xpol)
				event->point[i].x = pdata->screen_max_x - event->point[i].x;
			 if (pdata->ypol)
				event->point[i].y = pdata->screen_max_y - event->point[i].y;
			// printk("2222   posx =%d ,posy=%d \n", event->point[i].x, event->point[i].y);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 10);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->point[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->point[i].y);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 15);
		}

		for(i=0; i<MAX_CONTACTS; i++) {
			//printk("down_table=%d up_table=%d",down_table,up_table);
			if( ( (~down_table) & 1<<i) && !(up_table & 1<<i) )
			{
				input_mt_slot(data->input_dev, i);
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
			}
		}		
	}
	else
	{
		for ( i= 0; i<MAX_CONTACTS; ++i )
		{
			input_mt_slot(data->input_dev, i);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
		}		
	}

	up_table = ~down_table;
#else

	for(i=0; i<event->touch_point; i++)
	{
		if(event->point[i].status == 0 || event->point[i].status == 2 ) {
			input_mt_slot(data->input_dev, event->point[i].id);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, event->point[i].id);
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 200);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->point[i].x);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->point[i].y);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 100);
		}
		else if(event->point[i].status == 1) {
			input_mt_slot(data->input_dev, event->point[i].id);
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, -1);
		}
	}
#endif

	input_sync(data->input_dev);

}	/*end ft5x0x_report_value*/

static void ft5x0x_ts_pen_irq_work(struct work_struct *work)
{
	int ret = -1;
	
	ret = ft5x0x_read_data();	
	if (ret == 0) {	
		ft5x0x_report_value();
	}
}

static irqreturn_t ft5x0x_ts_interrupt(int irq, void *dev_id)
{
	struct ft5x0x_ts_dev *ft5x0x_ts = dev_id;
  	
	disable_irq_nosync(g_dev->irq);		
	timer_flag = 1;
	queue_work(ft5x0x_wq, &ft5x0x_ts->pen_event_work);
	 enable_irq(g_dev->irq);
	return IRQ_HANDLED;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_ts_suspend(struct early_suspend *handler)
{
	int ret, i;
	struct ft5x0x_ts_dev *ts;
	struct ft5x0x_platform_data *pdata = this_client->dev.platform_data;
	ts =  container_of(handler, struct ft5x0x_ts_dev, early_suspend);
	
	if(ts->irq)
		disable_irq_nosync(ts->irq);

	ret = cancel_work_sync(&ts->pen_event_work);
	if (ret && ts->irq) /* if work was pending disable-count is now 2 */
		enable_irq(ts->irq);
	// ==set mode ==, 
	
//	gpio_direction_output(RK30_PIN4_PD0, 0);
	for ( i= 0; i<MAX_CONTACTS; ++i )
	{
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(ts->input_dev);
	msleep(100);
     	ft5x0x_set_reg(FT5X0X_REG_PMODE, PMODE_HIBERNATE);
	gpio_set_value(pdata->reset_pin,GPIO_HIGH);
}

static void ft5x0x_ts_resume(struct early_suspend *handler)
{
	struct ft5x0x_ts_dev *ts;
	struct ft5x0x_platform_data *pdata = this_client->dev.platform_data;
	int i;
	ts =  container_of(handler, struct ft5x0x_ts_dev, early_suspend);
	// wake the mode

//	gpio_direction_output(RK30_PIN4_PD0, 0);
	gpio_set_value(pdata->reset_pin, GPIO_LOW);
	msleep(100);
	gpio_set_value(pdata->reset_pin, GPIO_HIGH);
	msleep(150);
#if USE_POINT
	down_table	= 0;
	up_table	= ~0;
#endif
	for ( i= 0; i<MAX_CONTACTS; ++i )
	{
		input_mt_slot(ts->input_dev, i);
		input_report_abs(ts->input_dev, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(ts->input_dev);
	msleep(100);
//	hrtimer_start(&ts->timer, ktime_set(0, (POLL_TIME+6)*1000000), HRTIMER_MODE_REL);
	if(ts->irq)
		enable_irq(ts->irq);
}
#endif  //CONFIG_HAS_EARLYSUSPEND

static int  ft5x0x_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_ts_dev *ft5x0x_ts = NULL;
	struct ft5x0x_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	unsigned char reg_version=0;
	unsigned char reg_value;
	int err = 0;

	if (pdata == NULL) {
		dev_err(&client->dev, "%s: platform data is null\n", __func__);
		goto exit_platform_data_null;
	}	

//	if (pdata->get_probe_state() == 1)
//		return 0;

	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	if(pdata->init_platform_hw)
		pdata->init_platform_hw();
	
	msleep(300);
	 this_client=client;
	
	ft5x0x_ts = (struct ft5x0x_ts_dev *)kzalloc(sizeof(*ft5x0x_ts), GFP_KERNEL);
	if (!ft5x0x_ts)	{
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}
	
	ft5x0x_ts->input_dev = input_dev;
	ft5x0x_ts->client = client;
	ft5x0x_ts->irq = client->irq;

	//avoid tp enter into upgrade mode.
	cmd_write(0x07,0x00,0x00,0x00,7);
	msleep(50); 			//leaf-2012-08-14

	//note by Aoker:
	//we can read Addr: 0x80, achieve current ensitivity,
	//and write the value back to 0x80, value is greater, the sensitive lower.

	err = i2c_master_reg8_recv(this_client, FT5X0X_REG_FIRMID, &reg_version, 1, FT5X0X_I2C_SPEED);
	if (err < 0) {
		printk(KERN_ALERT "FocalTech FT5X0X touch screen not found \n");
		i2c_set_clientdata(client, NULL);
		kfree(ft5x0x_ts);
		pdata->exit_platform_hw();
		goto exit_alloc_data_failed;
	}
	else
		printk("[FT5X0X]==========================\n");
		printk("[FT5X0X] firmware version = 0x%2x\n",reg_version);
		i2c_master_reg8_recv(this_client, FT5X0X_REG_REPORT_RATE, &reg_value, 1, 200*1000);
		printk("[FT5X0X] firmware report rate = %dHz\n", reg_value*10);
		i2c_master_reg8_recv(this_client, FT5X0X_REG_THRES, &reg_value, 1, 200*1000);
		printk("[FT5X0X] firmware threshold = %d\n", reg_value * 4);
		i2c_master_reg8_recv(this_client, FT5X0X_REG_NOISE_MODE, &reg_value, 1, 200*1000);
		printk("[FT5X0X] nosie mode = 0x%2x\n", reg_value);
		printk("[FT5X0X]==========================\n");

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);
	set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, input_dev->absbit);

	input_mt_init_slots(input_dev, MAX_CONTACTS);

	input_set_abs_params(input_dev,ABS_MT_POSITION_X, 0, pdata->screen_max_x + SCREEN_BOUNDARY_ADJUST_VALUE, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_POSITION_Y, 0, pdata->screen_max_y + SCREEN_BOUNDARY_ADJUST_VALUE, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev,ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);

	input_dev->name		= FT5X0X_NAME;		//dev_name(&client->dev)

	hrtimer_init(&ft5x0x_ts->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	ft5x0x_ts->timer.function = ft5x0x_ts_timer;
		
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
		"ft5x0x_ts_probe: failed to register input device: %s\n",
		dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	
	g_dev = ft5x0x_ts;
	
	i2c_set_clientdata(client, ft5x0x_ts);
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_ts_pen_irq_work);
/*	ft5x0x_ts->ts_workqueue = create_workqueue(FT5X0X_NAME);
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_create_singlethread;
	}
	*/
  //ft5x0x_set_reg(0x80,0x64);

  if(!ft5x0x_ts->irq)
  {
    dev_dbg(&ft5x0x_ts->client->dev, "no IRQ?\n");
    return -ENODEV;
  }
  else
  {
    ft5x0x_ts->irq = gpio_to_irq(ft5x0x_ts->irq);
  }
  	
  err = request_irq(ft5x0x_ts->irq, ft5x0x_ts_interrupt, IRQF_TRIGGER_FALLING/*IRQF_DISABLED*/, "ft5x0x_ts", ft5x0x_ts);
	if (err < 0) {
		dev_err(&client->dev, "ft5x0x_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}

	disable_irq_nosync(g_dev->irq);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ft5x0x_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
	ft5x0x_ts->early_suspend.suspend = ft5x0x_ts_suspend;
	ft5x0x_ts->early_suspend.resume	= ft5x0x_ts_resume;
	register_early_suspend(&ft5x0x_ts->early_suspend);
#endif
//wake the CTPM
//	__gpio_as_output(GPIO_FT5X0X_WAKE);		
//	__gpio_clear_pin(GPIO_FT5X0X_WAKE);		//set wake = 0,base on system
//	 msleep(100);
//	__gpio_set_pin(GPIO_FT5X0X_WAKE);			//set wake = 1,base on system
//	msleep(100);
//	ft5x0x_set_reg(0x88, 0x05); //5, 6,7,8
//	ft5x0x_set_reg(0x80, 30);
//	msleep(50);


#ifdef CONFIG_SUPPORT_FTS_CTP_UPG
	if ( reg_version  != fts_ctpm_get_upg_ver()   || reg_version == 0xA6 )  
	{
		printk("[LAIBAO] start upgrade new verison 0x%2x\n", fts_ctpm_get_upg_ver());
		msleep(200);
		err =  fts_ctpm_fw_upgrade_with_i_file();
		if (err == 0)
		{
			printk("[LAIBAO] ugrade successfuly.\n");
			msleep(300);
			 i2c_master_reg8_recv(this_client, FT5X0X_REG_FIRMID, &reg_version, 1, FT5X0X_I2C_SPEED);
			printk("FTS_DBG from old version 0x%2x to new version = 0x%2x\n", reg_version, reg_version);
		}
		else
		{
			printk("[LAIBAO]  ugrade fail err=%d, line = %d.\n",
					err, __LINE__);
		}
		msleep(2000);
	}
#endif

//	pdata->set_probe_state(1);
	printk("FocalTech FT5X0X touch screen probe successfully!\n");

    enable_irq(g_dev->irq);

    return 0;

exit_input_register_device_failed:
	input_free_device(input_dev);
exit_input_dev_alloc_failed:
	free_irq(client->irq, ft5x0x_ts);
	//free_irq(IRQ_EINT(6), ft5x0x_ts);
exit_irq_request_failed:
exit_platform_data_null:
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
//	destroy_workqueue(ft5x0x_ts->ts_workqueue);
/*
exit_create_singlethread:
	printk("==singlethread error =\n");
	i2c_set_clientdata(client, NULL);
	kfree(ft5x0x_ts);	*/
exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

static int __devexit ft5x0x_ts_remove(struct i2c_client *client)
{
	struct ft5x0x_ts_dev *ft5x0x_ts = i2c_get_clientdata(client);
	unregister_early_suspend(&ft5x0x_ts->early_suspend);
	free_irq(client->irq, ft5x0x_ts);
	input_unregister_device(ft5x0x_ts->input_dev);
	kfree(ft5x0x_ts);
	cancel_work_sync(&ft5x0x_ts->pen_event_work);
	destroy_workqueue(ft5x0x_wq);
	i2c_set_clientdata(client, NULL);
	return 0;
}

static const struct i2c_device_id ft5x0x_ts_id[] = {
	{ FT5X0X_NAME, 0 },{ }
};
MODULE_DEVICE_TABLE(i2c, ft5x0x_ts_id);

static struct i2c_driver ft5x0x_ts_driver = {
	.probe		= ft5x0x_ts_probe,
	.remove		= __devexit_p(ft5x0x_ts_remove),
	.id_table	= ft5x0x_ts_id,
	.driver	= {
		.name	= FT5X0X_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __init ft5x0x_ts_init(void)
{
	ft5x0x_wq = create_singlethread_workqueue("ft5x0x_wq");		//create a work queue and worker thread
	if (!ft5x0x_wq) {
		printk(KERN_ALERT "creat workqueue faiked\n");
		return -ENOMEM;
	}

	return i2c_add_driver(&ft5x0x_ts_driver);
}

static void __exit ft5x0x_ts_exit(void)
{
	i2c_del_driver(&ft5x0x_ts_driver);
	if (ft5x0x_wq)
		destroy_workqueue(ft5x0x_wq);		//release our work queue	
}

module_init(ft5x0x_ts_init);
module_exit(ft5x0x_ts_exit);

MODULE_AUTHOR("<wenfs@Focaltech-systems.com>");
MODULE_DESCRIPTION("FocalTech ft5x0x TouchScreen driver");
MODULE_LICENSE("GPL");

