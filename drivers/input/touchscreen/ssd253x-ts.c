#include <linux/interrupt.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>

#include <linux/input/mt.h>		//use slot B protocol, Android 4.0 system

#include <linux/init.h>
#include <linux/timer.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <mach/board.h>

#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif /* CONFIG_HAS_EARLYSUSPEND */

//#include <mach/gpio_v2.h>
#include <mach/irqs.h>
//#include <mach/script_v2.h>

//#define	CONFIG_TS_FUNCTION_CALLED_DEBUG			//Display the debug information whitch function is called
#define CONFIG_TS_PROBE_DEBUG		//Display the debug information in ssd253x_ts_probe function
//#define CONFIG_TS_I2C_TRANSFER_DEBUG		//Display the debug information of IIC transfer
//#define CONFIG_TPKEY_STATUS_DEBUG			//Display the debug information of Touch Key status
//#define CONFIG_TS_WORKQUEUE_DEBUG		//Display the debug ihnformation of creating work queue
//#define CONFIG_TS_COORDIATE_DEBUG		//
//#define CONFIG_TS_CUTEDGE_DEBUG			//

#define DEVICE_ID_REG                    2
#define VERSION_ID_REG                 3
#define AUTO_INIT_RST_REG             68
#define EVENT_STATUS                   121
#define EVENT_MSK_REG                 122
#define IRQ_MSK_REG                     123
#define FINGER01_REG                    124
#define EVENT_STACK                   	 128
#define EVENT_FIFO_SCLR               135
#define TIMESTAMP_REG                 136
#define SELFCAP_STATUS_REG         185

//static void* __iomem gpio_addr = NULL;
static int gpio_wakeup_hdle = 0;

#define SSD253X_I2C_NAME	"ssd253x-ts"

#include "ssd253x-ts_TP.h"

void deviceReset(struct i2c_client *client);
void deviceResume(struct i2c_client *client);
void deviceSuspend(struct i2c_client *client);
void SSD253xdeviceInit1(struct i2c_client *client);
void SSD253xdeviceInit(struct i2c_client *client);

static int ssd253x_ts_open(struct input_dev *dev);
static void ssd253x_ts_close(struct input_dev *dev);
static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg);
static int ssd253x_ts_resume(struct i2c_client *client);
#ifdef CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_early_suspend(struct early_suspend *h);
static void ssd253x_ts_late_resume(struct early_suspend *h);
#endif /* CONFIG_HAS_EARLYSUSPEND */

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer);
static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id);
static struct workqueue_struct *ssd253x_wq;
static struct ssd2533_platform_data *ssd2533_pdata;

int Ssd_record,Ssd_current,Ssd_Timer_flag;		//Change by Charles		//120613

struct ssl_ts_priv {
	struct i2c_client *client;
	struct input_dev *input;
	struct hrtimer timer;
	struct work_struct  ssl_work;
#ifdef	CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif

	int irq;
	int use_irq;
	int FingerNo;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];

	int Resolution;
	int EventStatus;
	int FingerDetect;

	int sFingerX[FINGERNO];
	int sFingerY[FINGERNO];
	int pFingerX[FINGERNO];
	int pFingerY[FINGERNO];
	int suspend_opend;
};

/***********************************************************
Read Data from TP through IIC
***********************************************************/
int ReadRegister(struct i2c_client *client,uint8_t reg,int ByteNo)
{
	unsigned char buf[4];
	struct i2c_msg msg[2];
	int ret = -1;

	memset(buf, 0xFF, sizeof(buf));
	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[0].scl_rate=SSD253X_I2C_RATE;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = ByteNo;
	msg[1].buf = buf;
	msg[1].scl_rate=SSD253X_I2C_RATE;

	ret = i2c_transfer(client->adapter, msg, 2);
	if(ret<0)	return -1;

	if(ByteNo==1) return (int)((unsigned int)buf[0]<<0);
	if(ByteNo==2) return (int)((unsigned int)buf[1]<<0)|((unsigned int)buf[0]<<8);
	if(ByteNo==3) return (int)((unsigned int)buf[2]<<0)|((unsigned int)buf[1]<<8)|((unsigned int)buf[0]<<16);
	if(ByteNo==4) return (int)((unsigned int)buf[3]<<0)|((unsigned int)buf[2]<<8)|((unsigned int)buf[1]<<16)|(buf[0]<<24);
	return 0;
}

/***********************************************************
Write Data to TP through IIC
***********************************************************/
int WriteRegister(struct i2c_client *client,uint8_t Reg,unsigned char Data1,unsigned char Data2,int ByteNo)
{
	struct i2c_msg msg;
	unsigned char buf[4];
	int ret = -1;

	buf[0]=Reg;
	buf[1]=Data1;
	buf[2]=Data2;
	buf[3]=0;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = ByteNo+1;
	msg.buf = (char *)buf;
	msg.scl_rate=SSD253X_I2C_RATE;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret;
}

void SSD253xdeviceInit1(struct i2c_client *client)
{
#ifdef	SSD2533FIXEDCODE
	int i;
	mdelay(600); //SSD2533 ESD2 EEPROM VERSION
	for(i=0;i<sizeof(ssd253xcfgTable1)/sizeof(ssd253xcfgTable1[0]);i++)
	{
		WriteRegister(	client,ssd253xcfgTable1[i].Reg,
				ssd253xcfgTable1[i].Data1,ssd253xcfgTable1[i].Data2,
				ssd253xcfgTable1[i].No);
	}
#endif
}

void SSD253xdeviceInit(struct i2c_client *client)
{
	int i;
/*
	for(i=0;i<sizeof(ssd253xcfg_Table1)/sizeof(ssd253xcfg_Table1[0]);i++)
	{
		WriteRegister(	client,ssd253xcfg_Table1[i].Reg,
				ssd253xcfg_Table1[i].Data1,ssd253xcfg_Table1[i].Data2,
				ssd253xcfg_Table1[i].No);
	}
	*/
	for (i=0; i<ssd2533_pdata->config_info_len; i++)
	{
		WriteRegister(client, 
			ssd2533_pdata->chipsetting_config[i].Reg,
			ssd2533_pdata->chipsetting_config[i].Data1,
			ssd2533_pdata->chipsetting_config[i].Data2,
			ssd2533_pdata->chipsetting_config[i].No);
	}

	mdelay(10);
}

void deviceReset(struct i2c_client *client)
{
	int i;

	gpio_set_value(ssd2533_pdata->gpio_rst, 0);
	mdelay(5);
	gpio_set_value(ssd2533_pdata->gpio_rst,1);
	mdelay(10);

	for(i=0;i<sizeof(Reset)/sizeof(Reset[0]);i++)
	{
		WriteRegister(	client,Reset[i].Reg,
				Reset[i].Data1,Reset[i].Data2,
				Reset[i].No);
	}
//	mdelay(100);
//	SSD253xdeviceInit1(client);
}

void deviceResume(struct i2c_client *client)
{	
#ifdef RST_PIN_RESUME
	//RST pin pull down
/*	gpio_set_value(ssd2533_pdata->gpio_rst, 0);
	mdelay(5);
	gpio_set_value(ssd2533_pdata->gpio_rst, 1);
	mdelay(2);
*/	deviceReset(client);
	mdelay(50);
	SSD253xdeviceInit(client);
#else
	int i;
	//int timeout=10;
	//int status;
	for(i=0;i<sizeof(Resume)/sizeof(Resume[0]);i++)
	{
		WriteRegister(	client,Resume[i].Reg,
				Resume[i].Data1,Resume[i].Data2,
				Resume[i].No);
		mdelay(100);
	}
	/*
	do {
		status=ReadRegister(client,0x26,1);
		printk("		deviceResume: status : %d !\n",status);
		if(status==Resume[2].Data1) break;
		mdelay(1);
	}while(timeout--); // Check the status
	*/
#endif
}

void deviceSuspend(struct i2c_client *client)
{	
#ifdef RST_PIN_RESUME
	//RST pin pull down
	gpio_set_value(ssd2533_pdata->gpio_rst, 0);
	mdelay(5);
	gpio_set_value(ssd2533_pdata->gpio_rst, 1);
	mdelay(2);
#else
	int i;
	//int timeout=10;
	//int status;
	
	/*
	WriteRegister(	client,Suspend[0].Reg,
			Suspend[0].Data1,Suspend[0].Data2,
			Suspend[0].No);
	do {
		status=ReadRegister(client,0x26,1);
		if(status==Suspend[0].Data1) break;
		mdelay(1);				
	}while(timeout--);
	*/
	
	for(i=0;i<sizeof(Suspend)/sizeof(Suspend[0]);i++)
	{
		WriteRegister(	client,Suspend[i].Reg,
				Suspend[i].Data1,Suspend[i].Data2,
				Suspend[i].No);
		mdelay(100);
	}
#endif
}

#define Mode RunningAverageMode
#define Dist RunningAverageDist
void RunningAverage(unsigned short *xpos,unsigned short *ypos,int No,struct ssl_ts_priv *ssl_priv)
{	
	int FilterMode[4][2]={{0,8},{5,3},{6,2},{7,1}};
	int dx,dy;
	int X,Y;

	X=*xpos;
	Y=*ypos;
	if((ssl_priv->pFingerX[No]!=0x0FFF)&&(X!=0x0FFF))
	{
		dx=abs(ssl_priv->pFingerX[No]-X);
		dy=abs(ssl_priv->pFingerY[No]-Y);
		if(dx+dy<Dist*64)
		{
			ssl_priv->pFingerX[No]=(FilterMode[Mode][0]*ssl_priv->pFingerX[No]+FilterMode[Mode][1]*X)/8;
			ssl_priv->pFingerY[No]=(FilterMode[Mode][0]*ssl_priv->pFingerY[No]+FilterMode[Mode][1]*Y)/8;
		}
		else
		{
			ssl_priv->pFingerX[No]=X;
			ssl_priv->pFingerY[No]=Y;
		}
	}
	else
	{
		ssl_priv->pFingerX[No]=X;
		ssl_priv->pFingerY[No]=Y;
	}
	*xpos=ssl_priv->pFingerX[No];
	*ypos=ssl_priv->pFingerY[No];
}

void FingerCheckSwap(int *FingerX,int *FingerY,int *FingerP,int FingerNo,int *sFingerX,int *sFingerY)
{
  	int i,j;
  	int index1,index2;
  	int Vx,Vy;
  	int Ux,Uy;
  	int R1x,R1y;
  	int R2x,R2y;
	for(i=0;i<FingerNo;i++)
  	{
 		index1=i;
	    	if( FingerX[index1]!=0xFFF)
		if(sFingerX[index1]!=0xFFF) 
		{
			for(j=i+1;j<FingerNo+3;j++)
			{
				index2=j%FingerNo;
	    			if( FingerX[index2]!=0xFFF)
				if(sFingerX[index2]!=0xFFF) 
		    		{
					Ux=sFingerX[index1]-sFingerX[index2];
					Uy=sFingerY[index1]-sFingerY[index2];      
					Vx= FingerX[index1]- FingerX[index2];
					Vy= FingerY[index1]- FingerY[index2];					

					R1x=Ux-Vx;
					R1y=Uy-Vy;
					R2x=Ux+Vx;
					R2y=Uy+Vy;
							
					R1x=R1x*R1x;
					R1y=R1y*R1y; 
					R2x=R2x*R2x;
					R2y=R2y*R2y;

					if(R1x+R1y>R2x+R2y)
				    	{
				    		Ux=FingerX[index1];
						Uy=FingerY[index1];
						Vx=FingerP[index1];
							          
						FingerX[index1]=FingerX[index2];
						FingerY[index1]=FingerY[index2];
						FingerP[index1]=FingerP[index2];
							
						FingerX[index2]=Ux;
						FingerY[index2]=Uy;
						FingerP[index2]=Vx;
					}
					break;
			    	}
			}
		}
  	}        
  	for(i=0;i<FingerNo;i++)
  	{
    		sFingerX[i]=FingerX[i];
    		sFingerY[i]=FingerY[i];
  	}
}

#ifdef USE_TOUCH_KEY
static void ssd2533_ts_send_keyevent(struct ssl_ts_priv *ssl_priv,u8 btn_status, int downup)
{
	switch(btn_status & 0x0f)
	{
		case TPKey_Flag_Mask[0]:
			input_report_key(ssl_priv->input, TPKey_code[0], downup);
			break;
		case TPKey_Flag_Mask[1]:
			input_report_key(ssl_priv->input, TPKey_code[1], downup);
			break;
		case TPKey_Flag_Mask[2]:
			input_report_key(ssl_priv->input, TPKey_code[2], downup);
			break;
		case TPKey_Flag_Mask[3]:
			input_report_key(ssl_priv->input, TPKey_code[3], downup);
			break;
		default:
			break;
	}
	#ifdef CONFIG_TPKEY_STATUS_DEBUG
	printk("send %x %x\n", btn_status, downup);
	#endif
}
#endif

#ifdef USE_CUT_EDGE
static int ssd253x_ts_cut_edge(unsigned short pos,unsigned short x_y)
{
	u8 cut_value = 10; //cut_value < 32
	if(pos == 0xfff)
	{
		return pos;
	}
	if(x_y) //xpos
	{
		#ifdef CONFIG_TS_CUTEDGE_DEBUG
			printk("X: Raw data %d\n",pos);
		#endif
		if(pos < 16)
			pos = cut_value + pos*(48 - cut_value) / 16;
		else if(pos > (XPOS_MAX - 16) )
			pos = XPOS_MAX + 16 + (pos - (XPOS_MAX -16))*(48 - cut_value) / 16;
		else
			pos = pos + 32;
			pos = ssd2533_pdata->screen_max_x * pos / (DRIVENO * 64);
		#ifdef CONFIG_TS_CUTEDGE_DEBUG
				printk("X: Cut edge data %d\n",pos);
		#endif
		return pos;
	}
	else    //ypos
	{

		#ifdef CONFIG_TS_CUTEDGE_DEBUG
			printk("Y: Raw data %d\n",pos);
		#endif
		if(pos < 16)
			pos = cut_value + pos*(48 - cut_value) / 16;
		else if(pos > (YPOS_MAX - 16) )
			pos = YPOS_MAX + 16 + (pos - (YPOS_MAX -16))*(48 - cut_value) / 16;
		else
			pos = pos + 32;
			pos = ssd2533_pdata->screen_max_y * pos / (SENSENO * 64);
		#ifdef CONFIG_TS_CUTEDGE_DEBUG
				printk("X: Cut edge data %d\n",pos);
		#endif
		return pos;
	}
	
	
}
#endif

#ifdef USE_TOUCH_KEY
static u8 btn_status_last = 0;
#endif
	
static void ssd253x_ts_work(struct work_struct *work)
{
	int i;
	unsigned short xpos=0, ypos=0, width=0;
	int FingerInfo;
	int EventStatus;
	int FingerX[FINGERNO];
	int FingerY[FINGERNO];
	int FingerP[FINGERNO];
	int clrFlag=0;
	int Ssd_Timer;
	#ifdef USE_TOUCH_KEY
	u8 btn_status;
	#endif
	

	struct ssl_ts_priv *ssl_priv = container_of(work,struct ssl_ts_priv,ssl_work);

	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_work!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif

	if (ssl_priv->suspend_opend == 1)
		return ;
	#ifdef USE_TOUCH_KEY
	btn_status = ReadRegister(ssl_priv->client,SELFCAP_STATUS_REG, 1);
	btn_status &= 0x0f;
	#ifdef CONFIG_TPKEY_STATUS_DEBUG
		printk("btn pressed:%x\n", btn_status & 0x0f);
	#endif
	if((ssl_priv->use_irq==HYBRID_INT) && (btn_status!=0))
	{
			hrtimer_start(&ssl_priv->timer, ktime_set(0, TimeoutInterupt), HRTIMER_MODE_REL);		//start the timeout interrupt
	}
	if (btn_status_last != btn_status)
	{
		if(btn_status)
		{
			btn_status_last = btn_status;
			ssd2533_ts_send_keyevent(ssl_priv,btn_status, 1);
		#ifdef CONFIG_TPKEY_STATUS_DEBUG
			printk("send %x btn_status_last%d \n", btn_status,btn_status_last);
		#endif
		}
		else
		{
			ssd2533_ts_send_keyevent(ssl_priv,btn_status_last, 0);
			btn_status_last = 0;
			#ifdef CONFIG_TPKEY_STATUS_DEBUG
					printk("btn_status_last %x \n", btn_status_last);
			#endif
		}
		return ;
	}
	#endif
	if(!Ssd_Timer_flag)
	{
		Ssd_Timer = ReadRegister(ssl_priv->client,TIMESTAMP_REG,2);
		if(!Ssd_record)                                      
		{
				Ssd_record = Ssd_Timer/1000;   			
		}
		
		Ssd_current = Ssd_Timer/1000;               
		if(Ssd_current < Ssd_record)
       {
       		Ssd_current += 0xffff/1000;
		}
		
		if((Ssd_current - Ssd_record) > 5)
		{
		WriteRegister(ssl_priv->client,AUTO_INIT_RST_REG,0x00,0x00,1);
		Ssd_Timer_flag = 1;
		Ssd_record = 0;
		}
	 }

	EventStatus = ReadRegister(ssl_priv->client,EVENT_STATUS,2)>>4;
	ssl_priv->FingerDetect=0;
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		if((EventStatus>>i)&0x1)
		{
			FingerInfo=ReadRegister(ssl_priv->client,FINGER01_REG+i,4);
			xpos = ((FingerInfo>>4)&0xF00)|((FingerInfo>>24)&0xFF);
			ypos = ((FingerInfo>>0)&0xF00)|((FingerInfo>>16)&0xFF);
			//width= ((FingerInfo>>4)&0x00F);	

			if(xpos!=0xFFF)
			{
				ssl_priv->FingerDetect++;
			#ifdef USE_CUT_EDGE
				xpos = ssd253x_ts_cut_edge(xpos, 1);
				ypos = ssd253x_ts_cut_edge(ypos, 0);
			#endif
			}
			else 
			{
				// This part is to avoid asyn problem when the finger leaves
				EventStatus=EventStatus&~(1<<i);
				clrFlag=1;
			}
		}
		else
		{
			xpos=ypos=0xFFF;
			//width=0;
			clrFlag=1;
		}
		FingerX[i]=xpos;
		FingerY[i]=ypos;
		//FingerP[i]=width;
	}
	if((ssl_priv->use_irq==HYBRID_INT) && (ssl_priv->FingerDetect!=0))
	{
			hrtimer_start(&ssl_priv->timer, ktime_set(0, TimeoutInterupt), HRTIMER_MODE_REL);		//start the timeout interrupt
	}
	if(clrFlag) WriteRegister(ssl_priv->client,EVENT_FIFO_SCLR,0x01,0x00,1);

	if(ssl_priv->input->id.product==0x2533)
	if(ssl_priv->input->id.version==0x0101) 
		FingerCheckSwap(FingerX,FingerY,FingerP,ssl_priv->FingerNo,ssl_priv->sFingerX,ssl_priv->sFingerY);

	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		xpos=FingerX[i];
		ypos=FingerY[i];
		//width=FingerP[i];
		if(ssl_priv->input->id.product==0x2533)
		{
			if(ssl_priv->input->id.version==0x0101) RunningAverage(&xpos,&ypos,i,ssl_priv);
			if(ssl_priv->input->id.version==0x0102) RunningAverage(&xpos,&ypos,i,ssl_priv);
		}

		if(xpos!=0xFFF)
		{
			#ifdef RK29xx_ANDROID2_3_REPORT
				input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);  
				input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, MAX_TOUCH_MAJOR);
				input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
				input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
				input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, MAX_WIDTH_MAJOR);//width);
				input_mt_sync(ssl_priv->input);
			#else
				if (ssd2533_pdata ->swap_xy)
					swap(xpos, ypos);
				if (ssd2533_pdata ->xpol)
					xpos = ssd2533_pdata ->screen_max_x -xpos;
				if (ssd2533_pdata ->ypol)
					ypos = ssd2533_pdata ->screen_max_y -ypos;
				
				input_mt_slot(ssl_priv->input, i);		//Slot B protocol
				input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, MAX_TOUCH_MAJOR); //Finger Size
				input_report_abs(ssl_priv->input, ABS_MT_POSITION_X, xpos);
				input_report_abs(ssl_priv->input, ABS_MT_POSITION_Y, ypos);
				input_report_abs(ssl_priv->input, ABS_MT_WIDTH_MAJOR, MAX_WIDTH_MAJOR); //Touch Size
			#endif

		//	#ifdef CONFIG_TS_COORDIATE_DEBUG
		//	if(i==0)
		//		printk("		ssd253x_ts_work: X = %d , Y = %d, W = 0x%x\n",xpos,ypos,width);
		//	#endif
		}
		else if(ssl_priv->FingerX[i]!=0xFFF)
		{
			#ifdef RK29xx_ANDROID2_3_REPORT
				input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(ssl_priv->input, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(ssl_priv->input);
			#else
				input_mt_slot(ssl_priv->input, i);
				input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, -1);
			#endif
			#ifdef CONFIG_TS_COORDIATE_DEBUG
				if(i==0) printk("	release	ssd253x_ts_work: X = 0x%x , Y = 0x%x, W = 0x%x\n",xpos,ypos,width);
			#endif
		}
		ssl_priv->FingerX[i]=FingerX[i];
		ssl_priv->FingerY[i]=FingerY[i];
		//ssl_priv->FingerP[i]=width;
	}		
	ssl_priv->EventStatus=EventStatus;	
	input_sync(ssl_priv->input);
}

static int ssd253x_init_platform_hw(void)
{
    if(gpio_request(ssd2533_pdata->gpio_rst, NULL) != 0){
      gpio_free(ssd2533_pdata->gpio_rst);
      printk("ssd2533_init_platform_hw gpio_request error\n");
      return -EIO;
    }

    if(gpio_request(ssd2533_pdata->gpio_irq, NULL) != 0){
      gpio_free(ssd2533_pdata->gpio_irq);
      printk("ssd2533_init_platform_hw gpio_request error\n");
      return -EIO;
    }
    gpio_pull_updown(ssd2533_pdata->gpio_irq, 1);
    gpio_direction_output(ssd2533_pdata->gpio_rst, 0);
    return 0;
}

static int ssd253x_ts_probe(struct i2c_client *client,const struct i2c_device_id *idp)
{
	struct ssl_ts_priv *ssl_priv;
	struct input_dev *ssl_input;
	int ret = 0;
	struct ssd2533_platform_data *pdata = client->dev.platform_data;
	int error = 0, i;		//,reg_val,err;

//	if (pdata->get_probe_state() == 1)
//		return error;
	
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
		printk("+-----------------------------------------+\n");
		printk("|	ssd253x_ts_probe!                 |\n");
		printk("+-----------------------------------------+\n");
	#endif

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: need I2C_FUNC_I2C\n");
		#endif
		return -ENODEV;
	}
	else
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: i2c Check OK!\n");
			printk("		ssd253x_ts_probe: i2c_client name : %s\n",client->name);
		#endif
	}

	ssl_priv = kzalloc(sizeof(*ssl_priv), GFP_KERNEL);
	if (!ssl_priv)
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: kzalloc Error!\n");
		#endif
		error=-ENODEV;
		goto	err0;
	}
	else
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: kzalloc OK!\n");
		#endif
	}

	ssd2533_pdata = client->dev.platform_data;
	ssd253x_init_platform_hw();		//Init RK29 GPIO

	dev_set_drvdata(&client->dev, ssl_priv);
	
	ssl_input = input_allocate_device();
	if (!ssl_input)
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: input_allocate_device Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: input_allocate_device OK\n");
		#endif
	}

//	ssl_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN)|BIT_MASK(EV_REP) ;
//	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_2);
	ssl_input->name = client->name;
	ssl_input->id.bustype = BUS_I2C;
	ssl_input->id.vendor  = 0x2878; // Modify for Vendor ID
	ssl_input->dev.parent = &client->dev;
	ssl_input->open = ssd253x_ts_open;
	ssl_input->close = ssd253x_ts_close;

	input_set_drvdata(ssl_input, ssl_priv);
	ssl_priv->client = client;
	ssl_priv->input = ssl_input;
	ssl_priv->use_irq = ENABLE_INT;
	ssl_priv->irq = ssd2533_pdata->gpio_irq;
	ssl_priv->FingerNo=FINGERNO;
	ssl_priv->Resolution=64;
	#ifdef RK29xx_ANDROID4_0_REPORT
	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
	ssl_input->absbit[0] = BIT(ABS_X) | BIT(ABS_Y); // for android

	__set_bit(EV_ABS, ssl_input->evbit);
	__set_bit(INPUT_PROP_DIRECT, ssl_input->propbit);
	set_bit(ABS_MT_POSITION_X, ssl_input->absbit);
	set_bit(ABS_MT_POSITION_Y, ssl_input->absbit);
	set_bit(ABS_MT_TOUCH_MAJOR, ssl_input->absbit);
	set_bit(ABS_MT_WIDTH_MAJOR, ssl_input->absbit);
	#endif
	
	for(i=0;i<ssl_priv->FingerNo;i++)
	{
		//ssl_priv->FingerP[i]=0;
		// For Finger Check Swap
		ssl_priv->sFingerX[i]=0xFFF;
		ssl_priv->sFingerY[i]=0xFFF;

		// For Adaptive Running Average
		ssl_priv->pFingerX[i]=0xFFF;
		ssl_priv->pFingerY[i]=0xFFF;
	}

	deviceReset(client);
	mdelay(200);
	ret = ReadRegister(client, DEVICE_ID_REG, 2);
	if (ret < 0)
	{
		printk(KERN_ALERT "ssd253x touch screen not found \n");
		gpio_free(pdata->gpio_rst);
		gpio_free(pdata->gpio_irq);
		kfree(ssl_priv);
		goto err0;
	}
	
	ssl_input->id.product = ReadRegister(client, DEVICE_ID_REG, 2);
	ssl_input->id.version = ReadRegister(client,VERSION_ID_REG, 2);
	printk("SSL Touchscreen Device ID  : 0x%04X\n",ssl_input->id.product);
	printk("SSL Touchscreen Version ID : 0x%04X\n",ssl_input->id.version);


	if(ssl_priv->input->id.product==0x2531)		ssl_priv->Resolution=32;
	else if(ssl_priv->input->id.product==0x2533)	ssl_priv->Resolution=64;
	else
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: ssl_input->id.product Error\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	SSD253xdeviceInit(client);
	WriteRegister(client,EVENT_FIFO_SCLR,0x01,0x00,1); // clear Event FiFo
	#ifdef CONFIG_TS_PROBE_DEBUG
		printk("		ssd253X_ts_probe: %04XdeviceInit OK!\n",ssl_input->id.product);
	#endif

	#ifdef RK29xx_ANDROID2_3_REPORT
   	ssl_input->evbit[0] = BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS) | BIT_MASK(EV_SYN) ;
	ssl_input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH) | BIT_MASK(BTN_2);
	input_set_abs_params(ssl_input, ABS_MT_TRACKING_ID, 0,MAX_TRACKID_ITEM, 0, 0);
	#else
	input_mt_init_slots(ssl_input, FINGERNO);
	#endif

	input_set_abs_params(ssl_input,ABS_MT_POSITION_X, 0, pdata->screen_max_x, 0, 0);
	input_set_abs_params(ssl_input,ABS_MT_POSITION_Y, 0, pdata->screen_max_y, 0, 0);
	input_set_abs_params(ssl_input,ABS_MT_TOUCH_MAJOR, 0, MAX_TOUCH_MAJOR, 0, 0);
	input_set_abs_params(ssl_input,ABS_MT_WIDTH_MAJOR, 0, MAX_WIDTH_MAJOR, 0, 0);


#ifdef USE_TOUCH_KEY
	set_bit(KEY_MENU, ssl_input->keybit);
	set_bit(KEY_HOME, ssl_input->keybit);
	set_bit(KEY_BACK, ssl_input->keybit);
	set_bit(KEY_SEARCH, ssl_input->keybit);
#endif
//	pdata->set_probe_state(1);
	printk("Install solomon ssd253x touch driver successfully\n");
	
	INIT_WORK(&ssl_priv->ssl_work, ssd253x_ts_work);

	if (ssl_priv->use_irq==HYBRID_INT)
	{
		// Options for different interrupt system
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_DISABLED|IRQF_TRIGGER_FALLING, client->name,ssl_priv);
//		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING, client->name,ssl_priv);
		error = request_irq(ssl_priv->irq, ssd253x_ts_isr, IRQF_TRIGGER_FALLING, client->name,ssl_priv);
		if(error)
		{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: request_irq Error!\n");
		#endif
			error=-ENODEV;
			goto err2;
		}
		else
		{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: request_irq OK!\n");
		#endif
		}	
		disable_irq(ssl_priv->irq);
	}
	{
		hrtimer_init(&ssl_priv->timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		ssl_priv->timer.function = ssd253x_ts_timer;
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: timer_init OK!\n");
		#endif
	}

	error = input_register_device(ssl_input);
	if(error)
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: input_register_device input Error!\n");
		#endif
		error=-ENODEV;
		goto	err1;
	}
	else
	{
		#ifdef CONFIG_TS_PROBE_DEBUG
			printk("		ssd253x_ts_probe: input_register_device input OK!\n");
		#endif
	}

#ifdef	CONFIG_HAS_EARLYSUSPEND
	ssl_priv->early_suspend.suspend = ssd253x_ts_early_suspend;
	ssl_priv->early_suspend.resume  = ssd253x_ts_late_resume;
	ssl_priv->early_suspend.level   = EARLY_SUSPEND_LEVEL_DISABLE_FB+1;
	register_early_suspend(&ssl_priv->early_suspend);
#endif

	return 0;
err2:	input_unregister_device(ssl_input);
err1:	input_free_device(ssl_input);
	kfree(ssl_priv);
//exit_gpio_wakeup_request_failed:
err0:	dev_set_drvdata(&client->dev, NULL);
	return error;
}

static int ssd253x_ts_open(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_open!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif	
//	deviceResume(ssl_priv->client);
	if(ssl_priv->use_irq!=POLLING_INT)
		{
			enable_irq(ssl_priv->irq);
		}
	else
		{
			hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
		ssl_priv->suspend_opend = 0;
	return 0;
}

static void ssd253x_ts_close(struct input_dev *dev)
{
	struct ssl_ts_priv *ssl_priv = input_get_drvdata(dev);
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_close!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
//	deviceSuspend(ssl_priv->client);
	
	ssl_priv->suspend_opend = 1;
	hrtimer_cancel(&ssl_priv->timer);
	if(ssl_priv->use_irq==HYBRID_INT)
		{
			disable_irq(ssl_priv->irq);
		}
}

static int ssd253x_ts_resume(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	int i;

	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_resume!                |\n");
	printk("+-----------------------------------------+\n");
	#endif
	Ssd_Timer_flag = 0;
	Ssd_record = 0;

	for ( i= 0; i<FINGERNO; ++i )
	{
		input_mt_slot(ssl_priv->input, i);
		input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(ssl_priv->input);

	deviceResume(client);
	ssl_priv->suspend_opend = 0;		
	if(ssl_priv->use_irq!=POLLING_INT) 
		{
			enable_irq(ssl_priv->irq);
		}
	else 
		{
			hrtimer_start(&ssl_priv->timer, ktime_set(1, 0), HRTIMER_MODE_REL);
		}
	return 0;
}

static int ssd253x_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	int i;

	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_suspend!               |\n");
	printk("+-----------------------------------------+\n");
	#endif

	for ( i= 0; i<FINGERNO; ++i )
	{
		input_mt_slot(ssl_priv->input, i);
		input_report_abs(ssl_priv->input, ABS_MT_TRACKING_ID, -1);
	}
	input_sync(ssl_priv->input);

	ssl_priv->suspend_opend = 1;
	hrtimer_cancel(&ssl_priv->timer);
	
	if (ssl_priv->use_irq==HYBRID_INT)
		{
			disable_irq(ssl_priv->irq);
		}
	Ssd_Timer_flag = 0;
	Ssd_record = 0;
	deviceSuspend(client);
	return 0;
}

#ifdef	CONFIG_HAS_EARLYSUSPEND
static void ssd253x_ts_late_resume(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_late_resume!           |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_resume(ssl_priv->client);
}
static void ssd253x_ts_early_suspend(struct early_suspend *h)
{
	struct ssl_ts_priv *ssl_priv = container_of(h, struct ssl_ts_priv, early_suspend);
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_early_suspend!         |\n");
	printk("+-----------------------------------------+\n");
	#endif
	ssd253x_ts_suspend(ssl_priv->client, PMSG_SUSPEND);
}
#endif

static int ssd253x_ts_remove(struct i2c_client *client)
{
	struct ssl_ts_priv *ssl_priv = dev_get_drvdata(&client->dev);
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_remove !               |\n");
	printk("+-----------------------------------------+\n");
	#endif
	//if((ssl_priv->use_irq==POLLING_INT)||(ssl_priv->use_irq==HYBRID_INT)) 
	hrtimer_cancel(&ssl_priv->timer);
	if (ssl_priv->use_irq==HYBRID_INT) 
		{
			free_irq(ssl_priv->irq, ssl_priv);
		}
	input_unregister_device(ssl_priv->input);
	input_free_device(ssl_priv->input);
	kfree(ssl_priv);
	dev_set_drvdata(&client->dev, NULL);
	return 0;
}

static irqreturn_t ssd253x_ts_isr(int irq, void *dev_id)
{
	struct ssl_ts_priv *ssl_priv = dev_id;
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_isr!                   |\n");
	printk("+-----------------------------------------+\n");
	#endif
	disable_irq_nosync(ssl_priv->irq);
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	enable_irq(ssl_priv->irq);		//Charles add,avoid the unbalance int
	return IRQ_HANDLED;
}

static enum hrtimer_restart ssd253x_ts_timer(struct hrtimer *timer)
{
	struct ssl_ts_priv *ssl_priv = container_of(timer, struct ssl_ts_priv, timer);
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_timer!                 |\n");
	printk("+-----------------------------------------+\n");
	#endif
	queue_work(ssd253x_wq, &ssl_priv->ssl_work);
	if(ssl_priv->use_irq==POLLING_INT) 
		{
			hrtimer_start(&ssl_priv->timer, ktime_set(0, MicroTimeTInterupt), HRTIMER_MODE_REL);
		}
	return HRTIMER_NORESTART;
}

static const struct i2c_device_id ssd253x_ts_id[] = {
	{ SSD253X_I2C_NAME, 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, ssd253x_ts_id);

static struct i2c_driver ssd253x_ts_driver = {
	.driver = {
		.name = SSD253X_I2C_NAME,
	},
	.probe = ssd253x_ts_probe,
	.remove = ssd253x_ts_remove,
#ifndef	CONFIG_HAS_EARLYSUSPEND
	.suspend = ssd253x_ts_suspend,
	.resume = ssd253x_ts_resume,
#endif
	.id_table = ssd253x_ts_id,
};

static char banner[] __initdata = KERN_INFO "SSL Touchscreen driver, (c) 2011 Solomon Systech Ltd.\n";
static int __init ssd253x_ts_init(void)
{
	int ret;
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	SSL_ts_init!                      |\n");
	printk("+-----------------------------------------+\n");
	#endif
	printk(banner);
	ssd253x_wq = create_singlethread_workqueue("ssd253x_wq");
	if (!ssd253x_wq)
	{
		#ifdef CONFIG_TS_WORKQUEUE_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue Error!\n");
		#endif
		return -ENOMEM;
	}
	else
	{
		#ifdef CONFIG_TS_WORKQUEUE_DEBUG
		printk("		ssd253x_ts_init: create_singlethread_workqueue OK!\n");
		#endif
	}
	ret=i2c_add_driver(&ssd253x_ts_driver);
	#ifdef CONFIG_TS_I2C_TRANSFER_DEBUG
	if(ret) printk("		ssd253x_ts_init: i2c_add_driver Error! \n");
	else    printk("		ssd253x_ts_init: i2c_add_driver OK! \n");
	#endif
	return ret;
}

static void __exit ssd253x_ts_exit(void)
{
	#ifdef CONFIG_TS_FUNCTION_CALLED_DEBUG
	printk("+-----------------------------------------+\n");
	printk("|	ssd253x_ts_exit!                  |\n");
	printk("+-----------------------------------------+\n");
	#endif
	i2c_del_driver(&ssd253x_ts_driver);
	if (ssd253x_wq) destroy_workqueue(ssd253x_wq);
}

module_init(ssd253x_ts_init);
module_exit(ssd253x_ts_exit);

MODULE_AUTHOR("Solomon Systech Ltd - Design Technology, Icarus Choi");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("ssd253x Touchscreen Driver 1.5_Charles@Raysens@20120613");
