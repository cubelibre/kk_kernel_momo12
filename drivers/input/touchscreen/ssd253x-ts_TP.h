/**************************************************************
使用前注意通道数，驱动默认使用通道是sense 
大于drive否则需要将使用到的DRIVENO与SENSENO调换
此情况包括0x66和0x67寄存器，但不必修改。
***************************************************************/
#define DRIVENO	38
#define SENSENO	23		//10 inch 23*38

#define POLLING_INT		0
#define HYBRID_INT		1

#define ENABLE_INT		HYBRID_INT	// 0->Polling, 1->Hybrid
#define EdgeDisable		1	// if Edge Disable, set it to 1, else reset to 0
#define RunningAverageMode	2	//{0,8},{5,3},{6,2},{7,1}
#define RunningAverageDist	4	// Threshold Between two consecutive points
#define MicroTimeTInterupt	20000000// 100Hz - 10,000,000us
#define TimeoutInterupt	25000000	// 25ms, Timer out 25ms after the last INT interrupt
#define FINGERNO		10

//#define SSDS53X_SCREEN_MAX_X    1024	//800
//#define SSDS53X_SCREEN_MAX_Y    600		//480
//#define SCREEN_MAX_X    1024		//800
//#define SCREEN_MAX_Y    600			//480

#define MAX_TOUCH_MAJOR		10		//Charles added
#define MAX_WIDTH_MAJOR		15		//Charles added
#define MAX_TRACKID_ITEM		10	//Charles added

//#define RK29xx_ANDROID2_3_REPORT		//if the Android system is V2.3
//#undef RK29xx_ANDROID2_3_REPORT
#define RK29xx_ANDROID4_0_REPORT		//if the Android system is V4.0
//#undef RK29xx_ANDROID4_0_REPORT

#define SSD253x_REPORT_COORD_ORIGIN		0x01
#define SSD253x_REPORT_RATE_TIME		0x0A //0x0F
//----------------------------------------//
//#define TOUCH_INT_PIN				RK29_PIN0_PA2			//define INT Pin
//#define TOUCH_RESET_PIN			RK29_PIN6_PC3			//define Reset Pin
//#define SW_INT_IRQNO_PIO    TOUCH_INT_PIN

#define	SSD253X_I2C_RATE	400*1000   //400KHz

//#define USE_TOUCH_KEY
#define RST_PIN_RESUME		//if define, use Reset Pin to Resume the TP
//#undef RST_PIN_RESUME

#ifdef USE_TOUCH_KEY
const unsigned int TPKey_Flag_Mask[4] = { 0x01,0x02,0x04,0x08};
const uint32_t TPKey_code[4] ={ KEY_SEARCH,KEY_MENU,KEY_HOME,KEY_BACK };
#endif

//#define USE_CUT_EDGE    //0x8b must be 0x00;  EdgeDisable set 0

#ifdef USE_CUT_EDGE
		#define XPOS_MAX (DRIVENO - EdgeDisable) *64
		#define YPOS_MAX (SENSENO - EdgeDisable) *64
#endif //USE_CUT_EDGE


// SSD2533 Setting
// Touch Panel Example table1 

// For SSD2533 Bug Version Only //
//#define	SSD2533FIXEDCODE
struct ChipSetting ssd253xcfgTable1[]={
{ 1, 0xA4, 0x00, 0x00},			//MCU prescaler default=01
{ 1, 0xD4, 0x08, 0x00},			//Dummy Code
{ 1, 0xD4, 0x08, 0x00},			//Set Osc frequency default=8, range 0 to F
};

struct ChipSetting Reset[]={
{ 0, 0x04, 0x00, 0x00},	// SSD2533
};

struct ChipSetting Resume[]={
{ 0, 0x04, 0x00, 0x00},	// SSD2533
{ 1, 0x25,	SSD253x_REPORT_RATE_TIME,	0x00}, // Set Operation Mode   //Charles change to define
};

struct ChipSetting Suspend[] ={
{ 1, 0x25, 0x00, 0x00}, // Set Operation Mode
{ 0, 0x05, 0x00, 0x00},	// SSD2533
};

