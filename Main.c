/*------------------------------------------------------------------------------------------------*/
/*                                                                                                */
/*           Copyright (C) 2018 NeuronBasic Co., Ltd. All rights reserved.                        */
/*                                                                                                */
/*------------------------------------------------------------------------------------------------*/
#include "nbsdk.h"
#include "User_Config.h"
#include "nb_motion_api.h"
#include "Basic.h"
#include "nb_detect_api.h"
#include "sensor_aeg.h"



/**************************************************************************
 *	Constants
 **************************************************************************/
#define BAUD_RATE 19200 // 230400//115200

#define HOST_SECTION __attribute__((section(".ver_host")))

#define PROJECT_TAG "Night"
#define PROJECT_MAJOR_VERSION 1
#define PROJECT_MINOR_VERSION 0
#define PROJECT_TEST_VERSION 0

#define TASK_DELAY_TIME 2

#define TRUE 1
#define FALSE 0

#define segment	0
#define   dig1 IOC3 //9 (個位數)
#define   dig2 IOC2 //6 (十位數)
#define   dig3 IOC4 

/* Macros */
#define DELAY_TIME 			2
#define STATE_BUF_SIZE 		6

/* Night Light Pin Define and PWM freq*/
#define MAIN_LIGHT_PIN  IOA1
#define IR_850NM_PIN IOA3
#define LEFT_TIME_LED_PIN IOC5
#define LOCK_STATE_LED_PIN IOA4
#define PWM_FREQ 1000000
/*Use for conditional judgement*/
#define MAX_LIGHT_LIMIT 100//最大亮度上限
#define MAX_LIGHT_LOWER_LIMIT 12//最大亮度下限，，，亮度分級12、25、50、100
#define LIGHT_LOWER_LIMIT 0//燈具佔空比下限
#define CLOSED 101// five state of light
#define OPENING 102
#define CLOSING 103
#define LIGHT_ADJUSTABLE 104
#define LIGHT_LOCKED 105

/**************************************************************************
 *	External Function
 **************************************************************************/
extern void printf_ctrl(u8 flag);

/**************************************************************************
 *	Global Data
 **************************************************************************/
static const char HOST_SECTION host_ver[] = "HOST-"__DATE__
											"-"__TIME__;
static TaskHandle_t xHandle_Win = NULL;
static TaskHandle_t xHandle_User = NULL;
static TaskHandle_t xHandle_Feature = NULL;
static TaskHandle_t xHandle_Result = NULL;
static TaskHandle_t xHandle_Seg = NULL;
static TaskHandle_t xHandle_softPWM = NULL;


unsigned char detect_enable = FALSE;
unsigned long int win_flow = 0;
unsigned long int detected_flow = 0;
unsigned long int frame_id = 0;
unsigned char use_state = 0;

u32 Sys_Tick = 0;

u8 task_init = FALSE;
/*Night light*/
int current_state = CLOSED;//初始化當前燈光狀態
int current_max_light = MAX_LIGHT_LIMIT;//初始化當前最大亮度
int main_light_duty_cycle = 0;//主燈佔空比，表示亮度
int left_time_led_duty_cycle;//剩餘時間LED佔空比，表示剩餘時間

unsigned long int last_frame = 0;//上一輪的幀數
unsigned long int current_frame;//此輪幀數
unsigned long int last_exp = 396;//上一輪曝光值
unsigned long int current_exp;//此輪曝光值
unsigned long int state_stored_frame = 0;//轉換狀態後累積的幀數
unsigned long int time_stored_frame = 0;//未識別到物件累積的幀數(時間)
unsigned long int light_stored_frame = 0;//轉換亮度後累積的幀數

#ifdef DEBUG_FOUND_DATA

#define DEBUG_FOUND_DATA_SAVE_MS			20
#define DEBUG_FOUND_DATA_OUT_MS				(2 * 60 * 1000)

uint32_t debug_found_data_save_tick = 0;
uint32_t debug_found_data_out_tick = 0;

// 1min ->60s->50fps
u8 debug_found_data[DEBUG_FOUND_DATA_OUT_MS / DEBUG_FOUND_DATA_SAVE_MS + 10] = {0};
u16 debug_found_data_item = 0;

#endif

u16 again_pools[] = {8, 2, 1, 1, 1};
u16 exposure_pools[] = {796, 796, 624, 340, 328};

u8 setting_idx = 0;

u32 last_detect_frame_id = 0;

u8 led_flag[3] = {0};
/**************************************************************************
 *	Function
 **************************************************************************/
void uart_puts(char *pdata);
void led_control(unsigned char a, unsigned char b , unsigned char c);
void custom_user(void);

/* Night light part start*/
int passed_frame_at_60_fps(void)
{
	return(current_exp+last_exp)/2/380*(current_frame-last_frame);
}

void closed_to_opening(void)
{
	if(NB_Found->get_group == 1)//detect face
	{
		current_state=OPENING;
		printf("State:OPENING\n");
	}
}

void do_opening(void)
{	
	if(main_light_duty_cycle<current_max_light)//漸亮
	{		
		if(current_exp<800)
		{
			main_light_duty_cycle+=1;
		}else{
			main_light_duty_cycle+=2;//案態環境時執行頻率會變低，因此多加一點
			if(main_light_duty_cycle>100)
			{
				main_light_duty_cycle=100;//超過100的部份去掉	
			}	
		}
	}else{
		current_state=LIGHT_ADJUSTABLE;
		state_stored_frame=0;
		time_stored_frame=0;
		printf("State:LIGHT_ADJUSTABLE\n");
	}
}


void do_closing(void)
{	
	if(main_light_duty_cycle>LIGHT_LOWER_LIMIT)
	{
		main_light_duty_cycle-=1;
	}
	else{
		current_max_light=MAX_LIGHT_LIMIT;
		current_state=CLOSED;
		printf("State:CLOSED\n");		
	}	
}

void closing_to_opening(void)
{
	if(NB_Found->get_group == 0&&state_stored_frame>=60)//detect hand
	{
		current_state=OPENING;
		printf("State:OPENING\n");
	}else{
		do_closing();
	}
}


void adjustable_to_closing(void)
{
	if(time_stored_frame>=10*60)//開燈持續10s無偵測到物件，秒數X幀數(60fps)
	{
		current_state=CLOSING;
		state_stored_frame=0;
		printf("State:CLOSING\n");
	}	
}

void locked_to_closing(void)
{
	current_state=CLOSING;
	state_stored_frame=0;
	printf("State:CLOSING\n");
}

void light_control(void)
{
	if(light_stored_frame>=60)
	{
		light_stored_frame=0;
		if(current_max_light<=MAX_LIGHT_LOWER_LIMIT)
		{
			current_max_light=100;
			main_light_duty_cycle=100;			
		}else{
			current_max_light/=2;
			main_light_duty_cycle/=2;
		}
		printf("current light:%d\n",current_max_light);
	}
}



void adjustable_to_locked(void)
{
	current_state=LIGHT_LOCKED;
	state_stored_frame=0;
	printf("State:LIGHT_LOCKED\n");
}

void locked_to_adjustable(void)
{
	current_state=LIGHT_ADJUSTABLE;
	time_stored_frame=0;
	state_stored_frame=0;
	printf("State:LIGHT_ADJUSTABLE\n");
}
/************Night light part end*********************/
void custom_user(void)
{
	#ifndef SUPPORT_OUTLCD
	//padshare
	// 全部设置为悬空输入
	sys_set_padshare(IOA0, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOA0,1);

	sys_set_padshare(IOA1, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_EN); 											   
	gpio_set_dir(IOA1,1);

	sys_set_padshare(IOA2, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOA2,1);

	sys_set_padshare(IOA3, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOA3,1);

	sys_set_padshare(IOA4, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOA4,GPIO_INPUT_DIRECTION);
    
	/* UART config*/
	sys_set_padshare(IOA5, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	sys_set_padshare(IOA6, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	

	sys_set_padshare(IOA7, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOA7,1);

	// Flash
	sys_set_padshare(IOB1, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 
	sys_set_padshare(IOB2, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
	sys_set_padshare(IOB3, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
	sys_set_padshare(IOB4, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
	sys_set_padshare(IOB5, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
	sys_set_padshare(IOB6, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);

	sys_set_padshare(IOB7, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOB7,GPIO_INPUT_DIRECTION);

	sys_set_padshare(IOC0, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOC0,1);

	sys_set_padshare(IOC1, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOC1,1);

	sys_set_padshare(IOC2, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOC2,1);

	sys_set_padshare(IOC3, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOC3,1);

	sys_set_padshare(IOC4, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOC4,1);

	sys_set_padshare(IOC5, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_EN); 											   
	gpio_set_dir(IOC5,GPIO_INPUT_DIRECTION);

	sys_set_padshare(IOC6, PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); 											   
	gpio_set_dir(IOC6,GPIO_INPUT_DIRECTION);
	#else
	sys_set_padshare(IOA0, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_PCLK
	sys_set_padshare(IOA2, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_VSYNC
	sys_set_padshare(IOA5, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_HREF
	//sys_set_padshare(IOB7, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_XCLK

	sys_set_padshare(IOA1, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT0
	sys_set_padshare(IOC6, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT1

	sys_set_padshare(IOC5, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT2
	sys_set_padshare(IOC4, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT3
	sys_set_padshare(IOC3, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT4
	sys_set_padshare(IOC2, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT5
	sys_set_padshare(IOC1, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT6
	sys_set_padshare(IOC0, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT7
	sys_set_padshare(IOA7, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT8    
	//sys_set_padshare(IOA6, PAD_FUNC4, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS); // CIS_DAT9

	#endif	
	
	#ifdef SUPPORT_OUTLCD
	// 用到的IO口配置
	//sys_set_padshare(IOA5, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
	//sys_set_padshare(IOA6, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);

	// sys_set_padshare(IOA3, PAD_FUNC5, PAD_PULL_UP, PAD_STRENGTH_DIS); 											   
	// gpio_set_dir(IOA3,GPIO_INPUT_DIRECTION);
	#endif

	#if 0
	// iic 主机
	//sys_set_padshare(IOA3, PAD_FUNC1, PAD_PULL_UP, PAD_STRENGTH_DIS); 											   
	//sys_set_padshare(IOA4, PAD_FUNC1, PAD_PULL_UP, PAD_STRENGTH_DIS); 	

	// iic 从机
	sys_set_padshare(IOA3, PAD_FUNC0, PAD_PULL_UP, PAD_STRENGTH_DIS); 											   
	sys_set_padshare(IOA4, PAD_FUNC0, PAD_PULL_UP, PAD_STRENGTH_DIS); 
			
	#endif
	
	
	// 使用一线模式操作spi flash
  // uncomment if need spi flash running on standard mode(1 line)
  sf_config_mode(QSPI_MODE_STD);	
	
  //printf("%s \n",__FUNCTION__);
  
}

void led_control(unsigned char a, unsigned char b , unsigned char c)
{
	if(led_flag[0] == 1 )
		{
			//printf("led_flag0= %d\n", led_flag[0]);
			if(a == ':' || a == '0')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,0);//G
				
			}
			else if(a == '1')
			{
				gpio_set_value(IOA0,0);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,0);//G
				
			}
			else if(a == '2')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,0);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == '3')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == '4')
			{
				gpio_set_value(IOA0,0);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == '5')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,0);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == '6')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,0);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == '7')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,0);//G
				
			}
			else if(a == '8')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == '9')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(a == 'f' || a == 'F')
			{
				gpio_set_value(IOA0,0);//A
				gpio_set_value(IOA1,0);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,0);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,0);//G
				gpio_set_value(dig1,0);
				udelay(800);
			}
			// udelay(3500);
		}
		if (led_flag[1] == 1)
		{
			//printf("led_flag1= %d\n", led_flag[1]);
			if(b == ':' || b == '0')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,0);//G
				
			}
			else if(b == '1')
			{
				gpio_set_value(IOA0,0);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,0);//G
				
			}
			else if(b == '2')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,0);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == '3')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == '4')
			{
				gpio_set_value(IOA0,0);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == '5')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,0);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == '6')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,0);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == '7')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,0);//G
				
			}
			else if(b == '8')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,1);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == '9')
			{
				gpio_set_value(IOA0,1);//A
				gpio_set_value(IOA1,1);//B
				gpio_set_value(IOC0,1);//F
				gpio_set_value(IOA2,1);//C
				gpio_set_value(IOA3,1);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,1);//G
				
			}
			else if(b == 'f' || b == 'F')
			{
				gpio_set_value(IOA0,0);//A
				gpio_set_value(IOA1,0);//B
				gpio_set_value(IOC0,0);//F
				gpio_set_value(IOA2,0);//C
				gpio_set_value(IOA3,0);//D
				gpio_set_value(IOA7,0);//E
				gpio_set_value(IOC1,0);//G
				gpio_set_value(dig2,0);
				udelay(800);
			}
			// udelay(3500);
		}

		if(led_flag[2] == 1)
		{	
			//printf("led_flag2= %d\n", led_flag[2]);
			if( c == 'A')
			{
				gpio_set_value(IOA0,0); //led a
				gpio_set_value(IOA1,0);// led B
				gpio_set_value(IOA2,0);// led C
				gpio_set_value(IOA3,0);// led D
				gpio_set_value(IOA7,1);// led E
					
			}
			else if( c == 'B')
			{
				gpio_set_value(IOA0,0);// led A
				gpio_set_value(IOA1,0);// led B
				gpio_set_value(IOA2,0);// led C
				gpio_set_value(IOA3,1);// led D
				gpio_set_value(IOA7,1);// led E

			}
			else if( c == 'C')
			{
				gpio_set_value(IOA0,0);// led A
				gpio_set_value(IOA1,0);// led B
				gpio_set_value(IOA2,1);// led C
				gpio_set_value(IOA3,1);// led D
				gpio_set_value(IOA7,1);// led E

			}
			else if( c == 'D')
			{
				gpio_set_value(IOA0,0);// led A
				gpio_set_value(IOA1,1);// led B
				gpio_set_value(IOA2,1);// led C
				gpio_set_value(IOA3,1);// led D
				gpio_set_value(IOA7,1);// led E
				
			}
			else if( c == 'E')
			{
				gpio_set_value(IOA0,1);// led A
				gpio_set_value(IOA1,1);// led B
				gpio_set_value(IOA2,1);// led C
				gpio_set_value(IOA3,1);// led D
				gpio_set_value(IOA7,1);// led E
				
			}

		}

}

static int sensor_setup(void)
{
	
	// printf("internal sensor.\n");
    //SetDebugLevel(DEBUG_OFF,1);
	printf_ctrl(0);
	InitializeGpioPorting();
	
	InitializeBase(19200,SUPPORT_SYS_96M,ENABLE,DISABLE,DISABLE);
	
	// uart at 115200
	// uart_set_baudrate(BAUD_RATE);
	
	/*init fe*/
	if (fe_init(FE_MODE_FE, 3, 0))
	{
		DEBUG(DEBUG_ERROR, "fe init fail\n");
	}
	fe_start();

	scaler_init(0);
	scaler_start();

	NB_Base_Init();

	User_Detect_Config_Init();
	NB_detect_buf_init();
	NB_detect_top_init();
	NB_detect_mid_init();
	NB_detect_bottom_init();

	NB_Motion_Init();
	//audio_play_init(DEV_PWM, IOC6); //IOC6
	//low_power();
	uart_init(19200,UART_LCR_8N1,2);
    
	#if segment
	custom_user();
	#endif

	sensor_aeg_custom_app_info_structure *aeg_config;
	aeg_config = (sensor_aeg_custom_app_info_structure *)nb_malloc(sizeof(sensor_aeg_custom_app_info_structure));
	u8 enable_gamma = TRUE;
	// 自定义Gama值 - Gama值数量默认为49
	unsigned char custom_gama_value_array[] = {0,21,42,60,61,68,78,78,86,94,95,101,110,110,116,123,124,130,137,137,143,149,149,155,161,165,170,175,
		176,180,184,188,189,193,197,198,202,206,211,211,216,221,226,230,234,239,244,250,255};
	// 写入自定义Gama值
	// 注：此部分必须再初始化前完成，否则将不生效
	sensor_aeg_update_def_gama_value(&custom_gama_value_array[0]);
	sensor_aeg_custom_app_info_structure app_info_str = { 0 };
	app_info_str.adjust_strategy = MACR_AEG_STRATEGY_METERING_SPOT;
	app_info_str.frm_rate = MACR_AEG_FRM_RATE_30FPS;
	app_info_str.frm_swicth_enable_flag = 0;
	app_info_str.ana_gain_max = 8;
	app_info_str.dete_info_filter = 1;
	app_info_str.md_info_filter = 1;
	app_info_str.valid_flag = 1;

	// 自定义信息更新成功，启动自定义应用
	if (sensor_aeg_update_custom_application_info(app_info_str) == 0)
	{
		sensor_aeg_init(MACR_AEG_APPLICATION_PROJECT_CUSTOM, MACR_AEG_AC_FREQ_CFG_50HZ,enable_gamma);
	}
	// 自定义信息更新失败，启动General应用
	else
	{
		sensor_aeg_init(MACR_AEG_APPLICATION_PROJECT_GENERAL, MACR_AEG_AC_FREQ_CFG_50HZ,enable_gamma);
	}

	nb_free(aeg_config);

	sensor_aeg_video_debug_label_enable(FALSE);

	return FALSE;
	
}


////////////////////////////////////////////////////////////////
// Motion_AEC & Set_Win task loop
//////////////////////////////////////////////////////////////////
static void Win_Task(void *parameters)
{	
	//printf("test\n");
	sensor_setup();

	u8 disable_motion = 0;
	u8 found_mode_exit = 0;
	u8 win_sl_rate = 0;
	char curr_aeg_work_mode = 0;

	u8 motor_run = 0;
	u32 label_data = 0;

	u8 flag_uart= 0;

//AE***0630>>>
	//#define STATE_BUF_SIZE 20
	int aeg_stable = 0, aeg_unstable = 0, region_average = 0;
	u8 sx = 5, ex = 25, sy = 5, ey = 25;
	
	int num = 0, areaVal = 0, regionVal = 0;
	

	// deadline by Kevin
	__raw_writel(0x01771B00,0x401D00D0);
	__raw_writel(0x0000F200,0x401D00D4);
	__raw_writel(0x1C535443,0x401D00D8);
//AE***0630<<<

    // Motion threshold
	__raw_writel(0x00080008,0x40090858);

	signed short temp = 0;
	signed short temp1 = 0;
	
	//mdelay(1000);	
	
    for (;;)
    {	

		if(detect_enable && task_init)
		{	
			if(md_wait_for_frame_finish(0) == 0) // True if in frame-end, else False. // Timeout in MS
			{		
				// 	
				//printf("************************************************ %ld \n",xTaskGetTickCount());
				frame_id = debug_get_frame_id_hex();
				// printf("1 now tick:%ld \n",xTaskGetTickCount());	

///////////////////////////////////////////////////////
// AE***0630>>>
				// set brightness				
				if (use_state == 0)
				{					   	
					// no detection
					get_global_area();
					get_partial_area_ave(sx, ex, sy, ey, &num, &areaVal, &regionVal);
					region_average = areaVal;		
					// Do AEG cycle when there is no detection after 10 frames
					if ((frame_id - last_detect_frame_id) % 10 == 0)
					{		
						setting_idx ++ ;
                        setting_idx %= 5;
					}
				}
				//SET PWM DUTY
				//HardPWM(IR_LED_IO,IR_PWM_FREQUENCY,IR_duty_val,IR_PWM_STRENGTH);
				
                
				sensor_set_analog_gain (again_pools[setting_idx]);
                sensor_set_exposure_row (exposure_pools[setting_idx]);
				 //debug_enable_label(3, 1, intToBcd(region_average * 10000), detect_config.detect_max_xpiont - 35, 5);
				// debug_enable_label(4, 1, intToBcd(IR_duty_val), detect_config.detect_max_xpiont - 35, 15);
				// label_data = sensor_get_exposure_row() + sensor_get_analog_gain() * 10000;
				 //debug_enable_label(5, 1, intToBcd(label_data),detect_config.detect_max_xpiont - 35, 25);
				//debug_enable_label(3, 1, intToBcd(background_ir), detect_config.detect_max_xpiont - 35, 25);
				//printf("background_overbright_count %d ,frame_id: %d \n",background_overbright_count, frame_id);
				// debug_enable_box(CH0217_DEBUG_MOTION_BOX_ID,0,CH0217_Found->x,CH0217_Found->y,CH0217_Found->x + CH0217_Found->w,CH0217_Found->y + CH0217_Found->h);
//AE***0630<<<
///////////////////////////////////////////////////////

				if(NB_buf_state)
				{
					NB_Base_Action_start(frame_id, disable_motion, found_mode_exit);

					NB_detect_top_win(1, &detected_flow, &win_flow, frame_id);

					// 获取Motion
					NB_Motion_Action(&detected_flow, frame_id);
				}

				// 根据外部跳线选择是否把Pin脚配置成串口
				// User_Auto_Select_Uart();

				// 显示debug的信息到屏幕上
				User_Image_Show_DebugMessage();

				// 屏幕上画框
				User_Image_Show_Box(frame_id);

				// 清除屏幕上的框
				User_Image_Clean_Box(frame_id);


			}
			
		}

	    vTaskDelay(TASK_DELAY_TIME);
    }

	vTaskDelete(NULL);
}


//////////////////////////////////////////////////////////////////
// For customers to respond to detection of face/body/etc...
//////////////////////////////////////////////////////////////////
void uart_puts(char *pdata)
{
	while(*pdata)
	{
		uart_putc(*pdata++);
 	}
}

static void User_Task(void *parameters)
{	
	
	static int DetectW_T_old = 0;
	
	u32 now_tick = 0;
		
	detect_enable = TRUE; // Must wait the md_wait_for_frame_finish() before setting to TRUE

	
	u8 u_flag = 0;
	u32 temp_rl = 0;
	int switcher = 0;


	/* Night light */
    sys_set_padshare(LOCK_STATE_LED_PIN,PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);//LOCK提示LED腳位設置
	gpio_set_dir(LOCK_STATE_LED_PIN, 1);
	sys_set_padshare(IR_850NM_PIN,PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_EN);//紅外補光LED腳位設置
	gpio_set_dir(IR_850NM_PIN, 1);
	/*主燈PWM設置*/
	sys_set_padshare(MAIN_LIGHT_PIN, PAD_FUNC3, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);//設置腳位使用PWM的function
	gpio_set_dir(MAIN_LIGHT_PIN, 1);//設置腳位為輸出狀態
	pwm_init(MAIN_LIGHT_PIN, 0);//pwm初始輸出為低電位
	pwm_enable(MAIN_LIGHT_PIN, 1);//啟用PWM
	/*剩餘時間LED PWM設置*/
	sys_set_padshare(LEFT_TIME_LED_PIN, PAD_FUNC3, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);//設置腳位使用PWM的function
	gpio_set_dir(LEFT_TIME_LED_PIN ,1);//設置腳位為輸出狀態
	pwm_init(LEFT_TIME_LED_PIN, 0);//pwm初始輸出為低電位
	pwm_enable(LEFT_TIME_LED_PIN, 1);//啟用PWM

	for (;;)
	{
		
		if(task_init)
		{
			/*Night light loop code start */
			current_frame=debug_get_frame_id_hex();//更新曝光與總經過幀數
			current_exp=NB_sensor_get_exposure();

			state_stored_frame+=passed_frame_at_60_fps();//更新frame數
			time_stored_frame+=passed_frame_at_60_fps();
			light_stored_frame+=passed_frame_at_60_fps();			
			if(left_time_led_duty_cycle<0)
			{
				left_time_led_duty_cycle=0;
			}	
				//太暗時開啟紅外LED，太亮時關閉
			if(NB_sensor_get_exposure()>=800)
			{
				gpio_set_value(IR_850NM_PIN,1);
			}else{
				gpio_set_value(IR_850NM_PIN,0);
			}
			// set PWM sample
			//pwm_set_sample(MAIN_LIGHT_PIN, PWM_SYSTEM_CLOCK / PWM_FREQ * main_light_duty_cycle / 100);//設置sample
			//pwm_set_sample(LEFT_TIME_LED_PIN, PWM_SYSTEM_CLOCK / PWM_FREQ * left_time_led_duty_cycle / 100);//設置sample
			
			//SET PWM DUTY
    		HardPWM(MAIN_LIGHT_PIN,PWM_FREQ,main_light_duty_cycle,0);
    		HardPWM(LEFT_TIME_LED_PIN,PWM_FREQ,left_time_led_duty_cycle,0);

				
			if((NB_Found->state == NB_Find_State_Find) || (NB_Found->state == NB_Find_State_Find_Follow))
			{			
				switch(current_state)
				{
					case CLOSED:
						closed_to_opening();
						break;
					case OPENING:
						do_opening();
						break;
					case LIGHT_ADJUSTABLE://調整最大亮度
						time_stored_frame=0;
						left_time_led_duty_cycle=100-time_stored_frame*100/(10*60);
						if((state_stored_frame>=60*2)&&(NB_Found->get_group==0))
						{								
							if((NB_Found->x+(NB_Found->x + NB_Found->w))/2>=150)//偵測框在鏡頭右側(人的左側)，變換亮度階級
							{
								light_control();	
							}else{
								adjustable_to_locked();
								//LOCK燈亮起，PIN:IOA4
								gpio_set_value(LOCK_STATE_LED_PIN,1);
							}
						}
						break;
					case LIGHT_LOCKED:
						if((state_stored_frame>=60*2)&&(NB_Found->get_group==0))//OOOOOOOO
						{
							gpio_set_value(LOCK_STATE_LED_PIN,0);//LOCK燈關閉
							if((NB_Found->x+(NB_Found->x + NB_Found->w))/2<150)//偵測框在鏡頭左側(人的右側)，切回至可調整狀態
							{
								locked_to_adjustable();
							}else{
								locked_to_closing();
							}
						}
						break;
					case CLOSING:
						closing_to_opening();
						break;
					default:
						printf("未知的燈光運作狀態");
				}
				//提示訊息
		
				/*
				#ifdef USE_DRAW_DEBUG_WIN
				if(NB_Found.x > 5)
				{
					temp_x = NB_Found.x - 5;
				}
				else
				{
					temp_x = 0;
				}
				if(NB_Found.y > 5)
				{
					temp_y = NB_Found.y - 5;
				}
				else
				{
					temp_y = 0;
				}
									
				temp_x_end = NB_Found.x + NB_Found.w + 5;
				if(temp_x_end >= SENSOR_OUTPUT_SIZE)
				{
					temp_x_end = SENSOR_OUTPUT_SIZE - 1;
				}
				
				temp_y_end = NB_Found.y + NB_Found.h + 5;
				if(temp_y_end >= SENSOR_OUTPUT_SIZE)
				{
					temp_y_end = SENSOR_OUTPUT_SIZE - 1;
				}				
				debug_enable_box(NB_DEBUG_FOUND_BOX_ID,1,temp_x,temp_y,temp_x_end,temp_y_end);
				#endif
				*/
					
				//printf("Found id:%ld g:%d l:%d x:%d y:%d \n",NB_Found.get_id,NB_Found.get_group,NB_Found.get_level,NB_Found.x,NB_Found.y);
					
				// Debug 识别到的特征值
				#if 0
				u16 i = 0;
				if(NB_Found.state == NB_Find_State_Find)
				{
					printf("Found Data: \n");
					for(i = 0;i < NB_model.group_size[NB_Found.get_group];i ++)						
					{
						printf("%d ",NB_Found.after_buf[i]);
					}
					printf("\n");
				}
				#endif
					
				// Debug 全部模型得分
				#if 0
				u8 i = 0;
				u8 j = 0;


				for(i = 0;i < detect_model.group_item;i ++)
				{
					for(j = 0;j < detect_model.group_level[i];j ++)
					{	
						if(NB_Found.score[i][j] > 0)
						{
							printf("Model id:%d g:%d l:%d score:%d \n",NB_Found.get_id,i,j,NB_Found.score[i][j]);
						}
						
					}
				}
				#endif
					
				NB_Found->state = NB_Find_State_No;
					
			}else if(!(NB_Found->state == NB_Find_State_Find))//未識別到
			{
				switch(current_state)
				{
					case CLOSED:
						break;
					case CLOSING:
						do_closing();
						break;
					case LIGHT_ADJUSTABLE:
						adjustable_to_closing();
						break;
					case LIGHT_LOCKED:
						break;
					case OPENING:
						break;
					default:
						printf("未知的燈光運作狀態");
				}
			}
			last_frame=current_frame;//在迴圈末尾設置，供下次迴圈計算使用
			last_exp=current_exp;	


			/*Night light loop code end*/
			now_tick = xTaskGetTickCount();
            //debug_enable_label(0, 1, intToBcd(temp_rl), 5, 260);
			// 有进行识别操作
			if(DetectW_T_old != NB_DetectW_T)
			{
				DetectW_T_old = NB_DetectW_T;

				if (NB_Found->state != NB_Find_State_No)
				{
					
					last_detect_frame_id = frame_id;
					
					use_state = 1;
					// 更新识别信息至AEG模块内部

					//sensor_aeg_update_dete_info(MACR_AEG_APP_DETE_MODE_OTHERS, NB_Found->get_id, NB_Found->x0, NB_Found->y0, NB_Found->score[NB_Found->get_group][NB_Found->get_level]);
					
					switcher = -1;
																                   					
					  
				}
				else if(frame_id > last_detect_frame_id + STATE_BUF_SIZE)
				{
					switcher = 0;
					use_state = 0;
				}			
				
				debug_enable_label(0, 1, intToBcd(temp_rl), 5, 260);
			    debug_enable_label(1, 1, intToBcd(NB_Found->get_group), 5, 280);
				
			}
			


            if (u_flag== 0 && switcher == -1)
			{
				if (NB_Found->get_group == 0) 
				{								
					uart_puts("R");
					u_flag = 1;
					temp_rl = -1;
					//mdelay(300);
				}
				else if (NB_Found->get_group == 1) 
				{								
					uart_puts("L");
					u_flag = 1;
					temp_rl = 11111111;
				}
			}
			else if(switcher == 0)
			{
				u_flag = 0;
			}

			NB_Found->state = NB_Find_State_No;
			// printf("switcher: %d, u_flag: %d", switcher, u_flag);

			if(now_tick > Sys_Tick + 10000)
			{
				Sys_Tick = now_tick;
							
				// printf("Run %lds NB_PlanWin_T:%d NB_FeW_T:%d NB_DetectW_T:%d \n",
				// 	Sys_Tick /1000,NB_PlanWin_T,NB_FeW_T,NB_DetectW_T);
			}
		}				

		vTaskDelay(TASK_DELAY_TIME);
	}
	vTaskDelete(NULL);
}

static void Result_Task(void *parameters)
{
	u8 select = 0;

	for (;;)
	{
		
		if (detect_enable && task_init && NB_buf_state)
		{
			select = NB_detect_top_check_result();

			if (select)
			{
				// detect
				NB_detect_top(select - 1, &detected_flow, &win_flow, frame_id);
			}
		}

		vTaskDelay(TASK_DELAY_TIME);
	}

	vTaskDelete(NULL);
}
//////////////////////////////////////////////////////////////////
// Get Feature task loop
//////////////////////////////////////////////////////////////////
static void Feature_Task(void *parameters)
{
	for (;;)
	{
		if (task_init)
		{
			NB_detect_top_febuf();
		}
		vTaskDelay(TASK_DELAY_TIME);
	}
	vTaskDelete(NULL);
}

static void Seg_Task(void *parameters)
{
		
	unsigned char cc,bb,aa,qq = 0;

	for(;;)
	{
		unsigned char read[3]= {0,0,0};
		int count = 0;
		int state = 0;
        
		uart_ctrl(2);
		sys_set_padshare(IOA5, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
		sys_set_padshare(IOA6, PAD_FUNC0, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);

		while (task_init)
		{
			if(state == 0)//FIFO
			{
				if(uart_tstc() == 1)
				{	
					count++;
					switch(count)
					{
						case 1:
							cc = uart_getc();

							if(isdigit(cc) && !isspace (cc) )
								read[0] = cc;
							else if(cc == 'f'|| cc == 'F')
							{
								read[0] = cc;
							}								
							else 
								count = 0;
							break;
						case 2:
							bb = uart_getc();

							if(isdigit(bb) && !isspace (bb))
								read[1] = bb;
							else if(bb == 'f'|| bb == 'F')
							{
								read[1] = bb;
							}
								
							else 
								count = 0;
							break;
						case 3:
							aa = uart_getc();

							if(isalpha(aa) && !isspace (aa))		
								read[2] = aa;
							count = 0;
							break;
					}


					if(read[2] == 'A' || read[2] == 'B' || read[2] == 'C' || read[2] == 'D' || read[2] == 'E'|| read[2] == 'F')
						qq = read[2];

					state = 1;
				}
			}


			gpio_set_value(dig1,0); gpio_set_value(dig2,0); gpio_set_value(dig3,0);
			led_flag[0] = 1;
			led_control(read[0],read[1],qq); 		
			gpio_set_value(dig1,1); gpio_set_value(dig2,0); gpio_set_value(dig3,0);
			vTaskDelay(3);
			led_flag[0] = 0;
			vTaskDelay(1);

			gpio_set_value(dig1,0); gpio_set_value(dig2,0); gpio_set_value(dig3,0);
			led_flag[1] = 1;
			led_control(read[0],read[1],qq); 		
			gpio_set_value(dig1,0); gpio_set_value(dig2,1); gpio_set_value(dig3,0);
			vTaskDelay(3);
			led_flag[1] = 0;
			vTaskDelay(1);

			gpio_set_value(dig1,0); gpio_set_value(dig2,0); gpio_set_value(dig3,0);
			led_flag[2] = 1;
			led_control(read[0],read[1],qq); 		
			gpio_set_value(dig1,0); gpio_set_value(dig2,0); gpio_set_value(dig3,1);
			vTaskDelay(3);
			led_flag[2] = 0;
			vTaskDelay(1);

			state = 0;
		}
	    vTaskDelay(TASK_DELAY_TIME + 3);		
	}
	vTaskDelete(NULL);
}


static void softPWM_Task(void *parameters)
{
	#define IR_LED_IO IOB7
	u8 IR_duty_val = 70;
	#define IR_PWM_STRENGTH 0				// 0 : 8mA / 1 : 12mA
	sys_set_padshare(IR_LED_IO,PAD_FUNC5, PAD_PULL_NO_PULL, PAD_STRENGTH_DIS);
	for (;;)
	{
		if (task_init)
		{
			gpio_set_dir(IR_LED_IO,0x1);
			if (perf_get_ticks()/PERF_TIMER_MS_CNT*20 % 100 < IR_duty_val){
				gpio_set_value(IR_LED_IO, HIGH);
			}
			else{
				gpio_set_value(IR_LED_IO, LOW);
			}
		}
		vTaskDelay(TASK_DELAY_TIME);
	}
	vTaskDelete(NULL);
}

static void winTaskInit(void)
{
	BaseType_t xTask_Win;
	xTask_Win = xTaskCreate(
		Win_Task,
		"Win_Task",
		configMINIMAL_STACK_SIZE * 8,
		NULL,
		tskIDLE_PRIORITY + 5,
		&xHandle_Win);
	if (xTask_Win != pdPASS)
	{
		DEBUG(DEBUG_ERROR, "Win_Task is NULL!\n");
		exit(FALSE);
	}
}

static void userTaskInit(void)
{
	BaseType_t xTask_User;
	xTask_User = xTaskCreate(
		User_Task,
		"User_Task",
		configMINIMAL_STACK_SIZE * 5,
		NULL,
		tskIDLE_PRIORITY + 5,
		&xHandle_User);
	if (xTask_User != pdPASS)
	{
		DEBUG(DEBUG_ERROR, "User_Task is NULL!\n");
		exit(FALSE);
	}
}

static void featureTaskInit(void)
{
	BaseType_t xTask_Feature;
	xTask_Feature = xTaskCreate(
		Feature_Task,
		"Feature_Task",
		configMINIMAL_STACK_SIZE * 5,
		NULL,
		tskIDLE_PRIORITY + 3,
		&xHandle_Feature);
	if (xTask_Feature != pdPASS)
	{
		DEBUG(DEBUG_ERROR, "Feature_Task is NULL!\n");
		exit(FALSE);
	}
}

static void resultTaskInit(void)
{
	BaseType_t xTask_Result;
	xTask_Result = xTaskCreate(
		Result_Task,
		"Result_Task",
		configMINIMAL_STACK_SIZE * 5,
		NULL,
		tskIDLE_PRIORITY + 3,
		&xHandle_Result);
	if (xTask_Result != pdPASS)
	{
		DEBUG(DEBUG_ERROR, "Result_Task is NULL!\n");
		exit(FALSE);
	}
}

static void segTaskInit(void)
{
	BaseType_t xTask_Seg;
	xTask_Seg = xTaskCreate(
		Seg_Task,
		"Seg_Task",
		configMINIMAL_STACK_SIZE * 1,
		NULL,
		tskIDLE_PRIORITY + 5, // 2
		&xHandle_Seg);
	if (xTask_Seg != pdPASS)
	{
		DEBUG(DEBUG_ERROR, "Seg_Task is NULL!\n");
		exit(FALSE);
	}                                                
}

static void softPWMTaskInit(void)
{
	BaseType_t xTask_softPWM;
	xTask_softPWM = xTaskCreate(
		softPWM_Task,
		"softPWM_Task",
		configMINIMAL_STACK_SIZE * 1,
		NULL,
		tskIDLE_PRIORITY + 6, // 2
		&xHandle_softPWM);
	if (xTask_softPWM != pdPASS)
	{
		DEBUG(DEBUG_ERROR, "softPWM_Task is NULL!\n");
		exit(FALSE);
	}                                                
}
int main(void)
{
	printf("****************************************\n");
	/* Please use nb_malloc & nb_free in main instead of using local array */
	uint8_t *pinfo = nb_malloc(64);
	ASSERT(pinfo);
	verNbsdkGet(pinfo);
	printf("\nNBSDK_VERSION:%s\n", pinfo);
	nb_free(pinfo);
	printf("Build @%s\n\n", host_ver);

	printf("Detection Version:%s\n", NB_Version);

	unsigned int aeg_ver_len;
	uint8_t *aeg_ver_array = (uint8_t *)nb_malloc(65);
	aeg_ver_len = sensor_aeg_get_version(aeg_ver_array, 64);

	if (aeg_ver_len > 0)
	{
		aeg_ver_array[aeg_ver_len] = '\0';
		printf("AEG Version: %s\n", aeg_ver_array);
	}
	else
	{
		printf("AEG Version is Error!!!\n");
	}
	nb_free(aeg_ver_array);

	printf("****************************************\n");

	winTaskInit();
	featureTaskInit();
	resultTaskInit();
	userTaskInit();
	softPWMTaskInit();

	#if segment
	segTaskInit();
	#endif
	//debug_task();

	task_init = TRUE;

	return 0;
}








