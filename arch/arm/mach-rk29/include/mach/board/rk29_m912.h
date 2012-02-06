/*
 * rk29_m911.h
 *
 * Overview:  
 *
 * Copyright (c) 2011, YiFang Digital
 *
 * Version:  1.0
 * Created:  02/22/2011 03:36:04 PM
 * Author:  zqqu <zqqu@yifangdigital.com>
 * Company:  YiFang Digital
 * History:
 *
 * 
 */

#ifndef __RK29_M911_H
#define __RK29_M911_H

#define NO_IOMUX_PINNAME  NULL
#define NO_IO_MUX_MODE		NULL
/***************************************************
 *
 *                      CPU
 *
 **************************************************/
#define	CPU_FREQ_1008M

/***************************************************
 *
 *                      KEY
 *
 **************************************************/
#define GPIO_MENU_KEY
#define GPIO_VOLUMEUP_KEY			RK29_PIN6_PA1
#define GPIO_VOLUMEDOWN_KEY			RK29_PIN6_PA2
#define GPIO_HOME_KEY
#define GPIO_SEARCH_KEY
#define GPIO_BACK_KEY
#define GPIO_CAMERA_KEY
#define GPIO_POWER_KEY				RK29_PIN6_PA7

#define GPIO_TOUCHKEY_INT   RK29_PIN0_PA4

#define PRESS_LEV_LOW			1
#define PRESS_LEV_HIGH			0

#if defined(CONFIG_KEYS_M901HR)
#define KEYS_MAP	{\
	{	\
		.desc	= "up",	\
		.code	= KEY_UP,	\
		.code_long_press = KEY_HOME, \
		.gpio	= RK29_PIN6_PA1,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{ \
		.desc	= "down",	\
		.code	= KEY_DOWN,	\
		.code_long_press = KEY_MENU, \
		.gpio	= RK29_PIN6_PA2,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
}
#elif defined(CONFIG_KEYS_RTM913HC)
#define KEYS_MAP	{\
	{ \
		.desc	= "vol+",	\
		.code	= KEY_VOLUMEUP,	\
		.gpio	= RK29_PIN6_PA3,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "vol-",	\
		.code	= KEY_VOLUMEDOWN,	\
		.gpio	= RK29_PIN6_PA4,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
}
#elif defined(CONFIG_KEYS_RTM915HC)
#define KEYS_MAP	{\
	{ \
		.desc	= "down",	\
		.code	= KEY_DOWN,	\
		.code_long_press = KEY_HOME, \
		.gpio	= RK29_PIN6_PA1,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "up",	\
		.code	= KEY_UP,	\
		.code_long_press = KEY_MENU, \
		.gpio	= RK29_PIN6_PA2,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{ \
		.desc	= "vol+",	\
		.code	= KEY_VOLUMEUP,	\
		.gpio	= RK29_PIN6_PA3,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "vol-",	\
		.code	= KEY_VOLUMEDOWN,	\
		.gpio	= RK29_PIN6_PA4,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
}
#elif defined(CONFIG_KEYS_RTM916HC)
#define KEYS_MAP	{\
	{ \
		.desc	= "home",	\
		.code	= KEY_BACK,	\
		.code_long_press = KEY_HOME, \
		.gpio	= RK29_PIN0_PA4,		\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{ \
		.desc	= "vol+",	\
		.code	= KEY_VOLUMEUP,	\
		.gpio	= RK29_PIN6_PA3,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "vol-",	\
		.code	= KEY_VOLUMEDOWN,	\
		.gpio	= RK29_PIN6_PA4,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
}
#else
#define KEYS_MAP	{\
	{ \
		.desc	= "vol+",	\
		.code	= KEY_VOLUMEUP,	\
		.gpio	= GPIO_VOLUMEUP_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "vol-",	\
		.code	= KEY_VOLUMEDOWN,	\
		.gpio	= GPIO_VOLUMEDOWN_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
}
#endif


/***************************************************
 *
 *                      TOUCH
 *
 **************************************************/
#define GPIO_TOUCH_EN RK29_PIN6_PB0
#define TOUCH_EN_LEVEL GPIO_HIGH
#define GPIO_TOUCH_INT   RK29_PIN0_PA2
#define GPIO_TOUCH_RST   RK29_PIN6_PC3
#define TOUCH_USE_I2C2	1
#define	TP_USE_WAKEUP_PIN


/***************************************************
 *
 *				    LCD  
 *
 **************************************************/
#define LCD_TXD_PIN          INVALID_GPIO
#define LCD_CLK_PIN          INVALID_GPIO
#define LCD_CS_PIN           INVALID_GPIO

#define FB_ID                       0
#define FB_DISPLAY_ON_PIN           RK29_PIN6_PD1
#define FB_LCD_STANDBY_PIN          RK29_PIN6_PD0
#define FB_LCD_CABC_EN_PIN          INVALID_GPIO
#define FB_MCU_FMK_PIN              INVALID_GPIO

#define FB_DISPLAY_ON_VALUE         GPIO_HIGH
#define FB_LCD_STANDBY_VALUE        GPIO_HIGH

#define OUT_CLK			 33000000
#define LCDC_ACLK       150000000     //29 lcdc axi DMA 撞楕

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888

/* Timing */
#define H_PW			48
#define H_BP			40
#define H_VD			800
#define H_FP			112

#define V_PW			3
#define V_BP			39
#define V_VD			600
#define V_FP			18

#define DCLK_POL		0
#define SWAP_RB			0

#define LCD_WIDTH       170
#define LCD_HEIGHT      128
/***************************************************
 *
 *				     BACKLIGHG
 *
 **************************************************/
 /*
 GPIO1B5_PWM0_NAME,       GPIO1L_PWM0
 GPIO5D2_PWM1_UART1SIRIN_NAME,  GPIO5H_PWM1
 GPIO2A3_SDMMC0WRITEPRT_PWM2_NAME,   GPIO2L_PWM2
 GPIO1A5_EMMCPWREN_PWM3_NAME,     GPIO1L_PWM3
 */
#define PWM_ID            0
#define PWM_MUX_NAME      GPIO1B5_PWM0_NAME
#define PWM_MUX_MODE      GPIO1L_PWM0
#define PWM_MUX_MODE_GPIO GPIO1L_GPIO1B5
#define PWM_EFFECT_VALUE  0
#define GPIO_BL_PWM		RK29_PIN1_PB5

//#define LCD_DISP_ON_PIN

#ifdef  LCD_DISP_ON_PIN
#define BL_EN_MUX_NAME    GPIOF34_UART3_SEL_NAME
#define BL_EN_MUX_MODE    IOMUXB_GPIO1_B34

#define BL_EN_PIN         GPIO0L_GPIO0A5
#define BL_EN_VALUE       GPIO_HIGH
#endif

/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	12

/*************************************************** *
 *
 *                  PWM REGULATOR
 *
 **************************************************/
#define REGULATOR_PWM_ID					2
#define REGULATOR_PWM_MUX_NAME      		GPIO2A3_SDMMC0WRITEPRT_PWM2_NAME
#define REGULATOR_PWM_MUX_MODE      					GPIO2L_PWM2
#define REGULATOR_PWM_MUX_MODE_GPIO 				GPIO2L_GPIO2A3
#define REGULATOR_PWM_GPIO				RK29_PIN2_PA3


/*************************************************** *
 *
 *                      GSENSOR
 *
 **************************************************/
#define MMA8452_INT_PIN   RK29_PIN0_PA3




/***************************************************
 *
 *                      WIFI & BT
 *
 **************************************************/
#if defined (CONFIG_RTL8192CU) || defined (CONFIG_RTL8192CU_FAC)
#define GPIO_WIFI_POWER       RK29_PIN6_PC0
#else
#define GPIO_WIFI_POWER       RK29_PIN5_PD6
#define GPIO_WIFI_RESET          RK29_PIN6_PC0
#define GPIO_BT_RESET            RK29_PIN6_PC4
#endif
/***************************************************
 *
 *                    SD/MMC
 *
 **************************************************/


/***************************************************
 *
 *                    USB
 *
 **************************************************/
#define GPIO_USB_INT			 RK29_PIN0_PA0
#define MASS_STORAGE_NAME "M912HC"
#define MASS_STORAGE_PRODUCT ""
#define USB_PRODUCT_ID			0x2910
#define ADB_PRODUCT_ID			0x0c02
#define VENDOR_ID				0x0bb4
#define ADB_PRODUCT_NAME		"rk2918"
#define ADB_MANUFACTURE_NAME	"RockChip"

/***************************************************
 *
 *                      POWER
 *
 **************************************************/
#define GPIO_POWER_ON			 RK29_PIN4_PA4


/***************************************************
 *
 *                      BATTERY 
 *
 **************************************************/
#define DC_DET_EFFECTIVE		1
#define CHG_OK_EFFECTIVE		1
#define GPIO_DC_DET			RK29_PIN4_PA1
#define GPIO_CHG_OK			RK29_PIN4_PA3
#define ADC_CLI_VALUE		15
#define CHARGE_FULL_GATE 		4150

//This parameter is for new battery driver//
#define	TIMER_MS_COUNTS		            50	//timers length(ms)

#define	SLOPE_SECOND_COUNTS	            15	//time interval(s) for computing voltage slope
#define	DISCHARGE_MIN_SECOND	        60	//minimum time interval for discharging 1% battery
#define	CHARGE_MIN_SECOND	            90	//minimum time interval for charging 1% battery
#define	CHARGE_MID_SECOND	            160	//time interval for charging 1% battery when battery capacity over 80%
#define	CHARGE_MAX_SECOND	            250	//max time interval for charging 1% battery
#define CHARGE_FULL_DELAY_TIMES         10  //delay time when charging FULL
#define USBCHARGE_IDENTIFY_TIMES        5   //time for identifying USB and Charge
#define STABLE_SECOND					8  //check ok µçÆ½»á»Î¶¯¡£¡£
#define SHUTDOWN_SECOND					20
#define SPEEDLOSE_SECOND                120 //play game rapid down

#define	NUM_VOLTAGE_SAMPLE	            ((SLOPE_SECOND_COUNTS * 1000) / TIMER_MS_COUNTS)	//samling numbers
#define	NUM_DISCHARGE_MIN_SAMPLE	    ((DISCHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	
#define	NUM_CHARGE_MIN_SAMPLE	        ((CHARGE_MIN_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MID_SAMPLE	        ((CHARGE_MID_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define	NUM_CHARGE_MAX_SAMPLE	        ((CHARGE_MAX_SECOND * 1000) / TIMER_MS_COUNTS)	    
#define NUM_CHARGE_FULL_DELAY_TIMES     ((CHARGE_FULL_DELAY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_USBCHARGE_IDENTIFY_TIMES    ((USBCHARGE_IDENTIFY_TIMES * 1000) / TIMER_MS_COUNTS)	
#define NUM_STABLE_SAMPLE				((STABLE_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SHUTD0WN_SAMPLE             ((SHUTDOWN_SECOND * 1000) / TIMER_MS_COUNTS)
#define NUM_SPEEDLOSE_SAMPLE  			((SPEEDLOSE_SECOND * 1000) / TIMER_MS_COUNTS)

#define BAT_2V5_VALUE	        2500
#define BATT_MAX_VOL_VALUE	    4190	//voltage of FULL battery
#define	BATT_ZERO_VOL_VALUE     3500	//voltage when poweroff
#define BATT_NOMAL_VOL_VALUE    3800

//define  divider resistors for ADC sampling, units as K
#define BAT_PULL_UP_R           549
#define BAT_PULL_DOWN_R         200


/***************************************************
 *
 *                      RTC
 *
 **************************************************/
#define GPIO_RTC_INT			 RK29_PIN0_PA1


/***************************************************
 *
 *                      AUDIO
 *
 **************************************************/
#define GPIO_SPK_CON			 RK29_PIN6_PB6
#define RTL5631_HP_HIGH			
#define DEF_VOL					0xc3
//#define OLD_HW_AUDIO
#define DEF_VOL_SPK				0xc6
#endif
