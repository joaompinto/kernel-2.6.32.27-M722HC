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

#ifndef __RK29_M900_H
#define __RK29_M900_H
#define NO_IOMUX_PINNAME  NULL
#define NO_IO_MUX_MODE		NULL

/***************************************************
 *
 *                      CPU
 *
 **************************************************/
#define	CPU_FREQ_912M

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


#define PRESS_LEV_LOW			1
#define PRESS_LEV_HIGH			0

#define KEYS_MAP	{\
	{	\
		.desc	= "play",	\
		.code	= KEY_POWER,	\
		.gpio	= GPIO_POWER_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
		.wakeup	= 1,	\
	},	\
	{ \
		.desc	= "vol+",	\
		.code	= KEY_VOLUMEUP,	\
		.gpio	= GPIO_VOLUMEUP_KEY,	\
		.active_low = PRESS_LEV_LOW,	\
	},	\
	{	\
		.desc	= "vol-",	\
		.code	= KEY_VOLUMEDOWN,	\
		.gpio	= GPIO_VOLUMEDOWN_KEY, \
		.active_low = PRESS_LEV_LOW,	\
	},	\
}

/***************************************************
 *
 *                      TOUCH SCREEN
 *
 **************************************************/
#define GPIO_TOUCH_EN	RK29_PIN6_PB0
#define GPIO_TOUCH_INT   RK29_PIN0_PA2
#define GPIO_TOUCH_RST   RK29_PIN6_PC3
#define TOUCH_USE_I2C2	1
#define	TP_USE_WAKEUP_PIN



/***************************************************
 *
 *                      TOUCH KEY
 *
 **************************************************/
#if 1
#define TOUCHKEY_ON_SCREEN
#if 0
#define TOUCH_KEY_MAP \
		{ \
				{850,	128, 	KEY_SEARCH},  /* search */ \
				{850,	95,	KEY_HOME},        /* home */ \
				{850,	55,	KEY_F1},      	  /* menu */ \
				{850,	5,	KEY_BACK	},    /* back */ \
				{0,0,0} \
		}

#define TOUCH_KEY_MAP_YINGMAO \
		{ \
				{834,	0, 	KEY_SEARCH},  /* search */ \
				{834,	54,	KEY_HOME},        /* home */ \
				{834,	90,	KEY_F1},      	  /* menu */ \
				{834,	125,	KEY_BACK	},    /* back */ \
				{0,0,0} \
		}
#endif

#define TOUCH_KEY_LED	RK29_PIN6_PA6
#else
#define GPIO_TOUCHKEY_INT   		RK29_PIN0_PA4
#endif

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

/* Base */
#define OUT_TYPE		SCREEN_RGB
#define OUT_FACE		OUT_P888
#define OUT_CLK			 50000000
#define LCDC_ACLK       150000000     //29 lcdc axi DMA 撞楕

/* Timing */
#define H_PW			1
#define H_BP			46
#define H_VD			800
#define H_FP			210

#define V_PW			3
#define V_BP			23
#define V_VD			600
#define V_FP			12

/* Other */
#define DCLK_POL		0
#define SWAP_RB			0

/***************************************************
 *
 *				     BACKLIGHT
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


/**the value of MIN_BACKLIGHT_SCALE must be between 0~10*/
#define MIN_BACKLIGHT_SCALE	7

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
#define GPIO_WIFI_POWER       RK29_PIN6_PC0


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
#define MASS_STORAGE_NAME		""
#define MASS_STORAGE_PRODUCT	  "M900HC"
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
#define	DC_CURRENT_IN_TWO_MODE          //This macro to distinguish some model such as A7/A8  will work in two charging current mode
#define GPIO_CURRENT_CONTROL		RK29_PIN2_PC6
#define DC_DET_EFFECTIVE		0
#define GPIO_DC_DET			RK29_PIN0_PA0
#define GPIO_CHG_OK			RK29_PIN4_PA3
#define ADC_ADD_VALUE		1
#define ADC_CLI_VALUE		5
#define CHARGE_FULL_GATE 		4150


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
#define DEF_VOL					0xc2
#define DEF_VOL_SPK				0xc6


/***************************************************
 *
 *                      HDMI
 *
 **************************************************/
#define ANX7150_ATTACHED_BUS	1	//attached to I2C3
#define	GPIO_ANX7150_RST	RK29_PIN2_PC7
#define ANX7150_RST_MUX_NAME	GPIO2C7_SPI1RXD_NAME	//NO_IOMUX_PINNAME
#define ANX7150_RST_MUX_MODE	GPIO2H_GPIO2C7	//NO_IO_MUX_MODE


/***************************************************
 *
 *                  CAMERA SENSOR
 *
 **************************************************/
#define SENSOR_NAME_0 RK29_CAM_SENSOR_NAME_OV5642			/* back camera sensor */
#define SENSOR_IIC_ADDR_0 	    0x78
#define SENSOR_IIC_ADAPTER_ID_0    1
#define SENSOR_POWER_PIN_0         INVALID_GPIO
#define SENSOR_RESET_PIN_0         INVALID_GPIO
#define SENSOR_POWERDN_PIN_0       RK29_PIN6_PB7
#define SENSOR_FALSH_PIN_0         INVALID_GPIO
#define SENSOR_POWERACTIVE_LEVEL_0 RK29_CAM_POWERACTIVE_L
#define SENSOR_RESETACTIVE_LEVEL_0 RK29_CAM_RESETACTIVE_L
#define SENSOR_POWERDNACTIVE_LEVEL_0 RK29_CAM_POWERDNACTIVE_H
#define SENSOR_FLASHACTIVE_LEVEL_0 RK29_CAM_FLASHACTIVE_L

#define SENSOR_NAME_1 RK29_CAM_SENSOR_NAME_NT99250			/* front camera sensor */
#define SENSOR_IIC_ADDR_1 	    0x6C
#define SENSOR_IIC_ADAPTER_ID_1    1
#define SENSOR_POWER_PIN_1         INVALID_GPIO
#define SENSOR_RESET_PIN_1         INVALID_GPIO
#define SENSOR_POWERDN_PIN_1       RK29_PIN5_PD7
#define SENSOR_FALSH_PIN_1         INVALID_GPIO
#define SENSOR_POWERACTIVE_LEVEL_1 RK29_CAM_POWERACTIVE_L
#define SENSOR_RESETACTIVE_LEVEL_1 RK29_CAM_RESETACTIVE_L
#define SENSOR_POWERDNACTIVE_LEVEL_1 RK29_CAM_POWERDNACTIVE_H
#define SENSOR_FLASHACTIVE_LEVEL_1 RK29_CAM_FLASHACTIVE_L

#endif
