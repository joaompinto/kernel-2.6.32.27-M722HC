/* arch/arm/mach-rk29/board-rk29.c
 *
 * Copyright (C) 2010 ROCKCHIP, Inc.
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
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/mmc/host.h>
#include <linux/android_pmem.h>
#include <linux/usb/android_composite.h>

#include <mach/hardware.h>
#include <asm/setup.h>
#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/flash.h>
#include <asm/hardware/gic.h>

#include <mach/iomux.h>
#include <mach/gpio.h>
#include <mach/irqs.h>
#include <mach/rk29_iomap.h>
#include <mach/board.h>
#include <mach/rk29_nand.h>
#include <mach/rk29_camera.h>                          /* ddl@rock-chips.com : camera support */
#include <media/soc_camera.h>                               /* ddl@rock-chips.com : camera support */
#include <mach/vpu_mem.h>
#include <mach/sram.h>
#include <mach/ddr.h>

#include <linux/regulator/rk29-pwm-regulator.h>
#include <linux/regulator/machine.h>

#include <linux/mtd/nand.h>
#include <linux/mtd/partitions.h>
#include <linux/vibrate_gpio.h>

#include "devices.h"
#include "../../../drivers/input/touchscreen/xpt2046_cbn_ts.h"


/* Set memory size of pmem */
#ifdef CONFIG_RK29_MEM_SIZE_M
#define SDRAM_SIZE          (CONFIG_RK29_MEM_SIZE_M * SZ_1M)
#else
#define SDRAM_SIZE          SZ_512M
#endif
#define PMEM_GPU_SIZE       SZ_64M
#define PMEM_VPU_SIZE       SZ_64M
#define PMEM_SKYPE_SIZE     SZ_8M
#define PMEM_CAM_SIZE       0x01300000
#ifdef CONFIG_VIDEO_RK29_WORK_IPP
#define MEM_CAMIPP_SIZE     SZ_8M//SZ_4M
#else
#define MEM_CAMIPP_SIZE     0
#endif

#define PMEM_UI_SIZE        SZ_32M
#define MEM_FB_SIZE         (3*SZ_2M)

#ifdef CONFIG_FB_WORK_IPP
#define MEM_FBIPP_SIZE      SZ_8M   //1920 x 1080 x 2 x 2  //RGB565 = x2;RGB888 = x4
#else
#define MEM_FBIPP_SIZE      0
#endif
#define PMEM_GPU_BASE       ((u32)RK29_SDRAM_PHYS + SDRAM_SIZE - PMEM_GPU_SIZE)
#define PMEM_UI_BASE        (PMEM_GPU_BASE - PMEM_UI_SIZE)
#define PMEM_VPU_BASE       (PMEM_UI_BASE - PMEM_VPU_SIZE)
#define PMEM_CAM_BASE       (PMEM_VPU_BASE - PMEM_CAM_SIZE)
#define MEM_CAMIPP_BASE     (PMEM_CAM_BASE - MEM_CAMIPP_SIZE)
#define MEM_FB_BASE         (MEM_CAMIPP_BASE - MEM_FB_SIZE)
#define MEM_FBIPP_BASE      (MEM_FB_BASE - MEM_FBIPP_SIZE)
#define PMEM_SKYPE_BASE      (MEM_FBIPP_BASE - PMEM_SKYPE_SIZE)
#define LINUX_SIZE          (PMEM_SKYPE_BASE - RK29_SDRAM_PHYS)

#define PREALLOC_WLAN_SEC_NUM           4
#define PREALLOC_WLAN_BUF_NUM           160
#define PREALLOC_WLAN_SECTION_HEADER    24

#define WLAN_SECTION_SIZE_0     (PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_1     (PREALLOC_WLAN_BUF_NUM * 128)
#define WLAN_SECTION_SIZE_2     (PREALLOC_WLAN_BUF_NUM * 512)
#define WLAN_SECTION_SIZE_3     (PREALLOC_WLAN_BUF_NUM * 1024)

#define WLAN_SKB_BUF_NUM        16

static struct sk_buff *wlan_static_skb[WLAN_SKB_BUF_NUM];

struct wifi_mem_prealloc {
        void *mem_ptr;
        unsigned long size;
};

extern struct sys_timer rk29_timer;

static int rk29_nand_io_init(void)
{
    return 0;
}

struct rk29_nand_platform_data rk29_nand_data = {
    .width      = 1,     /* data bus width in bytes */
    .hw_ecc     = 1,     /* hw ecc 0: soft ecc */
    .num_flash    = 1,
    .io_init   = rk29_nand_io_init,
};

#ifdef CONFIG_FB_RK29
static int rk29_lcd_io_init(void)
{
    int ret = 0;
    return ret;
}

static int rk29_lcd_io_deinit(void)
{
    int ret = 0;
    return ret;
}

static struct rk29lcd_info rk29_lcd_info = {
    .txd_pin  = LCD_TXD_PIN,
    .clk_pin = LCD_CLK_PIN,
    .cs_pin = LCD_CS_PIN,
    .io_init   = rk29_lcd_io_init,
    .io_deinit = rk29_lcd_io_deinit,
};


static int rk29_fb_io_init(struct rk29_fb_setting_info *fb_setting)
{
    int ret = 0;
    if(fb_setting->mcu_fmk_en && (FB_MCU_FMK_PIN != INVALID_GPIO))
    {
        ret = gpio_request(FB_MCU_FMK_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(FB_MCU_FMK_PIN);
            printk(">>>>>> FB_MCU_FMK_PIN gpio_request err \n ");
        }
        gpio_direction_input(FB_MCU_FMK_PIN);
    }
    if(fb_setting->disp_on_en && (FB_DISPLAY_ON_PIN != INVALID_GPIO))
    {
        ret = gpio_request(FB_DISPLAY_ON_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(FB_DISPLAY_ON_PIN);
            printk(">>>>>> FB_DISPLAY_ON_PIN gpio_request err \n ");
        }
    }

    if(fb_setting->disp_on_en && (FB_LCD_STANDBY_PIN != INVALID_GPIO))
    {
        ret = gpio_request(FB_LCD_STANDBY_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(FB_LCD_STANDBY_PIN);
            printk(">>>>>> FB_LCD_STANDBY_PIN gpio_request err \n ");
        }
    }

    if(FB_LCD_CABC_EN_PIN != INVALID_GPIO)
    {
        ret = gpio_request(FB_LCD_CABC_EN_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(FB_LCD_CABC_EN_PIN);
            printk(">>>>>> FB_LCD_CABC_EN_PIN gpio_request err \n ");
        }
        gpio_direction_output(FB_LCD_CABC_EN_PIN, 0);
        gpio_set_value(FB_LCD_CABC_EN_PIN, GPIO_LOW);
    }

    return ret;
}

static struct rk29fb_info rk29_fb_info = {
    .fb_id   = FB_ID,
    .disp_on_pin = FB_DISPLAY_ON_PIN,
    .disp_on_value = FB_DISPLAY_ON_VALUE,
    .standby_pin = FB_LCD_STANDBY_PIN,
    .standby_value = FB_LCD_STANDBY_VALUE,
    .mcu_fmk_pin = FB_MCU_FMK_PIN,
    .lcd_info = &rk29_lcd_info,
    .io_init   = rk29_fb_io_init,
};

/* rk29 fb resource */
static struct resource rk29_fb_resource[] = {
	[0] = {
        .name  = "lcdc reg",
		.start = RK29_LCDC_PHYS,
		.end   = RK29_LCDC_PHYS + RK29_LCDC_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
	    .name  = "lcdc irq",
		.start = IRQ_LCDC,
		.end   = IRQ_LCDC,
		.flags = IORESOURCE_IRQ,
	},
	[2] = {
	    .name   = "win1 buf",
        .start  = MEM_FB_BASE,
        .end    = MEM_FB_BASE + MEM_FB_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    #ifdef CONFIG_FB_WORK_IPP
    [3] = {
	    .name   = "win1 ipp buf",
        .start  = MEM_FBIPP_BASE,
        .end    = MEM_FBIPP_BASE + MEM_FBIPP_SIZE - 1,
        .flags  = IORESOURCE_MEM,
    },
    #endif
};

/*platform_device*/
struct platform_device rk29_device_fb = {
	.name		  = "rk29-fb",
	.id		  = 4,
	.num_resources	  = ARRAY_SIZE(rk29_fb_resource),
	.resource	  = rk29_fb_resource,
	.dev            = {
		.platform_data  = &rk29_fb_info,
	}
};

struct platform_device rk29_device_dma_cpy = {
	.name		  = "dma_memcpy",
	.id		  = 4,

};

#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name		= "pmem",
	.start		= PMEM_UI_BASE,
	.size		= PMEM_UI_SIZE,
	.no_allocator	= 0,
	.cached		= 1,
};

static struct platform_device android_pmem_device = {
	.name		= "android_pmem",
	.id		= 0,
	.dev		= {
		.platform_data = &android_pmem_pdata,
	},
};


static struct android_pmem_platform_data android_pmem_cam_pdata = {
	.name		= "pmem_cam",
	.start		= PMEM_CAM_BASE,
	.size		= PMEM_CAM_SIZE,
	.no_allocator	= 1,
	.cached		= 1,
};

static struct platform_device android_pmem_cam_device = {
	.name		= "android_pmem",
	.id		= 1,
	.dev		= {
		.platform_data = &android_pmem_cam_pdata,
	},
};


static struct vpu_mem_platform_data vpu_mem_pdata = {
	.name		= "vpu_mem",
	.start		= PMEM_VPU_BASE,
	.size		= PMEM_VPU_SIZE,
	.cached		= 1,
};

static struct platform_device rk29_vpu_mem_device = {
	.name		= "vpu_mem",
	.id		    = 2,
	.dev		= {
	.platform_data = &vpu_mem_pdata,
	},
};

static struct android_pmem_platform_data android_pmem_skype_pdata = {
	.name		= "pmem_skype",
	.start		= PMEM_SKYPE_BASE,
	.size		= PMEM_SKYPE_SIZE,
	.no_allocator	= 0,
	.cached		= 0,
};

static struct platform_device android_pmem_skype_device = {
	.name		= "android_pmem",
	.id		= 3,
	.dev		= {
		.platform_data = &android_pmem_skype_pdata,
	},
};

static struct platform_device rk29_v4l2_output_devce = {
	.name		= "rk29_vout",
};

/*HANNSTAR_P1003 touch*/
#if defined (CONFIG_HANNSTAR_P1003)
#define TOUCH_RESET_PIN RK29_PIN6_PC3
#define TOUCH_INT_PIN   RK29_PIN0_PA2

int p1003_init_platform_hw(void)
{
    if(gpio_request(TOUCH_RESET_PIN,NULL) != 0){
      gpio_free(TOUCH_RESET_PIN);
      printk("p1003_init_platform_hw gpio_request error\n");
      return -EIO;
    }

    if(gpio_request(TOUCH_INT_PIN,NULL) != 0){
      gpio_free(TOUCH_INT_PIN);
      printk("p1003_init_platform_hw gpio_request error\n");
      return -EIO;
    }
    gpio_pull_updown(TOUCH_INT_PIN, 1);
    gpio_direction_output(TOUCH_RESET_PIN, 0);
    msleep(500);
    gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
    msleep(500);
    gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);

    return 0;
}


struct p1003_platform_data p1003_info = {
  .model= 1003,
  .init_platform_hw= p1003_init_platform_hw,

};
#endif
#if defined (CONFIG_EETI_EGALAX)
#define TOUCH_RESET_PIN RK29_PIN6_PC3
#define TOUCH_INT_PIN   RK29_PIN0_PA2

static int EETI_EGALAX_init_platform_hw(void)
{
    if(gpio_request(TOUCH_RESET_PIN,NULL) != 0){
      gpio_free(TOUCH_RESET_PIN);
      printk("p1003_init_platform_hw gpio_request error\n");
      return -EIO;
    }

    if(gpio_request(TOUCH_INT_PIN,NULL) != 0){
      gpio_free(TOUCH_INT_PIN);
      printk("p1003_init_platform_hw gpio_request error\n");
      return -EIO;
    }
    gpio_pull_updown(TOUCH_INT_PIN, 1);
    gpio_direction_output(TOUCH_RESET_PIN, 0);
    msleep(500);
    gpio_set_value(TOUCH_RESET_PIN,GPIO_LOW);
    msleep(500);
    gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);

    return 0;
}


static struct eeti_egalax_platform_data eeti_egalax_info = {
  .model= 1003,
  .init_platform_hw= EETI_EGALAX_init_platform_hw,

};
#endif

static struct ts_hw_data ts_hw_info = {
	.reset_gpio = GPIO_TOUCH_RST,
#if defined (GPIO_TOUCH_EN)
	.touch_en_gpio = GPIO_TOUCH_EN,
#endif
};


#if defined (CONFIG_TOUCHSCREEN_A080SN03)
static struct a080sn03_platform_data a080sn03_info = {
	.reset_gpio = GPIO_TOUCH_RST,
};
#endif



/*ite7260 touch*/
#if defined (CONFIG_TOUCHSCREEN_IT7260)

struct ite7260_platform_data ite7260_info = {
	.model= 7260,
	.reset_gpio = GPIO_TOUCH_RST,
	.power = GPIO_TOUCH_EN,
};

#endif


/*MMA8452 gsensor*/
#if defined (CONFIG_GS_MMA8452)
#define MMA8452_INT_PIN   RK29_PIN0_PA3

static int mma8452_init_platform_hw(void)
{

    if(gpio_request(MMA8452_INT_PIN,NULL) != 0){
      gpio_free(MMA8452_INT_PIN);
      printk("mma8452_init_platform_hw gpio_request error\n");
      return -EIO;
    }
    gpio_pull_updown(MMA8452_INT_PIN, 1);
    return 0;
}


static struct mma8452_platform_data mma8452_info = {
  .model= 8452,
  .swap_xy = 0,

  .init_platform_hw= mma8452_init_platform_hw,

};
#endif

#if defined (CONFIG_BATTERY_BQ27510)
#define	DC_CHECK_PIN	RK29_PIN4_PA1
#define	LI_LION_BAT_NUM	2
static int bq27510_init_dc_check_pin(void){	
	if(gpio_request(DC_CHECK_PIN,"dc_check") != 0){      
		gpio_free(DC_CHECK_PIN);      
		printk("bq27510 init dc check pin request error\n");      
		return -EIO;    
	}	
	gpio_direction_input(DC_CHECK_PIN);	
	return 0;
}

struct bq27510_platform_data bq27510_info = {	
	.init_dc_check_pin = bq27510_init_dc_check_pin,	
	.dc_check_pin =  DC_CHECK_PIN,		
	.bat_num = LI_LION_BAT_NUM,
};
#endif

#if defined (CONFIG_EKT2201_TOUCHKEY)
struct ekt2201_platform_data ekt2201_info = {	
	.led_key = RK29_PIN6_PB4,
};
#endif

/*****************************************************************************************
 * i2c devices
 * author: kfx@rock-chips.com
*****************************************************************************************/
static int rk29_i2c0_io_init(void)
{
	rk29_mux_api_set(GPIO2B7_I2C0SCL_NAME, GPIO2L_I2C0_SCL);
	rk29_mux_api_set(GPIO2B6_I2C0SDA_NAME, GPIO2L_I2C0_SDA);
	return 0;
}

static int rk29_i2c1_io_init(void)
{
	rk29_mux_api_set(GPIO1A7_I2C1SCL_NAME, GPIO1L_I2C1_SCL);
	rk29_mux_api_set(GPIO1A6_I2C1SDA_NAME, GPIO1L_I2C1_SDA);
	return 0;
}
static int rk29_i2c2_io_init(void)
{
	rk29_mux_api_set(GPIO5D4_I2C2SCL_NAME, GPIO5H_I2C2_SCL);
	rk29_mux_api_set(GPIO5D3_I2C2SDA_NAME, GPIO5H_I2C2_SDA);
	return 0;
}

static int rk29_i2c3_io_init(void)
{
	rk29_mux_api_set(GPIO2B5_UART3RTSN_I2C3SCL_NAME, GPIO2L_I2C3_SCL);
	rk29_mux_api_set(GPIO2B4_UART3CTSN_I2C3SDA_NAME, GPIO2L_I2C3_SDA);
	return 0;
}

struct rk29_i2c_platform_data default_i2c0_data = {
	.bus_num    = 0,
	.flags      = 0,
	.slave_addr = 0xff,
	.scl_rate  = 400*1000,
	.mode 		= I2C_MODE_IRQ,
	.io_init = rk29_i2c0_io_init,
};

struct rk29_i2c_platform_data default_i2c1_data = {
	.bus_num    = 1,
	.flags      = 0,
	.slave_addr = 0xff,
	.scl_rate  = 400*1000,
	.mode 		= I2C_MODE_IRQ,
	.io_init = rk29_i2c1_io_init,
};

struct rk29_i2c_platform_data default_i2c2_data = {
	.bus_num    = 2,
	.flags      = 0,
	.slave_addr = 0xff,
	.scl_rate  = 400*1000,
	.mode 		= I2C_MODE_IRQ,
	.io_init = rk29_i2c2_io_init,
};

struct rk29_i2c_platform_data default_i2c3_data = {
	.bus_num    = 3,
	.flags      = 0,
	.slave_addr = 0xff,
	.scl_rate  = 400*1000,
	.mode 		= I2C_MODE_IRQ,
	.io_init = rk29_i2c3_io_init,
};

#if defined (CONFIG_TOUCHSCREEN_COASIA)
#define TOUCH_POWER_PIN RK29_PIN0_PD7
#define TOUCH_RESET_PIN RK29_PIN6_PC3
#define TOUCH_INT_PIN   RK29_PIN0_PA2

int coasia_init_platform_hw(void)
{
    int ret = 0;
    //power
    if(TOUCH_POWER_PIN != INVALID_GPIO)
    {
        //need iomux
        rk29_mux_api_set(GPIO0D7_FLASHCSN6_NAME, GPIO0H_GPIO0D7);
        ret = gpio_request(TOUCH_POWER_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(TOUCH_POWER_PIN);
            printk(">>>>>> BL_EN_PIN gpio_request err \n ");
        }
        gpio_direction_output(TOUCH_POWER_PIN, GPIO_LOW);
    }
    
    //int
    if(TOUCH_INT_PIN != INVALID_GPIO)
    {
        ret = gpio_request(TOUCH_INT_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(TOUCH_INT_PIN);
            printk(">>>>>> BL_EN_PIN gpio_request err \n ");
        }
        gpio_pull_updown(TOUCH_INT_PIN, GPIO_HIGH);
    }
    
    //reset
    if(TOUCH_RESET_PIN != INVALID_GPIO)
    {
        ret = gpio_request(TOUCH_RESET_PIN, NULL);
        if(ret != 0)
        {
            gpio_free(TOUCH_RESET_PIN);
            printk(">>>>>> BL_EN_PIN gpio_request err \n ");
        }
        gpio_direction_output(TOUCH_RESET_PIN, GPIO_LOW);
        msleep(500);
        gpio_set_value(TOUCH_RESET_PIN,GPIO_HIGH);
    }

    return 0;
}


struct coasia_platform_data coasia_info = {
  .model= 1003,
    .power_pin = TOUCH_POWER_PIN,
    .rest_pin  = TOUCH_RESET_PIN,
    .int_pin   = TOUCH_INT_PIN,
  .init_platform_hw= coasia_init_platform_hw,

};
#endif
#ifdef CONFIG_I2C0_RK29
static struct i2c_board_info __initdata board_i2c0_devices[] = {
#if defined (CONFIG_RK1000_CONTROL)
	{
		.type    		= "rk1000_control",
		.addr           = 0x40,
		.flags			= 0,
	},
#endif
#ifdef CONFIG_SENSORS_CM3212
	{
		/* light sensor */
		I2C_BOARD_INFO("cm3212", (0x70>>1)),
		//.platform_data = &cm3212,
	},
#endif
#if defined (CONFIG_SND_SOC_alc5621)
	{
		.type                   = "ALC5621",
		.addr                   = 0x1a,
		.flags                  = 0,
	},
#endif
#if defined (CONFIG_SND_SOC_alc5631)
	{
		.type                   = "rt5631",
		.addr                   = 0x1a,
		.flags                  = 0,
	},
#endif
#if defined (CONFIG_SND_SOC_RK1000)
	{
		.type    		= "rk1000_i2c_codec",
		.addr           = 0x60,
		.flags			= 0,
	},
#endif
#if defined (CONFIG_SND_SOC_WM8900)
	{
		.type    		= "wm8900",
		.addr           = 0x1A,
		.flags			= 0,
	},
#endif
#if defined (CONFIG_SND_SOC_WM8988)
	{
		.type    		= "wm8988",
		.addr           = 0x1A,
		.flags			= 0,
	},
#endif
#if defined (CONFIG_SND_SOC_ES8388)
	{
		.type    		= "es8388",
		.addr           = 0x10,
		.flags			= 0,
	},
#endif 
#if defined (CONFIG_BATTERY_STC3100)
	{
		.type    		= "stc3100",
		.addr           = 0x70,
		.flags			= 0,
	},
#endif
#if defined (CONFIG_BATTERY_BQ27510)
	{
		.type    		= "bq27510",
		.addr           = 0x55,
		.flags			= 0,
		.platform_data  = &bq27510_info,
	},
#endif
#if defined (CONFIG_RTC_HYM8563)
	{
		.type    		= "rtc_hym8563",
		.addr           = 0x51,
		.flags			= 0,
		.irq            = RK29_PIN0_PA1,
	},
#endif
#if defined (CONFIG_GT108M_TOUCHKEY)
    {
      .type           = "gt108m",
      .addr           = 0xB8,
      .flags          = 0,
      .irq            = GPIO_TOUCHKEY_INT,
      //.platform_data  = &gt108m_info,
    },
#endif
#if defined (CONFIG_EKT2201_TOUCHKEY)
    {
      .type           = "ekt2201",
      .addr           = 0x10,
      .flags          = 0,
      .irq            = GPIO_TOUCHKEY_INT,
      .platform_data  = &ekt2201_info,
    },
#endif
#if defined (CONFIG_SO381010_TOUCHKEY) || defined (CONFIG_SO381010_TOUCHKEY_M726)
    {
      .type           = "so381010",
      .addr           = 0x2c,
      .flags          = 0,
      .irq            = GPIO_TOUCHKEY_INT,
    },
#endif
#if defined (CONFIG_GS_MMA8452)
    {
      .type           = "gs_mma8452",
      .addr           = 0x1c,
      .flags          = 0,
      .irq            = MMA8452_INT_PIN,
      .platform_data  = &mma8452_info,
    },
#endif
#if defined (CONFIG_COMPASS_AK8973)
	{
		.type    		= "ak8973",
		.addr           = 0x1d,
		.flags			= 0,
		.irq			= RK29_PIN0_PA4,
	},
#endif
#if defined (CONFIG_COMPASS_AK8975)
	{
		.type    		= "ak8975",
		.addr           = 0x0d,
		.flags			= 0,
		.irq			= RK29_PIN0_PA4,
	},
#endif
};
#endif

#ifdef CONFIG_I2C1_RK29
static struct i2c_board_info __initdata board_i2c1_devices[] = {
#if defined (TOUCH_USE_I2C1)
#if defined (CONFIG_TOUCHSCREEN_FT5301)
    {
      .type           = "ft5x0x",
      .addr           = 0x38,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_FT5301_BYD)
    {
      .type           = "ft5x0x-byd",
      .addr           = 0x3f,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_A080SN03)
    {
      .type           = "raydium310",
      .addr           = 0x5c,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_GOODIX_HUIDING) 
    {
      .type           = "Goodix-TS",
      .addr           = 0x55,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif

#if defined (CONFIG_TOUCHSCREEN_IT7260)
    {
      .type           = "ite7260_touch",
      .addr           = 0x46,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ite7260_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_ST1564)
    {
      .type           = "st1564_ts",
      .addr           = 0x55,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#endif
#if defined (CONFIG_RK1000_CONTROL1)
	{
		.type			= "rk1000_control",
		.addr			= 0x40,
		.flags			= 0,
	},
#endif

#if ANX7150_ATTACHED_BUS==1
#if defined (CONFIG_ANX7150) || defined (CONFIG_ANX7150_NEW)
    {
		.type           = "anx7150",
        .addr           = 0x39,             //0x39, 0x3d
        .flags          = 0,
        .irq            = RK29_PIN1_PD7,
    },
#endif
#endif

};
#endif

#ifdef CONFIG_I2C2_RK29
static struct i2c_board_info __initdata board_i2c2_devices[] = {
#if defined (TOUCH_USE_I2C2)
#if defined (CONFIG_TOUCHSCREEN_COASIA)
    {
      .type           = "coasia_touch",
      .addr           = 0x5c,
      .flags          = I2C_M_NEED_DELAY,//I2C_M_IGNORE_NAK,
      .irq            = RK29_PIN0_PA2,
      .platform_data  = &coasia_info,
      .udelay         = 100,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_FT5301) 
    {
      .type           = "ft5x0x",
      .addr           = 0x38,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_FT5301_BYD)
    {
      .type           = "ft5x0x-byd",
      .addr           = 0x3f,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_A080SN03)
    {
      .type           = "raydium310",
      .addr           = 0x5c,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &a080sn03_info,
    },
#endif

#if defined (CONFIG_TOUCHSCREEN_GOODIX_HUIDING) 
    {
      .type           = "Goodix-TS",
      .addr           = 0x55,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif

#if defined (CONFIG_TOUCHSCREEN_IT7260)
    {
      .type           = "ite7260_touch",
      .addr           = 0x46,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ite7260_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_ST1564)
    {
      .type           = "st1564_ts",
      .addr           = 0x55,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#endif
#if defined (CONFIG_HANNSTAR_P1003)
    {
      .type           = "p1003_touch",
      .addr           = 0x04,
      .flags          = 0,
      .irq            = RK29_PIN0_PA2,
      .platform_data  = &p1003_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_PIXCIR)
    {
      .type           = "pixcir_ts",
      .addr           = 0xB8,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#if defined (CONFIG_EETI_EGALAX)
    {
      .type           = "egalax_i2c",
      .addr           = 0x04,
      .flags          = 0,
      .irq            = RK29_PIN0_PA2,
      .platform_data  = &eeti_egalax_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_ZINITIX)
    {
      .type           = "zinitix_touch",
      .addr           = 0x20,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
#if defined (CONFIG_TOUCHSCREEN_BU21020)
    {
      .type           = "bu21020",
      .addr           = 0x5c,
      .flags          = 0,
      .irq            = GPIO_TOUCH_INT,
      .platform_data  = &ts_hw_info,
    },
#endif
};
#endif

#ifdef CONFIG_I2C3_RK29
static struct i2c_board_info __initdata board_i2c3_devices[] = {

#if ANX7150_ATTACHED_BUS==3
#if defined (CONFIG_ANX7150) || defined (CONFIG_ANX7150_NEW)
		{
			.type			= "anx7150",
			.addr			= 0x39, 			//0x39, 0x3d
			.flags			= 0,
			.irq			= RK29_PIN1_PD7,
		},
#endif
#endif

};

#endif

#ifdef CONFIG_BATTERY_RK2918
struct rk2918_battery_platform_data rk2918_battery_platdata = {
        .dc_det_pin      = RK29_PIN4_PA1,
        .batt_low_pin    = INVALID_GPIO,
        .charge_set_pin  = INVALID_GPIO,
        .charge_ok_pin   = RK29_PIN4_PA3,

        .dc_det_level    = GPIO_LOW,
        .charge_ok_level = GPIO_HIGH,
};
#endif

/*****************************************************************************************
 * camera  devices
 * author: ddl@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_VIDEO_RK29
#ifndef SENSOR_NAME_0
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
#endif
#ifndef SENSOR_NAME_1
#define SENSOR_NAME_1 RK29_CAM_SENSOR_NAME_OV2659			/* front camera sensor */
#define SENSOR_IIC_ADDR_1 	    0x60
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

static int rk29_sensor_io_init(void);
static int rk29_sensor_io_deinit(int sensor);
static int rk29_sensor_ioctrl(struct device *dev,enum rk29camera_ioctrl_cmd cmd,int on);

static struct rk29camera_platform_data rk29_camera_platform_data = {
    .io_init = rk29_sensor_io_init,
    .io_deinit = rk29_sensor_io_deinit,
    .sensor_ioctrl = rk29_sensor_ioctrl,
    .gpio_res = {
        {
            .gpio_reset = SENSOR_RESET_PIN_0,
            .gpio_power = SENSOR_POWER_PIN_0,
            .gpio_powerdown = SENSOR_POWERDN_PIN_0,
            .gpio_flash = SENSOR_FALSH_PIN_0,
            .gpio_flag = (SENSOR_POWERACTIVE_LEVEL_0|SENSOR_RESETACTIVE_LEVEL_0|SENSOR_POWERDNACTIVE_LEVEL_0|SENSOR_FLASHACTIVE_LEVEL_0),
            .gpio_init = 0,
            .dev_name = SENSOR_NAME_0,
        }, {
            .gpio_reset = SENSOR_RESET_PIN_1,
            .gpio_power = SENSOR_POWER_PIN_1,
            .gpio_powerdown = SENSOR_POWERDN_PIN_1,
            .gpio_flash = SENSOR_FALSH_PIN_1,
            .gpio_flag = (SENSOR_POWERACTIVE_LEVEL_1|SENSOR_RESETACTIVE_LEVEL_1|SENSOR_POWERDNACTIVE_LEVEL_1|SENSOR_FLASHACTIVE_LEVEL_1),
            .gpio_init = 0,
            .dev_name = SENSOR_NAME_1,
        }
    },
	#ifdef CONFIG_VIDEO_RK29_WORK_IPP
	.meminfo = {
	    .name  = "camera_ipp_mem",
		.start = MEM_CAMIPP_BASE,
		.size   = MEM_CAMIPP_SIZE,
	}
	#endif
};

static int rk29_sensor_io_init(void)
{
    int ret = 0, i;
    unsigned int camera_reset = INVALID_GPIO, camera_power = INVALID_GPIO;
	unsigned int camera_powerdown = INVALID_GPIO, camera_flash = INVALID_GPIO;
	unsigned int camera_ioflag;

    for (i=0; i<2; i++) {
        camera_reset = rk29_camera_platform_data.gpio_res[i].gpio_reset;
        camera_power = rk29_camera_platform_data.gpio_res[i].gpio_power;
		camera_powerdown = rk29_camera_platform_data.gpio_res[i].gpio_powerdown;
        camera_flash = rk29_camera_platform_data.gpio_res[i].gpio_flash;
		camera_ioflag = rk29_camera_platform_data.gpio_res[i].gpio_flag;
		rk29_camera_platform_data.gpio_res[i].gpio_init = 0;

        if (camera_power != INVALID_GPIO) {
            ret = gpio_request(camera_power, "camera power");
            if (ret)
				goto sensor_io_int_loop_end;
			rk29_camera_platform_data.gpio_res[i].gpio_init |= RK29_CAM_POWERACTIVE_MASK;
            gpio_set_value(camera_reset, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
            gpio_direction_output(camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));

			//printk("\n%s....power pin(%d) init success(0x%x)  \n",__FUNCTION__,camera_power,(((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));

        }

        if (camera_reset != INVALID_GPIO) {
            ret = gpio_request(camera_reset, "camera reset");
            if (ret)
				goto sensor_io_int_loop_end;
			rk29_camera_platform_data.gpio_res[i].gpio_init |= RK29_CAM_RESETACTIVE_MASK;
            gpio_set_value(camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
            gpio_direction_output(camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));

			//printk("\n%s....reset pin(%d) init success(0x%x)\n",__FUNCTION__,camera_reset,((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));

        }

		if (camera_powerdown != INVALID_GPIO) {
            ret = gpio_request(camera_powerdown, "camera powerdown");
            if (ret)
				goto sensor_io_int_loop_end;
			rk29_camera_platform_data.gpio_res[i].gpio_init |= RK29_CAM_POWERDNACTIVE_MASK;
            gpio_set_value(camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
            gpio_direction_output(camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));

			//printk("\n%s....powerdown pin(%d) init success(0x%x) \n",__FUNCTION__,camera_powerdown,((camera_ioflag&RK29_CAM_POWERDNACTIVE_BITPOS)>>RK29_CAM_POWERDNACTIVE_BITPOS));
        }

		if (camera_flash != INVALID_GPIO) {
            ret = gpio_request(camera_flash, "camera flash");
            if (ret)
				goto sensor_io_int_loop_end;
			rk29_camera_platform_data.gpio_res[i].gpio_init |= RK29_CAM_FLASHACTIVE_MASK;
            gpio_set_value(camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
            gpio_direction_output(camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));

			//printk("\n%s....flash pin(%d) init success(0x%x) \n",__FUNCTION__,camera_flash,((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));

        }
		continue;
sensor_io_int_loop_end:
		rk29_sensor_io_deinit(i);
		continue;
    }

    return 0;
}

static int rk29_sensor_io_deinit(int sensor)
{
    unsigned int camera_reset = INVALID_GPIO, camera_power = INVALID_GPIO;
	unsigned int camera_powerdown = INVALID_GPIO, camera_flash = INVALID_GPIO;

    camera_reset = rk29_camera_platform_data.gpio_res[sensor].gpio_reset;
    camera_power = rk29_camera_platform_data.gpio_res[sensor].gpio_power;
	camera_powerdown = rk29_camera_platform_data.gpio_res[sensor].gpio_powerdown;
    camera_flash = rk29_camera_platform_data.gpio_res[sensor].gpio_flash;

	if (rk29_camera_platform_data.gpio_res[sensor].gpio_init & RK29_CAM_POWERACTIVE_MASK) {
	    if (camera_power != INVALID_GPIO) {
	        gpio_direction_input(camera_power);
	        gpio_free(camera_power);
	    }
	}

	if (rk29_camera_platform_data.gpio_res[sensor].gpio_init & RK29_CAM_RESETACTIVE_MASK) {
	    if (camera_reset != INVALID_GPIO)  {
	        gpio_direction_input(camera_reset);
	        gpio_free(camera_reset);
	    }
	}

	if (rk29_camera_platform_data.gpio_res[sensor].gpio_init & RK29_CAM_POWERDNACTIVE_MASK) {
	    if (camera_powerdown != INVALID_GPIO)  {
	        gpio_direction_input(camera_powerdown);
	        gpio_free(camera_powerdown);
	    }
	}

	if (rk29_camera_platform_data.gpio_res[sensor].gpio_init & RK29_CAM_FLASHACTIVE_MASK) {
	    if (camera_flash != INVALID_GPIO)  {
	        gpio_direction_input(camera_flash);
	        gpio_free(camera_flash);
	    }
	}

	rk29_camera_platform_data.gpio_res[sensor].gpio_init = 0;
    return 0;
}
static int rk29_sensor_ioctrl(struct device *dev,enum rk29camera_ioctrl_cmd cmd, int on)
{
    unsigned int camera_power=INVALID_GPIO,camera_reset=INVALID_GPIO, camera_powerdown=INVALID_GPIO,camera_flash = INVALID_GPIO;
	unsigned int camera_ioflag,camera_io_init;
	int ret = RK29_CAM_IO_SUCCESS;

    if(rk29_camera_platform_data.gpio_res[0].dev_name &&  (strcmp(rk29_camera_platform_data.gpio_res[0].dev_name, dev_name(dev)) == 0)) {
		camera_power = rk29_camera_platform_data.gpio_res[0].gpio_power;
		camera_reset = rk29_camera_platform_data.gpio_res[0].gpio_reset;
        camera_powerdown = rk29_camera_platform_data.gpio_res[0].gpio_powerdown;
		camera_flash = rk29_camera_platform_data.gpio_res[0].gpio_flash;
		camera_ioflag = rk29_camera_platform_data.gpio_res[0].gpio_flag;
		camera_io_init = rk29_camera_platform_data.gpio_res[0].gpio_init;
    } else if (rk29_camera_platform_data.gpio_res[1].dev_name && (strcmp(rk29_camera_platform_data.gpio_res[1].dev_name, dev_name(dev)) == 0)) {
    	camera_power = rk29_camera_platform_data.gpio_res[1].gpio_power;
        camera_reset = rk29_camera_platform_data.gpio_res[1].gpio_reset;
        camera_powerdown = rk29_camera_platform_data.gpio_res[1].gpio_powerdown;
		camera_flash = rk29_camera_platform_data.gpio_res[1].gpio_flash;
		camera_ioflag = rk29_camera_platform_data.gpio_res[1].gpio_flag;
		camera_io_init = rk29_camera_platform_data.gpio_res[1].gpio_init;
    }

 	switch (cmd)
 	{
 		case Cam_Power:
		{
			if (camera_power != INVALID_GPIO)  {
				if (camera_io_init & RK29_CAM_POWERACTIVE_MASK) {
			        if (on) {
			        	gpio_set_value(camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
						//printk("\n%s..%s..PowerPin=%d ..PinLevel = %x   \n",__FUNCTION__,dev_name(dev), camera_power, ((camera_ioflag&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
						msleep(10);
					} else {
						gpio_set_value(camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
						//printk("\n%s..%s..PowerPin=%d ..PinLevel = %x   \n",__FUNCTION__,dev_name(dev), camera_power, (((~camera_ioflag)&RK29_CAM_POWERACTIVE_MASK)>>RK29_CAM_POWERACTIVE_BITPOS));
					}
				} else {
					ret = RK29_CAM_EIO_REQUESTFAIL;
					printk("\n%s..%s..PowerPin=%d request failed!\n",__FUNCTION__,dev_name(dev),camera_reset);
				}
		    } else {
				ret = RK29_CAM_EIO_INVALID;
		    }
			break;
		}
		case Cam_Reset:
		{
			if (camera_reset != INVALID_GPIO) {
				if (camera_io_init & RK29_CAM_RESETACTIVE_MASK) {
					if (on) {
			        	gpio_set_value(camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
			        	//printk("\n%s..%s..ResetPin=%d ..PinLevel = %x \n",__FUNCTION__,dev_name(dev),camera_reset, ((camera_ioflag&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
					} else {
						gpio_set_value(camera_reset,(((~camera_ioflag)&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
		        		//printk("\n%s..%s..ResetPin= %d..PinLevel = %x   \n",__FUNCTION__,dev_name(dev), camera_reset, (((~camera_ioflag)&RK29_CAM_RESETACTIVE_MASK)>>RK29_CAM_RESETACTIVE_BITPOS));
			        }
				} else {
					ret = RK29_CAM_EIO_REQUESTFAIL;
					printk("\n%s..%s..ResetPin=%d request failed!\n",__FUNCTION__,dev_name(dev),camera_reset);
				}
		    } else {
				ret = RK29_CAM_EIO_INVALID;
		    }
			break;
		}

		case Cam_PowerDown:
		{
			if (camera_powerdown != INVALID_GPIO) {
				if (camera_io_init & RK29_CAM_POWERDNACTIVE_MASK) {
					if (on) {
			        	gpio_set_value(camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
			        	printk("\n\n********%s..%s..PowerDownPin=%d ..PinLevel = %x \n\n",__FUNCTION__,dev_name(dev),camera_powerdown, ((camera_ioflag&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
					} else {
						gpio_set_value(camera_powerdown,(((~camera_ioflag)&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
		        		printk("\n\n******** %s..%s..PowerDownPin= %d..PinLevel = %x \n\n",__FUNCTION__,dev_name(dev), camera_powerdown, (((~camera_ioflag)&RK29_CAM_POWERDNACTIVE_MASK)>>RK29_CAM_POWERDNACTIVE_BITPOS));
			        }
				} else {
					ret = RK29_CAM_EIO_REQUESTFAIL;
					printk("\n%s..%s..PowerDownPin=%d request failed!\n",__FUNCTION__,dev_name(dev),camera_powerdown);
				}
		    } else {
				ret = RK29_CAM_EIO_INVALID;
		    }
			break;
		}

		case Cam_Flash:
		{
			if (camera_flash != INVALID_GPIO) {
				if (camera_io_init & RK29_CAM_FLASHACTIVE_MASK) {
                    switch (on)
                    {
                        case Flash_Off:
                        {
                            gpio_set_value(camera_flash,(((~camera_ioflag)&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
		        		    //printk("\n%s..%s..FlashPin= %d..PinLevel = %x   \n",__FUNCTION__,dev_name(dev), camera_flash, (((~camera_ioflag)&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS)); 
		        		    break;
                        }

                        case Flash_On:
                        {
                            gpio_set_value(camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
			        	    //printk("\n%s..%s..FlashPin=%d ..PinLevel = %x \n",__FUNCTION__,dev_name(dev),camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
			        	    break;
                        }

                        case Flash_Torch:
                        {
                            gpio_set_value(camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
			        	    //printk("\n%s..%s..FlashPin=%d ..PinLevel = %x \n",__FUNCTION__,dev_name(dev),camera_flash, ((camera_ioflag&RK29_CAM_FLASHACTIVE_MASK)>>RK29_CAM_FLASHACTIVE_BITPOS));
			        	    break;
                        }

                        default:
                        {
                            printk("\n%s..%s..Flash command(%d) is invalidate \n",__FUNCTION__,dev_name(dev),on);
                            break;
                        }
                    }
				} else {
					ret = RK29_CAM_EIO_REQUESTFAIL;
					printk("\n%s..%s..FlashPin=%d request failed!\n",__FUNCTION__,dev_name(dev),camera_flash);
				}
		    } else {
				ret = RK29_CAM_EIO_INVALID;
		    }
			break;
		}

		default:
		{
			printk("%s cmd(0x%x) is unknown!\n",__FUNCTION__, cmd);
			break;
		}
 	}
    return ret;
}
static int rk29_sensor_power(struct device *dev, int on)
{
	rk29_sensor_ioctrl(dev,Cam_Power,on);
    return 0;
}
static int rk29_sensor_reset(struct device *dev)
{
	rk29_sensor_ioctrl(dev,Cam_Reset,1);
	msleep(2);
	rk29_sensor_ioctrl(dev,Cam_Reset,0);
	return 0;
}
static int rk29_sensor_powerdown(struct device *dev, int on)
{
	return rk29_sensor_ioctrl(dev,Cam_PowerDown,on);
}
#if (SENSOR_IIC_ADDR_0 != 0x00)
static struct i2c_board_info rk29_i2c_cam_info_0[] = {
	{
		I2C_BOARD_INFO(SENSOR_NAME_0, SENSOR_IIC_ADDR_0>>1)
	},
};

static struct soc_camera_link rk29_iclink_0 = {
	.bus_id		= RK29_CAM_PLATFORM_DEV_ID,
	.power		= rk29_sensor_power,
	.powerdown  = rk29_sensor_powerdown,
	.board_info	= &rk29_i2c_cam_info_0[0],
	.i2c_adapter_id	= SENSOR_IIC_ADAPTER_ID_0,
	.module_name	= SENSOR_NAME_0,
};

/*platform_device : soc-camera need  */
static struct platform_device rk29_soc_camera_pdrv_0 = {
	.name	= "soc-camera-pdrv",
	.id	= 0,
	.dev	= {
		.init_name = SENSOR_NAME_0,
		.platform_data = &rk29_iclink_0,
	},
};
#endif
static struct i2c_board_info rk29_i2c_cam_info_1[] = {
	{
		I2C_BOARD_INFO(SENSOR_NAME_1, SENSOR_IIC_ADDR_1>>1)
	},
};

static struct soc_camera_link rk29_iclink_1 = {
	.bus_id		= RK29_CAM_PLATFORM_DEV_ID,
	.power		= rk29_sensor_power,
	.powerdown  = rk29_sensor_powerdown,
	.board_info	= &rk29_i2c_cam_info_1[0],
	.i2c_adapter_id	= SENSOR_IIC_ADAPTER_ID_1,
	.module_name	= SENSOR_NAME_1,
};

/*platform_device : soc-camera need  */
static struct platform_device rk29_soc_camera_pdrv_1 = {
	.name	= "soc-camera-pdrv",
	.id	= 1,
	.dev	= {
		.init_name = SENSOR_NAME_1,
		.platform_data = &rk29_iclink_1,
	},
};


static u64 rockchip_device_camera_dmamask = 0xffffffffUL;
static struct resource rk29_camera_resource[] = {
	[0] = {
		.start = RK29_VIP_PHYS,
		.end   = RK29_VIP_PHYS + RK29_VIP_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.start = IRQ_VIP,
		.end   = IRQ_VIP,
		.flags = IORESOURCE_IRQ,
	}
};

/*platform_device : */
static struct platform_device rk29_device_camera = {
	.name		  = RK29_CAM_DRV_NAME,
	.id		  = RK29_CAM_PLATFORM_DEV_ID,               /* This is used to put cameras on this interface */
	.num_resources	  = ARRAY_SIZE(rk29_camera_resource),
	.resource	  = rk29_camera_resource,
	.dev            = {
		.dma_mask = &rockchip_device_camera_dmamask,
		.coherent_dma_mask = 0xffffffffUL,
		.platform_data  = &rk29_camera_platform_data,
	}
};
#endif
/*****************************************************************************************
 * backlight  devices
 * author: nzy@rock-chips.com
 *****************************************************************************************/
#ifdef CONFIG_BACKLIGHT_RK29_BL
static int rk29_backlight_io_init(void)
{
    int ret = 0;

    rk29_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
#ifdef BL_EN_PIN 
#ifdef BL_EN_MUX_NAME
	rk29_mux_api_set(BL_EN_MUX_NAME, BL_EN_MUX_MODE);
#endif

	ret = gpio_request(BL_EN_PIN, NULL);
	if(ret != 0)
	{
		gpio_free(BL_EN_PIN);
	}

	gpio_direction_output(BL_EN_PIN, 0);
	gpio_set_value(BL_EN_PIN, BL_EN_VALUE);
#endif
    return ret;
}

static int rk29_backlight_io_deinit(void)
{
	int ret = 0;
#ifdef  BL_EN_PIN
	gpio_free(BL_EN_PIN);
#endif
#ifdef BL_EN_MUX_NAME
	rk29_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
#endif
    return ret;
}

static int rk29_backlight_pwm_suspend(void)
{
	int ret = 0;
	rk29_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE_GPIO);
	if (ret = gpio_request(GPIO_BL_PWM, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
		return -1;
	}
	gpio_direction_output(GPIO_BL_PWM, GPIO_LOW);
	return ret;
}

static int rk29_backlight_pwm_resume(void)
{
	gpio_free(GPIO_BL_PWM);
	rk29_mux_api_set(PWM_MUX_NAME, PWM_MUX_MODE);
	return 0;
}

struct rk29_bl_info rk29_bl_info = {
    .pwm_id   = PWM_ID,
    .bl_ref   = PWM_EFFECT_VALUE,
    .io_init   = rk29_backlight_io_init,
    .io_deinit = rk29_backlight_io_deinit,
    .pwm_suspend = rk29_backlight_pwm_suspend,
    .pwm_resume = rk29_backlight_pwm_resume,
};
#endif
/*****************************************************************************************
* pwm voltage regulator devices
******************************************************************************************/
#if defined (CONFIG_RK29_PWM_REGULATOR)

#define REGULATOR_PWM_ID					2
#define REGULATOR_PWM_MUX_NAME      		GPIO2A3_SDMMC0WRITEPRT_PWM2_NAME
#define REGULATOR_PWM_MUX_MODE      					GPIO2L_PWM2
#define REGULATOR_PWM_MUX_MODE_GPIO 				GPIO2L_GPIO2A3
#define REGULATOR_PWM_GPIO				RK29_PIN2_PA3

static struct regulator_consumer_supply pwm_consumers[] = {
	{
		.supply = "vcore",
	}
};

static struct regulator_init_data rk29_pwm_regulator_data = {
	.constraints = {
		.name = "PWM2",
		.min_uV =  950000,
		.max_uV = 1400000,
		.apply_uV = 1,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS | REGULATOR_CHANGE_VOLTAGE,
	},
	.num_consumer_supplies = ARRAY_SIZE(pwm_consumers),
	.consumer_supplies = pwm_consumers,
};

static struct pwm_platform_data rk29_regulator_pwm_platform_data = {
	.pwm_id = REGULATOR_PWM_ID,
	.pwm_gpio = REGULATOR_PWM_GPIO,
	//.pwm_iomux_name[] = REGULATOR_PWM_MUX_NAME;
	.pwm_iomux_name = REGULATOR_PWM_MUX_NAME,
	.pwm_iomux_pwm = REGULATOR_PWM_MUX_MODE,
	.pwm_iomux_gpio = REGULATOR_PWM_MUX_MODE_GPIO,
	.init_data  = &rk29_pwm_regulator_data,
};

static struct platform_device rk29_device_pwm_regulator = {
	.name = "pwm-voltage-regulator",
	.id   = -1,
	.dev  = {
		.platform_data = &rk29_regulator_pwm_platform_data,
	},
};

#endif

#ifdef CONFIG_GPIO_VIBRATE
static int rk29_vibrator_init()
{
	int ret;

	ret = gpio_request(GPIO_VIB_CON,"vibrate con");

	return ret;
}

static void rk29_vibrator_exit()
{
}

static void rk29_vibrator_power_on()
{
}

static void rk29_vibrator_power_off()
{
}

static struct vib_gpio_platform_data rk29_vib_gpio_data = {
	.gpio = GPIO_VIB_CON,
	.max_timeout = 15000,
	.active_low = !VIB_CON_ENABLE,
	.initial_vibrate = 0,
	.init = rk29_vibrator_init,
	//.exit = rk29_vibrator_exit,
	//.power_on = rk29_vibrator_power_on,
	//.power_off = rk29_vibrator_power_off,
};

static struct platform_device rk29_vib_gpio = {
	.name = "vib-gpio",
	.id = -1,
	.dev = {
		.platform_data = &rk29_vib_gpio_data,
	}
};
#endif

#ifdef CONFIG_AR6003
static struct platform_device rk29sdk_wifi_ar6k_drv_device = {
	.name = "wlan_ar6000_drv",
	.id = 0,
	/* .dev = { */
	/* 	.platform_data = &rk29sdk_wifi_control, */
	/* }, */
};

static struct platform_device rk29sdk_wifi_ar6k_pm_device = {
	.name = "wlan_ar6000_pm",
	.id = 0,
	/* .dev = { */
	/* 	.platform_data = &rk29sdk_wifi_control, */
	/* }, */
};
#endif

#if defined(CONFIG_GPS_POWER)
static struct rk29_gps_data gps_data = {
	.power_io = GPIO_GPS_EN,
	.onoff_io = GPIO_GPS_ONOFF,
	.uart_id = GPS_UART_PORT,
};

struct platform_device rk29_device_gps = {
	.name	= "rk29_gps",
	.id	= 0,
	.dev			= {
		.platform_data	= &gps_data,
	},
};
#endif
/*****************************************************************************************
 * SDMMC devices
*****************************************************************************************/
#ifdef CONFIG_SDMMC0_RK29
static int rk29_sdmmc0_cfg_gpio(void)
{
	rk29_mux_api_set(GPIO1D1_SDMMC0CMD_NAME, GPIO1H_SDMMC0_CMD);
	rk29_mux_api_set(GPIO1D0_SDMMC0CLKOUT_NAME, GPIO1H_SDMMC0_CLKOUT);
	rk29_mux_api_set(GPIO1D2_SDMMC0DATA0_NAME, GPIO1H_SDMMC0_DATA0);
	rk29_mux_api_set(GPIO1D3_SDMMC0DATA1_NAME, GPIO1H_SDMMC0_DATA1);
	rk29_mux_api_set(GPIO1D4_SDMMC0DATA2_NAME, GPIO1H_SDMMC0_DATA2);
	rk29_mux_api_set(GPIO1D5_SDMMC0DATA3_NAME, GPIO1H_SDMMC0_DATA3);
	rk29_mux_api_set(GPIO2A2_SDMMC0DETECTN_NAME, GPIO2L_SDMMC0_DETECT_N);
	//rk29_mux_api_set(GPIO2A2_SDMMC0DETECTN_NAME, GPIO2L_GPIO2A2);
	rk29_mux_api_set(GPIO5D5_SDMMC0PWREN_NAME, GPIO5H_GPIO5D5);   ///GPIO5H_SDMMC0_PWR_EN);  ///GPIO5H_GPIO5D5);
	gpio_request(RK29_PIN5_PD5,"sdmmc");
#if 1
	gpio_set_value(RK29_PIN5_PD5,GPIO_HIGH);
	mdelay(100);
	gpio_set_value(RK29_PIN5_PD5,GPIO_LOW);
#else
	gpio_direction_output(RK29_PIN5_PD5,GPIO_LOW);
#endif
	return 0;
}

#define CONFIG_SDMMC0_USE_DMA
struct rk29_sdmmc_platform_data default_sdmmc0_data = {
	.host_ocr_avail = (MMC_VDD_25_26|MMC_VDD_26_27|MMC_VDD_27_28|MMC_VDD_28_29|MMC_VDD_29_30|
					   MMC_VDD_30_31|MMC_VDD_31_32|MMC_VDD_32_33|
					   MMC_VDD_33_34|MMC_VDD_34_35| MMC_VDD_35_36),
	.host_caps 	= (MMC_CAP_4_BIT_DATA|MMC_CAP_MMC_HIGHSPEED|MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc0_cfg_gpio,
	.dma_name = "sd_mmc",
#ifdef CONFIG_SDMMC0_USE_DMA
	.use_dma  = 1,
#else
	.use_dma = 0,
#endif
	.detect_irq = RK29_PIN2_PA2, // INVALID_GPIO
	.enable_sd_wakeup = 0,
};
#endif
#ifdef CONFIG_SDMMC1_RK29
#define CONFIG_SDMMC1_USE_DMA
static int rk29_sdmmc1_cfg_gpio(void)
{
	rk29_mux_api_set(GPIO1C2_SDMMC1CMD_NAME, GPIO1H_SDMMC1_CMD);
	rk29_mux_api_set(GPIO1C7_SDMMC1CLKOUT_NAME, GPIO1H_SDMMC1_CLKOUT);
	rk29_mux_api_set(GPIO1C3_SDMMC1DATA0_NAME, GPIO1H_SDMMC1_DATA0);
	rk29_mux_api_set(GPIO1C4_SDMMC1DATA1_NAME, GPIO1H_SDMMC1_DATA1);
	rk29_mux_api_set(GPIO1C5_SDMMC1DATA2_NAME, GPIO1H_SDMMC1_DATA2);
	rk29_mux_api_set(GPIO1C6_SDMMC1DATA3_NAME, GPIO1H_SDMMC1_DATA3);
	//rk29_mux_api_set(GPIO1C0_UART0CTSN_SDMMC1DETECTN_NAME, GPIO1H_SDMMC1_DETECT_N);
	return 0;
}

#ifdef CONFIG_WIFI_CONTROL_FUNC
static int rk29sdk_wifi_status(struct device *dev);
static int rk29sdk_wifi_status_register(void (*callback)(int card_presend, void *dev_id), void *dev_id);
#endif

#define RK29SDK_WIFI_SDIO_CARD_DETECT_N    RK29_PIN1_PD6

struct rk29_sdmmc_platform_data default_sdmmc1_data = {
	.host_ocr_avail = (MMC_VDD_25_26|MMC_VDD_26_27|MMC_VDD_27_28|MMC_VDD_28_29|
					   MMC_VDD_29_30|MMC_VDD_30_31|MMC_VDD_31_32|
					   MMC_VDD_32_33|MMC_VDD_33_34),
	.host_caps 	= (MMC_CAP_4_BIT_DATA|MMC_CAP_SDIO_IRQ|
				   MMC_CAP_MMC_HIGHSPEED|MMC_CAP_SD_HIGHSPEED),
	.io_init = rk29_sdmmc1_cfg_gpio,
	.dma_name = "sdio",
#ifdef CONFIG_SDMMC1_USE_DMA
	.use_dma  = 1,
#else
	.use_dma = 0,
#endif
#ifdef CONFIG_WIFI_CONTROL_FUNC
        .status = rk29sdk_wifi_status,
        .register_status_notify = rk29sdk_wifi_status_register,
#endif
#if 0
        .detect_irq = RK29SDK_WIFI_SDIO_CARD_DETECT_N,
#endif
};
#endif

#ifdef CONFIG_WIFI_CONTROL_FUNC

static int rk29sdk_wifi_cd = 0;   /* wifi virtual 'card detect' status */
static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;
int rk29sdk_wifi_power_state = 0;
int rk29sdk_bt_power_state = 0;

static int rk29sdk_wifi_status(struct device *dev)
{
        return rk29sdk_wifi_cd;
}

static int rk29sdk_wifi_status_register(void (*callback)(int card_present, void *dev_id), void *dev_id)
{
        if(wifi_status_cb)
                return -EAGAIN;
        wifi_status_cb = callback;
        wifi_status_cb_devid = dev_id;
        return 0;
}

static int rk29sdk_wifi_bt_gpio_control_init(void)
{
    if (gpio_request(GPIO_WIFI_POWER, "wifi_bt_power")) {
           pr_info("%s: request wifi_bt power gpio failed\n", __func__);
           return -1;
    }

    if (gpio_request(GPIO_WIFI_RESET, "wifi reset")) {
           pr_info("%s: request wifi reset gpio failed\n", __func__);
           gpio_free(GPIO_WIFI_POWER);
           return -1;
    }

#if defined(GPIO_BT_RESET)
    if (gpio_request(GPIO_BT_RESET, "bt reset")) {
          pr_info("%s: request bt reset gpio failed\n", __func__);
          gpio_free(GPIO_WIFI_RESET);
          return -1;
    }
#endif

    gpio_direction_output(GPIO_WIFI_POWER, GPIO_LOW);
    gpio_direction_output(GPIO_WIFI_RESET,    GPIO_LOW);
#if defined(GPIO_BT_RESET)
    gpio_direction_output(GPIO_BT_RESET,      GPIO_LOW);
#endif

    pr_info("%s: init finished\n",__func__);

    return 0;
}

static int rk29sdk_wifi_power(int on)
{
	printk("%s: %d\n", __func__, on);
	if (on){
		gpio_set_value(GPIO_WIFI_POWER, on);
        gpio_set_value(GPIO_WIFI_RESET, on);
		mdelay(100);
		printk("wifi turn on power\n");
	}else{
		if (!rk29sdk_bt_power_state){
			gpio_set_value(GPIO_WIFI_POWER, on);
			mdelay(100);
			printk("wifi shut off power\n");
		}else
		{
			printk("wifi shouldn't shut off power, bt is using it!\n");
		}
		gpio_set_value(GPIO_WIFI_RESET, on);

	}
	rk29sdk_wifi_power_state = on;
	return 0;
}

static int rk29sdk_wifi_reset_state;
static int rk29sdk_wifi_reset(int on)
{
        pr_info("%s: %d\n", __func__, on);
        gpio_set_value(GPIO_WIFI_RESET, on);
        mdelay(100);
        rk29sdk_wifi_reset_state = on;
        return 0;
}

int rk29sdk_wifi_set_carddetect(int val)
{
	pr_info("%s:%d\n", __func__, val);
	rk29sdk_wifi_cd = val;
	if (wifi_status_cb){
		wifi_status_cb(val, wifi_status_cb_devid);
	}else {
		pr_warning("%s, nobody to notify\n", __func__);
	}
	return 0;
}
EXPORT_SYMBOL(rk29sdk_wifi_set_carddetect);

static struct wifi_mem_prealloc wifi_mem_array[PREALLOC_WLAN_SEC_NUM] = {
	{NULL, (WLAN_SECTION_SIZE_0 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_1 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_2 + PREALLOC_WLAN_SECTION_HEADER)},
	{NULL, (WLAN_SECTION_SIZE_3 + PREALLOC_WLAN_SECTION_HEADER)}
};

static void *rk29sdk_mem_prealloc(int section, unsigned long size)
{
	if (section == PREALLOC_WLAN_SEC_NUM)
		return wlan_static_skb;

	if ((section < 0) || (section > PREALLOC_WLAN_SEC_NUM))
		return NULL;

	if (wifi_mem_array[section].size < size)
		return NULL;

	return wifi_mem_array[section].mem_ptr;
}

int __init rk29sdk_init_wifi_mem(void)
{
	int i;
	int j;

	for (i = 0 ; i < WLAN_SKB_BUF_NUM ; i++) {
		wlan_static_skb[i] = dev_alloc_skb(
				((i < (WLAN_SKB_BUF_NUM / 2)) ? 4096 : 8192));

		if (!wlan_static_skb[i])
			goto err_skb_alloc;
	}

	for (i = 0 ; i < PREALLOC_WLAN_SEC_NUM ; i++) {
		wifi_mem_array[i].mem_ptr =
			kmalloc(wifi_mem_array[i].size, GFP_KERNEL);

		if (!wifi_mem_array[i].mem_ptr)
			goto err_mem_alloc;
	}
	return 0;

err_mem_alloc:
	pr_err("Failed to mem_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		kfree(wifi_mem_array[j].mem_ptr);

	i = WLAN_SKB_BUF_NUM;

err_skb_alloc:
	pr_err("Failed to skb_alloc for WLAN\n");
	for (j = 0 ; j < i ; j++)
		dev_kfree_skb(wlan_static_skb[j]);

	return -ENOMEM;
}

static struct wifi_platform_data rk29sdk_wifi_control = {
	.set_power = rk29sdk_wifi_power,
	.set_reset = rk29sdk_wifi_reset,
	.set_carddetect = rk29sdk_wifi_set_carddetect,
	.mem_prealloc   = rk29sdk_mem_prealloc,
};

static struct platform_device rk29sdk_wifi_device = {
	.name = "bcm4329_wlan",
	.id = 1,
	.dev = {
		.platform_data = &rk29sdk_wifi_control,
	},
};
#endif


/* bluetooth rfkill device */
static struct platform_device rk29sdk_rfkill = {
        .name = "rk29sdk_rfkill",
        .id = -1,
};


#ifdef CONFIG_VIVANTE
static struct resource resources_gpu[] = {
    [0] = {
		.name 	= "gpu_irq",
        .start 	= IRQ_GPU,
        .end    = IRQ_GPU,
        .flags  = IORESOURCE_IRQ,
    },
    [1] = {
		.name = "gpu_base",
        .start  = RK29_GPU_PHYS,
        .end    = RK29_GPU_PHYS + RK29_GPU_SIZE,
        .flags  = IORESOURCE_MEM,
    },
    [2] = {
		.name = "gpu_mem",
        .start  = PMEM_GPU_BASE,
        .end    = PMEM_GPU_BASE + PMEM_GPU_SIZE,
        .flags  = IORESOURCE_MEM,
    },
};
static struct platform_device rk29_device_gpu = {
    .name             = "galcore",
    .id               = 0,
    .num_resources    = ARRAY_SIZE(resources_gpu),
    .resource         = resources_gpu,
};
#endif
#ifdef CONFIG_KEYS_RK29
extern struct rk29_keys_platform_data rk29_keys_pdata;
static struct platform_device rk29_device_keys = {
	.name		= "rk29-keypad",
	.id		= -1,
	.dev		= {
		.platform_data	= &rk29_keys_pdata,
	},
};
#endif

static void __init rk29_board_iomux_init(void)
{
	#ifdef CONFIG_UART0_RK29
	rk29_mux_api_set(GPIO1B7_UART0SOUT_NAME, GPIO1L_UART0_SOUT);
	rk29_mux_api_set(GPIO1B6_UART0SIN_NAME, GPIO1L_UART0_SIN);
	#ifdef CONFIG_UART0_CTS_RTS_RK29
	rk29_mux_api_set(GPIO1C1_UART0RTSN_SDMMC1WRITEPRT_NAME, GPIO1H_UART0_RTS_N);
	rk29_mux_api_set(GPIO1C0_UART0CTSN_SDMMC1DETECTN_NAME, GPIO1H_UART0_CTS_N);
	#endif
	#endif
	#ifdef CONFIG_UART1_RK29
	rk29_mux_api_set(GPIO2A5_UART1SOUT_NAME, GPIO2L_UART1_SOUT);
	rk29_mux_api_set(GPIO2A4_UART1SIN_NAME, GPIO2L_UART1_SIN);
	#endif
	#ifdef CONFIG_UART2_RK29
	rk29_mux_api_set(GPIO2B1_UART2SOUT_NAME, GPIO2L_UART2_SOUT);
	rk29_mux_api_set(GPIO2B0_UART2SIN_NAME, GPIO2L_UART2_SIN);
	#ifdef CONFIG_UART2_CTS_RTS_RK29
	rk29_mux_api_set(GPIO2A7_UART2RTSN_NAME, GPIO2L_UART2_RTS_N);
	rk29_mux_api_set(GPIO2A6_UART2CTSN_NAME, GPIO2L_UART2_CTS_N);
	#endif
	#endif
	#ifdef CONFIG_UART3_RK29
	rk29_mux_api_set(GPIO2B3_UART3SOUT_NAME, GPIO2L_UART3_SOUT);
	rk29_mux_api_set(GPIO2B2_UART3SIN_NAME, GPIO2L_UART3_SIN);
	#ifdef CONFIG_UART3_CTS_RTS_RK29
	rk29_mux_api_set(GPIO2B5_UART3RTSN_I2C3SCL_NAME, GPIO2L_UART3_RTS_N);
	rk29_mux_api_set(GPIO2B4_UART3CTSN_I2C3SDA_NAME, GPIO2L_UART3_CTS_N);
	#endif
	#endif
	#ifdef CONFIG_SPIM0_RK29
    rk29_mux_api_set(GPIO2C0_SPI0CLK_NAME, GPIO2H_SPI0_CLK);
	rk29_mux_api_set(GPIO2C1_SPI0CSN0_NAME, GPIO2H_SPI0_CSN0);
	rk29_mux_api_set(GPIO2C2_SPI0TXD_NAME, GPIO2H_SPI0_TXD);
	rk29_mux_api_set(GPIO2C3_SPI0RXD_NAME, GPIO2H_SPI0_RXD);
    #endif
    #ifdef CONFIG_SPIM1_RK29
    rk29_mux_api_set(GPIO2C4_SPI1CLK_NAME, GPIO2H_SPI1_CLK);
	rk29_mux_api_set(GPIO2C5_SPI1CSN0_NAME, GPIO2H_SPI1_CSN0);
	rk29_mux_api_set(GPIO2C6_SPI1TXD_NAME, GPIO2H_SPI1_TXD);
	rk29_mux_api_set(GPIO2C7_SPI1RXD_NAME, GPIO2H_SPI1_RXD);
    #endif
	#ifdef CONFIG_RK29_VMAC
    rk29_mux_api_set(GPIO4C0_RMIICLKOUT_RMIICLKIN_NAME, GPIO4H_RMII_CLKOUT);
    rk29_mux_api_set(GPIO4C1_RMIITXEN_MIITXEN_NAME, GPIO4H_RMII_TX_EN);
    rk29_mux_api_set(GPIO4C2_RMIITXD1_MIITXD1_NAME, GPIO4H_RMII_TXD1);
    rk29_mux_api_set(GPIO4C3_RMIITXD0_MIITXD0_NAME, GPIO4H_RMII_TXD0);
    rk29_mux_api_set(GPIO4C4_RMIIRXERR_MIIRXERR_NAME, GPIO4H_RMII_RX_ERR);
    rk29_mux_api_set(GPIO4C5_RMIICSRDVALID_MIIRXDVALID_NAME, GPIO4H_RMII_CSR_DVALID);
    rk29_mux_api_set(GPIO4C6_RMIIRXD1_MIIRXD1_NAME, GPIO4H_RMII_RXD1);
    rk29_mux_api_set(GPIO4C7_RMIIRXD0_MIIRXD0_NAME, GPIO4H_RMII_RXD0);

	rk29_mux_api_set(GPIO0A7_MIIMDCLK_NAME, GPIO0L_MII_MDCLK);
	rk29_mux_api_set(GPIO0A6_MIIMD_NAME, GPIO0L_MII_MD);
	#endif
	#ifdef CONFIG_RK29_PWM_REGULATOR
	rk29_mux_api_set(REGULATOR_PWM_MUX_NAME,REGULATOR_PWM_MUX_MODE);
	#endif
}

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_UART1_RK29
	&rk29_device_uart1,
#endif
#ifdef CONFIG_UART0_RK29
	&rk29_device_uart0,
#endif
#ifdef CONFIG_UART2_RK29
	&rk29_device_uart2,
#endif
#ifdef CONFIG_UART3_RK29
	&rk29_device_uart3,
#endif
#ifdef CONFIG_BATTERY_RK2918)
	&rk2918_device_battery,
#endif

#ifdef CONFIG_RK29_PWM_REGULATOR
	&rk29_device_pwm_regulator,
#endif
#ifdef CONFIG_SPIM0_RK29
    &rk29xx_device_spi0m,
#endif
#ifdef CONFIG_SPIM1_RK29
    &rk29xx_device_spi1m,
#endif
#ifdef CONFIG_ADC_RK29
	&rk29_device_adc,
#endif
#ifdef CONFIG_I2C0_RK29
	&rk29_device_i2c0,
#endif
#ifdef CONFIG_I2C1_RK29
	&rk29_device_i2c1,
#endif
#ifdef CONFIG_I2C2_RK29
	&rk29_device_i2c2,
#endif
#ifdef CONFIG_I2C3_RK29
	&rk29_device_i2c3,
#endif

#ifdef CONFIG_SND_RK29_SOC_I2S_2CH
        &rk29_device_iis_2ch,
#endif
#ifdef CONFIG_SND_RK29_SOC_I2S_8CH
        &rk29_device_iis_8ch,
#endif

#ifdef CONFIG_KEYS_RK29
	&rk29_device_keys,
#endif
#ifdef CONFIG_SDMMC0_RK29
	&rk29_device_sdmmc0,
#endif
#ifdef CONFIG_SDMMC1_RK29
	&rk29_device_sdmmc1,
#endif

#ifdef CONFIG_MTD_NAND_RK29XX
	&rk29xx_device_nand,
#endif

#ifdef CONFIG_WIFI_CONTROL_FUNC
        &rk29sdk_wifi_device,
#endif

#ifdef CONFIG_BT
        &rk29sdk_rfkill,
#endif

#ifdef CONFIG_MTD_NAND_RK29
	&rk29_device_nand,
#endif

#ifdef CONFIG_FB_RK29
	&rk29_device_fb,
	&rk29_device_dma_cpy,
#endif
#ifdef CONFIG_BACKLIGHT_RK29_BL
	&rk29_device_backlight,
#endif
#ifdef CONFIG_RK29_VMAC
	&rk29_device_vmac,
#endif
#ifdef CONFIG_VIVANTE
	&rk29_device_gpu,
#endif
#ifdef CONFIG_VIDEO_RK29
 	&rk29_device_camera,      /* ddl@rock-chips.com : camera support  */
 	#if (SENSOR_IIC_ADDR_0 != 0x00)
 	&rk29_soc_camera_pdrv_0,
 	#endif
 	&rk29_soc_camera_pdrv_1,
 	&android_pmem_cam_device,
#endif
  &android_pmem_skype_device,
	&android_pmem_device,
	&rk29_vpu_mem_device,
#ifdef CONFIG_USB20_OTG
	&rk29_device_usb20_otg,
#endif
#ifdef CONFIG_USB20_HOST
	&rk29_device_usb20_host,
#endif
#ifdef CONFIG_USB11_HOST
	&rk29_device_usb11_host,
#endif
#ifdef CONFIG_USB_ANDROID
	&android_usb_device,
	&usb_mass_storage_device,
#endif
#ifdef CONFIG_RK29_IPP
	&rk29_device_ipp,
#endif
#ifdef CONFIG_VIDEO_RK29XX_VOUT
	&rk29_v4l2_output_devce,
#endif
#ifdef CONFIG_AR6003
	&rk29sdk_wifi_ar6k_pm_device,
	&rk29sdk_wifi_ar6k_drv_device,
#endif
#if defined(CONFIG_GPS_POWER)
	&rk29_device_gps,
#endif
#if defined(CONFIG_GPIO_VIBRATE)
	&rk29_vib_gpio ,
#endif
};

/*****************************************************************************************
 * spi devices
 * author: cmc@rock-chips.com
 *****************************************************************************************/
static int rk29_vmac_register_set(void)
{
	//config rk29 vmac as rmii, 100MHz
	u32 value= readl(RK29_GRF_BASE + 0xbc);
	value = (value & 0xfff7ff) | (0x400);
	writel(value, RK29_GRF_BASE + 0xbc);
	return 0;
}

static int rk29_rmii_io_init(void)
{
	int err;

	//phy power gpio
	err = gpio_request(RK29_PIN6_PB0, "phy_power_en");
	if (err) {
		gpio_free(RK29_PIN6_PB0);
		printk("-------request RK29_PIN6_PB0 fail--------\n");
		return -1;
	}
	//phy power down
	gpio_direction_output(RK29_PIN6_PB0, GPIO_LOW);
	gpio_set_value(RK29_PIN6_PB0, GPIO_LOW);

	return 0;
}

static int rk29_rmii_io_deinit(void)
{
	//phy power down
	gpio_direction_output(RK29_PIN6_PB0, GPIO_LOW);
	gpio_set_value(RK29_PIN6_PB0, GPIO_LOW);
	//free
	gpio_free(RK29_PIN6_PB0);
	return 0;
}

static int rk29_rmii_power_control(int enable)
{
	if (enable) {
		//enable phy power
		gpio_direction_output(RK29_PIN6_PB0, GPIO_HIGH);
		gpio_set_value(RK29_PIN6_PB0, GPIO_HIGH);
	}
	else {
		gpio_direction_output(RK29_PIN6_PB0, GPIO_LOW);
		gpio_set_value(RK29_PIN6_PB0, GPIO_LOW);
	}
	return 0;
}

struct rk29_vmac_platform_data rk29_vmac_pdata = {
	.vmac_register_set = rk29_vmac_register_set,
	.rmii_io_init = rk29_rmii_io_init,
	.rmii_io_deinit = rk29_rmii_io_deinit,
	.rmii_power_control = rk29_rmii_power_control,
};

/*****************************************************************************************
 * spi devices
 * author: cmc@rock-chips.com
 *****************************************************************************************/
#define SPI_CHIPSELECT_NUM 2
static struct spi_cs_gpio rk29xx_spi0_cs_gpios[SPI_CHIPSELECT_NUM] = {
    {
		.name = "spi0 cs0",
		.cs_gpio = RK29_PIN2_PC1,
		.cs_iomux_name = GPIO2C1_SPI0CSN0_NAME,
		.cs_iomux_mode = GPIO2H_SPI0_CSN0,
	},
	{
		.name = "spi0 cs1",
		.cs_gpio = RK29_PIN1_PA4,
		.cs_iomux_name = GPIO1A4_EMMCWRITEPRT_SPI0CS1_NAME,//if no iomux,set it NULL
		.cs_iomux_mode = GPIO1L_SPI0_CSN1,
	}
};

static struct spi_cs_gpio rk29xx_spi1_cs_gpios[SPI_CHIPSELECT_NUM] = {
    {
		.name = "spi1 cs0",
		.cs_gpio = RK29_PIN2_PC5,
		.cs_iomux_name = GPIO2C5_SPI1CSN0_NAME,
		.cs_iomux_mode = GPIO2H_SPI1_CSN0,
	},
	{
		.name = "spi1 cs1",
		.cs_gpio = RK29_PIN1_PA3,
		.cs_iomux_name = GPIO1A3_EMMCDETECTN_SPI1CS1_NAME,//if no iomux,set it NULL
		.cs_iomux_mode = GPIO1L_SPI1_CSN1,
	}
};

static int spi_io_init(struct spi_cs_gpio *cs_gpios, int cs_num)
{
#if 1
	int i;
	if (cs_gpios) {
		for (i=0; i<cs_num; i++) {
			rk29_mux_api_set(cs_gpios[i].cs_iomux_name, cs_gpios[i].cs_iomux_mode);
		}
	}
#endif
	return 0;
}

static int spi_io_deinit(struct spi_cs_gpio *cs_gpios, int cs_num)
{
	return 0;
}

static int spi_io_fix_leakage_bug(void)
{
#if 0
	gpio_direction_output(RK29_PIN2_PC1, GPIO_LOW);
#endif
	return 0;
}

static int spi_io_resume_leakage_bug(void)
{
#if 0
	gpio_direction_output(RK29_PIN2_PC1, GPIO_HIGH);
#endif
	return 0;
}

struct rk29xx_spi_platform_data rk29xx_spi0_platdata = {
	.num_chipselect = SPI_CHIPSELECT_NUM,
	.chipselect_gpios = rk29xx_spi0_cs_gpios,
	.io_init = spi_io_init,
	.io_deinit = spi_io_deinit,
	.io_fix_leakage_bug = spi_io_fix_leakage_bug,
	.io_resume_leakage_bug = spi_io_resume_leakage_bug,
};

struct rk29xx_spi_platform_data rk29xx_spi1_platdata = {
	.num_chipselect = SPI_CHIPSELECT_NUM,
	.chipselect_gpios = rk29xx_spi1_cs_gpios,
	.io_init = spi_io_init,
	.io_deinit = spi_io_deinit,
	.io_fix_leakage_bug = spi_io_fix_leakage_bug,
	.io_resume_leakage_bug = spi_io_resume_leakage_bug,
};

/*****************************************************************************************
 * xpt2046 touch panel
 * author: cmc@rock-chips.com
 *****************************************************************************************/

#if defined (CONFIG_TOUCHSCREEN_800X600)
#define X_MAX	800
#define Y_MAX	600
#elif defined (CONFIG_TOUCHSCREEN_800X480)
#define X_MAX	800
#define Y_MAX	480
#endif


#if defined(CONFIG_TOUCHSCREEN_XPT2046_320X480_SPI)
static struct xpt2046_platform_data xpt2046_info = {
	.model			= 2046,
	.keep_vref_on 	= 1,
	.swap_xy		= 0,
	.x_min			= 0,
	.x_max			= 320,
	.y_min			= 0,
	.y_max			= 480,
	.debounce_max		= 7,
	.debounce_rep		= DEBOUNCE_REPTIME,
	.debounce_tol		= 20,
	.gpio_pendown		= GPIO_TOUCH_INT,
	.penirq_recheck_delay_usecs = 1,
};
#elif defined(CONFIG_TOUCHSCREEN_XPT2046_320X480_CBN_SPI)
static struct xpt2046_platform_data xpt2046_info = {
	.model			= 2046,
	.keep_vref_on 	= 1,
	.swap_xy		= 0,
	.x_min			= 0,
	.x_max			= 320,
	.y_min			= 0,
	.y_max			= 480,
	.debounce_max		= 7,
	.debounce_rep		= DEBOUNCE_REPTIME,
	.debounce_tol		= 20,
	.gpio_pendown		= GPIO_TOUCH_INT,
	.penirq_recheck_delay_usecs = 1,
};
#elif defined(CONFIG_TOUCHSCREEN_XPT2046_SPI)
static struct xpt2046_platform_data xpt2046_info = {
	.model			= 2046,
	.keep_vref_on 	= 1,
	.swap_xy		= 1,
	.x_min			= 0,
	.x_max			= X_MAX,
	.y_min			= 0,
	.y_max			= Y_MAX,
	.debounce_max		= 7,
	.debounce_rep		= DEBOUNCE_REPTIME,
	.debounce_tol		= 20,
	.gpio_pendown		= GPIO_TOUCH_INT,

	.penirq_recheck_delay_usecs = 1,
};
#elif defined(CONFIG_TOUCHSCREEN_XPT2046_CBN_SPI)
static struct xpt2046_platform_data xpt2046_info = {
	.model			= 2046,
	.keep_vref_on 	= 1,
	.swap_xy		= 1,
	.x_min			= 0,
	.x_max			= X_MAX,
	.y_min			= 0,
	.y_max			= Y_MAX, 
	.debounce_max		= 7,
	.debounce_rep		= DEBOUNCE_REPTIME,
	.debounce_tol		= 20,
	.gpio_pendown		= GPIO_TOUCH_INT,

	.penirq_recheck_delay_usecs = 1,
};
#endif

static struct spi_board_info board_spi_devices[] = {
#if defined(CONFIG_TOUCHSCREEN_XPT2046_320X480_SPI) || defined(CONFIG_TOUCHSCREEN_XPT2046_320X480_CBN_SPI)\
    ||defined(CONFIG_TOUCHSCREEN_XPT2046_SPI) || defined(CONFIG_TOUCHSCREEN_XPT2046_CBN_SPI)
	{
		.modalias	= "xpt2046_ts",
		.chip_select	= 0,
		.max_speed_hz	= 125 * 1000 * 26,/* (max sample rate @ 3V) * (cmd + data + overhead) */
		.bus_num	= 0,
		.irq = GPIO_TOUCH_INT,
		.platform_data = &xpt2046_info,
	},
#endif
};


static void __init rk29_gic_init_irq(void)
{
	gic_dist_init(0, (void __iomem *)RK29_GICPERI_BASE, 32);
	gic_cpu_init(0, (void __iomem *)RK29_GICCPU_BASE);
}

static void __init machine_rk29_init_irq(void)
{
	rk29_gic_init_irq();
	rk29_gpio_init();
}

static void rk29_pm_power_off(void)
{
	printk(KERN_ERR "rk29_pm_power_off start...\n");
	gpio_direction_output(GPIO_POWER_ON, GPIO_LOW);
	while (1);
}

static int rk29_bcm4329_power_on(void)
{
#if defined(WIFI_EXT_POWER)
	int err;

#if defined(WIFI_EXT_POWER_MUX_NAME)
    rk29_mux_api_set(WIFI_EXT_POWER_MUX_NAME, WIFI_EXT_POWER_GPIO_MODE);
#endif
	//phy power gpio
	err = gpio_request(WIFI_EXT_POWER, "b23_pwr_on");
	if (err) {
		gpio_free(WIFI_EXT_POWER);
		printk("-------request WIFI_EXT_POWER fail--------\n");
		return -1;
	}
	//phy power down
	gpio_direction_output(WIFI_EXT_POWER, GPIO_HIGH);
	gpio_set_value(WIFI_EXT_POWER, GPIO_HIGH);

#endif
#if defined(GPIO_WIFI_POWER_MUX_NAME)
    rk29_mux_api_set(GPIO_WIFI_POWER_MUX_NAME, GPIO_WIFI_POWER_GPIO_MODE);
#endif
	
	return 0;
}

static void __init machine_rk29_board_init(void)
{
	rk29_board_iomux_init();
	
	if (gpio_request(GPIO_BL_PWM, NULL)) {
		printk("func %s, line %d: request gpio fail\n", __FUNCTION__, __LINE__);
	}

    rk29_mux_api_set(PWM_MUX_NAME, GPIO1L_GPIO1B5);
	gpio_direction_output(GPIO_BL_PWM, GPIO_HIGH);
	gpio_set_value(GPIO_BL_PWM, GPIO_HIGH);
	mdelay(10);

	rk29_bcm4329_power_on();

#if 0
	gpio_request(GPIO_POWER_ON,"poweronpin");
	gpio_set_value(GPIO_POWER_ON, GPIO_HIGH);
	gpio_direction_output(GPIO_POWER_ON, GPIO_HIGH);
	pm_power_off = rk29_pm_power_off;
#else
	board_power_init();
#endif


	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_I2C0_RK29
	i2c_register_board_info(default_i2c0_data.bus_num, board_i2c0_devices,
			ARRAY_SIZE(board_i2c0_devices));
#endif
#ifdef CONFIG_I2C1_RK29
	i2c_register_board_info(default_i2c1_data.bus_num, board_i2c1_devices,
			ARRAY_SIZE(board_i2c1_devices));
#endif
#ifdef CONFIG_I2C2_RK29
	i2c_register_board_info(default_i2c2_data.bus_num, board_i2c2_devices,
			ARRAY_SIZE(board_i2c2_devices));
#endif
#ifdef CONFIG_I2C3_RK29
	i2c_register_board_info(default_i2c3_data.bus_num, board_i2c3_devices,
			ARRAY_SIZE(board_i2c3_devices));
#endif

	spi_register_board_info(board_spi_devices, ARRAY_SIZE(board_spi_devices));
        
#ifdef CONFIG_WIFI_CONTROL_FUNC
	rk29sdk_wifi_bt_gpio_control_init();
	rk29sdk_init_wifi_mem();
#endif
	board_usb_detect_init(GPIO_USB_INT);
}

static void __init machine_rk29_fixup(struct machine_desc *desc, struct tag *tags,
					char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = RK29_SDRAM_PHYS;
	mi->bank[0].node = PHYS_TO_NID(RK29_SDRAM_PHYS);
	mi->bank[0].size = LINUX_SIZE;
}

static void __init machine_rk29_mapio(void)
{
	rk29_map_common_io();
	rk29_setup_early_printk();
	rk29_sram_init();
	rk29_clock_init(periph_pll_288mhz);
	rk29_iomux_init();
}

MACHINE_START(RK29, "RK29board")
	/* UART for LL DEBUG */
	.phys_io	= RK29_UART1_PHYS,
	.io_pg_offst	= ((RK29_UART1_BASE) >> 18) & 0xfffc,
	.boot_params	= RK29_SDRAM_PHYS + 0x88000,
	.fixup		= machine_rk29_fixup,
	.map_io		= machine_rk29_mapio,
	.init_irq	= machine_rk29_init_irq,
	.init_machine	= machine_rk29_board_init,
	.timer		= &rk29_timer,
MACHINE_END
