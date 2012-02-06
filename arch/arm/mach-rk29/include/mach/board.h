/* arch/arm/mach-rk29/include/mach/board.h
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
#ifndef __ASM_ARCH_RK29_BOARD_H
#define __ASM_ARCH_RK29_BOARD_H

#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include <linux/timer.h>
#include <linux/notifier.h>

/*spi*/
struct spi_cs_gpio {
	const char *name;
	unsigned int cs_gpio;
	char *cs_iomux_name;
	unsigned int cs_iomux_mode;
};

struct rk29xx_spi_platform_data {
	int (*io_init)(struct spi_cs_gpio*, int);
	int (*io_deinit)(struct spi_cs_gpio*, int);
	int (*io_fix_leakage_bug)(void);
	int (*io_resume_leakage_bug)(void);
	struct spi_cs_gpio *chipselect_gpios;	
	u16 num_chipselect;
};

/*vmac*/
struct rk29_vmac_platform_data {
	int (*vmac_register_set)(void);
	int (*rmii_io_init)(void);
	int (*rmii_io_deinit)(void);
    int (*rmii_power_control)(int enable);
};

#define INVALID_GPIO        -1

struct rk29lcd_info{
    u32 lcd_id;
    u32 txd_pin;
    u32 clk_pin;
    u32 cs_pin;
    int (*io_init)(void);
    int (*io_deinit)(void);
};

struct rk29_fb_setting_info{
    u8 data_num;
    u8 vsync_en;
    u8 den_en;
    u8 mcu_fmk_en;
    u8 disp_on_en;
    u8 standby_en;
};

struct rk29fb_info{
    u32 fb_id;
    u32 disp_on_pin;
    u8 disp_on_value;
    u32 standby_pin;
    u8 standby_value;
    u32 mcu_fmk_pin;
    struct rk29lcd_info *lcd_info;
    int (*io_init)(struct rk29_fb_setting_info *fb_setting);
    int (*io_deinit)(void);
};

struct rk29_bl_info{
    u32 pwm_id;
    u32 bl_ref;
    int (*io_init)(void);
    int (*io_deinit)(void);
	int (*pwm_suspend)(void);
	int (*pwm_resume)(void);
    struct timer_list timer;  
    struct timer_list init_timer;  
    struct notifier_block freq_transition;
};

struct wifi_platform_data {
        int (*set_power)(int val);
        int (*set_reset)(int val);
        int (*set_carddetect)(int val);
        void *(*mem_prealloc)(int section, unsigned long size);
        int (*get_mac_addr)(unsigned char *buf);
};

struct rk29_sdmmc_platform_data {
	unsigned int host_caps;
	unsigned int host_ocr_avail;
	unsigned int use_dma:1;
	char dma_name[8];
	int (*io_init)(void);
	int (*io_deinit)(void);
	int (*status)(struct device *);
	int (*register_status_notify)(void (*callback)(int card_present, void *dev_id), void *dev_id);
        int detect_irq;
		int enable_sd_wakeup;
};
struct rk29_i2c_platform_data {
	int     bus_num;        
	unsigned int    flags;     
	unsigned int    slave_addr; 
	unsigned long   scl_rate;   
#define I2C_MODE_IRQ    0
#define I2C_MODE_POLL   1
	unsigned int    mode:1;
	int (*io_init)(void);
	int (*io_deinit)(void);
};

struct bq27510_platform_data {	
	int (*init_dc_check_pin)(void);	
	unsigned int dc_check_pin;	
	unsigned int bat_num;
};

/*i2s*/
struct rk29_i2s_platform_data {
	int (*io_init)(void);
	int (*io_deinit)(void);
};

/*p1003 touch */
struct p1003_platform_data {
    u16     model;

    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*p1003_platform_sleep)(void);
    int     (*p1003_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
};
struct eeti_egalax_platform_data{
	u16     model;

    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*eeti_egalax_platform_sleep)(void);
    int     (*eeti_egalax_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
};

/*sintex touch*/
struct sintek_platform_data {
	u16 	model;

	int 	(*get_pendown_state)(void);
	int 	(*init_platform_hw)(void);
	int 	(*sintek_platform_sleep)(void);
	int 	(*sintek_platform_wakeup)(void);
	void	(*exit_platform_hw)(void);
};

/*synaptics  touch*/
struct synaptics_platform_data {
	u16 	model;
	
	int 	(*get_pendown_state)(void);
	int 	(*init_platform_hw)(void);
	int 	(*sintek_platform_sleep)(void);
	int 	(*sintek_platform_wakeup)(void);
	void	(*exit_platform_hw)(void);
};

struct mma8452_platform_data {
    u16     model;
	u16     swap_xy;
    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*mma8452_platform_sleep)(void);
    int     (*mma8452_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
};
/*ite7260 touch */
struct ite7260_platform_data {
    int     (*get_pendown_state)(void);
    int     (*init_platform_hw)(void);
    int     (*ite7260_platform_sleep)(void);
    int     (*ite7260_platform_wakeup)(void);
    void    (*exit_platform_hw)(void);
	int 	reset_gpio;
	
	int 		power;
	int 		irq;
	unsigned short		model;
};

struct akm8975_platform_data {
	char layouts[3][3];
	char project_name[64];
	int gpio_DRDY;
};

/**battery*/
struct rk2918_battery_platform_data {
        int (*io_init)(void);
        int (*io_deinit)(void);

    int dc_det_pin;
    int batt_low_pin;
        int charge_ok_pin;
    int charge_set_pin;

        int dc_det_level;
    int batt_low_level;
    int charge_ok_level;
    int charge_set_level;
};


struct adc_battery_platform_data {
	int adc_chn;

	int (*io_init)(void);
	int (*io_deinit)(void);
	
	int dc_det_gpio;
	int batt_low_gpio;
	int chg_ok_gpio;
	int charge_set_gpio;

	int dc_det_level;
    int batt_low_level;
    int chg_ok_level;
    int charge_set_level;
};

struct ekt2201_platform_data {	
	int led_key;
};

/*coasia  touch*/
struct coasia_platform_data {
	u16 	model;
    u32     power_pin;
    u32     rest_pin;
    u32     int_pin;
	
	int 	(*get_pendown_state)(void);
	int 	(*init_platform_hw)(void);
	int 	(*coasia_platform_sleep)(void);
	int 	(*coasia_platform_wakeup)(void);
	void	(*exit_platform_hw)(void);
};

struct rk29_gps_data {
	int power_io;
	int onoff_io;
	int uart_id;
	int power_flag;
	struct semaphore power_sem;
};

struct ts_hw_data {
	int reset_gpio;
	int touch_en_gpio;
};

struct a080sn03_platform_data {
	int reset_gpio;
};

struct st1564_platform_data {
	int reset_gpio;
};


struct rk29_gpio_expander_info {
	unsigned int gpio_num;
	unsigned int pin_type;//GPIO_IN or GPIO_OUT
	unsigned int pin_value;//GPIO_HIGH or GPIO_LOW
};

struct tca6424_platform_data {
	/*  the first extern gpio number in all of gpio groups */
	unsigned int gpio_base;
	unsigned int gpio_pin_num;
	/*  the first gpio irq  number in all of irq source */

	unsigned int gpio_irq_start;
	unsigned int irq_pin_num;        //number of interrupt
	unsigned int tca6424_irq_pin;     //rk29 gpio
	unsigned int expand_port_group;
	unsigned int expand_port_pinnum;
	unsigned int rk_irq_mode;
	unsigned int rk_irq_gpio_pull_up_down;
	
	/* initial polarity inversion setting */
	uint16_t	invert;
	struct rk29_gpio_expander_info  *settinginfo;
	int  settinginfolen;
	void	*context;	/* param to setup/teardown */

	int		(*setup)(struct i2c_client *client,unsigned gpio, unsigned ngpio,void *context);
	int		(*teardown)(struct i2c_client *client,unsigned gpio, unsigned ngpio,void *context);
	char	**names;
	void    (*reseti2cpin)(void);
};

void __init rk29_setup_early_printk(void);
void __init rk29_map_common_io(void);
void __init rk29_clock_init(unsigned long ppll_rate);
void __init board_power_init(void);

#define BOOT_MODE_NORMAL		0
#define BOOT_MODE_FACTORY2		1
#define BOOT_MODE_RECOVERY		2
#define BOOT_MODE_CHARGE		3
#define BOOT_MODE_POWER_TEST		4
#define BOOT_MODE_OFFMODE_CHARGING	5
int board_boot_mode(void);
enum periph_pll{
	periph_pll_96mhz=96000000,
	periph_pll_144mhz=144000000,
	periph_pll_288mhz=288000000,
	periph_pll_300mhz=300000000,
};

/* for USB detection */
#ifdef CONFIG_USB_GADGET
int board_usb_detect_init(unsigned gpio);
#else
static int inline board_usb_detect_init(unsigned gpio) { return 0; }
#endif

/* for wakeup Android */
void rk28_send_wakeup_key(void);

/* for turn on_off touchkey led */
void on_off_touchkey_led(struct i2c_client *client, int on_off);

#endif
