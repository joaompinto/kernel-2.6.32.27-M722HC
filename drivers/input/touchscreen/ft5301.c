#include <linux/module.h>
#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/async.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/irq.h>
#include <mach/board.h>

//#define FT5X0X_DEBUG
#ifdef FT5X0X_DEBUG
#define DBG(fmt, args...)	printk("*** " fmt, ## args)
#else
#define DBG(fmt, args...)	do{}while(0)
#endif

#define EV_MENU					KEY_F1

#define FT5X0X_SPEED 200000
#define MAX_POINT  5

#define SCREEN_MAX_X 800
#define SCREEN_MAX_Y 480

#define PRESS_MAX 200
#define MULTI_TOUCH 1

#define VID_OF		0x51	//OuFei
#define VID_MD		0x53	//MuDong
#define VID_BYD		0x59
#define VID_YJ		0x80	
#define VID_YM		0xc0
static unsigned char g_vid;

#ifdef CONFIG_HAS_EARLYSUSPEND
static struct early_suspend ft5x0x_early_suspend;
#endif


static int  ft5x0x_probe(struct i2c_client *client, const struct i2c_device_id *id);

struct ts_event {
	u16	x1;
	u16	y1;
	u16	x2;
	u16	y2;
	u16	x3;
	u16	y3;
	u16	x4;
	u16	y4;
	u16	x5;
	u16	y5;
	u16	pressure;
	u16	w;
    u8  touch_point;
};

struct ft5x0x_data
{
	struct i2c_client *client;
	struct input_dev	*input_dev;
	spinlock_t 	lock;
	int			irq;
	int		reset_gpio;
	int		touch_en_gpio;
	int		last_points;
	struct ts_event		event;
	struct work_struct 	pen_event_work;
	struct workqueue_struct *ts_workqueue;
};

struct i2c_client *g_client;

int ft5x0x_rx_data(struct i2c_client *client, char *rxData, int length)
{
	int ret = 0;
	char reg = rxData[0];
	ret = i2c_master_reg8_recv(client, reg, rxData, length, FT5X0X_SPEED);
	return (ret > 0)? 0 : ret;
}

static int ft5x0x_tx_data(struct i2c_client *client, char *txData, int length)
{
	int ret = 0;
	char reg = txData[0];
	ret = i2c_master_reg8_send(client, reg, &txData[1], length-1, FT5X0X_SPEED);
	return (ret > 0)? 0 : ret;
}

char ft5x0x_read_reg(struct i2c_client *client, int addr)
{
	char tmp;
	int ret = 0;

	tmp = addr;
	ret = ft5x0x_rx_data(client, &tmp, 1);
	if (ret < 0) {
		return ret;
	}
	return tmp;
}

int ft5x0x_write_reg(struct i2c_client *client,int addr,int value)
{
	char buffer[3];
	int ret = 0;

	buffer[0] = addr;
	buffer[1] = value;
	ret = ft5x0x_tx_data(client, &buffer[0], 2);
	return ret;
}

static void ft5x0x_power_en(struct ft5x0x_data *ft5x0x, int on)
{
#if defined (GPIO_TOUCH_EN)
	if (on) {
		gpio_direction_output(ft5x0x->touch_en_gpio, 1);
		gpio_set_value(ft5x0x->touch_en_gpio, 1);
		mdelay(10);
	} else {
		gpio_direction_output(ft5x0x->touch_en_gpio, 0);
		gpio_set_value(ft5x0x->touch_en_gpio, 0);
		mdelay(10);
	}
#endif
}

static void ft5x0x_chip_reset(struct ft5x0x_data *ft5x0x)
{
    gpio_direction_output(ft5x0x->reset_gpio, 0);
    gpio_set_value(ft5x0x->reset_gpio, 1);
	mdelay(10);
    gpio_set_value(ft5x0x->reset_gpio, 0);
	mdelay(10);
    gpio_set_value(ft5x0x->reset_gpio, 1);
}

static int i2c_write_interface(unsigned char* pbt_buf, int dw_lenth)
{
    int ret;
    ret = i2c_master_send(g_client, pbt_buf, dw_lenth);
    if (ret <= 0) {
        printk("i2c_write_interface error\n");
        return -1;
    }

    return 0;
}

static int ft_cmd_write(unsigned char btcmd, unsigned char btPara1, unsigned char btPara2,
		unsigned char btPara3, int num)
{
    unsigned char write_cmd[4] = {0};

    write_cmd[0] = btcmd;
    write_cmd[1] = btPara1;
    write_cmd[2] = btPara2;
    write_cmd[3] = btPara3;
    return i2c_write_interface(&write_cmd, num);
}

static int ft5x0x_chip_init(struct i2c_client * client)
{
	int ret = 0;
	int w_value;
	char r_value;
	int reg;
	int i = 0, flag = 1;
	struct ft5x0x_data *ft5x0x = i2c_get_clientdata(client);

	ft5x0x_chip_reset(ft5x0x);
	ft5x0x_power_en(ft5x0x, 0);
    gpio_direction_output(ft5x0x->reset_gpio, 1);
    gpio_set_value(ft5x0x->reset_gpio, 1);
	mdelay(10);
	ft5x0x_power_en(ft5x0x, 1);
    ft_cmd_write(0x07,0x00,0x00,0x00,1);
	mdelay(100);

	r_value = ft5x0x_read_reg(client, 0xA8);//read touchpad ID for adjust touchkey place
	if (ret < 0) {
		printk(KERN_ERR "ft5x0x i2c rxdata failed\n");
		//goto out;
	}
	printk("ft5406 g_vid = 0x%X\n", r_value);
	g_vid = r_value;

	return ret;
}

static void key_led_ctrl(int on)
{
	#ifdef TOUCH_KEY_LED
		gpio_set_value(TOUCH_KEY_LED, on);
	#endif
}

static int g_screen_key=0;
#ifdef TOUCHKEY_ON_SCREEN

static unsigned char initkey_code[] =
{
    KEY_BACK, KEY_F1, KEY_HOME, KEY_SEARCH
};

typedef struct {
	int x;
	int y;
	int keycode;
} rect;

static int get_screen_key(int x, int y)
{
	const int span = 10;
	int idx;

#if defined(CONFIG_MACH_M803) || defined(CONFIG_MACH_M900) || defined(CONFIG_MACH_M900HW)
	if (g_vid == VID_YM) {
		rect rt[] = {
			 {834,	0, 	KEY_SEARCH},  		/* search */ 
			 {834,	54,	KEY_HOME},        	/* home */ 
			 {834,	90,	KEY_F1},      	  	/* menu */ 
			 {834,	125,KEY_BACK},	/* back */ 
			 {0,0,0},
		};

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}

	} else if (g_vid == VID_OF || g_vid == VID_YJ) {
		rect rt[] = {
			 {850,	128, 	KEY_BACK},  /* back */ 
			 {850,	95,	KEY_F1},        /* home */ 
			 {850,	55,	KEY_HOME},		/* menu */ 
			 {850,	5,	KEY_SEARCH},    /* search */ 
			 {0,0,0},
		};

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}

	} else if (g_vid == VID_MD || g_vid == 0x01) {
		rect rt[] = {
			{850,	128, 	KEY_SEARCH},  /* search */ 
			{850,	95,	KEY_HOME},        /* home */ 
			{850,	55,	KEY_F1},      	  /* menu */ 
			{850,	5,	KEY_BACK	},    /* back */ 
			{0,0,0}, 
		}; 

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}
	}

#elif defined(CONFIG_MACH_M803HD) || defined(CONFIG_MACH_M900HD) || defined(CONFIG_MACH_M900HDW) || defined(CONFIG_MACH_M807) || defined(CONFIG_MACH_M907) /* other product, not for m803 */
	if (g_vid == VID_YM) {
		rect rt[] = {
			 {1085,	5,	  KEY_SEARCH},    /* search */ 
			 {1085,	70,	  KEY_HOME},      /* home */ 
			 {1085,	120,  KEY_F1},        /* menu */ 
			 {1085,	165,  KEY_BACK},	  /* back */ 
			 {0,0,0},
		};

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}

	} else if (g_vid == VID_OF || g_vid == VID_YJ) {
		rect rt[] = {
			 {1085,	165, KEY_BACK},    /* back */ 
			 {1085,	120, KEY_F1},      /* home */ 
			 {1085,	70,  KEY_HOME},	   /* menu */ 
			 {1085,	5,   KEY_SEARCH},  /* search */ 
			 {0,0,0},
		};

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}

	} else if (g_vid == VID_MD || g_vid == 0x01) {
		rect rt[] = {
			{1085,	165,  KEY_SEARCH},  /* search */ 
			{1085,	120,  KEY_HOME},    /* home */ 
			{1085,	70,   KEY_F1},      /* menu */ 
			{1085,	5,    KEY_BACK	},  /* back */ 
			{0,0,0}, 
		}; 

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}
	}

#elif defined(CONFIG_MACH_M726HN) || defined(CONFIG_MACH_M732)
	if (g_vid == VID_OF || g_vid == VID_YJ) {
		rect rt[] = {
			 {840,	5,		KEY_BACK},  	/* back */ 
			 {840,	40,		KEY_F1},        /* home */ 
			 {840,	75,		KEY_HOME},		/* menu */ 
			 {840,	105,	KEY_SEARCH},    /* search */ 
			 {0,0,0},
		};

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}
	} else if (g_vid == VID_MD) {
		rect rt[] = {
			{845,	5, 		KEY_SEARCH},  	/* search */ 
			{845,	40,		KEY_HOME},      /* home */ 
			{845,	75,		KEY_F1},      	/* menu */ 
			{845,	105,	KEY_BACK},    	/* back */ 
			{0,0,0}, 
		}; 

		for(idx=0; rt[idx].keycode; idx++)
		{
			if(x >= rt[idx].x-span && x<= rt[idx].x+span)
				if(y >= rt[idx].y-span && y<= rt[idx].y+span)
					return rt[idx].keycode;
		}
	}
#else
	const rect rt[]=
#if defined (TOUCH_KEY_MAP)
	TOUCH_KEY_MAP;
#else
	{
		{845,	5, 	KEY_SEARCH},          /* search */
		{845,	40,	KEY_HOME},            /* home */
		{845,	75,	KEY_F1},              /* menu */
		{845,	104,	KEY_BACK	},    /* back */
		{0,0,0}
	};
#endif
	for(idx=0; rt[idx].keycode; idx++)
	{
		if(x >= rt[idx].x-span && x<= rt[idx].x+span)
			if(y >= rt[idx].y-span && y<= rt[idx].y+span)
				return rt[idx].keycode;
	}
#endif

	DBG("***x=%d, y=%d\n", x, y);
	return 0;
}


struct input_dev *tp_key_input;
static int report_screen_key(int down_up)
{
	struct input_dev * keydev=(struct input_dev *)tp_key_input;

	key_led_ctrl(down_up);
	input_report_key(keydev, g_screen_key, down_up);
	input_sync(keydev);
	if(!down_up) {
		g_screen_key=0;
	}
	return 0;
}
#endif


static int ft5x0x_process_points(struct ft5x0x_data *data)
{
	struct ts_event *event = &data->event;
	struct i2c_client *client = data->client;
	u8 start_reg=0x0;
	u8 buf[32] = {0};
	int ret = -1;
	int status = 0;
	int last_points;
	int id;
	int back_press = 0, search_press=0, menu_press=0, home_press=0;
	//static int key_point_down=0;
	//static int report_key_status;

	start_reg = 0;
	buf[0] = start_reg;

#if MULTI_TOUCH
	if (MAX_POINT == 5) {
		ret = ft5x0x_rx_data(client, buf, 31);
	} else {
		ret = ft5x0x_rx_data(client, buf, 13);
	}
#else
	ret = ft5x0x_rx_data(client, buf, 7);
#endif

    if (ret < 0) {
		printk("%s read_data i2c_rxdata failed: %d\n", __func__, ret);
		return ret;
	}

	memset(event, 0, sizeof(struct ts_event));

#if MULTI_TOUCH
	if (MAX_POINT == 5) {
		event->touch_point = buf[2] & 0x07;
	} else {
		event->touch_point = buf[2] & 0x03;
	}

	last_points = event->touch_point;
	if (data->last_points > event->touch_point) {
		event->touch_point = data->last_points;
	}
	data->last_points = last_points;

#else
	event->touch_point = buf[2] & 0x01;
#endif
#if MULTI_TOUCH
	if (event->touch_point == 0) {
		input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, 0);
		input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, 0);
		input_mt_sync(data->input_dev);
		input_sync(data->input_dev);

		DBG("release all points!!!!!!\n");
		return 0;
	}
#else
	if (event->touch_point == 0) {
		event->pressure = 0;
		event->w = 0;
	} else {
		event->pressure = 200;
		event->w = 50;
	}
#endif


#if MULTI_TOUCH
    switch (event->touch_point) {
		if (MAX_POINT == 5)	{
			case 5:
				event->x5 = (s16)(buf[0x1b] & 0x0F)<<8 | (s16)buf[0x1c];
				event->y5 = (s16)(buf[0x1d] & 0x0F)<<8 | (s16)buf[0x1e];
				status = (s16)((buf[0x1b] & 0xc0) >> 6);
				id = (buf[0x1d] & 0xf0) >> 4;
				if (status == 1) {
					event->pressure = 0;
					event->w = 0;
				} else {
					event->pressure = 200;
					event->w = 50;
				}
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id);			
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, event->w);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x5);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y5);
				input_mt_sync(data->input_dev);
				DBG("TOUCH_NO=%d: id=%d,(%d,%d), pressure=%d, w=%d\n",5,id,event->x5, event->y5, event->pressure, event->w);
			case 4:
				event->x4 = (s16)(buf[0x15] & 0x0F)<<8 | (s16)buf[0x16];
				event->y4 = (s16)(buf[0x17] & 0x0F)<<8 | (s16)buf[0x18];
				status = (s16)((buf[0x15] & 0xc0) >> 6);
				id = (buf[0x17] & 0xf0) >> 4;
				if (status == 1) {
					event->pressure = 0;
					event->w = 0;
				} else {
					event->pressure = 200;
					event->w = 50;
				}
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id);			
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, event->w);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x4);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y4);
				input_mt_sync(data->input_dev);
				DBG("TOUCH_NO=%d: id=%d,(%d,%d), pressure=%d, w=%d\n",4,id,event->x4, event->y4, event->pressure, event->w);
			case 3:
				event->x3 = (s16)(buf[0x0f] & 0x0F)<<8 | (s16)buf[0x10];
				event->y3 = (s16)(buf[0x11] & 0x0F)<<8 | (s16)buf[0x12];
				status = (s16)((buf[0x0f] & 0xc0) >> 6);
				id = (buf[0x11] & 0xf0) >> 4;
				if (status == 1) {
					event->pressure = 0;
					event->w = 0;
				} else {
					event->pressure = 200;
					event->w = 50;
				}
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id);			
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, event->w);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x3);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y3);
				input_mt_sync(data->input_dev);
				DBG("TOUCH_NO=%d: id=%d,(%d,%d), pressure=%d, w=%d\n",3,id,event->x3, event->y3, event->pressure, event->w);
		}
		case 2:
			event->x2 = (s16)(buf[0x09] & 0x0F)<<8 | (s16)buf[0x0A];
			event->y2 = (s16)(buf[0x0B] & 0x0F)<<8 | (s16)buf[0x0C];
			status = (s16)((buf[0x09] & 0xc0) >> 6);
			id = (buf[0x0B] & 0xf0) >> 4;
			if (status == 1) {
				event->pressure = 0;
				event->w = 0;
			} else {
				event->pressure = 200;
				event->w = 50;
			}
			input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id); 
			input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
			input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, event->w);
			input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x2);
			input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y2);
			input_mt_sync(data->input_dev);
			DBG("TOUCH_NO=%d;id=%d,(%d,%d), pressure=%d, w=%d\n",2,id,event->x2, event->y2, event->pressure, event->w);
		case 1:
			event->x1 = (s16)(buf[0x03] & 0x0F)<<8 | (s16)buf[0x04];
			event->y1 = (s16)(buf[0x05] & 0x0F)<<8 | (s16)buf[0x06];
			status = (s16)((buf[0x03] & 0xc0) >> 6);
			id = (buf[0x05] & 0xf0) >> 4;
			if (status == 1) {
				event->pressure = 0;
				event->w = 0;
			} else {
				event->pressure = 200;
				event->w = 50;
			}
			if (event->x1 < 810) {
				input_report_abs(data->input_dev, ABS_MT_TRACKING_ID, id); 
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev, ABS_MT_WIDTH_MAJOR, event->w);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->x1);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->y1);
				input_mt_sync(data->input_dev);			
				DBG("TOUCH_NO=%d: id=%d,(%d,%d), pressure=%d, w=%d\n",1,id,event->x1, event->y1, event->pressure, event->w);
			}
			
#ifdef TOUCHKEY_ON_SCREEN
			if ( event->x1 >= 810 && event->w ) {
				DBG("TOUCH_NO=%d: id=%d,(%d,%d), pressure=%d, w=%d\n",1,id,event->x1, event->y1, event->pressure, event->w);
				if (g_screen_key = get_screen_key(event->x1, event->y1)) {
					DBG("touch key = %x down\n", g_screen_key);
					report_screen_key(1);
					return 0;
				}
			}
			if(g_screen_key) {
				DBG("touch key up!\n");
				report_screen_key(0);
				return 0;
			}
#endif
			break;
		default:
			break;
	}
#else
    if (event->touch_point == 1) {
    	event->x1 = (s16)(buf[0x03] & 0x0F)<<8 | (s16)buf[0x04];
		event->y1 = (s16)(buf[0x05] & 0x0F)<<8 | (s16)buf[0x06];
    }
	input_report_abs(data->input_dev, ABS_PRESSURE, event->pressure);
	input_report_key(data->input_dev, BTN_TOUCH, (event->pressure!=0?1:0));
	if (event->touch_point == 1) {
		input_report_abs(data->input_dev, ABS_X, event->x1);
		input_report_abs(data->input_dev, ABS_Y, event->y1);
		DBG("report x = %d, y=%d, pressure = %d\n", event->x1, event->y1, event->pressure);
	} else {
		DBG("report pressure = %d\n",  event->pressure);
	}

#endif
	input_sync(data->input_dev);
	
#if defined(CONFIG_KEYS_RTM913HC)
	if(event->w) {
		on_off_touchkey_led(NULL, 1);
	} else {
		on_off_touchkey_led(NULL, 0);
	}
#endif

	return 0;
}


static void  ft5x0x_delaywork_func(struct work_struct *work)
{
	struct ft5x0x_data *ft5x0x = container_of(work, struct ft5x0x_data, pen_event_work);
	struct i2c_client *client = ft5x0x->client;

	ft5x0x_process_points(ft5x0x);
	enable_irq(client->irq);		
}

static irqreturn_t ft5x0x_interrupt(int irq, void *handle)
{
	struct ft5x0x_data *ft5x0x_ts = handle;

	//printk("Enter:%s %d\n",__FUNCTION__,__LINE__);
	disable_irq_nosync(irq);
	//if (!work_pending(&ft5x0x_ts->pen_event_work)) {
		queue_work(ft5x0x_ts->ts_workqueue, &ft5x0x_ts->pen_event_work);
	//}
	return IRQ_HANDLED;
}



static int ft5x0x_remove(struct i2c_client *client)
{
	struct ft5x0x_data *ft5x0x = i2c_get_clientdata(client);
	
    input_unregister_device(ft5x0x->input_dev);
    input_free_device(ft5x0x->input_dev);
    free_irq(client->irq, ft5x0x);
    kfree(ft5x0x); 
#ifdef CONFIG_HAS_EARLYSUSPEND
    unregister_early_suspend(&ft5x0x_early_suspend);
#endif      
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ft5x0x_suspend(struct early_suspend *h)
{
	int err;
	int w_value;
	int reg;
	struct ft5x0x_data *ft5x0x = i2c_get_clientdata(g_client);
	
#if defined(CONFIG_KEYS_RTM913HC)
	on_off_touchkey_led(NULL, 0);
#endif

	printk("==ft5x0x_ts_suspend=\n");
	key_led_ctrl(0);
#if defined(TP_USE_WAKEUP_PIN)
	w_value = 3;
	reg = 0xa5;
	err = ft5x0x_write_reg(g_client, reg, w_value);   /* enter sleep mode */
	if (err < 0) {
		printk("ft5x0x enter sleep mode failed\n");
	}
#endif
	disable_irq(g_client->irq);		
	ft5x0x_power_en(ft5x0x, 0);
}

static void ft5x0x_resume(struct early_suspend *h)
{
	struct ft5x0x_data *ft5x0x = i2c_get_clientdata(g_client);
	
#if defined(CONFIG_KEYS_RTM913HC)
	on_off_touchkey_led(NULL, 0);
#endif

	key_led_ctrl(0);
#if defined(TP_USE_WAKEUP_PIN)
	ft5x0x_chip_reset(ft5x0x);
#endif
	ft5x0x_power_en(ft5x0x, 1);
	enable_irq(g_client->irq);		
}
#else
static int ft5x0x_suspend(struct i2c_client *client, pm_message_t mesg)
{
	return 0;
}
static int ft5x0x_resume(struct i2c_client *client)
{
	return 0;
}
#endif

static const struct i2c_device_id ft5x0x_id[] = {
		{"ft5x0x", 0},
		{ }
};

static struct i2c_driver ft5x0x_driver = {
	.driver = {
		.name = "ft5x0x",
	    },
	.id_table 	= ft5x0x_id,
	.probe		= ft5x0x_probe,
	.remove		= __devexit_p(ft5x0x_remove),
#ifndef CONFIG_HAS_EARLYSUSPEND	
	.suspend = &ft5x0x_suspend,
	.resume = &ft5x0x_resume,
#endif	
};

static int ft5x0x_init_client(struct i2c_client *client)
{
	struct ft5x0x_data *ft5x0x;
	int ret;

	ft5x0x = i2c_get_clientdata(client);

	DBG("gpio_to_irq(%d) is %d\n",client->irq,gpio_to_irq(client->irq));
	if ( !gpio_is_valid(client->irq)) {
		DBG("+++++++++++gpio_is_invalid\n");
		return -EINVAL;
	}

	gpio_free(client->irq);
	ret = gpio_request(client->irq, "ft5x0x_int");
	if (ret) {
		DBG( "failed to request ft5x0x GPIO%d\n",gpio_to_irq(client->irq));
		return ret;
	}

    ret = gpio_direction_input(client->irq);
    if (ret) {
        DBG("failed to set ft5x0x  gpio input\n");
		return ret;
    }

	gpio_pull_updown(client->irq, GPIOPullUp);
	client->irq = gpio_to_irq(client->irq);
	ft5x0x->irq = client->irq;
	ret = request_irq(client->irq, ft5x0x_interrupt, IRQF_TRIGGER_FALLING, client->dev.driver->name, ft5x0x);
	DBG("request irq is %d,ret is  0x%x\n",client->irq,ret);
	if (ret ) {
		DBG(KERN_ERR "ft5x0x_init_client: request irq failed,ret is %d\n",ret);
        return ret;
	}
	//disable_irq(client->irq);
 
	return 0;
}


static int  ft5x0x_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct ft5x0x_data *ft5x0x_ts;
	struct ts_hw_data *pdata = client->dev.platform_data;
	int err = 0;
	int i;

	printk("%s enter\n",__FUNCTION__);



	ft5x0x_ts = kzalloc(sizeof(struct ft5x0x_data), GFP_KERNEL);
	if (!ft5x0x_ts) {
		DBG("[ft5x0x]:alloc data failed.\n");
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}
    
	g_client = client;
	ft5x0x_ts->client = client;
	ft5x0x_ts->last_points = 0;
	ft5x0x_ts->reset_gpio = pdata->reset_gpio;
	ft5x0x_ts->touch_en_gpio = pdata->touch_en_gpio;
	i2c_set_clientdata(client, ft5x0x_ts);

	gpio_free(ft5x0x_ts->reset_gpio);
	err = gpio_request(ft5x0x_ts->reset_gpio, "ft5x0x rst");
	if (err) {
		DBG( "failed to request ft5x0x reset GPIO%d\n",gpio_to_irq(client->irq));
		goto exit_alloc_gpio_rst_failed;
	}
	
#if defined (GPIO_TOUCH_EN)
	gpio_free(ft5x0x_ts->touch_en_gpio);
	err = gpio_request(ft5x0x_ts->touch_en_gpio, "ft5x0x power enable");
	if (err) {
		DBG( "failed to request ft5x0x power enable GPIO%d\n",gpio_to_irq(client->irq));
		goto exit_alloc_gpio_power_failed;
	}
#if defined (TOUCH_EN_MUX_NAME)
    rk29_mux_api_set(TOUCH_EN_MUX_NAME, TOUCH_EN_MUX_MODE_GPIO);
#endif
#endif

	err = ft5x0x_chip_init(client);
	if (err < 0) {
		printk(KERN_ERR
		       "ft5x0x_probe: ft5x0x chip init failed\n");
		goto exit_request_gpio_irq_failed;
	}
	INIT_WORK(&ft5x0x_ts->pen_event_work, ft5x0x_delaywork_func);
	ft5x0x_ts->ts_workqueue = create_singlethread_workqueue("ft5x0x_ts");
	if (!ft5x0x_ts->ts_workqueue) {
		err = -ESRCH;
		goto exit_request_gpio_irq_failed;
	}

	err = ft5x0x_init_client(client);
	if (err < 0) {
		printk(KERN_ERR
		       "ft5x0x_probe: ft5x0x_init_client failed\n");
		goto exit_request_gpio_irq_failed;
	}
		
	ft5x0x_ts->input_dev = input_allocate_device();
	if (!ft5x0x_ts->input_dev) {
		err = -ENOMEM;
		printk(KERN_ERR
		       "ft5x0x_probe: Failed to allocate input device\n");
		goto exit_input_allocate_device_failed;
	}

	ft5x0x_ts->input_dev->name = "ft5x0x-ts";
	ft5x0x_ts->input_dev->dev.parent = &client->dev;

	err = input_register_device(ft5x0x_ts->input_dev);
	if (err < 0) {
		printk(KERN_ERR
		       "ft5x0x_probe: Unable to register input device: %s\n",
		       ft5x0x_ts->input_dev->name);
		goto exit_input_register_device_failed;
	}

#ifdef TOUCHKEY_ON_SCREEN
	#ifdef TOUCH_KEY_LED
		err = gpio_request(TOUCH_KEY_LED, "key led");
		if (err < 0) {
			printk(KERN_ERR
			       "ft5x0x_probe: Unable to request gpio: %d\n",
			       TOUCH_KEY_LED);
			goto exit_input_register_device_failed;
		}
		gpio_direction_output(TOUCH_KEY_LED, GPIO_LOW);
		gpio_set_value(TOUCH_KEY_LED, GPIO_LOW);
	#endif
	
	tp_key_input = ft5x0x_ts->input_dev;
	for (i = 0; i < ARRAY_SIZE(initkey_code); i++)
		set_bit(initkey_code[i], ft5x0x_ts->input_dev->keybit);
#endif

#if MULTI_TOUCH
	set_bit(EV_SYN, ft5x0x_ts->input_dev->evbit);
	set_bit(EV_KEY, ft5x0x_ts->input_dev->evbit);
	set_bit(EV_ABS, ft5x0x_ts->input_dev->evbit);
	//set_bit(BTN_TOUCH, ft5x0x_ts->input_dev->keybit);
	//set_bit(BTN_2, ft5x0x_ts->input_dev->keybit);
	//set_bit(BTN_3, ft5x0x_ts->input_dev->keybit);
	//set_bit(BTN_4, ft5x0x_ts->input_dev->keybit);
	//set_bit(BTN_5, ft5x0x_ts->input_dev->keybit);

	//input_set_abs_params(ft5x0x_ts->input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	//input_set_abs_params(ft5x0x_ts->input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	//input_set_abs_params(ft5x0x_ts->input_dev, ABS_HAT0X, 0, SCREEN_MAX_X, 0, 0);
	//input_set_abs_params(ft5x0x_ts->input_dev, ABS_HAT0Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_POSITION_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_POSITION_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_WIDTH_MAJOR, 0, 50, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_MT_TRACKING_ID, 0, 4, 0, 0);
#else
	set_bit(EV_SYN, ft5x0x_ts->input_dev->evbit);
	set_bit(EV_KEY, ft5x0x_ts->input_dev->evbit);
	set_bit(EV_ABS, ft5x0x_ts->input_dev->evbit);
	set_bit(BTN_TOUCH, ft5x0x_ts->input_dev->keybit);

	input_set_abs_params(ft5x0x_ts->input_dev, ABS_X, 0, SCREEN_MAX_X, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_Y, 0, SCREEN_MAX_Y, 0, 0);
	input_set_abs_params(ft5x0x_ts->input_dev, ABS_PRESSURE, 0, 0, 0, 0);
#endif
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    ft5x0x_early_suspend.suspend = ft5x0x_suspend;
    ft5x0x_early_suspend.resume = ft5x0x_resume;
    ft5x0x_early_suspend.level = 0x2;
    register_early_suspend(&ft5x0x_early_suspend);
#endif
	
	return 0;

exit_input_register_device_failed:
	input_free_device(ft5x0x_ts->input_dev);
exit_input_allocate_device_failed:
    free_irq(client->irq, ft5x0x_ts);
exit_request_gpio_irq_failed:
	kfree(ft5x0x_ts);	
exit_alloc_gpio_power_failed:
#if defined (GPIO_TOUCH_EN)
	gpio_free(ft5x0x_ts->touch_en_gpio);
#endif
exit_alloc_gpio_rst_failed:
    gpio_free(ft5x0x_ts->reset_gpio);
exit_alloc_data_failed:
	printk("%s error\n",__FUNCTION__);
	return err;
}


static int __init ft5x0x_mod_init(void)
{

	printk("ft5x0x module init\n");
	return i2c_add_driver(&ft5x0x_driver);
}

static void __exit ft5x0x_mod_exit(void)
{
	i2c_del_driver(&ft5x0x_driver);
}


module_init(ft5x0x_mod_init);
module_exit(ft5x0x_mod_exit);

MODULE_DESCRIPTION("SO381010 Camera sensor driver");
MODULE_AUTHOR("lbt <kernel@rock-chips>");
MODULE_LICENSE("GPL");

