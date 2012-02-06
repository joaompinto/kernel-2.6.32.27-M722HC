#include <linux/input.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/fcntl.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <asm/types.h>
#include <mach/gpio.h>
#include <mach/iomux.h>
#include <linux/platform_device.h>
#include "rk29_gps.h"
#if 0
#define DBG(x...)	printk(KERN_INFO x)
#else
#define DBG(x...)
#endif

#define ENABLE  1
#define DISABLE 0

static struct rk29_gps_data *pgps;

static int rk29_gps_uart_to_gpio(int uart_id)
{
	if(uart_id == 3) {
		rk29_mux_api_set(GPIO2B3_UART3SOUT_NAME, GPIO2L_GPIO2B3); 			
		rk29_mux_api_set(GPIO2B2_UART3SIN_NAME, GPIO2L_GPIO2B2); 		

		gpio_request(RK29_PIN2_PB3, NULL);
		gpio_request(RK29_PIN2_PB2, NULL);

		gpio_direction_output(RK29_PIN2_PB3, GPIO_LOW);
		gpio_direction_output(RK29_PIN2_PB2, GPIO_LOW);
	}
	else if(uart_id == 2) {
		rk29_mux_api_set(GPIO2B1_UART2SOUT_NAME, GPIO2L_GPIO2B1); 			
		rk29_mux_api_set(GPIO2B0_UART2SIN_NAME, GPIO2L_GPIO2B0); 		

		gpio_request(RK29_PIN2_PB1, NULL);
		gpio_request(RK29_PIN2_PB0, NULL);

		gpio_direction_output(RK29_PIN2_PB1, GPIO_LOW);
		gpio_direction_output(RK29_PIN2_PB0, GPIO_LOW);
	}
	else if(uart_id == 1) {
		rk29_mux_api_set(GPIO2A5_UART1SOUT_NAME, GPIO2L_GPIO2A5); 			
		rk29_mux_api_set(GPIO2A4_UART1SIN_NAME, GPIO2L_GPIO2A4); 		

		gpio_request(RK29_PIN2_PA5, NULL);
		gpio_request(RK29_PIN2_PA4, NULL);

		gpio_direction_output(RK29_PIN2_PA5, GPIO_LOW);
		gpio_direction_output(RK29_PIN2_PA4, GPIO_LOW);
	}
	else {
		//to do
	}

	return 0;
}

static int rk29_gps_gpio_to_uart(int uart_id)
{
	if(uart_id == 3) {
		rk29_mux_api_set(GPIO2B3_UART3SOUT_NAME, GPIO2L_UART3_SOUT);
		rk29_mux_api_set(GPIO2B2_UART3SIN_NAME, GPIO2L_UART3_SIN); 

		gpio_request(RK29_PIN2_PB3, NULL);
		gpio_request(RK29_PIN2_PB2, NULL);

		gpio_direction_output(RK29_PIN2_PB3, GPIO_HIGH);
		gpio_direction_output(RK29_PIN2_PB2, GPIO_HIGH);
	}
	else if(uart_id == 2) {
		rk29_mux_api_set(GPIO2B1_UART2SOUT_NAME, GPIO2L_UART2_SOUT); 			
		rk29_mux_api_set(GPIO2B0_UART2SIN_NAME, GPIO2L_UART2_SIN); 		

		gpio_request(RK29_PIN2_PB1, NULL);
		gpio_request(RK29_PIN2_PB0, NULL);

		gpio_direction_output(RK29_PIN2_PB1, GPIO_HIGH);
		gpio_direction_output(RK29_PIN2_PB0, GPIO_HIGH);
	}
	else if(uart_id == 1) {
		rk29_mux_api_set(GPIO2A5_UART1SOUT_NAME, GPIO2L_UART1_SOUT); 			
		rk29_mux_api_set(GPIO2A4_UART1SIN_NAME, GPIO2L_UART1_SIN); 		

		gpio_request(RK29_PIN2_PA5, NULL);
		gpio_request(RK29_PIN2_PA4, NULL);

		gpio_direction_output(RK29_PIN2_PA5, GPIO_HIGH);
		gpio_direction_output(RK29_PIN2_PA4, GPIO_HIGH);
	}
	else {
		//to do
	}

	return 0;

}

int rk29_gps_suspend(struct platform_device *pdev,  pm_message_t state)
{
	struct rk29_gps_data *pdata = pdev->dev.platform_data;

	if(!pdata)
		return -1;
		
	if(pdata->power_flag == 1)
	{
		rk29_gps_uart_to_gpio(pdata->uart_id);
		pdata->power_down();	
		pdata->reset(GPIO_LOW);
	}
	
	printk("%s\n",__FUNCTION__);

	return 0;	
}

int rk29_gps_resume(struct platform_device *pdev)
{
	struct rk29_gps_data *pdata = pdev->dev.platform_data;

	if(!pdata)
		return -1;
	
	if(pdata->power_flag == 1)
	{
		pdata->reset(GPIO_LOW);
		mdelay(10);
		pdata->power_up();
		mdelay(500);
		pdata->reset(GPIO_HIGH);
		rk29_gps_gpio_to_uart(pdata->uart_id);
	}
	
	printk("%s\n",__FUNCTION__);

	return 0;
}

int rk29_gps_open(struct inode *inode, struct file *filp)
{
    DBG("rk29_gps_open\n");

	return 0;
}

int rk29_gps_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret = 0;
	struct rk29_gps_data *pdata = pgps;

	DBG("rk29_gps_ioctl: cmd = %d\n",cmd);

	ret = down_interruptible(&pdata->power_sem);
	if (ret < 0) {
		printk("%s: down power_sem error ret = %d\n", __func__, ret);
		return ret;
	}

	switch (cmd){
		case ENABLE:
			pdata->reset(GPIO_LOW);
			mdelay(10);
			pdata->power_up();
			mdelay(10);
			rk29_gps_gpio_to_uart(pdata->uart_id);
			mdelay(500);
			pdata->reset(GPIO_HIGH);
			pdata->power_flag = 1;
			break;
			
		case DISABLE:
			rk29_gps_uart_to_gpio(pdata->uart_id);
			pdata->power_down();
			pdata->reset(GPIO_LOW);
			pdata->power_flag = 0;
			break;
			
		default:
			printk("unknown ioctl cmd!\n");
			up(&pdata->power_sem);
			ret = -EINVAL;
			break;
	}

	up(&pdata->power_sem);

	return ret;
}


int rk29_gps_release(struct inode *inode, struct file *filp)
{
    	DBG("rk29_gps_release\n");
    
	return 0;
}

static struct file_operations rk29_gps_fops = {
	.owner   = THIS_MODULE,
	.open    = rk29_gps_open,
	.ioctl   = rk29_gps_ioctl,
	.release = rk29_gps_release,
};

static struct miscdevice rk29_gps_dev = 
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "rk29_gps",
    .fops = &rk29_gps_fops,
};

static int rk29_gps_probe(struct platform_device *pdev)
{
	int ret = 0;
	printk("\n\n=========================\n%s\n", __func__);
	struct rk29_gps_data *pdata = pdev->dev.platform_data;
	if(!pdata)
		return -1;
		
	ret = misc_register(&rk29_gps_dev);
	if (ret < 0){
		printk("rk29 gps register err!\n");
		return ret;
	}
	
	init_MUTEX(&pdata->power_sem);
	pdata->power_flag = 0;

	pgps = pdata;


	printk("%s:rk29 GPS initialized\n",__FUNCTION__);

	return ret;
}

static struct platform_driver rk29_gps_driver = {
	.probe	= rk29_gps_probe,
	.suspend  	= rk29_gps_suspend,
	.resume		= rk29_gps_resume,
	.driver	= {
		.name	= "rk29_gps",
		.owner	= THIS_MODULE,
	},
};

static int __init rk29_gps_init(void)
{
	return platform_driver_register(&rk29_gps_driver);
}

static void __exit rk29_gps_exit(void)
{
	platform_driver_unregister(&rk29_gps_driver);
}

module_init(rk29_gps_init);
module_exit(rk29_gps_exit);
MODULE_DESCRIPTION ("rk29 gps driver");
MODULE_LICENSE("GPL");

