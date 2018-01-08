#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"

#ifdef BUILD_LK
	#include <platform/upmu_common.h>
	#include <platform/upmu_hw.h>
	#include <platform/mt_gpio.h>
	#include <platform/mt_i2c.h>
	#include <platform/mt_pmic.h>
	#include <string.h>
#else
	/* #include <mach/mt_pm_ldo.h>	 hwPowerOn */
	#include <mt-plat/upmu_common.h>
	#include <mach/upmu_sw.h>
	#include <mach/upmu_hw.h>

	#include <mt-plat/mt_gpio.h>
#endif
/* #include <cust_gpio_usage.h> */
#define I2C_I2C_LCD_BIAS_CHANNEL 1

static const unsigned int BL_MIN_LEVEL = 20;
static LCM_UTIL_FUNCS lcm_util;

#define SET_RESET_PIN(v) (lcm_util.set_reset_pin((v)))
#define MDELAY(n) (lcm_util.mdelay(n))
#define UDELAY(n) (lcm_util.udelay(n))


/*/ ---------------------------------------------------------------------------
//  Local Functions
// ---------------------------------------------------------------------------*/

#define dsi_set_cmdq_V2(cmd, count, ppara, force_update) lcm_util.dsi_set_cmdq_V2(cmd, count, ppara, force_update)
#define dsi_set_cmdq(pdata, queue_size, force_update) lcm_util.dsi_set_cmdq(pdata, queue_size, force_update)
#define wrtie_cmd(cmd) lcm_util.dsi_write_cmd(cmd)
#define write_regs(addr, pdata, byte_nums) lcm_util.dsi_write_regs(addr, pdata, byte_nums)
#define read_reg(cmd) lcm_util.dsi_dcs_read_lcm_reg(cmd)
#define read_reg_v2(cmd, buffer, buffer_size) lcm_util.dsi_dcs_read_lcm_reg_v2(cmd, buffer, buffer_size)

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
/*****************************************************************************
 * Define
 *****************************************************************************/
#ifndef CONFIG_FPGA_EARLY_PORTING
#define NT_I2C_BUSNUM  I2C_I2C_LCD_BIAS_CHANNEL/*for I2C channel 0*/
#define I2C_ID_NAME "tps65132"
#define NT_ADDR 0x3E
/*****************************************************************************
 * GLobal Variable
 *****************************************************************************/
static struct i2c_board_info tps65132_board_info __initdata = {I2C_BOARD_INFO(I2C_ID_NAME, NT_ADDR)};
static struct i2c_client *tps65132_i2c_client;


/*****************************************************************************
 * Function Prototype
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int tps65132_remove(struct i2c_client *client);
/*****************************************************************************
 * Data Structure
 *****************************************************************************/

struct tps65132_dev	{
	struct i2c_client	*client;

};

static const struct i2c_device_id tps65132_id[] = {
	{ I2C_ID_NAME, 0 },
	{ }
};

/*#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36))
//static struct i2c_client_address_data addr_data = { .forces = forces,};
//#endif*/
static struct i2c_driver tps65132_iic_driver = {
	.id_table	= tps65132_id,
	.probe		= tps65132_probe,
	.remove		= tps65132_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "tps65132",
	},

};
/*****************************************************************************
 * Extern Area
 *****************************************************************************/



/*****************************************************************************
 * Function
 *****************************************************************************/
static int tps65132_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	pr_debug("tps65132_iic_probe\n");
	pr_debug("NT: info==>name=%s addr=0x%x\n", client->name, client->addr);
	tps65132_i2c_client  = client;
	return 0;
}


static int tps65132_remove(struct i2c_client *client)
{
	pr_debug("tps65132_remove\n");
	tps65132_i2c_client = NULL;
	i2c_unregister_device(client);
	return 0;
}

/*
static int tps65132_write_bytes(unsigned char addr, unsigned char value)
{
	int ret = 0;
	struct i2c_client *client = tps65132_i2c_client;
	char write_data[2] = {0};
	write_data[0] = addr;
	write_data[1] = value;
	ret = i2c_master_send(client, write_data, 2);
if (ret < 0)
	pr_debug("tps65132 write data fail !!\n");
return ret;
}
*/

/*
 * module load/unload record keeping
 */

static int __init tps65132_iic_init(void)
{

	pr_debug("tps65132_iic_init\n");
	i2c_register_board_info(NT_I2C_BUSNUM, &tps65132_board_info, 1);
	pr_debug("tps65132_iic_init2\n");
	i2c_add_driver(&tps65132_iic_driver);
	pr_debug("tps65132_iic_init success\n");
	return 0;
}

static void __exit tps65132_iic_exit(void)
{
	pr_debug("tps65132_iic_exit\n");
	i2c_del_driver(&tps65132_iic_driver);
}


module_init(tps65132_iic_init);
module_exit(tps65132_iic_exit);

MODULE_AUTHOR("Xiaokuan Shi");
MODULE_DESCRIPTION("MTK TPS65132 I2C Driver");
MODULE_LICENSE("GPL");
#endif
#endif

/*/static unsigned char lcd_id_pins_value = 0xFF;*/
static const unsigned char LCD_MODULE_ID = 0x01; /*  haobing modified 2013.07.11*/
/*/ ---------------------------------------------------------------------------
//  Local Constants
// ---------------------------------------------------------------------------
//#define LCM_DSI_CMD_MODE*/
#ifdef CONFIG_FPGA_EARLY_PORTING
#define FRAME_WIDTH (480)
#define FRAME_HEIGHT (800)
#else
#define FRAME_WIDTH (1080)
#define FRAME_HEIGHT (1920)
#endif
/*
#ifndef CONFIG_FPGA_EARLY_PORTING
#define GPIO_65132_EN GPIO_LCD_BIAS_ENP_PIN
#endif
*/
#define REGFLAG_DELAY 0xFC
#define REGFLAG_UDELAY 0xFB

#define REGFLAG_END_OF_TABLE 0xFD   /* END OF REGISTERS MARKER*/
#define REGFLAG_RESET_LOW 0xFE
#define REGFLAG_RESET_HIGH 0xFF

LCM_DSI_MODE_SWITCH_CMD lcm_switch_mode_cmd;

#ifndef TRUE
	#define TRUE 1
#endif

#ifndef FALSE
	#define FALSE 0
#endif
/*static unsigned int lcm_esd_test = FALSE;      ///only for ESD test
// ---------------------------------------------------------------------------
//  Local Variables
// ---------------------------------------------------------------------------*/


struct LCM_setting_table {
	unsigned char cmd;
	unsigned char count;
	unsigned char para_list[64];
};

static struct LCM_setting_table lcm_suspend_setting[] = {
#ifndef LCM_DSI_CMD_MODE
	/* switch to CMD mode for fitting DSI state */
	{0xB3, 1, {0x04} },
	{REGFLAG_DELAY, 120, {} },
#endif
	{0x28, 0, {} },
	{REGFLAG_DELAY, 20, {} },
	{0x10, 0, {} },
	{0xB0, 1, {0x00} },
	{0xB1, 1, {0x01} },
	{REGFLAG_DELAY, 80, {} },
};

/*update initial param for IC nt35520 0.01*/
static struct LCM_setting_table lcm_initialization_setting[] = {

	/*unlock manufacturing command write.*/
	{0xB0, 1, {0x00} },
	/*remove NVM reload after sleep*/
	{0xD6, 1, {0x01} },
#ifndef LCM_DSI_CMD_MODE
	/*video mode*/
	{0xB3, 1, {0x14} },
#endif
	/*display brightness*/
	{0x51, 1, {0xFF} },
	/*LED pwm output enable*/
	{0x53, 1, {0x0C} },

	/*Analog Gamma Setting*/
	{0xC7, 30, {0x01, 0x0C, 0x14, 0x1E, 0x2D, 0x3C, 0x48, 0x58, 0x3D, 0x44, 0x4F, 0x5C, 0x65, 0x6D, 0x75,
	0x01, 0x0C, 0x14, 0x1D, 0x2C, 0x39, 0x44, 0x54, 0x39, 0x41, 0x4D, 0x5A, 0x63, 0x6B, 0x74} },

	/*Digital Gamma Setting*/
	{0xC8, 19, {0x01, 0x00, 0xFF, 0xFF, 0x00, 0xFC, 0x00, 0x00, 0xFF, 0xFF, 0x00, 0xFC, 0x00, 0x00, 0xFF,
	0xFF, 0x00, 0xFC, 0x00} },

	/*Enable TE*/
	{0x35, 1, {0x00} },
/* mipi clock*/
	{0xB6, 2, {0x3A, 0xD3} },

	/* Display ON*/
	{0x29, 0, {} },
	/*Sleep Out*/
	{0x11, 0, {} },
	{REGFLAG_DELAY, 120, {} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};

static struct LCM_setting_table lcm_backlight_level_setting[] = {
	{0x51, 1, {0xFF} },
	{REGFLAG_END_OF_TABLE, 0x00, {} }
};
static void push_table(struct LCM_setting_table *table, unsigned int count, unsigned char force_update)
{
	unsigned int i;

	for (i = 0; i < count; i++) {
		unsigned cmd;

		cmd = table[i].cmd;

			switch (cmd) {

			case REGFLAG_DELAY:
				MDELAY(table[i].count);
				break;

			case REGFLAG_UDELAY:
				UDELAY(table[i].count);
				break;

			case REGFLAG_END_OF_TABLE:
				break;

			default:
				dsi_set_cmdq_V2(cmd, table[i].count, table[i].para_list, force_update);
		}
	}
}

/* ---------------------------------------------------------------------------
//  LCM Driver Implementations
// ---------------------------------------------------------------------------*/

void lcm_set_util_funcs(const LCM_UTIL_FUNCS *util)
{
  memcpy(&g_LCM_UTIL_FUNCS, util, 0xD8);
}

void lcm_get_params(LCM_PARAMS *params)
{
  memset(params, 0, 0x378);
  params->dsi.LANE_NUM = 4;
  params->dsi.vertical_sync_active = 4;
  params->dsi.horizontal_sync_active = 4;
  params->dsi.horizontal_backporch = 50;
  params->dsi.horizontal_frontporch = 96;
  params->type = 2;
  params->dsi.data_format.format = 2;
  params->dsi.PS = 2;
  params->dsi.lcm_esd_check_table[0].cmd = 10;
  params->physical_width = 62;
  params->dsi.mode = 1;
  params->dsi.vertical_backporch = 20;
  params->dsi.esd_check_enable = 1;
  params->dsi.customization_esd_check_enable = 1;
  params->dsi.lcm_esd_check_table[0].count = 1;
  params->dsi.lcm_esd_check_table[0].para_list[0] = -100;
  params->dsi.lcm_esd_check_table[1].count = 1;
  params->dsi.cont_clock = 1;
  params->width = 720;
  params->height = 1280;
  params->physical_height = 111;
  params->dsi.vertical_frontporch = 36;
  params->dsi.vertical_active_line = 1280;
  params->dsi.horizontal_active_pixel = 720;
  params->dsi.lcm_esd_check_table[1].cmd = 13;
  params->dsi.PLL_CLOCK = 225;
}

void lcm_init(void)
{
  int array[4];

//  printk("@@tianma@kernel lihl lcm_init \n\n");
  mt_set_gpio_mode(0x92, 0);
  mt_set_gpio_dir(0x92, 1);
  mt_set_gpio_out(0x9, 1);
  MDELAY(2);
  mt_set_gpio_out(0x92, 0);
  MDELAY(2);
  mt_set_gpio_out(0x92, 1);
  MDELAY(100);
  array[0] = 0x110500;
  dsi_set_cmdq(array);
  MDELAY(120);
  array[0] = 0x290500;
  dsi_set_cmdq(array);
  MDELAY(20);
}

void lcm_suspend(void)
{
  int array[4];

//  printk("@@tianma@kernel lihl lcm_suspend \n\n");
  array[0] = 0x280500;
  dsi_set_cmdq(array);
  MDELAY(30);
  array[0] = 0x100500;
  dsi_set_cmdq(array);
  MDELAY(120);
}

void lcm_resume(void)
{
//  printk("@@tianma@kernel lihl lcm_resume \n\n");
  lcm_init();
}

void lcm_init_power(void)
{
//  printk("@@tianma@kernel lihl lcm_init_power\n\n");
  tpd_ft_turnon_power();
  MDELAY(5);
}

void lcm_suspend_power(void)
{
//  printk("@@tianma@kernel tangwei ft8606 lcm_suspend_power\n\n");
  tpd_ft_turnoff_power();
}

void lcm_resume_power(void)
{
//  printk("@@tianma@kernel lihl lcm_resume_power\n\n");
  lcm_init_power();
}

unsigned int lcm_compare_id(void)
{
  return 1;
}

