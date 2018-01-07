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

