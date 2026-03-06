#include "my_lvgl_task.h"
#include "lvgl.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"
#include "cmsis_os.h"
#include "ui.h"



void my_gui_task(void *argument)
{
	lv_init();
	lv_port_disp_init();
	lv_port_indev_init();
	
	ui_init();
	
	while(1)
	{
//		lv_timer_handler();
		osDelay(20);
	}
	
	
}




