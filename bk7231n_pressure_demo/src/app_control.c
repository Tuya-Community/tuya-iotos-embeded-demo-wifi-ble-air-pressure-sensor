/*
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @LastEditors: zgw
 * @file name: sht21.h
 * @Description: SHT21 IIC drive src file
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2020-12-16 18:51:29
 * @LastEditTime: 2021-03-22 17:51:54
 */

#include "app_control.h"
#include "tuya_gpio.h"
#include "tuya_uart.h"
#include "BkDriverUart.h"
#include "sys_timer.h"
#include "uni_time.h"
#include "BMP180.h"
/***********************************************************
*************************types define***********************
***********************************************************/
typedef enum
{
    LOW = 0,
    HIGH,
}default_level;

APP_CTRL_DATA_T app_ctrl_data = {0};

/***********************************************************
*************************IO control device define***********
***********************************************************/

/***********************************************************
*************************about adc init*********************
***********************************************************/
#define IIC_SDA_PIN                         (6)
#define IIC_SCL_PIN                         (7)

STATIC bmp180_init_t bmp180_init_param = {IIC_SDA_PIN, IIC_SCL_PIN};


/***********************************************************
*************************about iic init*********************
***********************************************************/

/***********************************************************
*************************function***************************
***********************************************************/

STATIC VOID __ctrl_gpio_init(CONST TY_GPIO_PORT_E port, CONST BOOL_T high)
{   
    //Set IO port as output mode
    tuya_gpio_inout_set(port, FALSE);
    //Set IO port level
    tuya_gpio_write(port, high);
}


VOID app_device_init(VOID)
{
    bmp180_init(&bmp180_init_param);
    tuya_hal_system_sleep(500);
}



VOID app_ctrl_handle(VOID)
{   
	UINT_T v_uncomp_press_u32 = 0;

    v_uncomp_press_u32 = bmp180_get_uncomp_pressure(1);
    app_ctrl_data.pressure_value = bmp180_get_pressure(v_uncomp_press_u32);

    PR_NOTICE("------------- pressure = %d ---------",app_ctrl_data.pressure_value);
}

VOID app_ctrl_all_off(VOID)
{   

}

 