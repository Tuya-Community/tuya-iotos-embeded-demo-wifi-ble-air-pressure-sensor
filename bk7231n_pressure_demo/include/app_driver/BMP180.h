/*
 * @file name: 
 * @Descripttion: 
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2021-03-22 15:58:06
 * @LastEditors: zgw
 * @LastEditTime: 2021-03-22 17:11:41
 */

#ifndef __BMP180_H__
#define __BMP180_H__

#include "uni_log.h"
#include "tuya_cloud_error_code.h"


/*BMP180 I2C Address*/
#define BMP180_I2C_ADDR                           (0xEE >> 0)

/***************************************************************/
/**\name    ERROR CODE DEFINITIONS    */
/***************************************************************/
#define E_BMP_NULL_PTR                            ((INT8_T) - 127)
#define E_BMP_COMM_RES                            ((INT8_T) - 1)
#define E_BMP_OUT_OF_RANGE                        ((INT8_T) - 2)

/***************************************************************/
/**\name    CONSTANTS       */
/***************************************************************/
#define   BMP180_RETURN_FUNCTION_TYPE               INT8_T
#define   BMP180_INIT_VALUE                       ((UINT8_T)0)
#define   BMP180_INITIALIZE_OVERSAMP_SETTING_U8X  ((UINT8_T)0)
#define   BMP180_INITIALIZE_SW_OVERSAMP_U8X       ((UINT8_T)0)
#define   BMP180_INITIALIZE_NUMBER_OF_SAMPLES_U8X ((UINT8_T)1)
#define   BMP180_GEN_READ_WRITE_DATA_LENGTH       ((UINT8_T)1)
#define   BMP180_TEMPERATURE_DATA_LENGTH          ((UINT8_T)2)
#define   BMP180_PRESSURE_DATA_LENGTH             ((UINT8_T)3)
#define   BMP180_SW_OVERSAMP_U8X                  ((UINT8_T)1)
#define   BMP180_OVERSAMP_SETTING_U8X             ((UINT8_T)3)
#define   BMP180_2MS_DELAY_U8X                    (2)
#define   BMP180_3MS_DELAY_U8X                    (3)
#define   BMP180_AVERAGE_U8X                      (3)
#define   BMP180_INVALID_DATA                     (0)
#define   BMP180_CHECK_DIVISOR                    (0)
#define   BMP180_DATA_MEASURE                     (3)
#define   BMP180_CALCULATE_TRUE_PRESSURE          (8)
#define   BMP180_CALCULATE_TRUE_TEMPERATURE       (8)
#define BMP180_SHIFT_BIT_POSITION_BY_01_BIT       (1)
#define BMP180_SHIFT_BIT_POSITION_BY_02_BITS      (2)
#define BMP180_SHIFT_BIT_POSITION_BY_04_BITS      (4)
#define BMP180_SHIFT_BIT_POSITION_BY_06_BITS      (6)
#define BMP180_SHIFT_BIT_POSITION_BY_08_BITS      (8)
#define BMP180_SHIFT_BIT_POSITION_BY_11_BITS      (11)
#define BMP180_SHIFT_BIT_POSITION_BY_12_BITS      (12)
#define BMP180_SHIFT_BIT_POSITION_BY_13_BITS      (13)
#define BMP180_SHIFT_BIT_POSITION_BY_15_BITS      (15)
#define BMP180_SHIFT_BIT_POSITION_BY_16_BITS      (16)

/***************************************************************/
/**\name    REGISTER ADDRESS DEFINITION       */
/***************************************************************/
/*register definitions */

#define BMP180_PROM_START__ADDR       (0xAA)
#define BMP180_PROM_DATA__LEN         (22)

#define BMP180_CHIP_ID_REG            (0xD0)
#define BMP180_VERSION_REG            (0xD1)

#define BMP180_CTRL_MEAS_REG          (0xF4)
#define BMP180_ADC_OUT_MSB_REG        (0xF6)
#define BMP180_ADC_OUT_LSB_REG        (0xF7)

#define BMP180_SOFT_RESET_REG         (0xE0)

/* temperature measurement */
#define BMP180_T_MEASURE              (0x2E)

/* pressure measurement*/
#define BMP180_P_MEASURE              (0x34)

/* TO be spec'd by GL or SB*/
#define BMP180_TEMP_CONVERSION_TIME   (5)

#define BMP180_PARAM_MG               (3038)
#define BMP180_PARAM_MH               (-7357)
#define BMP180_PARAM_MI               (3791)

/****************************************************/
/**\name    ARRAY SIZE DEFINITIONS      */
/***************************************************/
#define BMP180_TEMPERATURE_DATA_BYTES (2)
#define BMP180_PRESSURE_DATA_BYTES    (3)
#define BMP180_TEMPERATURE_LSB_DATA   (1)
#define BMP180_TEMPERATURE_MSB_DATA   (0)
#define BMP180_PRESSURE_MSB_DATA      (0)
#define BMP180_PRESSURE_LSB_DATA      (1)
#define BMP180_PRESSURE_XLSB_DATA     (2)

#define BMP180_CALIB_DATA_SIZE        (22)
#define BMP180_CALIB_PARAM_AC1_MSB    (0)
#define BMP180_CALIB_PARAM_AC1_LSB    (1)
#define BMP180_CALIB_PARAM_AC2_MSB    (2)
#define BMP180_CALIB_PARAM_AC2_LSB    (3)
#define BMP180_CALIB_PARAM_AC3_MSB    (4)
#define BMP180_CALIB_PARAM_AC3_LSB    (5)
#define BMP180_CALIB_PARAM_AC4_MSB    (6)
#define BMP180_CALIB_PARAM_AC4_LSB    (7)
#define BMP180_CALIB_PARAM_AC5_MSB    (8)
#define BMP180_CALIB_PARAM_AC5_LSB    (9)
#define BMP180_CALIB_PARAM_AC6_MSB    (10)
#define BMP180_CALIB_PARAM_AC6_LSB    (11)
#define BMP180_CALIB_PARAM_B1_MSB     (12)
#define BMP180_CALIB_PARAM_B1_LSB     (13)
#define BMP180_CALIB_PARAM_B2_MSB     (14)
#define BMP180_CALIB_PARAM_B2_LSB     (15)
#define BMP180_CALIB_PARAM_MB_MSB     (16)
#define BMP180_CALIB_PARAM_MB_LSB     (17)
#define BMP180_CALIB_PARAM_MC_MSB     (18)
#define BMP180_CALIB_PARAM_MC_LSB     (19)
#define BMP180_CALIB_PARAM_MD_MSB     (20)
#define BMP180_CALIB_PARAM_MD_LSB     (21)

struct bmp180_calib_param_t
{
	INT16_T ac1; /**<calibration ac1 data*/
	INT16_T ac2; /**<calibration ac2 data*/
	INT16_T ac3; /**<calibration ac3 data*/
	UINT16_T ac4; /**<calibration ac4 data*/
	UINT16_T ac5; /**<calibration ac5 data*/
	UINT16_T ac6; /**<calibration ac6 data*/
	INT16_T b1; /**<calibration b1 data*/
	INT16_T b2; /**<calibration b2 data*/
	INT16_T mb; /**<calibration mb data*/
	INT16_T mc; /**<calibration mc data*/
	INT16_T md; /**<calibration md data*/
};
extern struct bmp180_calib_param_t calib_param;


typedef struct 
{
    UCHAR_T SDA_PIN;            ///< SDA pin
    UCHAR_T SCL_PIN;            ///< SCL pin 
}bmp180_init_t;

VOID bmp180_init(bmp180_init_t* param);
UINT16_T bmp180_get_uncomp_temperature(void);
UINT_T bmp180_get_uncomp_pressure(UINT8_T oss_mode);
INT16_T bmp180_get_temperature(UINT_T v_uncomp_temperature_u32);
INT_T bmp180_get_pressure(UINT_T v_uncomp_pressure_u32);

#endif