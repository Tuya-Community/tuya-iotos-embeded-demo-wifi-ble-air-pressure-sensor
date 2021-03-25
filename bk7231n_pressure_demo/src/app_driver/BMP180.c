/*
 * @file name: 
 * @Descripttion: 
 * @Author: zgw
 * @email: liang.zhang@tuya.com
 * @Copyright: HANGZHOU TUYA INFORMATION TECHNOLOGY CO.,LTD
 * @Company: http://www.tuya.com
 * @Date: 2021-03-22 15:58:20
 * @LastEditors: zgw
 * @LastEditTime: 2021-03-22 17:34:48
 */

#include "BMP180.h"
#include "soc_i2c.h"

struct bmp180_calib_param_t calib_param;
UINT8_T oversamp_setting = 0;
UINT8_T BMP180_param[BMP180_PROM_DATA__LEN];
INT_T param_b5;

STATIC int __iic_start(VOID)
{
    vI2CSDASet();
    vI2CSCLSet();

    vI2CDelay(5);

    vI2CSDAReset();

    vI2CDelay(5);

    vI2CSCLReset();

    return 0;
}


STATIC int __iic_stop(VOID)
{   
    vI2CSCLReset();
    vI2CSDAReset();

    vI2CDelay(5);
    
    vI2CSCLSet();
    vI2CSDASet();

    vI2CDelay(5);

    return 0;      
}


STATIC VOID __iic_send_ack(VOID)
{   
    vI2CSCLReset();
        
    vI2CSDAReset();

    vI2CDelay(2);
    vI2CSCLSet();

    vI2CDelay(2);

    vI2CSCLReset();
}

STATIC VOID __iic_send_nack(VOID)
{   
    vI2CSCLReset();
        
    vI2CSDASet();

    vI2CDelay(2);
    vI2CSCLSet();

    vI2CDelay(2);

    vI2CSCLReset();
}

STATIC UINT8_T __iic_recv_ack(VOID)
{   
    UINT8_T ucErrTime=0;
    UINT8_T recv_ack;
    vI2CSDARelease();
    vI2CSCLSet();
    vI2CDelay(5);
    
    while(ucI2CSDAInputRead()) {
        ucErrTime++;
        if(ucErrTime>250) {
			__iic_stop();
            PR_NOTICE("-------iic ack error-----");
			return 1;
		}
    }

    
    vI2CSCLReset();

    return 0;
}

STATIC VOID __iic_send_byte(UCHAR_T sendbyte)
{
    UCHAR_T i = 0;
    vI2CSCLReset;
    for(i = 0x80; i > 0; i >>= 1)
    {
        if((sendbyte & i) == 0) {
            vI2CSDAReset();
        } else {
            vI2CSDASet();
        }
        vI2CDelay(2);   
        vI2CSCLSet();
        vI2CDelay(2); //vI2CDelay(5);
        vI2CSCLReset();
        vI2CDelay(2);
    }
}

STATIC UINT8_T __iic_read_byte(UCHAR_T ack)
{
    UCHAR_T i = 0;
    
    UCHAR_T readbyte = 0;
    vI2CSDARelease();
    for(i = 0x80; i > 0; i >>= 1)
    {
        vI2CSCLReset();
        vI2CDelay(2);
        vI2CSCLSet();
        if(ucI2CSDAInputRead()) {
            readbyte |= i;    
        }
        vI2CDelay(2);
    }
    if(!ack) {
        __iic_send_nack(); 
    }else {
        __iic_send_ack();
    }
    
    return readbyte;    
}

VOID bmp180_iic_write(UINT8_T drv_addr, UINT8_T reg_addr, UINT8_T reg_data)
{   
	__iic_start();
	__iic_send_byte(drv_addr);	   //发送从机地址写命令
	__iic_recv_ack();
	__iic_send_byte(reg_addr);
	__iic_recv_ack();
    __iic_send_byte(reg_data);
	__iic_recv_ack();
    __iic_stop();
}

VOID bmp180_iic_read(UINT8_T drv_addr, UINT8_T reg_addr, UINT8_T *p_data)
{   
	__iic_start();
	__iic_send_byte(drv_addr);	   //发送从机地址写命令
	__iic_recv_ack();
	__iic_send_byte(reg_addr);
	__iic_recv_ack();

    __iic_start();
    __iic_send_byte(drv_addr + 1);
	__iic_recv_ack();

    *p_data = __iic_read_byte(0);

    __iic_stop();
}

VOID bmp180_iic_write_mulbyte(UINT8_T drv_addr,UINT16_T read_addr,UINT8_T *pBuffer,UINT16_T num)
{
    __iic_start();  
	__iic_send_byte(drv_addr);	
	__iic_recv_ack();
	__iic_send_byte(read_addr);	    
	__iic_recv_ack();

    while(num--)
	{ 										  		   
		__iic_send_byte(*pBuffer);						   
		__iic_recv_ack(); 
		pBuffer++;
	}
	__iic_stop();
	vI2CDelay(100);
}

VOID bmp180_iic_read_mulbyte(UINT8_T drv_addr,UINT16_T read_addr,UINT8_T *pBuffer,UINT16_T num)
{
	__iic_start();  
	__iic_send_byte(drv_addr);	
	__iic_recv_ack();
	__iic_send_byte(read_addr);	    
	__iic_recv_ack();	    
	
	__iic_start();
	__iic_send_byte(drv_addr+1);		   
	__iic_recv_ack();
	while(num)
	{
		if(num == 1)
		{
			*pBuffer = __iic_read_byte(0);	
		}
		else
		{
			*pBuffer=__iic_read_byte(1);
		}
		pBuffer++;
		num--;
	}
	__iic_stop();
}

VOID bmp180_get_calib_param(VOID)
{
    bmp180_iic_read_mulbyte(BMP180_I2C_ADDR, BMP180_PROM_START__ADDR, BMP180_param, BMP180_PROM_DATA__LEN);
    calib_param.ac1 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_AC1_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC1_LSB]);
    calib_param.ac2 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_AC2_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC2_LSB]);
    calib_param.ac3 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_AC3_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC3_LSB]);
    calib_param.ac4 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_AC4_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC4_LSB]);
    calib_param.ac5 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_AC5_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC5_LSB]);
    calib_param.ac6 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_AC6_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_AC6_LSB]);

    /*parameters B1,B2*/
    calib_param.b1 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_B1_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_B1_LSB]);
    calib_param.b2 =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_B2_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_B2_LSB]);

    /*parameters MB,MC,MD*/
    calib_param.mb =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_MB_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_MB_LSB]);
    calib_param.mc =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_MC_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_MC_LSB]);
    calib_param.md =
        (INT16_T)((((UINT_T)((INT8_T)BMP180_param[BMP180_CALIB_PARAM_MD_MSB])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
              BMP180_param[BMP180_CALIB_PARAM_MD_LSB]);	
}

INT16_T bmp180_get_temperature(UINT_T v_uncomp_temperature_u32)
{
    INT16_T v_temperature_s16 = BMP180_INIT_VALUE;
    INT_T v_x1_s32, v_x2_s32 = BMP180_INIT_VALUE;

    /* calculate temperature*/
    v_x1_s32 = (((INT_T) v_uncomp_temperature_u32 - (INT_T) calib_param.ac6) * (INT_T) calib_param.ac5) >>
               BMP180_SHIFT_BIT_POSITION_BY_15_BITS;
    if (v_x1_s32 == BMP180_CHECK_DIVISOR && calib_param.md == BMP180_CHECK_DIVISOR)
    {
        return BMP180_INVALID_DATA;
    }

    /* executed only the divisor is not zero*/
    v_x2_s32 = ((INT_T) calib_param.mc << BMP180_SHIFT_BIT_POSITION_BY_11_BITS) /
               (v_x1_s32 + calib_param.md);
    param_b5 = v_x1_s32 + v_x2_s32;
    v_temperature_s16 =
        ((param_b5 + BMP180_CALCULATE_TRUE_TEMPERATURE) >> BMP180_SHIFT_BIT_POSITION_BY_04_BITS);

    return v_temperature_s16;
}

INT_T bmp180_get_pressure(UINT_T v_uncomp_pressure_u32)
{
    INT_T v_pressure_s32, v_x1_s32, v_x2_s32, v_x3_s32, v_b3_s32, v_b6_s32 = BMP180_INIT_VALUE;
	UINT_T v_b4_u32, v_b7_u32 = BMP180_INIT_VALUE;

	v_b6_s32 = param_b5 - 4000;

	/*****calculate B3************/
	v_x1_s32 = (v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_12_BITS;
	v_x1_s32 *= calib_param.b2;
	v_x1_s32 >>= BMP180_SHIFT_BIT_POSITION_BY_11_BITS;
	v_x2_s32 = (calib_param.ac2 * v_b6_s32);
	v_x2_s32 >>= BMP180_SHIFT_BIT_POSITION_BY_11_BITS;
	v_x3_s32 = v_x1_s32 + v_x2_s32;
	v_b3_s32 = (((((INT_T)calib_param.ac1) * 4 + v_x3_s32) << oversamp_setting) + 2) >>
	BMP180_SHIFT_BIT_POSITION_BY_02_BITS;

	/*****calculate B4************/
	v_x1_s32 = (calib_param.ac3 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_13_BITS;
	v_x2_s32 = (calib_param.b1 * ((v_b6_s32 * v_b6_s32) >> BMP180_SHIFT_BIT_POSITION_BY_12_BITS)) >>
	BMP180_SHIFT_BIT_POSITION_BY_16_BITS;
	v_x3_s32 = ((v_x1_s32 + v_x2_s32) + 2) >> BMP180_SHIFT_BIT_POSITION_BY_02_BITS;
	v_b4_u32 = (calib_param.ac4 * (UINT_T)(v_x3_s32 + 32768)) >> BMP180_SHIFT_BIT_POSITION_BY_15_BITS;
	v_b7_u32 = ((UINT_T)(v_uncomp_pressure_u32 - v_b3_s32) * (50000 >> oversamp_setting));
	if (v_b7_u32 < 0x80000000)
	{
		if (v_b4_u32 != BMP180_CHECK_DIVISOR)
		{
			v_pressure_s32 = (v_b7_u32 << BMP180_SHIFT_BIT_POSITION_BY_01_BIT) / v_b4_u32;
		}
		else
		{
			return BMP180_INVALID_DATA;
		}
	}
	else
	{
		if (v_b4_u32 != BMP180_CHECK_DIVISOR)
		{
			v_pressure_s32 = (v_b7_u32 / v_b4_u32) << BMP180_SHIFT_BIT_POSITION_BY_01_BIT;
		}
		else
		{
			return BMP180_INVALID_DATA;
		}
	}
	v_x1_s32 = v_pressure_s32 >> BMP180_SHIFT_BIT_POSITION_BY_08_BITS;
	v_x1_s32 *= v_x1_s32;
	v_x1_s32 = (v_x1_s32 * BMP180_PARAM_MG) >> BMP180_SHIFT_BIT_POSITION_BY_16_BITS;
	v_x2_s32 = (v_pressure_s32 * BMP180_PARAM_MH) >> BMP180_SHIFT_BIT_POSITION_BY_16_BITS;

	/*pressure in Pa*/
	v_pressure_s32 += (v_x1_s32 + v_x2_s32 + BMP180_PARAM_MI) >> BMP180_SHIFT_BIT_POSITION_BY_04_BITS;

	return v_pressure_s32;
}

UINT16_T bmp180_get_uncomp_temperature(void)
{
    UINT16_T v_ut_u16 = BMP180_INIT_VALUE;
    /* Array holding the temperature LSB and MSB data*/
    UINT8_T v_data_u8[BMP180_TEMPERATURE_DATA_BYTES] = {0,0};
    UINT8_T v_ctrl_reg_data_u8 = BMP180_T_MEASURE;
    
    /* used to return the bus communication results*/
    
    bmp180_iic_write_mulbyte(BMP180_I2C_ADDR,BMP180_CTRL_MEAS_REG,&v_ctrl_reg_data_u8,BMP180_GEN_READ_WRITE_DATA_LENGTH);
		vI2CDelay(500);
	bmp180_iic_read_mulbyte(BMP180_I2C_ADDR,BMP180_ADC_OUT_MSB_REG,v_data_u8,BMP180_TEMPERATURE_DATA_LENGTH);
		
    v_ut_u16 =
        (UINT16_T)((((INT_T)((INT8_T)v_data_u8[BMP180_TEMPERATURE_MSB_DATA])) << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
                (v_data_u8[BMP180_TEMPERATURE_LSB_DATA]));

    return v_ut_u16;
}

UINT_T bmp180_get_uncomp_pressure(UINT8_T oss_mode)
{
	/*j included for loop*/
	//uint8_t v_j_u8 = BMP180_INIT_VALUE;
	UINT_T v_up_u32 = BMP180_INIT_VALUE;
	
	/*get the calculated pressure data*/
	//UINT_T v_sum_u32 = BMP180_INIT_VALUE;
	UINT8_T v_data_u8[BMP180_PRESSURE_DATA_BYTES] = { BMP180_INIT_VALUE, BMP180_INIT_VALUE, BMP180_INIT_VALUE };
	UINT8_T v_ctrl_reg_data_u8 = BMP180_INIT_VALUE;

	/* used to return the bus communication results*/

	v_ctrl_reg_data_u8 = BMP180_P_MEASURE + ( oss_mode << BMP180_SHIFT_BIT_POSITION_BY_06_BITS);
    bmp180_iic_write_mulbyte(BMP180_I2C_ADDR,BMP180_CTRL_MEAS_REG,&v_ctrl_reg_data_u8,BMP180_GEN_READ_WRITE_DATA_LENGTH);
	vI2CDelay((BMP180_2MS_DELAY_U8X + (BMP180_3MS_DELAY_U8X << (oss_mode)))*10);
    bmp180_iic_read_mulbyte(BMP180_I2C_ADDR,BMP180_ADC_OUT_MSB_REG,v_data_u8,BMP180_PRESSURE_DATA_LENGTH);

    v_up_u32 =
            (UINT_T)((((UINT_T)v_data_u8[BMP180_PRESSURE_MSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_16_BITS) |
                            ((UINT_T) v_data_u8[BMP180_PRESSURE_LSB_DATA] << BMP180_SHIFT_BIT_POSITION_BY_08_BITS) |
                            (UINT_T) v_data_u8[BMP180_PRESSURE_XLSB_DATA]) >>
                        (BMP180_CALCULATE_TRUE_PRESSURE - oss_mode));

	return v_up_u32;
}

VOID bmp180_init(bmp180_init_t* param)
{
    UINT8_T id;
    UINT8_T i;

    i2c_pin_t i2c_config = {
        .ucSDA_IO = param ->SDA_PIN,
        .ucSCL_IO = param ->SCL_PIN,
    };
    
    opSocI2CInit(&i2c_config);          /* SDA&SCL GPIO INIT */
    vI2CDelay(100);

    bmp180_iic_read_mulbyte(BMP180_I2C_ADDR, BMP180_CHIP_ID_REG, &id, 1);
    if(id == 0x55) {
        PR_NOTICE("bmp180 init success");
        oversamp_setting = 1;
        bmp180_get_calib_param();
    }else {
        PR_NOTICE("bmp180 init failed");
    }
}