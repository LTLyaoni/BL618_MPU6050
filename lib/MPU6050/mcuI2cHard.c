
#ifndef mcuI2cHard.c 
#define mcuI2cHard.c 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "mcuI2cHard.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/
static struct bflb_device_s *i2c0;       //i2c0外设句柄

struct bflb_i2c_msg_s msgs[2];           //I2C发送数据结构体数组

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
//初始化I2C的gpio引脚，
void mcu_i2c_hard_IIC_Init(){ 

    struct bflb_device_s* gpio;

    gpio = bflb_device_get_by_name("gpio");
    /* I2C0_SDA */
    bflb_gpio_init(gpio, GPIO_PIN_27, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2C0_SCL */
    bflb_gpio_init(gpio, GPIO_PIN_26, GPIO_FUNC_I2C0 | GPIO_ALTERNATE | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);

    i2c0 = bflb_device_get_by_name("i2c0");
    bflb_i2c_init(i2c0, 400000);
    bflb_i2c_link_txdma(i2c0, true);
    bflb_i2c_link_rxdma(i2c0, true);

}


//I2C写入数据函数

static uint8_t mcu_i2c_hard_i2c_write(char reg_addr, char buff, char buf_size)
{
    /* Write data */
    // msgs[0].addr = I2C_SLAVE_ADDR;
    // msgs[0].flags = I2C_M_NOSTOP;
    // msgs[0].buffer = ®_addr;
    // msgs[0].length = buf_size;

    // msgs[1].addr = I2C_SLAVE_ADDR;
    // msgs[1].flags = 0;
    // msgs[1].buffer = &buff;
    // msgs[1].length = buf_size;

    // bflb_i2c_transfer(i2c0, msgs, 2);

    // bflb_mtimer_delay_ms(100);

    return 1;
}




/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* mcuI2cHard.c */
