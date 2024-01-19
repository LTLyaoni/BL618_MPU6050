/**
 * *****************************************************************************
 * @file        mpu6050.h
 * @brief
 * @author       ()
 * @date        2023-12-30
 * @copyright   yaoni
 * *****************************************************************************
 */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C"
{
#endif

/*----------------------------------include-----------------------------------*/
#include "stdio.h"

#if defined(CONFIG_MPU_I2C_SOFT) && CONFIG_MPU_I2C_SOFT
// #define MPU_I2C_TYPE SOFT
#include "mcuI2cSoft.h"

#define _MCU_I2C_FUNC_DEFINE(_func, ...) mcu_i2c_soft_##_func(__VA_ARGS__)
#else
// #define MPU_I2C_TYPE HARD
#define _MCU_I2C_FUNC_DEFINE(_func, ...) mcu_i2c_hard_##_func(__VA_ARGS__)
#endif

    /**
     *@defgroup mpu6050驱动程序mpu605驱动程序功能
     *@brief mpu6050驱动程序模块
     * @
     */
    /**
     *@addtogroup mpu6050基本驱动程序
     * @{
     */
    /**
     *@brief mpu6050地址枚举定义
     */
    typedef enum
    {
        MPU6050_ADDRESS_AD0_LOW = 0xD0,  /**< AD0 pin set LOW */
        MPU6050_ADDRESS_AD0_HIGH = 0xD2, /**< AD0 pin set HIGH */
    } mpu6050_address_t;

    /**
     * @brief mpu6050唤醒频率枚举定义
     */
    typedef enum
    {
        MPU6050_WAKE_UP_FREQUENCY_1P25_HZ = 0x00, /**< 1.25Hz */
        MPU6050_WAKE_UP_FREQUENCY_5_HZ = 0x01,    /**< 5Hz */
        MPU6050_WAKE_UP_FREQUENCY_20_HZ = 0x02,   /**< 20Hz */
        MPU6050_WAKE_UP_FREQUENCY_40_HZ = 0x03,   /**< 40Hz */
    } mpu6050_wake_up_frequency_t;

    /**
     * @brief mpu6050布尔枚举定义
     */
    typedef enum
    {
        MPU6050_BOOL_FALSE = 0x00, /**< 禁用功能 */
        MPU6050_BOOL_TRUE = 0x01,  /**< 启用功能 */
    } mpu6050_bool_t;

    /**
     * @brief mpu6050源枚举定义
     */
    typedef enum
    {
        MPU6050_SOURCE_ACC_X = 0x05,  /**< 加速度计 x */
        MPU6050_SOURCE_ACC_Y = 0x04,  /**< 加速度计 y */
        MPU6050_SOURCE_ACC_Z = 0x03,  /**< 加速度计 z */
        MPU6050_SOURCE_GYRO_X = 0x02, /**< 陀螺仪 x */
        MPU6050_SOURCE_GYRO_Y = 0x01, /**< 陀螺仪 y */
        MPU6050_SOURCE_GYRO_Z = 0x00, /**< 陀螺仪 z */
    } mpu6050_source_t;

    /**
     * @brief mpu6050时钟源枚举定义
     */
    typedef enum
    {
        MPU6050_CLOCK_SOURCE_INTERNAL_8MHZ = 0x00,      /**< internal 8MHz */
        MPU6050_CLOCK_SOURCE_PLL_X_GYRO = 0x01,         /**< pll with x axis gyroscope reference */
        MPU6050_CLOCK_SOURCE_PLL_Y_GYRO = 0x02,         /**< pll with y axis gyroscope reference */
        MPU6050_CLOCK_SOURCE_PLL_Z_GYRO = 0x03,         /**< pll with z axis gyroscope reference */
        MPU6050_CLOCK_SOURCE_PLL_EXT_32P768_KHZ = 0x04, /**< pll extern 32.768 KHz */
        MPU6050_CLOCK_SOURCE_PLL_EXT_19P2_MHZ = 0x05,   /**< pll extern 19.2 MHz */
        MPU6050_CLOCK_SOURCE_STOP_CLOCK = 0x07,         /**< 停止时钟 */
    } mpu6050_clock_source_t;

    /**
     * @brief mpu6050信号路径重置枚举定义
     */
    typedef enum
    {
        MPU6050_SIGNAL_PATH_RESET_TEMP = 0x00,  /**< 温度传感器模拟和数字信号路径 */
        MPU6050_SIGNAL_PATH_RESET_ACCEL = 0x01, /**< 加速度计模拟和数字信号路径 */
        MPU6050_SIGNAL_PATH_RESET_GYRO = 0x02,  /**< 陀螺仪模拟和数字信号路径 */
    } mpu6050_signal_path_reset_t;

    /**
     * @brief mpu6050外部同步枚举定义
     */
    typedef enum
    {
        MPU6050_EXTERN_SYNC_INPUT_DISABLED = 0x00, /**< input disabled */
        MPU6050_EXTERN_SYNC_TEMP_OUT_L = 0x01,     /**< temp out low */
        MPU6050_EXTERN_SYNC_GYRO_XOUT_L = 0x02,    /**< gyro xout low */
        MPU6050_EXTERN_SYNC_GYRO_YOUT_L = 0x03,    /**< gyro yout low */
        MPU6050_EXTERN_SYNC_GYRO_ZOUT_L = 0x04,    /**< gyro zout low */
        MPU6050_EXTERN_SYNC_ACCEL_XOUT_L = 0x05,   /**< accel xout low */
        MPU6050_EXTERN_SYNC_ACCEL_YOUT_L = 0x06,   /**< accel yout low */
        MPU6050_EXTERN_SYNC_ACCEL_ZOUT_L = 0x07,   /**< accel zout low */
    } mpu6050_extern_sync_t;

    /**
     * @brief mpu6050低通滤波器枚举定义
     */
    typedef enum                        /**<           accelerometer                     gyroscope             */
    {                                   /**< bandwidth(Hz) fs(KHz) delay(ms)   bandwidth(Hz) fs(KHz) delay(ms) */
      MPU6050_LOW_PASS_FILTER_0 = 0x00, /**<      260         1         0          256          8      0.98    */
      MPU6050_LOW_PASS_FILTER_1 = 0x01, /**<      184         1       2.0          188          1       1.9    */
      MPU6050_LOW_PASS_FILTER_2 = 0x02, /**<       94         1       3.0           98          1       2.8    */
      MPU6050_LOW_PASS_FILTER_3 = 0x03, /**<       44         1       4.9           42          1       4.8    */
      MPU6050_LOW_PASS_FILTER_4 = 0x04, /**<       21         1       8.5           20          1       8.3    */
      MPU6050_LOW_PASS_FILTER_5 = 0x05, /**<       10         1      13.8           10          1      13.4    */
      MPU6050_LOW_PASS_FILTER_6 = 0x06, /**<        5         1      19.0            5          1      18.6    */
    } mpu6050_low_pass_filter_t;

    /**
     * @brief mpu6050轴枚举定义
     */
    typedef enum
    {
        MPU6050_AXIS_Z = 0x05, /**< z */
        MPU6050_AXIS_Y = 0x06, /**< y */
        MPU6050_AXIS_X = 0x07, /**< x */
    } mpu6050_axis_t;

    /**
     * @brief mpu6050陀螺仪量程枚举定义
     */
    typedef enum
    {
        MPU6050_GYROSCOPE_RANGE_250DPS = 0x00,  /**< ±250 dps */
        MPU6050_GYROSCOPE_RANGE_500DPS = 0x01,  /**< ±500 dps */
        MPU6050_GYROSCOPE_RANGE_1000DPS = 0x02, /**< ±1000 dps */
        MPU6050_GYROSCOPE_RANGE_2000DPS = 0x03, /**< ±2000 dps */
    } mpu6050_gyroscope_range_t;

    /**
     * @brief mpu6050加速度计量程枚举定义
     */
    typedef enum
    {
        MPU6050_ACCELEROMETER_RANGE_2G = 0x00,  /**< ±2 g */
        MPU6050_ACCELEROMETER_RANGE_4G = 0x01,  /**< ±4 g */
        MPU6050_ACCELEROMETER_RANGE_8G = 0x02,  /**< ±8 g */
        MPU6050_ACCELEROMETER_RANGE_16G = 0x03, /**< ±16 g */
    } mpu6050_accelerometer_range_t;

    /**
     * @brief mpu6050 fifo枚举定义
     */
    typedef enum
    {
        MPU6050_FIFO_TEMP = 0x07,  /**< temperature */
        MPU6050_FIFO_XG = 0x06,    /**< gyroscope x */
        MPU6050_FIFO_YG = 0x05,    /**< gyroscope y */
        MPU6050_FIFO_ZG = 0x04,    /**< gyroscope z */
        MPU6050_FIFO_ACCEL = 0x03, /**< accelerometer */
    } mpu6050_fifo_t;

    /**
     * @brief mpu6050引脚级枚举定义
     */
    typedef enum
    {
        MPU6050_PIN_LEVEL_HIGH = 0x00, /**< active low */
        MPU6050_PIN_LEVEL_LOW = 0x01,  /**< active high */
    } mpu6050_pin_level_t;

    /**
     * @brief mpu6050引脚类型枚举定义
     */
    typedef enum
    {
        MPU6050_PIN_TYPE_PUSH_PULL = 0x00,  /**< push pull */
        MPU6050_PIN_TYPE_OPEN_DRAIN = 0x01, /**< open drain */
    } mpu6050_pin_type_t;

    /**
     * @brief mpu6050中断枚举定义
     */
    typedef enum
    {
        MPU6050_INTERRUPT_MOTION = 6,        /**< motion */
        MPU6050_INTERRUPT_FIFO_OVERFLOW = 4, /**< fifo overflow */
        MPU6050_INTERRUPT_I2C_MAST = 3,      /**< i2c master */
        MPU6050_INTERRUPT_DMP = 1,           /**< dmp */
        MPU6050_INTERRUPT_DATA_READY = 0,    /**< data ready */
    } mpu6050_interrupt_t;

    /**
     * @brief mpu6050 iic从枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_SLAVE_0 = 0x00, /**< slave0 */
        MPU6050_IIC_SLAVE_1 = 0x01, /**< slave1 */
        MPU6050_IIC_SLAVE_2 = 0x02, /**< slave2 */
        MPU6050_IIC_SLAVE_3 = 0x03, /**< slave3 */
        MPU6050_IIC_SLAVE_4 = 0x04, /**< slave4 */
    } mpu6050_iic_slave_t;

    /**
     * @brief mpu6050 iic时钟枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_CLOCK_348_KHZ = 0x00, /**< 348 kHz */
        MPU6050_IIC_CLOCK_333_KHZ = 0x01, /**< 333 kHz */
        MPU6050_IIC_CLOCK_320_KHZ = 0x02, /**< 320 kHz */
        MPU6050_IIC_CLOCK_308_KHZ = 0x03, /**< 308 kHz */
        MPU6050_IIC_CLOCK_296_KHZ = 0x04, /**< 296 kHz */
        MPU6050_IIC_CLOCK_286_KHZ = 0x05, /**< 286 kHz */
        MPU6050_IIC_CLOCK_276_KHZ = 0x06, /**< 276 kHz */
        MPU6050_IIC_CLOCK_267_KHZ = 0x07, /**< 267 kHz */
        MPU6050_IIC_CLOCK_258_KHZ = 0x08, /**< 258 kHz */
        MPU6050_IIC_CLOCK_500_KHZ = 0x09, /**< 500 kHz */
        MPU6050_IIC_CLOCK_471_KHZ = 0x0A, /**< 471 kHz */
        MPU6050_IIC_CLOCK_444_KHZ = 0x0B, /**< 444 kHz */
        MPU6050_IIC_CLOCK_421_KHZ = 0x0C, /**< 421 kHz */
        MPU6050_IIC_CLOCK_400_KHZ = 0x0D, /**< 400 kHz */
        MPU6050_IIC_CLOCK_381_KHZ = 0x0E, /**< 381 kHz */
        MPU6050_IIC_CLOCK_364_KHZ = 0x0F, /**< 364 kHz */
    } mpu6050_iic_clock_t;

    /**
     * @brief mpu6050 iic读取模式枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_READ_MODE_RESTART = 0x00,        /**< restart */
        MPU6050_IIC_READ_MODE_STOP_AND_START = 0x01, /**< stop and start */
    } mpu6050_iic_read_mode_t;

    /**
     * @brief mpu6050 iic模式枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_MODE_WRITE = 0x00, /**< write */
        MPU6050_IIC_MODE_READ = 0x01,  /**< read */
    } mpu6050_iic_mode_t;

    /**
     * @brief mpu6050 iic事务模式枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_TRANSACTION_MODE_DATA = 0x00,     /**< 仅数据 */
        MPU6050_IIC_TRANSACTION_MODE_REG_DATA = 0x01, /**< 在读取或写入数据之前写入寄存器地址 */
    } mpu6050_iic_transaction_mode_t;

    /**
     * @brief mpu6050 iic4事务模式枚举定义
     */
    typedef enum
    {
        MPU6050_IIC4_TRANSACTION_MODE_DATA = 0x00, /**< 仅数据 */
        MPU6050_IIC4_TRANSACTION_MODE_REG = 0x01,  /**< 仅注册 */
    } mpu6050_iic4_transaction_mode_t;

    /**
     * @brief mpu6050 iic组顺序枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_GROUP_ORDER_EVEN = 0x00, /**< 当清除为0时，来自寄存器地址0和1、2和3的字节，等等（偶数、然后奇数寄存器地址）被配对以形成一个字。*/
        MPU6050_IIC_GROUP_ORDER_ODD = 0x01,  /**< 当设置为1时，来自寄存器地址的字节被配对为1和2、3和4，等等（奇数、然后偶数寄存器地址）被配对以形成一个字。*/
    } mpu6050_iic_group_order_t;

    /**
     * @brief mpu6050 iic状态枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_STATUS_PASS_THROUGH = 0x80,  /**< pass through */
        MPU6050_IIC_STATUS_IIC_SLV4_DONE = 0x40, /**< slave4 done */
        MPU6050_IIC_STATUS_IIC_LOST_ARB = 0x20,  /**< lost arbitration */
        MPU6050_IIC_STATUS_IIC_SLV4_NACK = 0x10, /**< slave4 nack */
        MPU6050_IIC_STATUS_IIC_SLV3_NACK = 0x08, /**< slave3 nack */
        MPU6050_IIC_STATUS_IIC_SLV2_NACK = 0x04, /**< slave2 nack */
        MPU6050_IIC_STATUS_IIC_SLV1_NACK = 0x02, /**< slave1 nack */
        MPU6050_IIC_STATUS_IIC_SLV0_NACK = 0x01, /**< slave0 nack */
    } mpu6050_iic_status_t;

    /**
     * @brief mpu6050 iic延迟枚举定义
     */
    typedef enum
    {
        MPU6050_IIC_DELAY_ES_SHADOW = 7, /**< delays shadowing of external sensor data until
                                              all data has been received */
        MPU6050_IIC_DELAY_SLAVE_4 = 4,   /**< slave 4 */
        MPU6050_IIC_DELAY_SLAVE_3 = 3,   /**< slave 3 */
        MPU6050_IIC_DELAY_SLAVE_2 = 2,   /**< slave 2 */
        MPU6050_IIC_DELAY_SLAVE_1 = 1,   /**< slave 1 */
        MPU6050_IIC_DELAY_SLAVE_0 = 0,   /**< slave 0 */
    } mpu6050_iic_delay_t;

    /**
     * @}
     */

    /**
     * @addtogroup mpu6050_dmp_driver
     * @{
     */

    /**
     * @brief mpu6050 dmp中断模式枚举定义
     */
    typedef enum
    {
        MPU6050_DMP_INTERRUPT_MODE_CONTINUOUS = 0x00, /**< continuous mode */
        MPU6050_DMP_INTERRUPT_MODE_GESTURE = 0x01,    /**< gesture mode */
    } mpu6050_dmp_interrupt_mode_t;

    /**
     * @brief mpu6050 dmp功能枚举定义
     */
    typedef enum
    {
        MPU6050_DMP_FEATURE_TAP = 0x001,            /**< feature tap */
        MPU6050_DMP_FEATURE_ORIENT = 0x002,         /**< feature orient */
        MPU6050_DMP_FEATURE_3X_QUAT = 0x004,        /**< feature 3x quat */
        MPU6050_DMP_FEATURE_PEDOMETER = 0x008,      /**< feature pedometer */
        MPU6050_DMP_FEATURE_6X_QUAT = 0x010,        /**< feature 6x quat */
        MPU6050_DMP_FEATURE_GYRO_CAL = 0x020,       /**< feature gyro cal */
        MPU6050_DMP_FEATURE_SEND_RAW_ACCEL = 0x040, /**< feature send raw accel */
        MPU6050_DMP_FEATURE_SEND_RAW_GYRO = 0x080,  /**< feature send raw gyro */
        MPU6050_DMP_FEATURE_SEND_CAL_GYRO = 0x100,  /**< feature send cal gyro */
    } mpu6050_dmp_feature_t;

    /**
     * @brief mpu6050 dmp抽头枚举定义
     */
    typedef enum
    {
        MPU6050_DMP_TAP_X_UP = 0x01,   /**< tap x up */
        MPU6050_DMP_TAP_X_DOWN = 0x02, /**< tap x down */
        MPU6050_DMP_TAP_Y_UP = 0x03,   /**< tap y up */
        MPU6050_DMP_TAP_Y_DOWN = 0x04, /**< tap y down */
        MPU6050_DMP_TAP_Z_UP = 0x05,   /**< tap z up */
        MPU6050_DMP_TAP_Z_DOWN = 0x06, /**< tap z down */
    } mpu6050_dmp_tap_t;

    /**
     * @brief mpu6050 dmp定向枚举定义
     */
    typedef enum
    {
        MPU6050_DMP_ORIENT_PORTRAIT = 0x00,          /**< portrait */
        MPU6050_DMP_ORIENT_LANDSCAPE = 0x01,         /**< landscape */
        MPU6050_DMP_ORIENT_REVERSE_PORTRAIT = 0x02,  /**< reverse portrait */
        MPU6050_DMP_ORIENT_REVERSE_LANDSCAPE = 0x03, /**< reverse landscape */
    } mpu6050_dmp_orient_t;

    /**
     * @}
     */

    /**
     * @addtogroup mpu6050_basic_driver
     * @{
     */

    /**
     * @brief mpu6050句柄结构定义
     */
    typedef struct mpu6050_handle_s
    {
        uint8_t iic_addr;                                                            /**< iic设备地址 */
        uint8_t (*iic_init)(void);                                                   /**< 指向iic_init函数地址 */
        uint8_t (*iic_deinit)(void);                                                 /**< 指向iic_deinit函数地址 */
        uint8_t (*iic_read)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);  /**< 指向iic_read函数地址 */
        uint8_t (*iic_write)(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len); /**< 指向iic_write函数地址 */
        void (*delay_ms)(uint32_t ms);                                               /**< 指向delay_ms函数地址 */
        void (*debug_print)(const char *const fmt, ...);                             /**< 指向debug_print函数地址 */
        void (*receive_callback)(uint8_t type);                                      /**< 指向receive_callback函数地址 */
        void (*dmp_tap_callback)(uint8_t count, uint8_t direction);                  /**< 指向dmp_tap_callback函数地址 */
        void (*dmp_orient_callback)(uint8_t orientation);                            /**< 指向dmp_orient_callback函数地址 */
        uint8_t inited;                                                              /**< inited标志 */
        uint8_t dmp_inited;                                                          /**< dmp inited标志 */
        uint16_t orient;                                                             /**< 朝向 */
        uint16_t mask;                                                               /**< mask */
        uint8_t buf[1024];                                                           /**< 内部缓冲器 */
    } mpu6050_handle_t;

    /**
     * @brief mpu6050信息结构定义
     */
    typedef struct mpu6050_info_s
    {
        char chip_name[32];         /**< 芯片名称 */
        char manufacturer_name[32]; /**< 制造商名称 */
        char interface[8];          /**< 芯片接口名称 */
        float supply_voltage_min_v; /**< 芯片最小供电电压 */
        float supply_voltage_max_v; /**< 芯片最大供电电压 */
        float max_current_ma;       /**< 芯片最大电流 */
        float temperature_min;      /**< 芯片最低工作温度 */
        float temperature_max;      /**< 芯片最高工作温度 */
        uint32_t driver_version;    /**< 驱动程序版本 */
    } mpu6050_info_t;

/**
 * @}
 */

/**
 *@defgroup mpu6050_link_driver mpu6050链接驱动程序功能
 *@brief mpu6050链接驱动程序模块
 *@ingroup mpu6050_driver
 * @{
 */
/**
 *@brief初始化mpu6050_handle_t结构
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]STRUCTURE是mpu6050_handle_t
 *@note none
 */
#define DRIVER_MPU6050_LINK_INIT(HANDLE, STRUCTURE) memset(HANDLE, 0, sizeof(STRUCTURE))

/**
 *@brief link iic_init函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向iic_init函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_IIC_INIT(HANDLE, FUC) (HANDLE)->iic_init = FUC

/**
 *@brief link iic_deinit函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向iic_deinit函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_IIC_DEINIT(HANDLE, FUC) (HANDLE)->iic_deinit = FUC

/**
 *@brief link iic_read函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向iic_read函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_IIC_READ(HANDLE, FUC) (HANDLE)->iic_read = FUC

/**
 *@brief link iic_write函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向iic_write函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_IIC_WRITE(HANDLE, FUC) (HANDLE)->iic_write = FUC

/**
 *@brief link delay_ms函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向delay_ms函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_DELAY_MS(HANDLE, FUC) (HANDLE)->delay_ms = FUC

/**
 *@brief link debug_print函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向debug_print函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_DEBUG_PRINT(HANDLE, FUC) (HANDLE)->debug_print = FUC

/**
 *@brief link receive_callback函数
 *@param[in]HANDLE指向mpu6050句柄结构
 *@param[in]FUC指向receive_callback函数地址
 *@note none
 */
#define DRIVER_MPU6050_LINK_RECEIVE_CALLBACK(HANDLE, FUC) (HANDLE)->receive_callback = FUC

    /*----------------------------------function----------------------------------*/

    /**
     *@defgroup mpu6050_basic_driver mpu6050基本驱动程序功能
     *@brief mpu6050基本驱动程序模块
     *@ingroup mpu6050_driver
     * @{
     */
    /**
     *@brief获取芯片信息
     *@param[out]*info指向一个mpu6050信息结构
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *@note none
     */
    uint8_t mpu6050_info(mpu6050_info_t *info);

    /**
     *@brief设置芯片地址引脚
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]addr_pin是芯片地址引脚
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *@note none
     */
    uint8_t mpu6050_set_addr_pin(mpu6050_handle_t *handle, mpu6050_address_t addr_pin);

    /**
     *@brief获取芯片地址引脚
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*addr_pin指向芯片地址引脚缓冲区
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *@note none
     */
    uint8_t mpu6050_get_addr_pin(mpu6050_handle_t *handle, mpu6050_address_t *addr_pin);

    /**
     *@brief irq处理程序
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1次运行失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_irq_handler(mpu6050_handle_t *handle);

    /**
     *@brief初始化芯片
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1 iic初始化失败
     *-2句柄为NULL
     *-3个链接函数为NULL
     *-4复位失败
     *-5 id无效
     *@note none
     */
    uint8_t mpu6050_init(mpu6050_handle_t *handle);

    /**
     *@brief关闭芯片
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1 iic卸载失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4进入睡眠模式失败
     *@note none
     */
    uint8_t mpu6050_deinit(mpu6050_handle_t *handle);

    /**
     *@brief读取数据
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]**accel_raw指向一个accel原始数据缓冲区
     *@param[out]**accel_g指向已转换的accel数据缓冲区
     *@param[out]**gyro_raw指向陀螺仪原始数据缓冲区
     *@param[out]**gyro_dps指向转换后的陀螺仪数据缓冲区
     *@param[in，out]*len指向一个长度缓冲区
     *@返回状态代码
     *-0成功
     *-1读取失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4长度为零
     *-5 dmp正在运行
     *-6 fifo conf错误
     *@note none
     */
    uint8_t mpu6050_read(mpu6050_handle_t *handle, int16_t (*accel_raw)[3], float (*accel_g)[3],
                         int16_t (*gyro_raw)[3], float (*gyro_dps)[3], uint16_t *len);

    /**
     *@brief读取温度
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*raw指向原始数据缓冲区
     *@param[out]*degree指向转换后的degree数据缓冲区
     *@返回状态代码
     *-0成功
     *-1读取失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_read_temperature(mpu6050_handle_t *handle, int16_t(*raw), float *degrees);

    /**
     *@brief启用或禁用fifo
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1套fifo失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_fifo(mpu6050_handle_t *handle, mpu6050_bool_t enable);
    /**
     *@brief获取fifo状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取fifo失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_fifo(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief force重置fifo
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1强制fifo重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_force_fifo_reset(mpu6050_handle_t *handle);

    /**
     *@brief启用或禁用iic主模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1套iic主机失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_master(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取iic主机状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic主机失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_master(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief重置fifo
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1 fifo重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_fifo_reset(mpu6050_handle_t *handle);

    /**
     *@brief获取fifo重置状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取fifo重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_fifo_reset(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief重置iic主控制器
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1 iic主重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_iic_master_reset(mpu6050_handle_t *handle);

    /**
     *@brief获取iic主机重置状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic主重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_master_reset(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief重置所有传感器
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1传感器复位失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_sensor_reset(mpu6050_handle_t *handle);

    /**
     *@brief获取传感器重置状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取传感器重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_sensor_reset(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief重置芯片
     *@param[in]*handle指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1设备重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_device_reset(mpu6050_handle_t *handle);

    /**
     *@brief获取设备重置状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取设备重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_device_reset(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief设置芯片时钟源
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]clock_source是芯片主时钟源
     *@返回状态代码
     *-0成功
     *-1组时钟源失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_clock_source(mpu6050_handle_t *handle, mpu6050_clock_source_t clock_source);

    /**
     *@brief获取芯片时钟源
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*clock_source指向时钟源缓冲区
     *@返回状态代码
     *-0成功
     *-1获取时钟源失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_clock_source(mpu6050_handle_t *handle, mpu6050_clock_source_t *clock_source);

    /**
     *@brief启用或禁用温度传感器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组温度传感器故障
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_temperature_sensor(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取温度传感器状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取温度传感器故障
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_temperature_sensor(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用循环唤醒模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组循环唤醒失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_cycle_wake_up(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取循环唤醒模式状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取循环唤醒失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_cycle_wake_up(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用睡眠模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组睡眠失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_sleep(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取睡眠状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1睡眠失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_sleep(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief将源设置为待机模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]source是输入源
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1设置待机模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_standby_mode(mpu6050_handle_t *handle, mpu6050_source_t source, mpu6050_bool_t enable);

    /**
     *@brief获取源模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]source是输入源
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取待机模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_standby_mode(mpu6050_handle_t *handle, mpu6050_source_t source, mpu6050_bool_t *enable);

    /**
     *@brief设置唤醒频率
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]frequency是唤醒频率
     *@返回状态代码
     *-0成功
     *-1设置唤醒频率失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_wake_up_frequency(mpu6050_handle_t *handle, mpu6050_wake_up_frequency_t frequency);

    /**
     *@brief获取唤醒频率
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*frequency指向唤醒频率缓冲区
     *@返回状态代码
     *-0成功
     *-1获取唤醒频率失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_wake_up_frequency(mpu6050_handle_t *handle, mpu6050_wake_up_frequency_t *frequency);

    /**
     *@brief获取fifo计数器值
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*count指向fifo计数缓冲区
     *@返回状态代码
     *-0成功
     *-1获取fifo计数失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_fifo_count(mpu6050_handle_t *handle, uint16_t *count);

    /**
     *@brief fifo读取字节
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*buf指向一个数据缓冲区
     *@param[in]len是缓冲区长度
     *@返回状态代码
     *-0成功
     *-1个fifo读取失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_fifo_get(mpu6050_handle_t *handle, uint8_t *buf, uint16_t len);

    /**
     *@brief fifo写入字节
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]*buf指向数据缓冲区
     *@param[in]len是缓冲区长度
     *@返回状态代码
     *-0成功
     *-1个fifo写入失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_fifo_set(mpu6050_handle_t *handle, uint8_t *buf, uint16_t len);

    /**
     *@brief设置信号路径重置
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]path是信号路径
     *@返回状态代码
     *-0成功
     *-1设置信号路径重置失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_signal_path_reset(mpu6050_handle_t *handle, mpu6050_signal_path_reset_t path);

    /**
     *@brief设置采样速率分配器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]d是采样速率除法器
     *@返回状态代码
     *-0成功
     *-1套采样速率分配器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_sample_rate_divider(mpu6050_handle_t *handle, uint8_t d);

    /**
     *@brief获取采样速率分配器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*d指向采样速率分配器缓冲区
     *@返回状态代码
     *-0成功
     *-1获取采样速率分配器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_sample_rate_divider(mpu6050_handle_t *handle, uint8_t *d);

    /**
     *@brief设置外部同步类型
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]sync是外部同步类型
     *@返回状态代码
     *-0成功
     *-1集外部同步失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_extern_sync(mpu6050_handle_t *handle, mpu6050_extern_sync_t sync);

    /**
     *@brief获取外部同步类型
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*sync指向外部同步类型缓冲区
     *@返回状态代码
     *-0成功
     *-1获取外部同步失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_extern_sync(mpu6050_handle_t *handle, mpu6050_extern_sync_t *sync);

    /**
     *@brief设置低通滤波器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]filter是低通滤波器
     *@返回状态代码
     *-0成功
     *-1套低通滤波器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_low_pass_filter(mpu6050_handle_t *handle, mpu6050_low_pass_filter_t filter);

    /**
     *@brief获取低通滤波器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*filter指向一个低通滤波器缓冲区
     *@返回状态代码
     *-0成功
     *-1获取低通滤波器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_low_pass_filter(mpu6050_handle_t *handle, mpu6050_low_pass_filter_t *filter);

    /**
     *@brief设置陀螺仪测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]轴是测试的轴
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1套陀螺仪测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_gyroscope_test(mpu6050_handle_t *handle, mpu6050_axis_t axis, mpu6050_bool_t enable);

    /**
     *@brief获取陀螺仪测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]轴是测试的轴
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取陀螺仪测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_gyroscope_test(mpu6050_handle_t *handle, mpu6050_axis_t axis, mpu6050_bool_t *enable);

    /**
     *@brief设置陀螺仪范围
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]range是陀螺仪的范围
     *@返回状态代码
     *-0成功
     *-1组陀螺仪范围失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_gyroscope_range(mpu6050_handle_t *handle, mpu6050_gyroscope_range_t range);

    /**
     *@brief获取陀螺仪范围
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*范围指向陀螺仪范围缓冲区
     *@返回状态代码
     *-0成功
     *-1获取陀螺仪范围失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_gyroscope_range(mpu6050_handle_t *handle, mpu6050_gyroscope_range_t *range);

    /**
     *@brief设置加速度计测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]轴是测试的轴
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组加速度计测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_accelerometer_test(mpu6050_handle_t *handle, mpu6050_axis_t axis, mpu6050_bool_t enable);

    /**
     *@brief获取加速度计测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]轴是测试的轴
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取加速度计测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_accelerometer_test(mpu6050_handle_t *handle, mpu6050_axis_t axis, mpu6050_bool_t *enable);

    /**
     *@brief设置加速度计范围
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]range是加速度计的范围
     *@返回状态代码
     *-0成功
     *-1组加速度计范围失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_accelerometer_range(mpu6050_handle_t *handle, mpu6050_accelerometer_range_t range);

    /**
     *@brief获取加速度计范围
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*范围指向加速度计范围缓冲区
     *@返回状态代码
     *-0成功
     *-1获取加速计范围失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_accelerometer_range(mpu6050_handle_t *handle, mpu6050_accelerometer_range_t *range);

    /**
     *@brief启用或禁用fifo功能
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]fifo是fifo类型
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组fifo启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_fifo_enable(mpu6050_handle_t *handle, mpu6050_fifo_t fifo, mpu6050_bool_t enable);

    /**
     *@brief获取fifo功能状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]fifo是fifo类型
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取fifo启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_fifo_enable(mpu6050_handle_t *handle, mpu6050_fifo_t fifo, mpu6050_bool_t *enable);

    /**
     *@brief设置中断级别
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]level是中断级别
     *@返回状态代码
     *-0成功
     *-1设置中断级别失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_interrupt_level(mpu6050_handle_t *handle, mpu6050_pin_level_t level);

    /**
     *@brief获取中断级别
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*level指向中断级缓冲区
     *@返回状态代码
     *-0成功
     *-1获取中断级别失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_interrupt_level(mpu6050_handle_t *handle, mpu6050_pin_level_t *level);

    /**
     *@brief设置中断引脚类型
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]type是中断引脚类型
     *@返回状态代码
     *-0成功
     *-1设置中断引脚类型失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_interrupt_pin_type(mpu6050_handle_t *handle, mpu6050_pin_type_t type);

    /**
     *@brief获取中断引脚类型
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*type指向pin类型的缓冲区
     *@返回状态代码
     *-0成功
     *-1获取中断引脚类型失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_interrupt_pin_type(mpu6050_handle_t *handle, mpu6050_pin_type_t *type);

    /**
     *@brief启用或禁用中断锁存器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1设置中断锁存失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_interrupt_latch(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取中断锁存状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取中断锁存失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_interrupt_latch(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用中断读取清除
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1设置中断读取清除失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_interrupt_read_clear(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取中断读取清除状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取中断读取清除失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_interrupt_read_clear(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief设置fsync中断级别
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]level是设置的级别
     *@返回状态代码
     *-0成功
     *-1设置fsync中断级别失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_fsync_interrupt_level(mpu6050_handle_t *handle, mpu6050_pin_level_t level);

    /**
     *@brief获取fsync中断级别
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*level指向一个设置级别的缓冲区
     *@返回状态代码
     *-0成功
     *-1获取fsync中断级别失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_fsync_interrupt_level(mpu6050_handle_t *handle, mpu6050_pin_level_t *level);

    /**
     *@brief启用或禁用fsync中断
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1设置fsync中断失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_fsync_interrupt(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取fsync中断状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取fsync中断失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_fsync_interrupt(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用iic旁路
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1套iic旁路失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_bypass(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取iic旁路状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic旁路失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_bypass(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用中断
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]type是设置的中断类型
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1设置中断失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_interrupt(mpu6050_handle_t *handle, mpu6050_interrupt_t type, mpu6050_bool_t enable);

    /**
     *@brief获取中断状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]type是设置的中断类型
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取中断失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_interrupt(mpu6050_handle_t *handle, mpu6050_interrupt_t type, mpu6050_bool_t *enable);

    /**
     *@brief获取中断状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*status指向一个状态缓冲区
     *@返回状态代码
     *-0成功
     *-1获取中断状态失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_interrupt_status(mpu6050_handle_t *handle, uint8_t *status);

    /**
     *@brief设置陀螺仪x测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1套陀螺仪x测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4数据超过0x1F
     *@note none
     */
    uint8_t mpu6050_set_gyroscope_x_test(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取陀螺仪x测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取陀螺仪x测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_gyroscope_x_test(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief设置陀螺仪y测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1套陀螺仪y测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4数据超过0x1F
     *@note none
     */
    uint8_t mpu6050_set_gyroscope_y_test(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取陀螺仪y测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取陀螺仪y测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_gyroscope_y_test(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief设置陀螺仪z测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1套陀螺仪z测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4数据超过0x1F
     *@note none
     */
    uint8_t mpu6050_set_gyroscope_z_test(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取陀螺仪z测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取陀螺仪z测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_gyroscope_z_test(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief设置加速度计x测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1组加速度计x测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4数据超过0x1F
     *@note none
     */
    uint8_t mpu6050_set_accelerometer_x_test(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取加速度计x测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取加速度计x测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_accelerometer_x_test(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief设置加速度计y测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1组加速度计y测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4数据超过0x1F
     *@note none
     */
    uint8_t mpu6050_set_accelerometer_y_test(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取加速度计y测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取加速度计y测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_accelerometer_y_test(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief设置加速度计z测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1组加速度计z测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4数据超过0x1F
     *@note none
     */
    uint8_t mpu6050_set_accelerometer_z_test(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取加速度计z测试
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取加速度计z测试失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_accelerometer_z_test(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief设置动作阈值
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]threshold是设置的阈值
     *@返回状态代码
     *-0成功
     *-1设置运动阈值失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_motion_threshold(mpu6050_handle_t *handle, uint8_t threshold);

    /**
     *@brief获取动作阈值
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*threshold指向阈值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取运动阈值失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_motion_threshold(mpu6050_handle_t *handle, uint8_t *threshold);

    /**
     *@brief将运动阈值转换为寄存器原始数据
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]mg是运动阈值
     *@param[out]*reg指向一个寄存器原始缓冲区
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_motion_threshold_convert_to_register(mpu6050_handle_t *handle, float mg, uint8_t *reg);

    /**
     *@brief将寄存器原始数据转换为运动阈值
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]reg是寄存器的原始数据
     *@param[out]*mg指向运动阈值缓冲区
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_motion_threshold_convert_to_data(mpu6050_handle_t *handle, uint8_t reg, float *mg);

    /**
     *@brief设置动作持续时间
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]duration是设置的持续时间
     *@返回状态代码
     *-0成功
     *-1设置运动持续时间失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_motion_duration(mpu6050_handle_t *handle, uint8_t duration);

    /**
     *@brief获取运动持续时间
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*duration指向一个duration缓冲区
     *@返回状态代码
     *-0成功
     *-1获取运动持续时间失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_motion_duration(mpu6050_handle_t *handle, uint8_t *duration);

    /**
     *@brief将运动持续时间转换为寄存器原始数据
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]ms是运动持续时间
     *@param[out]*reg指向一个寄存器原始缓冲区
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_motion_duration_convert_to_register(mpu6050_handle_t *handle, uint8_t ms, uint8_t *reg);

    /**
     *@brief将寄存器原始数据转换为运动持续时间
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]reg是寄存器的原始数据
     *@param[out]*ms指向运动持续时间缓冲区
     *@返回状态代码
     *-0成功
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_motion_duration_convert_to_data(mpu6050_handle_t *handle, uint8_t reg, uint8_t *ms);

    /**
     *@brief启用或禁用强制加速样本
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组力加速样本失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_force_accel_sample(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief运行自检
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*gyro_offset_raw指向陀螺仪偏移原始缓冲区
     *@param[out]*accel_offset_raw指向一个accel offset原始缓冲区
     *@返回状态代码
     *-0成功
     *-1自检失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_self_test(mpu6050_handle_t *handle, int32_t gyro_offset_raw[3], int32_t accel_offset_raw[3]);

    /**
     *@brief设置iic时钟
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]clk是iic时钟
     *@返回状态代码
     *-0成功
     *-1组iic时钟失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_clock(mpu6050_handle_t *handle, mpu6050_iic_clock_t clk);

    /**
     *@brief获取iic时钟
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*clk指向iic时钟缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic时钟失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_clock(mpu6050_handle_t *handle, mpu6050_iic_clock_t *clk);

    /**
     *@brief启用或禁用iic多主机
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1套iic多主机失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_multi_master(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取iic多主机状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic多主机失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_multi_master(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用iic等待外部传感器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组iic等待外部传感器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_wait_for_external_sensor(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取iic等待外部传感器状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic等待外部传感器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_wait_for_external_sensor(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief设置iic读取模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]模式是读取模式
     *@返回状态代码
     *-0成功
     *-1设置iic读取模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_read_mode(mpu6050_handle_t *handle, mpu6050_iic_read_mode_t mode);

    /**
     *@brief获取iic读取模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*mode指向读取模式缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic读取模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_read_mode(mpu6050_handle_t *handle, mpu6050_iic_read_mode_t *mode);

    /**
     *@brief启用或禁用iic-fifo
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组iic fifo启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_fifo_enable(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_bool_t enable);

    /**
     *@brief获取iic-fifo状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic fifo启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_fifo_enable(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_bool_t *enable);

    /**
     *@brief设置iic模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]模式是iic模式
     *@返回状态代码
     *-0成功
     *-1套iic模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_mode(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_iic_mode_t mode);

    /**
     *@brief获取iic模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*mode指向iic模式缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_mode(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_iic_mode_t *mode);

    /**
     *@brief设置iic地址
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]addr_7bit是iic地址
     *@返回状态代码
     *-0成功
     *-1设置iic地址失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_address(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t addr_7bit);

    /**
     *@brief获取iic地址
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*addr_7bit指向iic地址缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic地址失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_address(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t *addr_7bit);

    /**
     *@brief设置iic寄存器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]reg是iic寄存器
     *@返回状态代码
     *-0成功
     *-1组iic寄存器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_register(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t reg);
    /**
     *@brief获取iic寄存器
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*reg指向iic寄存器缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic寄存器失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_register(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t *reg);

    /**
     *@brief设置iic数据
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]data是集合数据
     *@返回状态代码
     *-0成功
     *-1组iic数据输出失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_data_out(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t data);

    /**
     *@brief获取iic数据
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*data指向一个设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic数据失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_data_out(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t *data);

    /**
     *@brief启用或禁用iic
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1集iic启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_enable(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_bool_t enable);

    /**
     *@brief获取iic状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_enable(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用iic字节交换
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组iic字节交换失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_byte_swap(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_bool_t enable);

    /**
     *@brief获取iic字节交换状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic字节交换失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_byte_swap(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_bool_t *enable);

    /**
     *@brief设置iic交易模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]模式是iic事务模式
     *@返回状态代码
     *-0成功
     *-1设置iic事务模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_transaction_mode(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_iic_transaction_mode_t mode);

    /**
     *@brief获取iic交易模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*mode指向iic事务模式缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic事务模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_transaction_mode(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_iic_transaction_mode_t *mode);

    /**
     *@brief设置iic组顺序
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]order是组顺序
     *@返回状态代码
     *-0成功
     *-1组iic组订单失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_set_iic_group_order(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_iic_group_order_t order);
    /**
     *@brief获取iic组订单
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*order指向一个组订单缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic组订单失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_group_order(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, mpu6050_iic_group_order_t *order);

    /**
     *@brief设置iic传输长度
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[in]len是iic传输的长度
     *@返回状态代码
     *-0成功
     *-1套iic传输len失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *-5 len>0xF
     *@note none
     */
    uint8_t mpu6050_set_iic_transferred_len(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t len);

    /**
     *@brief获取iic传输长度
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]slave是iic slave编号
     *@param[out]*len指向iic传输长度缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic传输len失败
     *-2句柄为NULL
     *-3句柄未初始化
     *-4个无效从属
     *@note none
     */
    uint8_t mpu6050_get_iic_transferred_len(mpu6050_handle_t *handle, mpu6050_iic_slave_t slave, uint8_t *len);

    /**
     *@brief获取iic状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*status指向一个状态缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic状态失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_status(mpu6050_handle_t *handle, uint8_t *status);

    /**
     *@brief启用或禁用iic延迟
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]delay是iic延迟
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组iic延迟启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic_delay_enable(mpu6050_handle_t *handle, mpu6050_iic_delay_t delay, mpu6050_bool_t enable);

    /**
     *@brief获取iic延迟状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]delay是iic延迟
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic延迟启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic_delay_enable(mpu6050_handle_t *handle, mpu6050_iic_delay_t delay, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用iic4
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1设置iic4启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic4_enable(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取iic4状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic4启用失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic4_enable(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief启用或禁用iic4中断
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]enable是布尔值
     *@返回状态代码
     *-0成功
     *-1组iic4中断失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic4_interrupt(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     *@brief获取iic4中断状态
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*enable指向布尔值缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic4中断失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic4_interrupt(mpu6050_handle_t *handle, mpu6050_bool_t *enable);

    /**
     *@brief设置iic4交易模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[in]模式是事务模式
     *@返回状态代码
     *-0成功
     *-1设置iic4事务模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_set_iic4_transaction_mode(mpu6050_handle_t *handle, mpu6050_iic4_transaction_mode_t mode);

    /**
     *@brief获取iic4交易模式
     *@param[in]*handle指向mpu6050句柄结构
     *@param[out]*mode指向事务模式缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic4事务模式失败
     *-2句柄为NULL
     *-3句柄未初始化
     *@note none
     */
    uint8_t mpu6050_get_iic4_transaction_mode(mpu6050_handle_t *handle, mpu6050_iic4_transaction_mode_t *mode);
    /**
     *@brief设置iic延迟
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]delay是iic延迟
     *@返回状态代码
     *-0成功
     *-1组iic延迟失败
     *-2句柄为空
     *-3句柄未初始化
     *-4延迟>0x1F
     *@注释无
     */
    uint8_t mpu6050_set_iic_delay(mpu6050_handle_t *handle, uint8_t delay);

    /**
     *@brief获取iic延迟
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*延迟指向iic延迟缓冲器
     *@返回状态代码
     *-0成功
     *-1获取iic延迟失败
     *-2句柄为空
     *-3句柄未初始化
     *@注释无
     */
    uint8_t mpu6050_get_iic_delay(mpu6050_handle_t *handle, uint8_t *delay);

    /**
     *@brief设置iic4数据
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]data是设置的数据
     *@返回状态代码
     *-0成功
     *-1组iic4数据输出失败
     *-2句柄为空
     *-3句柄未初始化
     *@注释无
     */
    uint8_t mpu6050_set_iic4_data_out(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief取出iic4数据
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*数据指向设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1获取iic4数据输出失败
     *-2句柄为空
     *-3句柄未初始化
     *@注释无
     */
    uint8_t mpu6050_get_iic4_data_out(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@brief在中设置iic4数据
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]data是设置的数据
     *@返回状态代码
     *-0成功
     *-1个设置iic4数据失败
     *-2句柄为空
     *-3句柄未初始化
     *@注释无
     */
    uint8_t mpu6050_set_iic4_data_in(mpu6050_handle_t *handle, uint8_t data);

    /**
     *@brief获取iic4数据
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*数据指向设置的数据缓冲区
     *@返回状态代码
     *-0成功
     *-1在中获取iic4数据失败
     *-2句柄为空
     *-3句柄未初始化
     *@注释无
     */
    uint8_t mpu6050_get_iic4_data_in(mpu6050_handle_t *handle, uint8_t *data);

    /**
     *@简要读取外部传感器数据
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*数据指向数据缓冲区
     *@param[in]len是数据长度
     *@返回状态代码
     *-0成功
     *-1读取外部传感器数据失败
     *-2句柄为空
     *-3句柄未初始化
     *-4长度>24
     *@注释无
     */
    uint8_t mpu6050_read_extern_sensor_data(mpu6050_handle_t *handle, uint8_t *data, uint8_t len);

    /**
     *“@}”
     */
    /**
     *@defgroup mpu6050_dmp_driver mpu6050 dmp驱动程序函数
     *@brief mpu6050 dmp驱动程序模块
     *@ingroup mpu6050_driver
     *@{
     */
    /**
     *@brief加载dmp固件
     *@param[in]*句柄指向mpu6050句柄结构
     *@返回状态代码
     *-0成功
     *-1加载固件失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp正在运行
     *-5代码比较错误
     *-6套程序启动失败
     *@注释无
     */
    uint8_t mpu6050_dmp_load_firmware(mpu6050_handle_t *handle);

    /**
     *@brief dmp设置计步器行走时间
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]ms是行走时间
     *@返回状态代码
     *-0成功
     *-1 dmp设置计步器行走时间失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_pedometer_walk_time(mpu6050_handle_t *handle, uint32_t ms);

    /**
     *@brief dmp获取计步器步行时间
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*ms指向行走时间缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取计步器行走时间失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_pedometer_walk_time(mpu6050_handle_t *handle, uint32_t *ms);

    /**
     *@brief dmp设置计步器步数
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]count是步长计数
     *@返回状态代码
     *-0成功
     *-1 dmp设置计步器步数失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_pedometer_step_count(mpu6050_handle_t *handle, uint32_t count);

    /**
     *@brief dmp获取计步器步数
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*count指向步进计数缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取计步器步长计数失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_pedometer_step_count(mpu6050_handle_t *handle, uint32_t *count);

    /**
     *@brief dmp设置抖动拒绝超时
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]ms是拒绝超时
     *@返回状态代码
     *-0成功
     *-1 dmp设置抖动拒绝超时失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_shake_reject_timeout(mpu6050_handle_t *handle, uint16_t ms);

    /**
     *@brief dmp获取抖动拒绝超时
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*ms指向拒绝超时缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取抖动拒绝超时失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_shake_reject_timeout(mpu6050_handle_t *handle, uint16_t *ms);

    /**
     *@brief dmp设置抖动拒绝时间
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]ms是抖动拒绝时间
     *@返回状态代码
     *-0成功
     *-1 dmp设置抖动拒绝时间失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_shake_reject_time(mpu6050_handle_t *handle, uint16_t ms);

    /**
     *@brief dmp获取抖动拒绝时间
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*ms指向抖动拒绝时间缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取抖动拒绝时间失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_shake_reject_time(mpu6050_handle_t *handle, uint16_t *ms);

    /**
     *@brief dmp设置抖动拒绝阈值
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]dps是抖动拒绝阈值
     *@返回状态代码
     *-0成功
     *-1 dmp设置抖动拒绝阈值失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_shake_reject_thresh(mpu6050_handle_t *handle, uint16_t dps);

    /**
     *@brief dmp获得抖动拒绝阈值
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*dps指向抖动拒绝阈值dps缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取抖动拒绝阈值失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_shake_reject_thresh(mpu6050_handle_t *handle, uint16_t *dps);

    /**
     *@brief dmp设置抽头之间的最大时间以注册为多抽头
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]ms是延迟时间
     *@返回状态代码
     *-0成功
     *-1 dmp设置抽头时间多重失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_tap_time_multi(mpu6050_handle_t *handle, uint16_t ms);

    /**
     *@brief dmp获取点击之间的最大时间以注册为多点击
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*ms指向延迟时间缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取抽头时间多重失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_tap_time_multi(mpu6050_handle_t *handle, uint16_t *ms);

    /**
     *@brief dmp设置抽头时间
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]ms是抽头时间
     *@返回状态代码
     *-0成功
     *-1 dmp设置抽头时间失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_tap_time(mpu6050_handle_t *handle, uint16_t ms);

    /**
     *@brief dmp获取抽头时间
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*ms指向抽头时间缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取抽头时间失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_tap_time(mpu6050_handle_t *handle, uint16_t *ms);

    /**
     *@brief dmp设置最小抽头计数
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]cnt是抽头计数器
     *@返回状态代码
     *-0成功
     *-1 dmp设置最小抽头计数失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *-5 cnt必须介于1和4之间
     *@注释1<=cnt<=4
     */
    uint8_t mpu6050_dmp_set_min_tap_count(mpu6050_handle_t *handle, uint8_t cnt);

    /**
     *@brief dmp获取最小点击计数
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[out]*cnt指向抽头计数器缓冲区
     *@返回状态代码
     *-0成功
     *-1 dmp获取最小抽头计数失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_get_min_tap_count(mpu6050_handle_t *handle, uint8_t *cnt);

    /**
     *@brief dmp启用或禁用攻丝轴
     *@param[in]*句柄指向mpu6050句柄结构
     *@param[in]轴是设置的轴
     *@param[in]enable是一个布尔值
     *@返回状态代码
     *-0成功
     *-1 dmp设置抽头轴失败
     *-2句柄为空
     *-3句柄未初始化
     *-4 dmp未初始化
     *@注释无
     */
    uint8_t mpu6050_dmp_set_tap_axes(mpu6050_handle_t *handle, mpu6050_axis_t axis, mpu6050_bool_t enable);

    /**
     * @brief      dmp get the tap axes status
     * @param[in]  *handle points to an mpu6050 handle structure
     * @param[in]  axis is the set axis
     * @param[out] *enable points to a bool value buffer
     * @return     status code
     *             - 0 success
     *             - 1 dmp get tap axes failed
     *             - 2 handle is NULL
     *             - 3 handle is not initialized
     *             - 4 dmp is not inited
     * @note       none
     */
    uint8_t mpu6050_dmp_get_tap_axes(mpu6050_handle_t *handle, mpu6050_axis_t axis, mpu6050_bool_t *enable);

    /**
     * @brief     dmp set the tap thresh
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] axis is the set axis
     * @param[in] mg_ms is the set thresh
     * @return    status code
     *            - 0 success
     *            - 1 dmp set tap thresh failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     *            - 5 mg/ms > 1600
     *            - 6 invalid axis
     * @note      none
     */
    uint8_t mpu6050_dmp_set_tap_thresh(mpu6050_handle_t *handle, mpu6050_axis_t axis, uint16_t mg_ms);

    /**
     * @brief      dmp get the tap thresh
     * @param[in]  *handle points to an mpu6050 handle structure
     * @param[in]  axis is the set axis
     * @param[out] *mg_ms points to an mg/ms thresh buffer
     * @return     status code
     *             - 0 success
     *             - 1 dmp get tap thresh failed
     *             - 2 handle is NULL
     *             - 3 handle is not initialized
     *             - 4 dmp is not inited
     *             - 5 invalid axis
     * @note       none
     */
    uint8_t mpu6050_dmp_get_tap_thresh(mpu6050_handle_t *handle, mpu6050_axis_t axis, uint16_t *mg_ms);

    /**
     * @brief     dmp set the fifo rate
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] rate is the set rate
     * @return    status code
     *            - 0 success
     *            - 1 dmp set fifo rate failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     *            - 5 rate > 200
     * @note      none
     */
    uint8_t mpu6050_dmp_set_fifo_rate(mpu6050_handle_t *handle, uint16_t rate);

    /**
     * @brief      dmp get the fifo rate
     * @param[in]  *handle points to an mpu6050 handle structure
     * @param[out] *rate points to a rate buffer
     * @return     status code
     *             - 0 success
     *             - 1 dmp get fifo rate failed
     *             - 2 handle is NULL
     *             - 3 handle is not initialized
     *             - 4 dmp is not inited
     * @note       none
     */
    uint8_t mpu6050_dmp_get_fifo_rate(mpu6050_handle_t *handle, uint16_t *rate);

    /**
     * @brief     dmp enable or disable gyro calibrate
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] enable is a bool value
     * @return    status code
     *            - 0 success
     *            - 1 dmp set gyro calibrate failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_gyro_calibrate(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     * @brief     dmp enable or disable generate 3 axis quaternions from dmp
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] enable is a bool value
     * @return    status code
     *            - 0 success
     *            - 1 dmp set 3x quaternion failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_3x_quaternion(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     * @brief     dmp enable or disable generate 6 axis quaternions from dmp
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] enable is a bool value
     * @return    status code
     *            - 0 success
     *            - 1 dmp set 6x quaternion failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_6x_quaternion(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     * @brief     dmp set the interrupt mode
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] mode is the dmp interrupt mode
     * @return    status code
     *            - 0 success
     *            - 1 dmp set interrupt mode failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_interrupt_mode(mpu6050_handle_t *handle, mpu6050_dmp_interrupt_mode_t mode);

    /**
     * @brief     dmp set the gyro bias
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] *bias points to a bias buffer
     * @return    status code
     *            - 0 success
     *            - 1 dmp set gyro bias failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_gyro_bias(mpu6050_handle_t *handle, int32_t bias[3]);

    /**
     * @brief     dmp set the accel bias
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] *bias points to a bias buffer
     * @return    status code
     *            - 0 success
     *            - 1 dmp set accel bias failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_accel_bias(mpu6050_handle_t *handle, int32_t bias[3]);

    /**
     * @brief     dmp set the orientation
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] *mat points to an orientation matrix buffer
     * @return    status code
     *            - 0 success
     *            - 1 dmp set orientation failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_orientation(mpu6050_handle_t *handle, int8_t mat[9]);

    /**
     * @brief     dmp enable or disable the dmp feature
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] mask is the set mask
     * @return    status code
     *            - 0 success
     *            - 1 dmp set feature failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      mask can be MPU6050_DMP_FEATURE_TAP, MPU6050_DMP_FEATURE_ORIENT
     *            MPU6050_DMP_FEATURE_3X_QUAT, MPU6050_DMP_FEATURE_PEDOMETER
     *            MPU6050_DMP_FEATURE_6X_QUAT, MPU6050_DMP_FEATURE_GYRO_CAL
     *            MPU6050_DMP_FEATURE_SEND_RAW_ACCEL, MPU6050_DMP_FEATURE_SEND_RAW_GYRO
     *            MPU6050_DMP_FEATURE_SEND_CAL_GYRO or combination
     */
    uint8_t mpu6050_dmp_set_feature(mpu6050_handle_t *handle, uint16_t mask);

    /**
     * @brief         dmp read the data
     * @param[in]     *handle points to an mpu6050 handle structure
     * @param[out]    *accel_raw points to an accel raw buffer
     * @param[out]    *accel_g points to an accel g buffer
     * @param[out]    *gyro_raw points to a gyro raw buffer
     * @param[out]    *gyro_dps points to a gyro dps buffer
     * @param[out]    *quat points to a quat buffer
     * @param[out]    *pitch points to a pitch buffer
     * @param[out]    *roll points to a roll buffer
     * @param[out]    *yaw points to a yaw buffer
     * @param[in,out] *l points to a length buffer
     * @return        status code
     *                - 0 success
     *                - 1 dmp get fifo rate failed
     *                - 2 handle is NULL
     *                - 3 handle is not initialized
     *                - 4 dmp is not inited
     *                - 5 quat check error
     *                - 6 fifo overflow
     *                - 7 fifo data is too little
     *                - 8 no data
     * @note          none
     */
    uint8_t mpu6050_dmp_read(mpu6050_handle_t *handle,
                             int16_t (*accel_raw)[3], float (*accel_g)[3],
                             int16_t (*gyro_raw)[3], float (*gyro_dps)[3],
                             int32_t (*quat)[4],
                             float *pitch, float *roll, float *yaw,
                             uint16_t *l);

    /**
     * @brief     dmp set the tap callback
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] *callback points to a callback function address
     * @return    status code
     *            - 0 success
     *            - 1 dmp set tap callback failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_tap_callback(mpu6050_handle_t *handle, void (*callback)(uint8_t count, uint8_t direction));

    /**
     * @brief     dmp set the orient callback
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] *callback points to a callback function address
     * @return    status code
     *            - 0 success
     *            - 1 dmp set orient callback failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_orient_callback(mpu6050_handle_t *handle, void (*callback)(uint8_t orientation));

    /**
     * @brief     enable or disable the dmp
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] enable is a bool value
     * @return    status code
     *            - 0 success
     *            - 1 dmp set enable failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     *            - 4 dmp is not inited
     * @note      none
     */
    uint8_t mpu6050_dmp_set_enable(mpu6050_handle_t *handle, mpu6050_bool_t enable);

    /**
     * @brief      dmp gyro accel raw offset convert
     * @param[in]  *handle points to an mpu6050 handle structure
     * @param[in]  *gyro_offset_raw points to a gyro offset raw buffer
     * @param[in]  *accel_offset_raw points to an accel offset raw buffer
     * @param[out] *gyro_offset points to a gyro offset buffer
     * @param[out] *accel_offset points to an accel offset buffer
     * @return     status code
     *             - 0 success
     *             - 1 dmp set enable failed
     *             - 2 handle is NULL
     *             - 3 handle is not initialized
     *             - 4 dmp is not inited
     * @note       none
     */
    uint8_t mpu6050_dmp_gyro_accel_raw_offset_convert(mpu6050_handle_t *handle,
                                                      int32_t gyro_offset_raw[3], int32_t accel_offset_raw[3],
                                                      int32_t gyro_offset[3], int32_t accel_offset[3]);

    /**
     * @}
     */

    /**
     * @defgroup mpu6050_extern_driver mpu6050 extern driver function
     * @brief    mpu6050 extern driver modules
     * @ingroup  mpu6050_driver
     * @{
     */

    /**
     * @brief     set the chip register
     * @param[in] *handle points to an mpu6050 handle structure
     * @param[in] reg is the register address
     * @param[in] *buf points to a data buffer
     * @param[in] len is the data buffer length
     * @return    status code
     *            - 0 success
     *            - 1 write failed
     *            - 2 handle is NULL
     *            - 3 handle is not initialized
     * @note      none
     */
    uint8_t mpu6050_set_reg(mpu6050_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len);

    /**
     * @brief      get the chip register
     * @param[in]  *handle points to an mpu6050 handle structure
     * @param[in]  reg is the register address
     * @param[out] *buf points to a data buffer
     * @param[in]  len is the data buffer length
     * @return     status code
     *             - 0 success
     *             - 1 read failed
     *             - 2 handle is NULL
     *             - 3 handle is not initialized
     * @note       none
     */
    uint8_t mpu6050_get_reg(mpu6050_handle_t *handle, uint8_t reg, uint8_t *buf, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
