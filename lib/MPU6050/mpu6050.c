
#ifndef mpu6050.c 
#define mpu6050.c 

#ifdef __cplusplus
extern "C"{
#endif

/*----------------------------------include-----------------------------------*/
#include "mpu6050.h"
/*-----------------------------------macro------------------------------------*/

/*----------------------------------typedef-----------------------------------*/

/*----------------------------------variable----------------------------------*/

/*-------------------------------------os-------------------------------------*/

/*----------------------------------function----------------------------------*/
void MPU_6050_Init(void) {

    // u8 res;
	// IIC_Init();//初始化IIC总线
	// MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X80);	//复位MPU6050
    // delay_ms(100);
	// MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X00);	//唤醒MPU6050 
	// MPU_Set_Gyro_Fsr(3);					//陀螺仪传感器,±2000dps
	// MPU_Set_Accel_Fsr(0);					//加速度传感器,±2g
	// MPU_Set_Rate(50);						//设置采样率50Hz
	// MPU_Write_Byte(MPU_INT_EN_REG,0X00);	//关闭所有中断
	// MPU_Write_Byte(MPU_USER_CTRL_REG,0X00);	//I2C主模式关闭
	// MPU_Write_Byte(MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	// MPU_Write_Byte(MPU_INTBP_CFG_REG,0X80);	//INT引脚低电平有效
	// res=MPU_Read_Byte(MPU_DEVICE_ID_REG);
	// if(res==MPU_ADDR)//器件ID正确
	// {
	// 	MPU_Write_Byte(MPU_PWR_MGMT1_REG,0X01);	//设置CLKSEL,PLL X轴为参考
	// 	MPU_Write_Byte(MPU_PWR_MGMT2_REG,0X00);	//加速度与陀螺仪都工作
	// 	MPU_Set_Rate(50);						//设置采样率为50Hz
 	// }else return 1;
	// return 0;


	// HAL_StatusTypeDef status;
	// HAL_I2C_StateTypeDef flag;
	// unsigned char pdata;
	// //检查设备是否准备好  地址   检查 次数 超时时间  
	// status=HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS_W, 10, HAL_MAX_DELAY);
	// printf("status is %d\n",status);
	// //检查总线是否准备好
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// //发送写寄存器命令
	// pdata=0x80; //复位MPU
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); //写0x80复位
	// status=HAL_I2C_IsDeviceReady(&hi2c1, ADDRESS_W, 10, HAL_MAX_DELAY);
	// printf("status is %d\n",status);
	
	// HAL_Delay(500);  //复位后需要等待一段时间 等待芯片复位完成 
	
	// //检查总线是否准备好
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// //唤醒MPU
	// pdata=0x01; // 7位 1 重启  6位睡眠模式1 睡眠 2 唤醒   3位 为u你懂传感器0 开启    0-2位 时钟选择  01 PLL使用XZ轴陀螺作为参数
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); //写0x80复位
	// //设陀螺仪 寄存器  2000  3
	// pdata=3<<3; //设置3 为量程2000  右移3位 到对应的 3 4 位寄存器中
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_GYRO_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// //设置加速度传感器量程 2g
	// pdata=0; 
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_ACCEL_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// //陀螺仪采样分频设置
	// pdata=19; //1000/50-1  这个还需要查资料查看  原因 和计算方法
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_SAMPLE_RATE_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// //关闭所有中断
	// pdata=0;
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_INT_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// //I2C 主模式的 外接磁力传感器接口关闭
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// pdata=0;
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_USER_CTRL_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// //关闭FIFO 
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// pdata=0;
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_FIFO_EN_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// //中断旁路设置 低电平有效
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// pdata=0X80;
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_INTBP_CFG_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// //设置低通滤波器
	// MPU_Set_LPF(50/2);
	// //读器件ID  默认是 0x68 
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// pdata=MPU_DEVICE_ID_REG;
	// HAL_I2C_Mem_Read(&hi2c1, ADDRESS_R, MPU_DEVICE_ID_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	// printf("ID is %X \n",pdata);
	// //使能 陀螺仪 和加速度 工作
	// flag=HAL_I2C_GetState(&hi2c1);
	// printf("flag %X \n",flag);
	// pdata=0;
	// HAL_I2C_Mem_Write(&hi2c1, ADDRESS_W, MPU_PWR_MGMT2_REG, 1, &pdata, 1, HAL_MAX_DELAY);
//寄存器输出调试观看
/*
	//读取寄存器1 
	flag=HAL_I2C_GetState(&hi2c1);
	printf("flag %X \n",flag);
	pdata=0;
	HAL_I2C_Mem_Read(&hi2c1, ADDRESS_R, MPU_PWR_MGMT1_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	printf("0x6B REG1 %02X \n",pdata);
	//读取温度传感器
	HAL_Delay(500);
	flag=HAL_I2C_GetState(&hi2c1);
	printf("flag %X \n",flag);
	pdata=0xff;
	HAL_I2C_Mem_Read(&hi2c1, ADDRESS_R, MPU_TEMP_OUTH_REG, 1, &pdata, 1, HAL_MAX_DELAY); 
	printf("hight bit %02X \n",pdata);
	HAL_Delay(500);
	flag=HAL_I2C_GetState(&hi2c1);
	printf("flag %X \n",flag);
	pdata=0xff;
	HAL_I2C_Mem_Read(&hi2c1, ADDRESS_R, MPU_TEMP_OUTH_REG+1, 1, &pdata, 1, HAL_MAX_DELAY); 
	printf("low bit %02X \n",pdata);
*/



}
/*------------------------------------test------------------------------------*/

#ifdef __cplusplus
}
#endif

#endif	/* mpu6050.c */
