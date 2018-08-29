#include "mpu6050.h"
#include "delay.h"
#include "usart.h"

//void PMU6050_WriteReg(u8 reg_add,u8 reg_dat)
//{
//	SI2C_Start();                                                   //��������
//	SI2C_SendByte(MPU6050_SLAVE_ADDRESS);										//д��������ַ
//	SI2C_RecvACK();                                                  //�ȴ�Ӧ��
//	SI2C_SendByte(reg_add);											            //д���ݵ�ַ
//	SI2C_RecvACK(); 
//		SI2C_SendByte(reg_dat);									               //д����
//		SI2C_RecvACK();                                              //�ȴ�Ӧ��
//	  SI2C_Stop();          
//}

void PMU6050_WriteReg(u8 reg_add,u8 reg_dat)
{
	I2C_Start();
	I2C_Send_Byte(MPU6050_SLAVE_ADDRESS);
	I2C_Wait_Ack();
	I2C_Send_Byte(reg_add);
	I2C_Wait_Ack();
	I2C_Send_Byte(reg_dat);
	I2C_Wait_Ack();
	I2C_Stop();
}

void PMU6050_ReadData(u8 reg_add,unsigned char*Read,u8 num)
{
	unsigned char i;
	
	I2C_Start();
	I2C_Send_Byte(MPU6050_SLAVE_ADDRESS);
	I2C_Wait_Ack();
	I2C_Send_Byte(reg_add);
	I2C_Wait_Ack();
	
	I2C_Start();
	I2C_Send_Byte(MPU6050_SLAVE_ADDRESS+1);
	I2C_Wait_Ack();
	
	for(i=0;i<(num-1);i++){
		*Read=I2C_Read_Byte(1);
		Read++;
	}
	*Read=I2C_Read_Byte(0);
	I2C_Stop();
}

//  Single_Write(MPU6050_Addr,PWR_MGMT_1, 0x00);	
//	Single_Write(MPU6050_Addr,SMPLRT_DIV, 0x07);
//	Single_Write(MPU6050_Addr,CONFIG, 0x06);
//	Single_Write(MPU6050_Addr,GYRO_CONFIG, 0x18);
//	Single_Write(MPU6050_Addr,ACCEL_CONFIG, 0x01);
//void MPU6050_Init(void)
//{ 
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x03);	     //�������״̬
//	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x00);	    //�����ǲ�����
//	PMU6050_WriteReg(MPU6050_RA_CONFIG , 0x00);	
//	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x18);	  //���ü��ٶȴ�����������16Gģʽ
//	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
//}
//void MPU6050_PWR_MGMT_1_INIT(void)
//{
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x80);
//	delay_ms(100);
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);
//	delay_ms(100);
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x09);
//	delay_ms(100);
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x04);
//	delay_ms(100);
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);
//	delay_ms(100);
//	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x08);
//}
void MPU6050_Init(void)
{
//	MPU6050_PWR_MGMT_1_INIT();
	PMU6050_WriteReg(MPU6050_RA_PWR_MGMT_1, 0x00);	     //�������״̬
	PMU6050_WriteReg(MPU6050_RA_SMPLRT_DIV , 0x07);	    //�����ǲ�����
	PMU6050_WriteReg(MPU6050_RA_CONFIG , 0x06);	
	PMU6050_WriteReg(MPU6050_RA_ACCEL_CONFIG , 0x01);	  //���ü��ٶȴ�����������16Gģʽ
	PMU6050_WriteReg(MPU6050_RA_GYRO_CONFIG, 0x18);     //�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
}
void MPU6050ReadID(void)
{
	unsigned char Re = 0;
    PMU6050_ReadData(MPU6050_RA_WHO_AM_I,&Re,1);    //��������ַ
     printf("%d\r\n",Re);
}
void MPU6050ReadAcc(short *accData)
{
    u8 buf[6];
    PMU6050_ReadData(MPU6050_ACC_OUT, buf, 6);
    accData[0] = (buf[0] << 8) | buf[1];
    accData[1] = (buf[2] << 8) | buf[3];
    accData[2] = (buf[4] << 8) | buf[5];
}
void MPU6050ReadGyro(short *gyroData)
{
    u8 buf[6];
    PMU6050_ReadData(MPU6050_GYRO_OUT,buf,6);
    gyroData[0] = (buf[0] << 8) | buf[1];
    gyroData[1] = (buf[2] << 8) | buf[3];
    gyroData[2] = (buf[4] << 8) | buf[5];
}

void MPU6050ReadTemp(short *tempData)
{
	u8 buf[2];
    PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
    *tempData = (buf[0] << 8) | buf[1];
}

void MPU6050_ReturnTemp(short*Temperature)
{
	short temp3;
	u8 buf[2];
	
	PMU6050_ReadData(MPU6050_RA_TEMP_OUT_H,buf,2);     //��ȡ�¶�ֵ
  temp3= (buf[0] << 8) | buf[1];
	*Temperature=(((double) (temp3 + 13200)) / 280)-13;
}
