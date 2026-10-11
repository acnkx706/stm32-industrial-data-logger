#include "stm32f10x.h"                  // Device header
#include "MyI2C.h"
#include "MPU6050_Reg.h"
#include "MPU6050.h"
#define MPU6050_ADDRESS 		0XD0
#define MPU6050_GYRO_CAL_N       100
#define MPU6050_OFF_AX      (+70)
#define MPU6050_OFF_AY      (-43)
#define MPU6050_OFF_AZ      (-160)
static int16_t s_off_ax = MPU6050_OFF_AX;      /* 常量：出厂标定 */
static int16_t s_off_ay = MPU6050_OFF_AY;
static int16_t s_off_az = MPU6050_OFF_AZ;
static int16_t s_off_gx = 0, s_off_gy = 0, s_off_gz = 0;   /* 开机标定后才有值 */
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data)
{
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(Data);
	MyI2C_ReceiveAck();
	MyI2C_Stop();
}

uint8_t MPU6050_ReadReg(uint8_t RegAddress)
{
	uint8_t Data;
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);
	MyI2C_ReceiveAck();
	MyI2C_SendByte(RegAddress);
	MyI2C_ReceiveAck();
	
	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS|0x01);
	MyI2C_ReceiveAck();
	Data=MyI2C_ReceiveByte();
	MyI2C_SendAck(1);
	MyI2C_Stop();
	return Data;
}

/* 从 reg 开始连续读 len 个字节（利用 MPU6050 寄存器地址自动递增 ✓）
 * ★ 关键：最后一个字节必须回 NACK，其余回 ACK ——
 *   这是 I2C 告诉从机"我读完了"的唯一方式 ✓ 弄错会多读/时序错乱 ✗ */
static void MPU6050_ReadBurst(uint8_t reg, uint8_t *buf, uint8_t len)
{
	uint8_t i;

	MyI2C_Start();
	MyI2C_SendByte(MPU6050_ADDRESS);          /* 0xD0 = 写地址 */
	MyI2C_ReceiveAck();
	MyI2C_SendByte(reg);
	MyI2C_ReceiveAck();

	MyI2C_Start();                            /* ★ 重复起始(Restart)，中间不 Stop ✓ */
	MyI2C_SendByte(MPU6050_ADDRESS | 0x01);   /* 0xD1 = 读地址 */
	MyI2C_ReceiveAck();

	for (i = 0; i < len; i++)
	{
		buf[i] = MyI2C_ReceiveByte();
		MyI2C_SendAck((i == (uint8_t)(len - 1)) ? 1 : 0);   /* 最后 1 字节 NACK(1) ✓ */
	}
	MyI2C_Stop();
}

void MPU6050_Init(void)
{
	MyI2C_Init();
	MPU6050_WriteReg(MPU6050_PWR_MGMT_1,0x01);
	MPU6050_WriteReg(MPU6050_PWR_MGMT_2,0x00);
	MPU6050_WriteReg(MPU6050_SMPLRT_DIV,0x09);
	MPU6050_WriteReg(MPU6050_CONFIG,0x06);
	MPU6050_WriteReg(MPU6050_GYRO_CONFIG,0x18);
	MPU6050_WriteReg(MPU6050_ACCEL_CONFIG,0x18);
}

uint8_t MPU6050_GetID(void)
{
	return MPU6050_ReadReg(MPU6050_WHO_AM_I);
}

static void MPU6050_ReadRaw(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                            int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                            int16_t *Temp)              /* ★ 新增一个出参 */
{
	uint8_t b[14];

	MPU6050_ReadBurst(MPU6050_ACCEL_XOUT_H, b, 14);     /* 0x3B 起，一次读完 ✓ */

	*AccX  = (int16_t)((b[0]  << 8) | b[1]);
	*AccY  = (int16_t)((b[2]  << 8) | b[3]);
	*AccZ  = (int16_t)((b[4]  << 8) | b[5]);
	*Temp  = (int16_t)((b[6]  << 8) | b[7]);            /* 0x41/0x42 ✓ */
	*GyroX = (int16_t)((b[8]  << 8) | b[9]);
	*GyroY = (int16_t)((b[10] << 8) | b[11]);
	*GyroZ = (int16_t)((b[12] << 8) | b[13]);
}

void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ,int16_t *Temp)
{
	MPU6050_ReadRaw(AccX, AccY, AccZ, GyroX, GyroY, GyroZ, Temp);
	*AccX  -= s_off_ax;
	*AccY  -= s_off_ay;
	*AccZ  -= s_off_az;
	*GyroX -= s_off_gx;
	*GyroY -= s_off_gy;
	*GyroZ -= s_off_gz;
}
void MPU6050_CalibrateGyro(void)
{
	int32_t sum_gx = 0, sum_gy = 0, sum_gz = 0;
	int16_t ax, ay, az, gx, gy, gz,tdummy;
	uint8_t i;
	for (i = 0; i < MPU6050_GYRO_CAL_N; i++)
	{
		MPU6050_ReadRaw(&ax, &ay, &az, &gx, &gy, &gz,&tdummy);
		sum_gx += gx;  sum_gy += gy;  sum_gz += gz;
	}
	s_off_gx = (int16_t)(sum_gx / MPU6050_GYRO_CAL_N);
	s_off_gy = (int16_t)(sum_gy / MPU6050_GYRO_CAL_N);
	s_off_gz = (int16_t)(sum_gz / MPU6050_GYRO_CAL_N);
}
void MPU6050_GetAccOffset(int16_t *OffAccX,int16_t *OffAccY,int16_t *OffAccZ)
{
	*OffAccX = s_off_ax;   *OffAccY = s_off_ay;   *OffAccZ = s_off_az;
}
void MPU6050_GetGyroOffset(int16_t *OffGyroX,int16_t *OffGyroY,int16_t *OffGyroZ)
{
	*OffGyroX = s_off_gx;   *OffGyroY = s_off_gy;   *OffGyroZ = s_off_gz;
}

/* 芯片内部温度(die) -> ×100 °C 整数
 * 手册：340 LSB/°C，偏移 36.53 °C
 *   x100 = raw/340*100 + 3653 = raw*5/17 + 3653   （100/340 约分 = 5/17 ✓ 无精度损失）
 * ★ 先用 int32 乘：raw=8500 时 8500*5=42500 会溢出 int16 ✗ */
int16_t MPU6050_TempX100(int16_t raw)
{
	return (int16_t)(((int32_t)raw * 5) / 17 + 3653);
}
