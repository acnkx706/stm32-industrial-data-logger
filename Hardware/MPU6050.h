#ifndef __MPU6050_H
#define __MPU6050_H
#define MPU6050_ACC_1G   2048
void MPU6050_Calibrate(void);
void MPU6050_GetOffset(int16_t *ax, int16_t *ay, int16_t *az,
                       int16_t *gx, int16_t *gy, int16_t *gz);
void MPU6050_Init(void);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
uint8_t MPU6050_GetID(void);
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ,int16_t *Temp);
void MPU6050_CalibrateGyro(void);
void MPU6050_GetAccOffset(int16_t *OffAccX,int16_t *OffAccY,int16_t *OffAccZ);
void MPU6050_GetGyroOffset(int16_t *OffGyroX,int16_t *OffGyroY,int16_t *OffGyroZ);
void MPU6050_ReadRaw(int16_t *AccX, int16_t *AccY, int16_t *AccZ,
                            int16_t *GyroX, int16_t *GyroY, int16_t *GyroZ,
                            int16_t *Temp) ;

int16_t MPU6050_TempX100(int16_t raw);
#endif