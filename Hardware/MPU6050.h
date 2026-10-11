#ifndef __MPU6050_H
#define __MPU6050_H
#define MPU6050_ACC_1G   2048
/* ★★ 已删（2026-10-11 独立审查 F24）✗：
 *   void MPU6050_Calibrate(void);
 *   void MPU6050_GetOffset(int16_t *ax, ...);
 *   原因：这两个【有声明、无定义】✗ —— MPU6050.c 里没有实现，全项目也没有调用点 ✓
 *     属于重构时被遗弃的旧接口残骸 ✓ 现在真正有的是：
 *       MPU6050_CalibrateGyro() / MPU6050_GetAccOffset() / MPU6050_GetGyroOffset() ✓
 *   ★ 留着它的危害：将来一调用就是 L6218E: Undefined symbol ✓（潜伏型 ✗）*/
void MPU6050_Init(void);
uint8_t MPU6050_ReadReg(uint8_t RegAddress);
uint8_t MPU6050_GetID(void);
void MPU6050_WriteReg(uint8_t RegAddress,uint8_t Data);
void MPU6050_GetData(int16_t *AccX,int16_t *AccY,int16_t *AccZ,int16_t *GyroX,int16_t *GyroY,int16_t *GyroZ,int16_t *Temp);
void MPU6050_CalibrateGyro(void);
void MPU6050_GetAccOffset(int16_t *OffAccX,int16_t *OffAccY,int16_t *OffAccZ);
void MPU6050_GetGyroOffset(int16_t *OffGyroX,int16_t *OffGyroY,int16_t *OffGyroZ);

/* ★★ 已删（2026-10-11 独立审查 F25）✗：
 *   void MPU6050_ReadRaw(int16_t *AccX, ...);
 *   原因：★ 头文件宣称它是【外部链接】✗ 而 MPU6050.c:85 的实现是 【static】✗✓
 *     → 头文件在【撒谎】✓ 任何别的 .c 文件调用它都会 Undefined symbol ✓
 *   处置：选择【保持 static】✓ —— 因为 MPU6050_GetData() 已经是对外封装 ✓
 *     ReadRaw 是内部细节 ✓ 不该对外暴露 ✓✓ */

int16_t MPU6050_TempX100(int16_t raw);
#endif
