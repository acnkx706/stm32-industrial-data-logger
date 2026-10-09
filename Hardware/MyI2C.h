#ifndef __MyI2C_H
#define __MyI2C_H

#define MYI2C_GPIO       GPIOA
#define MYI2C_SCL_PIN    GPIO_Pin_2
#define MYI2C_SDA_PIN    GPIO_Pin_3

void MyI2C_W_SCL(uint8_t BitValue);
void MyI2C_W_SDA(uint8_t BitValue);
uint8_t MyI2C_R_SDA(void);
void MyI2C_Init(void);
uint8_t MyI2C_ReceiveAck(void);
void MyI2C_SendAck(uint8_t AckBit);
uint8_t MyI2C_ReceiveByte(void);
void MyI2C_SendByte(uint8_t Byte);
void MyI2C_Stop(void);
void MyI2C_Start(void);


#endif
