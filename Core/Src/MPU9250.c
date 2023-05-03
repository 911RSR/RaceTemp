// https://www.invensense.com/products/motion-tracking/9-axis/mpu-9250/
#include "MPU9250.h"
#include "main.h"

#define    MPU9250_ADDRESS            0x68 // device address given in datasheet
#define    MAG_ADDRESS                0x0C

#define    GYRO_FULL_SCALE_250_DPS    0x00  
#define    GYRO_FULL_SCALE_500_DPS    0x08
#define    GYRO_FULL_SCALE_1000_DPS   0x10
#define    GYRO_FULL_SCALE_2000_DPS   0x18

#define    ACC_FULL_SCALE_2_G        0x00
#define    ACC_FULL_SCALE_4_G        0x08
#define    ACC_FULL_SCALE_8_G        0x10
#define    ACC_FULL_SCALE_16_G       0x18

const  uint32_t Timeout =10;  // HAL  what unit?


// Write a byte (Data) in device (Address) at register (Register)
void write_byte(uint16_t Address, uint16_t Register, uint8_t Data)
{
	HAL_I2C_Mem_Write( (void*) MPU9250_dev, Address, Register, I2C_MEMADD_SIZE_8BIT, &Data, 1, Timeout);
	//HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_TypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
}


void MPU9250_setup()
{
  //hi2c1.begin();
  write_byte(MPU9250_ADDRESS,29,0x06);   // Set accelerometers low pass filter at 5Hz
  write_byte(MPU9250_ADDRESS,26,0x06);   // Set gyroscope low pass filter at 5Hz
  write_byte(MPU9250_ADDRESS,27,GYRO_FULL_SCALE_1000_DPS);  // Configure gyroscope range
  write_byte(MPU9250_ADDRESS,28,ACC_FULL_SCALE_4_G);  // Configure accelerometers range
  write_byte(MPU9250_ADDRESS,0x37,0x02); // Set bypass mode for the magnetometers
  write_byte(MAG_ADDRESS,0x0A,0x16);     // Request continuous magnetometer measurements in 16 bits
}

// This function read Nbytes bytes from I2C device at address Address. 
// Put read bytes starting at register Register in the Data array. 
void I2C_Read(uint16_t Address, uint16_t Register, uint16_t Nbytes, uint8_t* Data)
{
	HAL_I2C_Mem_Read((void*)MPU9250_dev, Address, Register, I2C_MEMADD_SIZE_8BIT, Data, Nbytes, Timeout);
	//HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout);
}

void MPU9250_read( int16_t acc[], int16_t gyro[], int16_t mag[])
{
  uint8_t buf[14];
  I2C_Read(MPU9250_ADDRESS,0x3B,14,buf);  // Read accelerometer and gyroscope

  // Create 16 bits values from 8 bits data
  // Accelerometer
  acc[0]=buf[0]<<8 | buf[1];
  acc[1]=buf[2]<<8 | buf[3];
  acc[2]=buf[4]<<8 | buf[5];

  //*temp=buf[6]<<8 | buf[7];

  // Gyroscope
  gyro[0]=buf[8]<<8 | buf[9];
  gyro[1]=buf[10]<<8 | buf[11];
  gyro[2]=buf[12]<<8 | buf[13];

  // Read magnetometer data  
  I2C_Read(MAG_ADDRESS,0x03,7,buf);
  mag[0]=buf[3]<<8 | buf[2];
  mag[1]=buf[1]<<8 | buf[0];
  mag[2]=buf[5]<<8 | buf[4];
}

