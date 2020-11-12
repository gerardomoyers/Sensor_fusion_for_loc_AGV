/*
 * Sensors_functions.h
 *
 *  Created on: Mar 17, 2020
 *      Author: Gerardo
 */

///// Includes
#include "main.h"
#include "cmsis_os.h"
#include "uarts.h"
#include "math.h"

/*Defines*/

//// HCSR4 defines
#define US_In_Port_left GPIOD
#define US_In_Pin_left  GPIO_PIN_2
#define US_In_Port_right GPIOE
#define US_In_Pin_right  GPIO_PIN_12
#define US_Out_Port  GPIOC
#define US_Out_Pin GPIO_PIN_12
#define TIMEMAX_Echo 50000


//////// IMU defines
#define MPU6050_ADDRESS  0b1101000




/* End Defines*/

#ifndef INC_SENSORS_FUNCTIONS_H_
#define INC_SENSORS_FUNCTIONS_H_

void Init_localization(float val_x, float val_y, float val_Vx, float val_Vy);

//////// HCSR4 functions
void HCSR4_Init(_Bool sensors[6]);
uint16_t HCSR4_Meas(uint16_t distance, _Bool sensors[6]);


//////// IMU functions
void i2c_write_register(I2C_TypeDef *i2c, uint8_t i2c_address, uint8_t reg, uint8_t value);
void i2c_read_registers(I2C_TypeDef *i2c, uint8_t i2c_address, uint8_t byte_count, uint8_t first_reg, uint8_t *rx_buffer);
void mpu6050_read_sensors(void);
void IMU_star_com (void);
uint16_t IMU_Meas(uint16_t count);


/////// Motors Function

void PID_Init(float Kp_, float Ki_, float Kd_);
void PID_UpdateError(double cte);
float PID_TotalError();

/////////// Radar functions
void getdata_Radar(char rx_buffer[LINEMAX + 1]);
void getPos_Radar(void);



/* End Functions*/



#endif /* INC_SENSORS_FUNCTIONS_H_ */
