/*
 * Sensors_functions.c
 *
 *  Created on: Mar 17, 2020
 *      Author: Gerardo
 */



/* Includes*/

#include "Sensors_functions.h"
#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* End Includes*/


/* External Variables*/

/**/

/* Functions */

void Init_localization(float val_x, float val_y, float val_Vx, float val_Vy){
	Radarx = val_x;
	Radary = val_y;
	speedUWBx = val_Vx;
	speedUWBy = val_Vy;
	speedradarx = val_Vx;
	speedradary = val_Vy;

	float dummyH [4][4] = {{(weight_UWBx + weight_radarx),0,0,0},
			 {0,(weight_UWBy + weight_radary),0,0},
			 {0,0,1,0},
		 	 {0,0,0,1}};
	float dummyF[4][4] = {{1,0,deltatime_imu,0},
			 {0,1,0,deltatime_imu},
			 {0,0,1,0},
		 	 {0,0,0,1}};
	float dummyQ [4][4] = {{0.00000533,0,0,0},
						{0,0.00000036,0,0},
						{0,0,0.00043956,0},
						{0,0,0,0.00034941}};

	float dummyR [4][4] = {{(weight_UWBx * 0.007161239 + weight_radarx * 0.072271677),0,0,0},
						{0,(weight_UWBy * 0.00206330 + weight_radary * 0.013577092),0,0},
						{0,0,weight_speedUWBx * 0.08474402 + weight_speedradarx * 0.000850232,0},
						{0,0,0,weight_speedUWBy * 0.05404885 + weight_speedradary * 0.000695134}};
	float dummyX[4] = {val_x, val_y, val_Vx, val_Vy};
	float dummyP [4][4] = {{0.1,0,0,0},
							{0,0.1,0,0},
							{0,0,0,0},
							{0,0,0,0}};

	memcpy(jacobian_F,dummyF,sizeof(dummyF));
	memcpy(jacobian_H,dummyH,sizeof(dummyH));
	memcpy(standar_dev_Q,dummyQ,sizeof(dummyQ));
	memcpy(standar_dev_R,dummyR,sizeof(dummyR));
	memcpy(states_X,dummyX,sizeof(dummyX));
	memcpy(covariance_P,dummyP,sizeof(dummyP));

}

//////// HCSR4 funcitons
void HCSR4_Init(_Bool sensors[6]){
	sensors[4] = true;
	HAL_GPIO_WritePin(US_Out_Port, US_Out_Pin, 0);// Init Ultrasound
}

uint16_t HCSR4_Meas(uint16_t distance, _Bool sensors[6]){

	uint16_t time_le = 0;
	uint16_t time_ri = 0;
	uint16_t timeout = 0;
	_Bool wait_for_echo = false;
	HAL_GPIO_WritePin(US_Out_Port, US_Out_Pin, 1);
	DelayMicro(10);
	HAL_GPIO_WritePin(US_Out_Port, US_Out_Pin, 0);


	while(wait_for_echo == false && timeout < TIMEMAX_Echo){

		if(HAL_GPIO_ReadPin(US_In_Port_left, US_In_Pin_left)){
			while(HAL_GPIO_ReadPin(US_In_Port_left, US_In_Pin_left) && timeout < TIMEMAX_Echo ){
				if(HAL_GPIO_ReadPin(US_In_Port_right, US_In_Pin_right)){
					time_ri++;
				}
				time_le++;
				timeout++;
				DelayMicro(1);
			}
			if (HAL_GPIO_ReadPin(US_In_Port_right, US_In_Pin_right)){
				while(HAL_GPIO_ReadPin(US_In_Port_right, US_In_Pin_right) && timeout < TIMEMAX_Echo ){
					time_ri++;
					timeout++;
					DelayMicro(1);
				}
			}
			wait_for_echo = true;
			time_ri = time_le + 500;
			uint16_t timeless = (time_le > time_ri) ? time_ri : time_le;
			USlr = (time_le > time_ri) ? USright : USleft;
			distance = 2.51* (timeless/58);
			sensors[4] = true;
		}else if(HAL_GPIO_ReadPin(US_In_Port_right, US_In_Pin_right)){
			while(HAL_GPIO_ReadPin(US_In_Port_right, US_In_Pin_right) && timeout < TIMEMAX_Echo ){
				if(HAL_GPIO_ReadPin(US_In_Port_left, US_In_Pin_left)){
					time_le++;
				}
				time_ri++;
				timeout++;
				DelayMicro(1);
			}
			if (HAL_GPIO_ReadPin(US_In_Port_left, US_In_Pin_left)){
				while(HAL_GPIO_ReadPin(US_In_Port_left, US_In_Pin_left) && timeout < TIMEMAX_Echo ){
					time_le++;
					timeout++;
					DelayMicro(1);
				}
			}
			wait_for_echo = true;
			time_le = time_ri +500 ;
			uint16_t timeless = (time_le > time_ri) ? time_ri : time_le;
			USlr = (time_le > time_ri) ? USright : USleft;
			distance = 2.51* (timeless/58);
			sensors[4] = true;
		}
		timeout++;
		DelayMicro(1);
	}
	return distance;
}



/////////////// IMU Functions

/**
 * Writes to one register of an I2C device
 *
 * @param i2c           I2C1 or I2C2
 * @param i2c_address   I2C device address
 * @param reg           Register being written to
 * @param value         Value for the register
 */
void i2c_write_register(I2C_TypeDef *i2c, uint8_t i2c_address, uint8_t reg, uint8_t value) {

	// write two bytes with a start bit and a stop bit
	i2c->CR2 = (i2c_address << 1) | I2C_CR2_START | I2C_CR2_AUTOEND | (2 << 16); //33685712
	while((i2c->ISR & I2C_ISR_TXIS) == 0);
	i2c->TXDR = reg;
	while((i2c->ISR & I2C_ISR_TXIS) == 0);
	i2c->TXDR = value;
	while(i2c->ISR & I2C_ISR_BUSY);

}

/**
 * Read the specified number of bytes from an I2C device
 *
 * @param i2c           I2C1 or I2C2
 * @param i2c_address   I2C device address
 * @param byte_count    Number of bytes to read
 * @param first_reg     First register to read from
 * @param rx_buffer     Pointer to an array of uint8_t's where values will be stored
 */
void i2c_read_registers(I2C_TypeDef *i2c, uint8_t i2c_address, uint8_t byte_count, uint8_t first_reg, uint8_t *rx_buffer) {

	// write one byte (the register number) with a start bit but no stop bit
	i2c->CR2 = (i2c_address << 1) | I2C_CR2_START | (1 << 16);
	while((i2c->ISR & I2C_ISR_TXIS) == 0);
	i2c->TXDR = first_reg;
	while((i2c->ISR & I2C_ISR_TC) == 0);

	// read the specified number of bytes with a start bit and a stop bit
	i2c->CR2 = (i2c_address << 1) | I2C_CR2_RD_WRN | I2C_CR2_START | I2C_CR2_AUTOEND | (byte_count << 16);

	// wait for the bytes to arrive
	while(byte_count-- > 0) {
		while((i2c->ISR & I2C_ISR_RXNE) == 0);
		*rx_buffer++ = i2c->RXDR;
	}
}



void mpu6050_read_sensors(void) {

	// read the sensor values
	uint8_t rx_buffer[201];

	i2c_read_registers(I2C1, MPU6050_ADDRESS, 20, 0x3B, rx_buffer);

	// extract the raw values
	int16_t  accel_x  = rx_buffer[0]  << 8 | rx_buffer[1];
	int16_t  accel_y  = rx_buffer[2]  << 8 | rx_buffer[3];
	int16_t  accel_z  = rx_buffer[4]  << 8 | rx_buffer[5];
	int16_t  gyro_x   = rx_buffer[8]  << 8 | rx_buffer[9];
	int16_t  gyro_y   = rx_buffer[10] << 8 | rx_buffer[11];
	int16_t  gyro_z   = rx_buffer[12] << 8 | rx_buffer[13];

	// convert accelerometer readings into G's
	accel_x_g = accel_x / 2048.0f;
	accel_y_g = accel_y / 2048.0f;
	accel_z_g = accel_z / 2048.0f;

	// convert gyro readings into Radians per second
	gyro_x_rad = gyro_x /  939.650784f;
	gyro_y_rad = gyro_y / 939.650784f;
	gyro_z_rad = gyro_z / 939.650784f;//16.4 2000dps





}

void IMU_star_com (void){


	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x6B, 0x00);                    // exit sleep
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x19, 7);                     // sample rate = 8kHz / 110 = 72.7Hz
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x1B, 0x18);                    // gyro full scale = +/- 200(x00) 500(x08) 1000(x10) 2000(x18)dps
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x1C, 0x18);                    // accelerometer full scale = +/- 2(x00) 4(x08) 8 (x10) 16(x18)g
	i2c_write_register(I2C1, MPU6050_ADDRESS,  0x38, 0x01);                    // enable INTA interrupt
}

uint16_t IMU_Meas(uint16_t count){
	if(!offset){
		  	if(count <= 100){
		  		mean_acc_x += accel_x_g;
		  		mean_acc_y += accel_y_g;
		  		mean_acc_z += accel_z_g;
		  		mean_gyro_x += gyro_x_rad;
		  		mean_gyro_y += gyro_y_rad;
		  		mean_gyro_z += gyro_z_rad;
		  		count++;
		  	}else{
				accel_x_offset = mean_acc_x/100;
				accel_y_offset = mean_acc_y/100;
				accel_z_offset = mean_acc_z/100;
				gyro_x_offset = mean_gyro_x/100;
				gyro_y_offset = mean_gyro_y/100;
				gyro_z_offset = mean_gyro_z/100;
				offset = true;
				count = 0;
		  	}
	}else if(Mean++ >= 9 && offset){
		float weightgyro = 0.02;//4
		mean_acc_x += accel_x_g;
		mean_acc_y += accel_y_g;
		mean_acc_z += accel_z_g;
		mean_gyro_x += gyro_x_rad;
		mean_gyro_y += gyro_y_rad;
		mean_gyro_z += gyro_z_rad;
		accel_x_g2 = (mean_acc_x/10 - accel_x_offset)*9.81;
		accel_y_g2 = (mean_acc_y/10 - accel_y_offset)*9.81;
		accel_z_g2 = (mean_acc_z/10 - accel_z_offset)*9.81;
		gyro_x_rad2 = (mean_gyro_x/10 - gyro_x_offset);
		gyro_y_rad2 = (mean_gyro_y/10 - gyro_y_offset);
		gyro_z_rad2 = (mean_gyro_z/10 - gyro_z_offset);
//		float gyro_conv_x = 1/sqrt( 1 + pow(1/tan(gyro_y_rad2),2)*pow(1/cos(gyro_y_rad2),2) );
	//	float gyro_conv_y = 1/sqrt( 1 + pow(1/tan(gyro_x_rad2),2)*pow(1/cos(gyro_x_rad2),2) );
		gyro_conv_x = sin(gyro_y_rad2)/sqrt( 1 + pow(tan(gyro_x_rad2),2)*pow(cos(gyro_y_rad2),2) );
		gyro_conv_y = sin(gyro_x_rad2)/sqrt( 1 + pow(tan(gyro_y_rad2),2)*pow(cos(gyro_x_rad2),2) );
		gyro_conv_z = gyro_z_rad2/fabs(gyro_z_rad2)*sqrt(1 - pow(gyro_conv_x,2) - pow(gyro_conv_y,2) );

		gyro_x_rad2 = gyro_x_rad2 *180/3.14159;
		gyro_y_rad2 = gyro_y_rad2 *180/3.14159;
		gyro_z_rad2 = gyro_z_rad2 *180/3.14159;
		mean_acc_x = 0;
		mean_acc_y = 0;
		mean_acc_z = 0;
		mean_gyro_x = 0;
		mean_gyro_y = 0;
		mean_gyro_z = 0;

		accel_x_g2 = (accel_x_g2 + 9.81 * gyro_conv_x * weightgyro) / (1 + weightgyro);
		accel_y_g2 = (accel_y_g2 + 9.81 * gyro_conv_y * weightgyro) / (1 + weightgyro);
		accel_z_g2 = (accel_z_g2 + 9.81 * gyro_conv_z * weightgyro) / (1 + weightgyro);

		deltatime_imu = (HAL_GetTick() - start_imu)/100;


		distx_rec = fabs(accel_x_g2*deltatime_imu); // 0.04
		anglez_rec = gyro_z_rad2 * deltatime_imu;

			if (moving){
				anglezactual += anglez_rec;

				if(anglez < -IMU_angle_acc || anglez > IMU_angle_acc){
					anglez -= anglez_rec;
					turning = !turning;
				}else{
					turning = false;
					disth -= distx_rec;
					anglez -= anglez_rec;
				}
			}

			Mean = 0;
	}else {
			mean_acc_x += accel_x_g;
			mean_gyro_z += gyro_z_rad;
	}
	return count;
}


// Control Motors

void PID_Init(float Kp_, float Ki_, float Kd_) {

    Kp = Kp_;
    Ki = Ki_;
    Kd = Kd_;

    p_error = 0;
    i_error = 0;
    d_error = 0;
    previous_e = 0;
}

void PID_UpdateError(double cte) {

    p_error = cte;
    i_error += p_error;
    d_error = cte - previous_e;
    previous_e = cte;
}

float PID_TotalError() {

    return -Kp * p_error - Ki * i_error - Kd * d_error;
}



// Radar Functions

void getdata_Radar(char rx_buffer[LINEMAX + 1])
{
	uint8_t TLVs = rx_buffer[32];
	uint16_t idx = 40;//36;
	uint16_t num_det = 0;
	if(count_ch_freq_radar > 20){
		if(actual_freq_radar2 == 1){
			actual_freq_radar = 5;
		}else{
			actual_freq_radar++;
			actual_freq_radar = (actual_freq_radar > 10) ? 10 : actual_freq_radar;
		}
		count_ch_freq_radar = 0;
	}

	for(uint8_t tempTLV = 1; tempTLV<= TLVs; tempTLV++)
	{
		if ((uint8_t)rx_buffer[idx] == tempTLV)
		{
			idx += 4;
			uint32_t TLVlength = rx_buffer[idx++] + (rx_buffer[idx++] << 8) + (rx_buffer[idx++] << 16) + (rx_buffer[idx++] << 24);
			uint16_t tempidx = idx;
			num_det = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
			uint16_t xyzQformat = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
			xyzQformat = powf(2,xyzQformat);

			switch(tempTLV)
			{
			case 1:
				numObj = num_det;

				for (uint16_t temp_num_det = 1; temp_num_det <= num_det; temp_num_det++ )
				{
					Objects_radar[temp_num_det -1].speed = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					Objects_radar[temp_num_det -1].peak = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					//idx += 2; // 0;
					Objects_radar[temp_num_det - 1].xobj = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					Objects_radar[temp_num_det - 1].yobj = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					Objects_radar[temp_num_det - 1].zobj = rx_buffer[idx++] + (rx_buffer[idx++] << 8);

					Objects_radar[temp_num_det - 1].speed = (Objects_radar[temp_num_det - 1].speed > 32767) ? Objects_radar[temp_num_det - 1].speed - 65536 : Objects_radar[temp_num_det - 1].speed;
					Objects_radar[temp_num_det - 1].xobj = (Objects_radar[temp_num_det - 1].xobj > 32767) ? Objects_radar[temp_num_det - 1].xobj - 65536 : Objects_radar[temp_num_det - 1].xobj;
					Objects_radar[temp_num_det - 1].yobj = (Objects_radar[temp_num_det - 1].yobj > 32767) ? Objects_radar[temp_num_det - 1].yobj - 65536 : Objects_radar[temp_num_det - 1].yobj;
					Objects_radar[temp_num_det - 1].zobj = (Objects_radar[temp_num_det - 1].zobj > 32767) ? Objects_radar[temp_num_det - 1].zobj - 65536 : Objects_radar[temp_num_det - 1].zobj;

					Objects_radar[temp_num_det - 1].speed /= xyzQformat;
					Objects_radar[temp_num_det - 1].peak /= xyzQformat;
					Objects_radar[temp_num_det - 1].xobj /= xyzQformat;
					Objects_radar[temp_num_det - 1].yobj /= xyzQformat;
					Objects_radar[temp_num_det - 1].zobj /= xyzQformat;

					if(count_ch_freq_radar == 0){
						if (fabs(Objects_radar[temp_num_det - 1].xobj) < 1 && fabs(Objects_radar[temp_num_det - 1].yobj) < 1 || Objects_radar[temp_num_det - 1].speed < - 3){
							actual_freq_radar = 1;
						}
					}
					actual_freq_radar2 = actual_freq_radar;

					if (idx >= TLVlength + tempidx)
						break;
				}
				break;
			case 2:
				numClu = num_det;
				/*for (uint16_t temp_num_det = 1; temp_num_det <= num_det; temp_num_det++ )
				{

					Clusters_radar[temp_num_det - 1].xclu = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					Clusters_radar[temp_num_det - 1].yclu = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					idx += 2;   // z val
					Clusters_radar[temp_num_det - 1].xsizeclu = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					Clusters_radar[temp_num_det - 1].ysizeclu = rx_buffer[idx++] + (rx_buffer[idx++] << 8);
					idx += 2; //z size

					Clusters_radar[temp_num_det - 1].xclu = (Clusters_radar[temp_num_det - 1].xclu > 32767)
								? Clusters_radar[temp_num_det - 1].xclu - 65536 : Clusters_radar[temp_num_det - 1].xclu;
					Clusters_radar[temp_num_det - 1].yclu = (Clusters_radar[temp_num_det - 1].yclu > 32767)
								? Clusters_radar[temp_num_det - 1].yclu - 65536 : Clusters_radar[temp_num_det - 1].yclu;

					Clusters_radar[temp_num_det - 1].xclu /= xyzQformat;
					Clusters_radar[temp_num_det - 1].yclu /= xyzQformat;
					Clusters_radar[temp_num_det - 1].xsizeclu /= xyzQformat;
					Clusters_radar[temp_num_det - 1].ysizeclu /= xyzQformat;

					if (idx >= TLVlength + tempidx)
												break;
				}*/
				break;
			}
			idx = tempidx + TLVlength;
		}
	}
} // end getdata_radar

void getPos_Radar(void){

	//attempt with no map

	moveallx = 0;
	moveally = 0;
	speedall = 0;
	meanmove = 0;
	gap = 0.056; //0.8
	uint16_t objmax = numObj_prev;
	objmax = (numObj_prev2 > objmax) ? numObj_prev2 : objmax;
	update_factor = 15;

	//if(anglez > -IMU_angle_acc*2 && anglez < IMU_angle_acc*2)
		//gap = 0.05;
	/*if(strcmp(Direction, "move") == 0){
		gap = 0.5;
	}else{
		gap = 0.1;
	}*/



	for (uint16_t temp_obj_prev = objmax; temp_obj_prev > 0; temp_obj_prev--){
		moveobjx = 0; // move on realworld x of the prev obj
		moveobjy = 0; // move on realworld y of the prev obj
		moveobjx2 = 0; // move on realworld x of the prev 2 obj
		moveobjy2 = 0; // move on realworld y of the prev 2 obj
		speedobj = 0;
		objinrange = 0; // counter of objects detected near to this previous detection
		objinrange2 = 0; // counter of objects detected near to this previous detection
//	float speedlimit = sqrtf(states_X[2]^2 + states_X[3]^2);
	//	float speedlimit = 0.8;
		if(fabs(Objects_radar_previous[temp_obj_prev - 1].speed) <= 0.8 || (Objects_radar_previous[temp_obj_prev - 1].speed <= 0 && frontback)
				||  (Objects_radar_previous[temp_obj_prev - 1].speed > 0 && frontback == false) ){

			for (uint16_t temp_obj = numObj; temp_obj > 0; temp_obj--){
				if(temp_obj_prev <= numObj_prev)
				if(Objects_radar[temp_obj - 1].xobj >= Objects_radar_previous[temp_obj_prev - 1].xobj - gap &&
				   Objects_radar[temp_obj - 1].xobj <= Objects_radar_previous[temp_obj_prev - 1].xobj + gap &&
				   Objects_radar[temp_obj - 1].yobj >= Objects_radar_previous[temp_obj_prev - 1].yobj - gap &&
				   Objects_radar[temp_obj - 1].yobj <= Objects_radar_previous[temp_obj_prev - 1].yobj + gap &&
				   fabs(Objects_radar[temp_obj - 1].speed) <= 1 && ((Objects_radar[temp_obj - 1].speed <= 0 && frontback)
					||  (Objects_radar[temp_obj - 1].speed >= 0 && frontback == false)) ){

					speedobj +=  Objects_radar[temp_obj-1].speed;
					moveobjx += ((Objects_radar_previous[temp_obj_prev - 1].xobj - Objects_radar[temp_obj - 1].xobj ));
					moveobjy += ((Objects_radar_previous[temp_obj_prev - 1].yobj - Objects_radar[temp_obj - 1].yobj ));
					objinrange++;
				} // end if previous detections

				if(temp_obj_prev <= numObj_prev2)
				if(Objects_radar[temp_obj - 1].xobj >= Objects_radar_previous_2[temp_obj_prev - 1].xobj - gap &&
				   Objects_radar[temp_obj - 1].xobj <= Objects_radar_previous_2[temp_obj_prev - 1].xobj + gap &&
				   Objects_radar[temp_obj - 1].yobj >= Objects_radar_previous_2[temp_obj_prev - 1].yobj - gap &&
				   Objects_radar[temp_obj - 1].yobj <= Objects_radar_previous_2[temp_obj_prev - 1].yobj + gap &&
				   fabs(Objects_radar[temp_obj - 1].speed) <= 0.8 && ((Objects_radar[temp_obj - 1].speed <= 0 && frontback)
					||  (Objects_radar[temp_obj - 1].speed >= 0 && frontback == false)) ){

					speedobj +=  Objects_radar[temp_obj-1].speed;
					moveobjx2 += ((Objects_radar_previous_2[temp_obj_prev - 1].xobj - Objects_radar[temp_obj - 1].xobj )/(2));
					moveobjy2 += ((Objects_radar_previous_2[temp_obj_prev - 1].yobj - Objects_radar[temp_obj - 1].yobj )/(2));
					objinrange2++;
				} // end if previous2 detections

			}
		}
		if (objinrange > 0){
			speedall += (speedobj / objinrange);
			if(objinrange2 > 0){//2
				moveallx += (((moveobjx / objinrange) + (moveobjx2 / objinrange2) ) / 2);
				moveally += (((moveobjy / objinrange) + (moveobjy2 / objinrange2) ) / 2);
			}else{
				moveallx += (moveobjx / objinrange);
				moveally += (moveobjy / objinrange);
			}

			speedall = (-speedall);
			meanmove++;
		}
	}
	if (meanmove > 0){
	switch (heading){

		case 1: //North
			speedradarx = (meanmove > 0) ? sin(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradarx;
			speedradary = (meanmove > 0) ? cos(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradary;
	//		Radarx = (meanmove > 0) ? Radarx + moveallx/meanmove : Radarx;
	//		Radary = (meanmove > 0) ? Radary + moveally/meanmove : Radary;
			Radarx = (meanmove > 0) ? states_X[0] + moveallx/meanmove : Radarx;
			Radary = (meanmove > 0) ? states_X[1] + moveally/meanmove : Radary;
			break;
		case 2: // West
			speedradarx = (meanmove > 0) ? sin(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradarx;
			speedradary = (meanmove > 0) ? cos(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradary;
	//		Radarx = (meanmove > 0) ? Radarx - moveally/meanmove : Radarx;
	//		Radary = (meanmove > 0) ? Radary - moveallx/meanmove : Radary;
			Radarx = (meanmove > 0) ? states_X[0] - moveally/meanmove : Radarx;
			Radary = (meanmove > 0) ? states_X[1] - moveallx/meanmove : Radary;
			break;
		case 3: // South
			speedradarx = (meanmove > 0) ? sin(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradarx;
			speedradary = (meanmove > 0) ? cos(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradary;
	//		Radarx = (meanmove > 0) ? Radarx - moveallx/meanmove : Radarx;
	//		Radary = (meanmove > 0) ? Radary - moveally/meanmove : Radary;
			Radarx = (meanmove > 0) ? states_X[0] - moveallx/meanmove : Radarx;
			Radary = (meanmove > 0) ? states_X[1] - moveally/meanmove : Radary;
			break;
		case 4: //East
			speedradarx = (meanmove > 0) ? sin(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradarx;
			speedradary = (meanmove > 0) ? cos(angle_accumulated * 3.14159 / 180) * speedall/meanmove  : speedradary;
	//		Radarx = (meanmove > 0) ? Radarx + moveally/meanmove : Radarx;
	//		Radary = (meanmove > 0) ? Radary + moveallx/meanmove : Radary;
			Radarx = (meanmove > 0) ? states_X[0] + moveally/meanmove : Radarx;
			Radary = (meanmove > 0) ? states_X[1] + moveallx/meanmove : Radary;
			break;
		case 5:
			break;

		}

	}

	if(Objects_radar_previous[0].xobj == 0){
		memcpy(Objects_radar_previous,Objects_radar,sizeof(Objects_radar));
		numObj_prev = numObj;
		numClu_prev = numClu;

		memcpy(Objects_radar_previous_2,Objects_radar,sizeof(Objects_radar));
	    numObj_prev2 = numObj;
	    numClu_prev2 = numClu;

	}else if(counter_previous_update++ >= update_factor - 1){
		memcpy(Objects_radar_previous_2,Objects_radar_previous,sizeof(Objects_radar_previous));
		numObj_prev2 = numObj_prev;
		numClu_prev2 = numClu_prev;

		memcpy(Objects_radar_previous,Objects_radar,sizeof(Objects_radar));
	    numObj_prev = numObj;
	    numClu_prev = numClu;

		counter_previous_update = 0;
	}




}


/* End Functions*/
