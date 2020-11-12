/*
 * EKF.h
 *
 *  Created on: Aug 17, 2020
 *      Author: Gerardo.Moyers
 */
///// Includes
#include "main.h"
#include "cmsis_os.h"
#include "uarts.h"
#include "math.h"

/*Defines*/

#ifndef INC_EKF_H_
#define INC_EKF_H_

void matrix_multiplication(float Matrix_A[100],float Matrix_B[100],float Matrix_result[100], uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb);//( uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb);
void matrix_Transpose (float Mat_A[100],float Mat_B[100],  uint8_t row, uint8_t col);
void matrix_sum(float Matrix_A[100],float Matrix_B[100],float Matrix_result[100], uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb);//( uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb);
void matrix_sub(float Matrix_A[100],float Matrix_B[100],float Matrix_result[100], uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb);//( uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb);
void InverseOfMatrix(float Matrix_A_A[100], int order);//(int order) ;
void EKF_prediction (void);
void EKF_update (void);


#endif /* INC_EKF_H_ */
