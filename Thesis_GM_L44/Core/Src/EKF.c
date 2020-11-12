/*
 * EKF.c
 *
 *  Created on: Aug 17, 2020
 *      Author: Gerardo.Moyers
 */


/* Includes*/

#include "EKF.h"
#include <stdio.h>
#include <stdlib.h>
#include "main.h"

/* End Includes*/

/////////////////////////////// Sensor fusion functions

// Matrix multiplication
void matrix_multiplication(float Matrix_A[100],float Matrix_B[100],float Matrix_result[100], uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb){

	  for (uint8_t i = 0; i < rowa; i++) {
	          for (uint8_t j = 0; j < colb; j++) {
	        	  Matrix_result[i*colb + j] = 0;

	              for (uint8_t k = 0; k < cola; k++) {
	            	  Matrix_result[i*colb + j] += Matrix_A[i*cola + k] * Matrix_B[k*colb + j];
	              }
	          }
	      }
}

void matrix_Transpose (float Mat_A[100],float Mat_B[100],   uint8_t row, uint8_t col){

	float Mat_transpose[col][row];
	  for (uint8_t i = 0; i < row; i++) {
	          for (uint8_t j = 0; j < col; j++) {
	        	  Mat_transpose[j][i] =  Mat_A[i*col + j];
	          }
	      }
		memcpy(Mat_B,Mat_transpose,sizeof(Mat_transpose));
}

void matrix_sum(float Matrix_A[100],float Matrix_B[100],float Matrix_result[100], uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb){

	uint8_t rows = (rowa > rowb) ? rowb : rowa;
	  for (uint8_t i = 0; i < rows; i++) {
	          for (uint8_t j = 0; j < colb; j++) {
	        	  if(colb == 1){
	        		  for (uint8_t k = 0; k < cola; k++){
	        			  Matrix_result[i*cola + k] = Matrix_A[i*cola + k] + Matrix_B[i];
	        		  }
	        	  }else{
	            	  Matrix_result[i*colb + j] = Matrix_A[i*cola + j] + Matrix_B[i*colb + j];
	        	  }
	          }
	      }
}

void matrix_sub(float Matrix_A[100],float Matrix_B[100],float Matrix_result[100], uint8_t rowa, uint8_t cola, uint8_t rowb, uint8_t colb){

	uint8_t rows = (rowa > rowb) ? rowb : rowa;
	  for (uint8_t i = 0; i < rows; i++) {
	          for (uint8_t j = 0; j < colb; j++) {
	        	  if(colb == 1){
	        		  for (uint8_t k = 0; k < cola; k++){
	        			  Matrix_result[i*cola + k] = Matrix_A[i*cola + k] - Matrix_B[i];
	        		  }
	        	  }else{
	            	  Matrix_result[i*colb + j] = Matrix_A[i*cola + j] - Matrix_B[i*colb + j];
	        	  }
	          }
	      }
}


void InverseOfMatrix(float Matrix_A_A[100], int order) {
    // Matrix Declaration.

    float temp;
    float ident [order * order];
    float Matrix_A [order * order];

	  memcpy(Matrix_A,Matrix_A_A,sizeof(Matrix_A));


    // Create the augmented matrix
    // Add the identity matrix
    // of order at the end of original matrix.
    for (uint8_t i = 0; i < order; i++) {

        for (uint8_t j = 0; j < order; j++) {

            // Add '1' at the diagonal places of
            // the matrix to create a identity matirx
            if (j == i)
                ident[i*order + j] = 1;
            else
            	ident[i*order + j] = 0;
        }
    }

    // Replace a row by sum of itself and a
    // constant multiple of another row of the matrix
    for (uint8_t i = 0; i < order; i++) {
    	if(Matrix_A[i*order + i] == 0){
    		for (uint8_t k = i + 1; k < order; k++) {
    			if(Matrix_A[k*order + i] != 0){
    				float norm =  Matrix_A[k*order + i];
    				 for (uint8_t j = 0; j < order; j++) {

						// Swapping of the row, if above
						// condition satisfied.
						temp = Matrix_A[i*order + j];
						Matrix_A[i*order + j] = Matrix_A[k*order + j]/norm;
						Matrix_A[k*order + j] = temp;

						temp = ident[i*order + j];
						ident[i*order + j] = ident[k*order + j]/norm;
						ident[k*order + j] = temp;
					}
    				 break;
    			}
			}
		}// end if pivot

        for (uint8_t j = 0; j < order; j++) {

            if (j != i) {

                temp = Matrix_A[j*order + i] / Matrix_A[i*order + i];
                for (int k = 0; k < order; k++) {

                	Matrix_A[j*order + k] -= Matrix_A[i*order + k] * temp;
                	ident[j*order + k] -= ident[i*order + k] * temp;

                }
            }else{
            	temp = Matrix_A[i*order + i];
                for (int k = 0; k < order; k++) {

                	Matrix_A[i*order + k] /=  temp;
                	ident[j*order + k] /= temp;

                }
            }
        }
    }

	  memcpy(Matrix_A_A,ident,sizeof(ident));

}

void EKF_prediction (void){

	float result_1 [4][4];
	float result_2 [4][4];
	float F_transpose [4][4];
	int dummyw = TIM7 -> CNT;
	float w1  = dummyw % 462;
	float w2  = dummyw % 120;
	float w3  = dummyw % 73;
	float w4  = dummyw % 65;
	float w[4] = {w1/100000 - 0.00231,
			w2/100000 - 0.00060,
			w3/100000 - 0.00032624,
			w4/100000 - 0.00065249};
	deltatime_EKF = deltatime_imu/10;

	// Predict x
	states_X[2] = /*states_X_previous[2] +*/ (accel_x_g2 + w[0]) * deltatime_EKF * (sinf(angle_accumulated * 3.14159 / 180) + w[2]);
	states_X[3] = /*states_X_previous[3] +*/ (accel_x_g2 + w[0]) * deltatime_EKF * (cosf(angle_accumulated * 3.14159 / 180) + w[3]);
	states_X[0] = states_X_previous[0] + states_X[2] * deltatime_EKF + 1/2 * (accel_x_g2 + w[0]) * pow(deltatime_EKF,2);
	states_X[1] = states_X_previous[1] + states_X[3] * deltatime_EKF + 1/2 * (accel_y_g2 + w[1]) * pow(deltatime_EKF,2);



	//Predict P
	matrix_multiplication(jacobian_F, covariance_P, result_1, 4, 4, 4, 4); // F*P
	matrix_Transpose(jacobian_F, F_transpose, 4, 4);//F'
	matrix_multiplication(result_1, F_transpose, result_2, 4, 4, 4, 4);//F*P*F'
	matrix_sum(result_2, standar_dev_Q, covariance_P, 4, 4, 4, 4);// P = F*P*F' + Q

}

void EKF_update (void){

	float Kalman_gain_K[4][4]; // Kalman Gain P H' (S)^-1,   S = H P H' + R
	float H_transpose [4][4];
	float result_1 [4][4];
	float result_2 [4][4];
	float Identity_Matrix[4][4] = {{1,0,0,0},
		  				 {0,1,0,0},
		  				 {0,0,1,0},
		  			 	 {0,0,0,1}};

	// Calculate S
	matrix_multiplication(jacobian_H, covariance_P, result_1, 4, 4, 4, 4); // H*P
	matrix_Transpose(jacobian_H,H_transpose,4, 4);// H'
	matrix_multiplication(result_1, H_transpose, result_2, 4, 4, 4, 4);// H*P*H'
	matrix_sum(result_2, standar_dev_R, result_2, 4, 4, 4, 4); // S

	//Calculate K
	matrix_multiplication(covariance_P, H_transpose, result_1,4, 4, 4, 4); // P * H'
	InverseOfMatrix(result_2,4); // S^-1
	matrix_multiplication(result_1, result_2, Kalman_gain_K, 4, 4, 4, 4); // Kalman gain
	// update x
	matrix_sub(measurementes_Z, states_X, result_1, 4, 1, 4, 1);// z-h(xk)
	matrix_multiplication(Kalman_gain_K, result_1, result_2, 4, 4, 4, 1);//
	matrix_sum(states_X, result_2, states_X, 4, 1, 4, 1);

	//update P
	matrix_multiplication(Kalman_gain_K, jacobian_H, result_1, 4, 4, 4, 4); // K*H
	matrix_sub(Identity_Matrix, result_1, Identity_Matrix, 4, 4, 4, 4);// I - KH
	matrix_multiplication(Identity_Matrix, covariance_P, result_2, 4, 4, 4, 4); // P+
	memcpy(covariance_P,result_2,sizeof(result_2));


	//
}


