/*Includes*/
#include "main.h"
#include "cmsis_os.h"
#include "uarts.h"



/*Variable */
uint8_t syncRadar = 0b00000000;
uint16_t counterRadar = 0x00;
uint32_t lengthdataRadar = 0x00;


/*functions*/


void USART_IMU_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	 static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	 static int rx_index = 0;
	 if (USART_IMU->ISR & USART_ISR_ORE) // Overrun Error
		 USART_IMU->ICR = USART_ICR_ORECF;
	 if (USART_IMU->ISR & USART_ISR_NE) // Noise Error
		 USART_IMU->ICR = USART_ICR_NCF;
	 if (USART_IMU->ISR & USART_ISR_FE) // Framing Error
		 USART_IMU->ICR = USART_ICR_FECF;
	 if (USART_IMU->ISR & USART_ISR_RXNE) // Received character?
	 {
		 char rx = (char)(USART_IMU->RDR & 0xFF);
		 if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		 {
			 if (rx_index != 0) // Line has some content?
			 {
				 //rx_buffer[rx_index++] = 0; // Add NUL if required down stream
				 QueueBuffer(rx_buffer, rx_index,3); // Copy to queue from live dynamic receive buffer
				 //HAL_UART_Transmit(&hlpuart1, rx_buffer, rx_index, 1000);
				 rx_index = 0; // Reset content pointer
			 }
		 }
		 else
		 {
			 if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
			 rx_index = 0;
			 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		 }
	 }

}


void USART_UWB_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	 static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	 static int rx_index = 0;
	 if (USART_UWB->ISR & USART_ISR_ORE) // Overrun Error
		 USART_UWB->ICR = USART_ICR_ORECF;
	 if (USART_UWB->ISR & USART_ISR_NE) // Noise Error
		 USART_UWB->ICR = USART_ICR_NCF;
	 if (USART_UWB->ISR & USART_ISR_FE) // Framing Error
		 USART_UWB->ICR = USART_ICR_FECF;
	 if (USART_UWB->ISR & USART_ISR_RXNE) // Received character?
	 {
		 char rx = (char)(USART_UWB->RDR & 0xFF);
		 if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		 {
			 if (rx_index != 0) // Line has some content?
			 {
				 //rx_buffer[rx_index++] = 0; // Add NUL if required down stream
				 QueueBuffer(rx_buffer, rx_index, 1); // Copy to queue from live dynamic receive buffer
				 //HAL_UART_Transmit(&hlpuart1, rx_buffer, rx_index, 1000);
				 rx_index = 0; // Reset content pointer
			 }
		 }
		 else
		 {
			 if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
			 rx_index = 0;
			 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		 }
	 }

}


void USART_Radar_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	 static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	 static int rx_index = 0;
	 if (USART_Radar->ISR & USART_ISR_ORE) // Overrun Error
		 USART_Radar->ICR = USART_ICR_ORECF;
	 if (USART_Radar->ISR & USART_ISR_NE) // Noise Error
		 USART_Radar->ICR = USART_ICR_NCF;
	 if (USART_Radar->ISR & USART_ISR_FE) // Framing Error
		 USART_Radar->ICR = USART_ICR_FECF;
	 if (USART_Radar->ISR & USART_ISR_RXNE) // Received character?
	 {
		 char rx = (char)(USART_Radar->RDR & 0xFF);

		 if ( (syncRadar == 0xFF) && counterRadar == lengthdataRadar-1 && counterRadar > 15)
		 {
			 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
			 QueueBuffer(rx_buffer, rx_index, 2); // Copy to queue from live dynamic receive buffer
			 rx_index = 0; // Reset content pointer
			 counterRadar = 0;
			 lengthdataRadar = 0;
			 syncRadar = 0b00000000;

		 }else
		 {
			 if ((rx_index == LINEMAX)) // If resync or overflows pull back to start
			 rx_index = 0;
			 if(syncRadar == 0xFF)
			 {
				 if (counterRadar >= 12 && counterRadar <= 15){
					 uint32_t templengthradar = rx << (8 * (counterRadar - 12));
					 lengthdataRadar += templengthradar;
				 }
				 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
				 counterRadar++;
			 }else{
				 if (counterRadar)
				 {
					 uint8_t tempsync[8] = {0x02, 0x01, 0x04, 0x03, 0x06, 0x05, 0x08, 0x07};

					 if (rx == tempsync[counterRadar])
					 {
						 syncRadar += (1 << counterRadar);
						 counterRadar++;
						 rx_buffer[rx_index++] = rx; // Copy to buffer and increment

					 }else
					 {
						 syncRadar = 0b00000000;
						 counterRadar = 0x00;
						 rx_index = 0;
					 }

				 }else
				 {
					 if(rx == 0x02){
						 syncRadar++;
						 counterRadar++;
						 rx_buffer[rx_index++] = rx; // Copy to buffer and increment

					 }
				 }
			 }

		 }


	 }

}

void USART_Lidar_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	 static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	 static int rx_index = 0;
	 if (USART_Lidar->ISR & USART_ISR_ORE) // Overrun Error
		 USART_Lidar->ICR = USART_ICR_ORECF;
	 if (USART_Lidar->ISR & USART_ISR_NE) // Noise Error
		 USART_Lidar->ICR = USART_ICR_NCF;
	 if (USART_Lidar->ISR & USART_ISR_FE) // Framing Error
		 USART_Lidar->ICR = USART_ICR_FECF;
	 if (USART_Lidar->ISR & USART_ISR_RXNE) // Received character?
	 {
		 char rx = (char)(USART_Lidar->RDR & 0xFF);
		 if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		 {
			 if (rx_index != 0) // Line has some content?
			 {
				 //rx_buffer[rx_index++] = 0; // Add NUL if required down stream
				 QueueBuffer(rx_buffer, rx_index, 0); // Copy to queue from live dynamic receive buffer
				 //HAL_UART_Transmit(&hlpuart1, rx_buffer, rx_index, 1000);
				 rx_index = 0; // Reset content pointer
			 }
		 }
		 else
		 {
			 if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
			 rx_index = 0;
			 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		 }
	 }

}


void USART_US_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	 static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	 static int rx_index = 0;
	 if (USART_US->ISR & USART_ISR_ORE) // Overrun Error
		 USART_US->ICR = USART_ICR_ORECF;
	 if (USART_US->ISR & USART_ISR_NE) // Noise Error
		 USART_US->ICR = USART_ICR_NCF;
	 if (USART_US->ISR & USART_ISR_FE) // Framing Error
		 USART_US->ICR = USART_ICR_FECF;
	 if (USART_US->ISR & USART_ISR_RXNE) // Received character?
	 {
		 char rx = (char)(USART_US->RDR & 0xFF);
		 if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		 {
			 if (rx_index != 0) // Line has some content?
			 {
				 //rx_buffer[rx_index++] = 0; // Add NUL if required down stream
				 QueueBuffer(rx_buffer, rx_index, 4); // Copy to queue from live dynamic receive buffer
				 //HAL_UART_Transmit(&hlpuart1, rx_buffer, rx_index, 1000);
				 rx_index = 0; // Reset content pointer
			 }
		 }
		 else
		 {
			 if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
			 rx_index = 0;
			 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		 }
	 }

}


void USART_PC_IRQHandler(void) // Sync and Queue NMEA Sentences
{
	 static char rx_buffer[LINEMAX + 1]; // Local holding buffer to build line, w/NUL
	 static int rx_index = 0;
	 if (USART_PC->ISR & USART_ISR_ORE) // Overrun Error
		 USART_PC->ICR = USART_ICR_ORECF;
	 if (USART_PC->ISR & USART_ISR_NE) // Noise Error
		 USART_PC->ICR = USART_ICR_NCF;
	 if (USART_PC->ISR & USART_ISR_FE) // Framing Error
		 USART_PC->ICR = USART_ICR_FECF;
	 if (USART_PC->ISR & USART_ISR_RXNE) // Received character?
	 {
		 char rx = (char)(USART_PC->RDR & 0xFF);
		 if ((rx == '\r') || (rx == '\n')) // Is this an end-of-line condition, either will suffice?
		 {
			 if (rx_index != 0) // Line has some content?
			 {
				 //rx_buffer[rx_index++] = 0; // Add NUL if required down stream
				 QueueBuffer(rx_buffer, rx_index, 5); // Copy to queue from live dynamic receive buffer
				 //HAL_UART_Transmit(&hlpuart1, rx_buffer, rx_index, 1000);
				 rx_index = 0; // Reset content pointer
			 }
		 }
		 else
		 {
			 if ((rx == '$') || (rx_index == LINEMAX)) // If resync or overflows pull back to start
			 rx_index = 0;
			 rx_buffer[rx_index++] = rx; // Copy to buffer and increment
		 }
	 }

}


