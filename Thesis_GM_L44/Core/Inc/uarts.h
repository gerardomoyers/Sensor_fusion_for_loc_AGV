/*
 * uarts.h
 *
 *  Created on: Feb 12, 2020
 *      Author: Gerardo.Moyers
 */
/*Defines*/
#define false 0
#define true 1

#define USART_PC_IRQHandler LPUART1_IRQHandler
#define USART_PC LPUART1
#define USART_UWB_IRQHandler USART2_IRQHandler
#define USART_UWB USART2
#define USART_IMU_IRQHandler UART4_IRQHandler
#define USART_IMU UART4
#define USART_Radar_IRQHandler USART3_IRQHandler
#define USART_Radar USART3
#define USART_Lidar_IRQHandler  USART1_IRQHandler
#define USART_Lidar USART1
#define USART_US_IRQHandler UART5_IRQHandler
#define USART_US UART5




#ifdef USART_CR1_TXEIE_TXFNFIE // FIFO Support (L4R9)
#define USART_CR1_TXEIE USART_CR1_TXEIE_TXFNFIE
#define USART_ISR_TXE USART_ISR_TXE_TXFNF
#define USART_CR1_RXNEIE USART_CR1_RXNEIE_RXFNEIE
#define USART_ISR_RXNE USART_ISR_RXNE_RXFNE
#endif

#define LINEMAX 2000 // Maximal allowed/expected line length


#ifndef INC_UARTS_H_
#define INC_UARTS_H_


void USART_UWB_IRQHandler(void);



#endif /* INC_UARTS_H_ */
