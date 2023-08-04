//
// Created by ShiF on 2023/8/4.
//

#ifndef GREEN_FIND_OPENMV_H
#define GREEN_FIND_OPENMV_H
void mv_Init(void);
void mv_getdata(UART_HandleTypeDef *huart);
void MV_UartIrqHandler(UART_HandleTypeDef* huart);
#endif //GREEN_FIND_OPENMV_H
