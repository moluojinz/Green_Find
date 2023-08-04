//
// Created by ShiF on 2023/8/4.
//

#include "stm32f4xx_hal.h"
#include "tim.h"
#include "usart.h"
#include "string.h"
#include "openmv.h"
#include "A_headfile.h"
/**
 * DMA传输    huart1
 * mv_rx    PA10
 * mv_tx    PA9
 */
#define MV_RVSIZE 255
char MVRvBuff[MV_RVSIZE] = {0};  //存放串口6（调试用）接收的第一手数据
char MVTBuff[MV_RVSIZE] = {0};    //进行一定变换
void mv_Init(void) {
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart1, (uint8_t *) MVRvBuff, MV_RVSIZE);
}


void mv_getdata(UART_HandleTypeDef *huart) {
    HAL_UART_DMAStop(huart);
    //解包预处理
    //将串口收到的数据进行处理，新的数组以数字开头，便于之后字符转浮点数的运算
    uint8_t data_length = MV_RVSIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);   //计算接收到的数据长度
    if (MVRvBuff[0] == 0x2c && MVRvBuff[1] == 0x12) {
        memcpy(MVTBuff, &MVRvBuff[2], 4);
        servo_tar.asis_x = (short) (MVTBuff[0] | MVTBuff[1] << 8);
        servo_tar.asis_y = (short) (MVTBuff[2] | MVTBuff[3] << 8);

    }

    memset(MVRvBuff, 0, data_length);                                            //清零接收缓冲区
    data_length = 0;
    HAL_UART_Receive_DMA(huart, (uint8_t *) MVRvBuff, MV_RVSIZE);
}

void MV_UartIrqHandler(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART1)                                   //判断是否是串口1
    {
        if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET)   //判断是否是空闲中断
        {
            __HAL_UART_CLEAR_IDLEFLAG(huart);                     //清楚空闲中断标志（否则会一直不断进入中断）
            mv_getdata(huart);                          //调用中断处理函数

        }
    }
}

