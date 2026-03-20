#include "bsp_uart.h"
#include "string.h"
#include "usart.h"

/* 来自main的外部DMA句柄 */
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

/* 存储uart句柄结构的指针map*/
static UART_Instance_t *g_uart_instances[3] = {NULL, NULL, NULL};

/* 静态函数原型 */
static UART_Status_t UART_DMA_Stop_Receive(const UART_Instance_t *uart_ins);

/**
 * @brief  UART发送任务函数
 * @param  argument: 指向UART句柄结构的指针
 * @retval 无
 */
static void UART_TxTask(const void *argument)
{
    UART_Instance_t *uart_ins = (UART_Instance_t *)argument;

    while (1)
    {
        /* 等待发送请求 */
        const osEvent event = osMailGet(uart_ins->txMailHandle, osWaitForever);
        if (event.status == osEventMail) {
            uint8_t *block = (uint8_t*)event.value.p;
            uint16_t len = block[0] | (block[1] << 8);
            uint8_t *data = block + 2;

            if (len > 0) {
                /* 获取缓冲区互斥量以确保线程安全 */
                if (osSemaphoreWait(uart_ins->txSemaphore, 1000) == osOK)
                {
                    /* 开始DMA发送 */
                    if(HAL_UART_Transmit_DMA(uart_ins->handle, data, len) != HAL_OK) {
                        /* 发送失败，立即释放信号量 */
                        osSemaphoreRelease(uart_ins->txSemaphore);
                        if(uart_ins->ErrorCallback != NULL) {
                            uart_ins->ErrorCallback(HAL_UART_GetError(uart_ins->handle));
                        }
                    }
                }

                osMailFree(uart_ins->txMailHandle, block);
            }
        }
    }
}

/**
 * @brief  停止DMA接收
 * @param  uart_ins: 指向UART句柄结构的指针
 * @retval 停止结果 (UART_Status_t)
 */
static UART_Status_t UART_DMA_Stop_Receive(const UART_Instance_t *uart_ins)
{
    if(uart_ins == NULL || uart_ins->handle->hdmarx == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }

    /* 禁用DMA流 */
    __HAL_DMA_DISABLE(uart_ins->handle->hdmarx);
    while(uart_ins->handle->hdmarx->Instance->CR & DMA_SxCR_EN) {
        __HAL_DMA_DISABLE(uart_ins->handle->hdmarx);
    }

    return UART_OK;
}

/**
 * @brief  使用DMA和回调初始化UART外设
 * @param  uart_ins: 指向UART句柄结构的指针
 * @param  huart: 指向HAL UART句柄
 * @param  rxCallback: 接收回调函数指针
 * @param  errorCallback: 错误回调函数指针（可为NULL）
 * @param txBufferSize: 发送缓冲区大小
 * @param rxBufferSize: 接收缓冲区大小
 * @retval 初始化结果 (UART_Status_t)
 */
UART_Status_t BSP_UART_Init(UART_Instance_t *uart_ins,
                           UART_HandleTypeDef *huart,
                           const UART_RxCallback_t rxCallback,
                           const UART_ErrorCallback_t errorCallback,
                           const uint16_t txBufferSize,
                           const uint16_t rxBufferSize)
{
    if(uart_ins == NULL || huart == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }

    /* 验证DMA句柄 */
    if (huart->hdmarx == NULL) {
        return UART_ERROR_INVALID_PARAM;
    }
    if (huart->hdmatx == NULL && txBufferSize != 0) {
        return UART_ERROR_INVALID_PARAM;
    }


    /* 检查UART实例是否已初始化 */
    if(uart_ins->handle != NULL) {
        return UART_ERROR_ALREADY_INIT;
    }

    /* 检查Buffer设置是否超过上限*/
    if (txBufferSize > TX_BUFFER_SIZE || rxBufferSize > RX_BUFFER_SIZE) {
        return UART_ERROR_INVALID_PARAM;
    }

    /* 初始化UART句柄结构 */
    uart_ins->handle = huart;
    uart_ins->RxCallback = rxCallback;
    uart_ins->ErrorCallback = errorCallback;
    uart_ins->rxBufferSize = rxBufferSize;
    uart_ins->txBufferSize = txBufferSize;
    uart_ins->StackSize = txBufferSize * 4;
    uart_ins->txBusy = 0;

    /* 加入map */
    if(huart->Instance == USART1) {
        g_uart_instances[0] = uart_ins;
    } else if(huart->Instance == USART3) {
        g_uart_instances[1] = uart_ins;
    } else if(huart->Instance == USART6) {
        g_uart_instances[2] = uart_ins;
    }

    /* 清空缓冲区 */
    memset(uart_ins->rxBuffer, 0, sizeof(uart_ins->rxBuffer));

    if (txBufferSize != 0)
    {
        /* 创建用于线程安全缓冲区访问的互斥量 */
        osSemaphoreDef(bufferMutex);
        uart_ins->txSemaphore = osSemaphoreCreate(osSemaphore(bufferMutex), 1);
        if(uart_ins->txSemaphore == NULL) {
            return UART_ERROR;
        }

        /* 创建用于发送请求的消息队列 */
        osMailQDef(txMail, uart_ins->txBufferSize, uint8_t);
        uart_ins->txMailHandle = osMailCreate(osMailQ(txMail), NULL);
        if(uart_ins->txMailHandle == NULL) {
            return UART_ERROR;
        }

        /* 创建发送任务 */
        osThreadDef(UARTtxTask, UART_TxTask, osPriorityNormal, 1, uart_ins->StackSize);
        uart_ins->txTaskHandle = osThreadCreate(osThread(UARTtxTask), uart_ins);
        if(uart_ins->txTaskHandle == NULL) {
            return UART_ERROR;
        }
    }


    /* 启用UART DMA接收 */
    SET_BIT(uart_ins->handle->Instance->CR3, USART_CR3_DMAR);
    __HAL_UART_ENABLE_IT(uart_ins->handle, UART_IT_IDLE);
    /* 配置DMA用于双缓冲接收 */
    UART_DMA_Stop_Receive(uart_ins);

    /* 配置DMA参数 */
    uart_ins->handle->hdmarx->Instance->PAR = (uint32_t)&uart_ins->handle->Instance->DR;
    uart_ins->handle->hdmarx->Instance->M0AR = (uint32_t)(uart_ins->rxBuffer[0]);
    uart_ins->handle->hdmarx->Instance->M1AR = (uint32_t)(uart_ins->rxBuffer[1]);
    uart_ins->handle->hdmarx->Instance->NDTR = uart_ins->rxBufferSize;

    /* 启用双缓冲模式 */
    SET_BIT(uart_ins->handle->hdmarx->Instance->CR, DMA_SxCR_DBM);
    __HAL_DMA_ENABLE(uart_ins->handle->hdmarx);
    /* 开始DMA接收 */
    return UART_OK;
}

/**
 * @brief  通过UART使用DMA和FreeRTOS任务发送数据
 * @param  uart_ins: 指向UART句柄结构的指针
 * @param  pData: 指向数据缓冲区的指针
 * @param  Size: 要发送的数据大小
 * @param  Timeout: 超时值（毫秒）
 * @retval 发送结果 (UART_Status_t)
 */
UART_Status_t BSP_UART_Transmit_To_Mail(const UART_Instance_t *uart_ins, const uint8_t *pData, const uint16_t Size, const uint32_t Timeout)
{
    if(uart_ins == NULL || pData == NULL || Size == 0 || Size > TX_BUFFER_SIZE || uart_ins->txBufferSize == 0) {
        return UART_ERROR_INVALID_PARAM;
    }

    /* 获取邮箱地址 */
    uint8_t* mail_ptr = osMailAlloc(uart_ins->txMailHandle, 0);
    if (!mail_ptr)
        return UART_ERROR;

    /* 准备消息结构 */
    mail_ptr[0] = Size & 0xFF;
    mail_ptr[1] = (Size >> 8) & 0xFF;
    memcpy(mail_ptr+2, pData, Size);

    /* 发送消息到传输队列 */
    const osStatus status = osMailPut(uart_ins->txMailHandle, mail_ptr);

    if(status == osOK) {
        return UART_OK;
    }
    osMailFree(uart_ins->txMailHandle, mail_ptr);
    return UART_ERROR;
}

/**
 * @brief  UART DMA发送完成回调
 * @param  uart_ins: 指向UART句柄结构的指针
 * @retval 无
 */
void BSP_UART_TxCpltCallback(UART_Instance_t *uart_ins)
{
    if(uart_ins == NULL || uart_ins->txBufferSize == 0) {
        return;
    }
    /* 释放标志位 */
    osSemaphoreRelease(uart_ins->txSemaphore);
    /* 清除发送忙标志 */
    uart_ins->txBusy = 0;
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    for(int i = 0; i < 3; i++) {
        if(g_uart_instances[i] != NULL &&
           g_uart_instances[i]->handle == huart) {
            BSP_UART_TxCpltCallback(g_uart_instances[i]);
            break;
        }
    }
}

/**
 * @brief  UART中断处理程序（从HAL_UART_IRQHandler调用）
 * @param  uart_ins: 指向UART句柄结构的指针
 * @retval 无
 */
void BSP_UART_IRQHandler(const UART_Instance_t *uart_ins)
{
    if(uart_ins == NULL || uart_ins->handle == NULL) {
        return;
    }

    /* 检查IDLE线路检测 */
    if(__HAL_UART_GET_FLAG(uart_ins->handle, UART_FLAG_IDLE))
    {
        __HAL_UART_CLEAR_IDLEFLAG(uart_ins->handle);

        /* 处理接收到的数据 */
        uint16_t receivedLength;

        /* 确定当前使用哪个缓冲区 */
        if((uart_ins->handle->hdmarx->Instance->CR & DMA_SxCR_CT) == RESET)
        {
            UART_DMA_Stop_Receive(uart_ins);
            receivedLength = uart_ins->rxBufferSize - uart_ins->handle->hdmarx->Instance->NDTR;
            uart_ins->handle->hdmarx->Instance->NDTR = uart_ins->rxBufferSize;
            uart_ins->handle->hdmarx->Instance->CR |= DMA_SxCR_CT;

            /* 如果已注册则调用接收回调 */
            if(uart_ins->RxCallback != NULL && receivedLength > 0) {
                uart_ins->RxCallback(uart_ins->rxBuffer[0], receivedLength);
            }
            __HAL_DMA_ENABLE(uart_ins->handle->hdmarx);
        } else {
            UART_DMA_Stop_Receive(uart_ins);
            receivedLength = uart_ins->rxBufferSize - uart_ins->handle->hdmarx->Instance->NDTR;
            uart_ins->handle->hdmarx->Instance->NDTR = uart_ins->rxBufferSize;
            uart_ins->handle->hdmarx->Instance->CR &= ~(DMA_SxCR_CT);

            /* 如果已注册则调用接收回调 */
            if(uart_ins->RxCallback != NULL && receivedLength > 0) {
                uart_ins->RxCallback(uart_ins->rxBuffer[1], receivedLength);
            }
            __HAL_DMA_ENABLE(uart_ins->handle->hdmarx);
        }
    }

    /* 如果需要，处理其他UART中断 */
    if(__HAL_UART_GET_FLAG(uart_ins->handle, UART_FLAG_RXNE)) {
        __HAL_UART_CLEAR_FLAG(uart_ins->handle, UART_FLAG_RXNE);
    }
}

UART_Instance_t* get_uart_ins_map(const uint8_t uart)
// 仅给stm32f4xx_it使用，原本把USARTx_IRQHandler放在本文件下每次重新生成CubeMX都把it里的都删一遍太麻烦了
{
    return g_uart_instances[uart];
}