#include "fdcan.h"

FDCAN_HandleTypeDef hfdcan2;

void MX_FDCAN2_Init(void)
{
    hfdcan2.Instance = FDCAN2;
    hfdcan2.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    hfdcan2.Init.Mode = FDCAN_MODE_NORMAL;
    hfdcan2.Init.AutoRetransmission = ENABLE;
    hfdcan2.Init.TransmitPause = DISABLE;
    hfdcan2.Init.ProtocolException = DISABLE;
    hfdcan2.Init.NominalPrescaler = 20U;
    hfdcan2.Init.NominalSyncJumpWidth = 2U;
    hfdcan2.Init.NominalTimeSeg1 = 13U;
    hfdcan2.Init.NominalTimeSeg2 = 2U;
    hfdcan2.Init.DataPrescaler = 20U;
    hfdcan2.Init.DataSyncJumpWidth = 2U;
    hfdcan2.Init.DataTimeSeg1 = 13U;
    hfdcan2.Init.DataTimeSeg2 = 2U;
    hfdcan2.Init.MessageRAMOffset = 0U;
    hfdcan2.Init.StdFiltersNbr = 0U;
    hfdcan2.Init.ExtFiltersNbr = 1U;
    hfdcan2.Init.RxFifo0ElmtsNbr = 8U;
    hfdcan2.Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxFifo1ElmtsNbr = 0U;
    hfdcan2.Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.RxBuffersNbr = 0U;
    hfdcan2.Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    hfdcan2.Init.TxEventsNbr = 0U;
    hfdcan2.Init.TxBuffersNbr = 0U;
    hfdcan2.Init.TxFifoQueueElmtsNbr = 8U;
    hfdcan2.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
    hfdcan2.Init.TxElmtSize = FDCAN_DATA_BYTES_8;
    if (HAL_FDCAN_Init(&hfdcan2) != HAL_OK)
    {
        Error_Handler();
    }
}

void HAL_FDCAN_MspInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

    if (fdcanHandle->Instance != FDCAN2) return;

    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_FDCAN;
    PeriphClkInitStruct.FdcanClockSelection = RCC_FDCANCLKSOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
        Error_Handler();
    }

    __HAL_RCC_FDCAN_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_FDCAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    HAL_NVIC_SetPriority(FDCAN2_IT0_IRQn, 5U, 0U);
    HAL_NVIC_EnableIRQ(FDCAN2_IT0_IRQn);
}

void HAL_FDCAN_MspDeInit(FDCAN_HandleTypeDef *fdcanHandle)
{
    if (fdcanHandle->Instance != FDCAN2) return;
    __HAL_RCC_FDCAN_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_5 | GPIO_PIN_13);
    HAL_NVIC_DisableIRQ(FDCAN2_IT0_IRQn);
}
