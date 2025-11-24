/**
  ******************************************************************************
  * @file    stm32f1xx_hal_spi.h
  * @author  MCD Application Team
  * @brief   Header file of SPI HAL module.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2016 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F1xx_HAL_SPI_H
#define __STM32F1xx_HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal_def.h"

/** @addtogroup STM32F1xx_HAL_Driver
  * @{
  */

/** @addtogroup SPI
  * @{
  */

/* Exported types ------------------------------------------------------------*/
/** @defgroup SPI_Exported_Types SPI Exported Types
  * @{
  */

/**
  * @brief  SPI Configuration Structure definition
  */
typedef struct
{
  uint32_t Mode;                /*!< Specifies the SPI operating mode.
                                     This parameter can be a value of @ref SPI_Mode */

  uint32_t Direction;           /*!< Specifies the SPI bidirectional mode state.
                                     This parameter can be a value of @ref SPI_Direction */

  uint32_t DataSize;            /*!< Specifies the SPI data size.
                                     This parameter can be a value of @ref SPI_Data_Size */

  uint32_t CLKPolarity;         /*!< Specifies the serial clock steady state.
                                     This parameter can be a value of @ref SPI_Clock_Polarity */

  uint32_t CLKPhase;            /*!< Specifies the clock active edge for the bit capture.
                                     This parameter can be a value of @ref SPI_Clock_Phase */

  uint32_t NSS;                 /*!< Specifies whether the NSS signal is managed by
                                     hardware (NSS pin) or by software using the SSI bit.
                                     This parameter can be a value of @ref SPI_Slave_Select_management */

  uint32_t BaudRatePrescaler;   /*!< Specifies the Baud Rate prescaler value which will be
                                     used to configure the transmit and receive SCK clock.
                                     This parameter can be a value of @ref SPI_BaudRate_Prescaler
                                     @note The communication clock is derived from the master
                                     clock. The slave clock does not need to be set. */

  uint32_t FirstBit;            /*!< Specifies whether data transfers start from MSB or LSB bit.
                                     This parameter can be a value of @ref SPI_MSB_LSB_transmission */

  uint32_t TIMode;              /*!< Specifies if the TI mode is enabled or not.
                                     This parameter can be a value of @ref SPI_TI_mode */

  uint32_t CRCCalculation;      /*!< Specifies if the CRC calculation is enabled or not.
                                     This parameter can be a value of @ref SPI_CRC_Calculation */

  uint32_t CRCPolynomial;       /*!< Specifies the polynomial used for the CRC calculation.
                                     This parameter must be an odd number between Min_Data = 1 and Max_Data = 65535 */
} SPI_InitTypeDef;

/**
  * @brief  HAL SPI State structure definition
  */
typedef enum
{
  HAL_SPI_STATE_RESET      = 0x00U,    /*!< Peripheral not Initialized                         */
  HAL_SPI_STATE_READY      = 0x01U,    /*!< Peripheral Initialized and ready for use           */
  HAL_SPI_STATE_BUSY       = 0x02U,    /*!< an internal process is ongoing                     */
  HAL_SPI_STATE_BUSY_TX    = 0x03U,    /*!< Data Transmission process is ongoing               */
  HAL_SPI_STATE_BUSY_RX    = 0x04U,    /*!< Data Reception process is ongoing                  */
  HAL_SPI_STATE_BUSY_TX_RX = 0x05U,    /*!< Data Transmission and Reception process is ongoing */
  HAL_SPI_STATE_ERROR      = 0x06U,    /*!< SPI error state                                    */
  HAL_SPI_STATE_ABORT      = 0x07U     /*!< SPI abort is ongoing                               */
} HAL_SPI_StateTypeDef;

/**
  * @brief  SPI handle Structure definition
  */
typedef struct __SPI_HandleTypeDef
{
  SPI_TypeDef                *Instance;      /*!< SPI registers base address               */

  SPI_InitTypeDef            Init;           /*!< SPI communication parameters             */

  uint8_t                    *pTxBuffPtr;    /*!< Pointer to SPI Tx transfer Buffer        */

  uint16_t                   TxXferSize;     /*!< SPI Tx Transfer size                     */

  __IO uint16_t              TxXferCount;    /*!< SPI Tx Transfer Counter                  */

  uint8_t                    *pRxBuffPtr;    /*!< Pointer to SPI Rx transfer Buffer        */

  uint16_t                   RxXferSize;     /*!< SPI Rx Transfer size                     */

  __IO uint16_t              RxXferCount;    /*!< SPI Rx Transfer Counter                  */

  void                       (*RxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Rx ISR */

  void                       (*TxISR)(struct __SPI_HandleTypeDef *hspi);   /*!< function pointer on Tx ISR */

  DMA_HandleTypeDef          *hdmatx;        /*!< SPI Tx DMA Handle parameters             */

  DMA_HandleTypeDef          *hdmarx;        /*!< SPI Rx DMA Handle parameters             */

  HAL_LockTypeDef            Lock;           /*!< Locking object                           */

  __IO HAL_SPI_StateTypeDef  State;          /*!< SPI communication state                  */

  __IO uint32_t              ErrorCode;      /*!< SPI Error code                           */

} SPI_HandleTypeDef;

/**
  * @}
  */

/* Exported constants --------------------------------------------------------*/
/** @defgroup SPI_Exported_Constants SPI Exported Constants
  * @{
  */

/** @defgroup SPI_Error_Code SPI Error Code
  * @{
  */
#define HAL_SPI_ERROR_NONE              0x00000000U   /*!< No error                               */
#define HAL_SPI_ERROR_MODF              0x00000001U   /*!< MODF error                             */
#define HAL_SPI_ERROR_CRC               0x00000002U   /*!< CRC error                              */
#define HAL_SPI_ERROR_OVR               0x00000004U   /*!< OVR error                              */
#define HAL_SPI_ERROR_DMA               0x00000008U   /*!< DMA transfer error                     */
#define HAL_SPI_ERROR_FLAG              0x00000010U   /*!< Flag: RXNE,TXE, BSY error              */
#define HAL_SPI_ERROR_ABORT             0x00000020U   /*!< Error during Abort procedure           */
/**
  * @}
  */

/** @defgroup SPI_Mode SPI Mode
  * @{
  */
#define SPI_MODE_SLAVE                  0x00000000U
#define SPI_MODE_MASTER                 (SPI_CR1_MSTR | SPI_CR1_SSI)
/**
  * @}
  */

/** @defgroup SPI_Direction SPI Direction
  * @{
  */
#define SPI_DIRECTION_2LINES            0x00000000U
#define SPI_DIRECTION_2LINES_RXONLY     SPI_CR1_RXONLY
#define SPI_DIRECTION_1LINE             SPI_CR1_BIDIMODE
/**
  * @}
  */

/** @defgroup SPI_Data_Size SPI Data Size
  * @{
  */
#define SPI_DATASIZE_8BIT               0x00000000U
#define SPI_DATASIZE_16BIT              SPI_CR1_DFF
/**
  * @}
  */

/** @defgroup SPI_Clock_Polarity SPI Clock Polarity
  * @{
  */
#define SPI_POLARITY_LOW                0x00000000U
#define SPI_POLARITY_HIGH               SPI_CR1_CPOL
/**
  * @}
  */

/** @defgroup SPI_Clock_Phase SPI Clock Phase
  * @{
  */
#define SPI_PHASE_1EDGE                 0x00000000U
#define SPI_PHASE_2EDGE                 SPI_CR1_CPHA
/**
  * @}
  */

/** @defgroup SPI_Slave_Select_management SPI Slave Select management
  * @{
  */
#define SPI_NSS_SOFT                    SPI_CR1_SSM
#define SPI_NSS_HARD_INPUT              0x00000000U
#define SPI_NSS_HARD_OUTPUT             ((uint32_t)0x00040000U)
/**
  * @}
  */

/** @defgroup SPI_BaudRate_Prescaler SPI BaudRate Prescaler
  * @{
  */
#define SPI_BAUDRATEPRESCALER_2         0x00000000U
#define SPI_BAUDRATEPRESCALER_4         (SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_8         (SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_16        (SPI_CR1_BR_1 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_32        (SPI_CR1_BR_2)
#define SPI_BAUDRATEPRESCALER_64        (SPI_CR1_BR_2 | SPI_CR1_BR_0)
#define SPI_BAUDRATEPRESCALER_128       (SPI_CR1_BR_2 | SPI_CR1_BR_1)
#define SPI_BAUDRATEPRESCALER_256       (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0)
/**
  * @}
  */

/** @defgroup SPI_MSB_LSB_transmission SPI MSB LSB transmission
  * @{
  */
#define SPI_FIRSTBIT_MSB                0x00000000U
#define SPI_FIRSTBIT_LSB                SPI_CR1_LSBFIRST
/**
  * @}
  */

/** @defgroup SPI_TI_mode SPI TI mode
  * @{
  */
#define SPI_TIMODE_DISABLE              0x00000000U
#define SPI_TIMODE_ENABLE               SPI_CR2_SSOE
/**
  * @}
  */

/** @defgroup SPI_CRC_Calculation SPI CRC Calculation
  * @{
  */
#define SPI_CRCCALCULATION_DISABLE      0x00000000U
#define SPI_CRCCALCULATION_ENABLE       SPI_CR1_CRCEN
/**
  * @}
  */

/** @defgroup SPI_Flags_definition SPI Flags definition
  * @{
  */
#define SPI_FLAG_RXNE                   SPI_SR_RXNE   /* SPI status flag: Rx buffer not empty flag */
#define SPI_FLAG_TXE                    SPI_SR_TXE    /* SPI status flag: Tx buffer empty flag */
#define SPI_FLAG_CRCERR                 SPI_SR_CRCERR /* SPI status flag: CRC error flag */
#define SPI_FLAG_MODF                   SPI_SR_MODF   /* SPI status flag: Mode fault flag */
#define SPI_FLAG_OVR                    SPI_SR_OVR    /* SPI status flag: Overrun flag */
#define SPI_FLAG_BSY                    SPI_SR_BSY    /* SPI status flag: Busy flag */
#define SPI_FLAG_FRE                    SPI_SR_FRE    /* SPI status flag: Frame format error flag */
/**
  * @}
  */

/** @defgroup SPI_Interrupt_definition SPI Interrupt definition
  * @{
  */
#define SPI_IT_TXE                      SPI_CR2_TXEIE
#define SPI_IT_RXNE                     SPI_CR2_RXNEIE
#define SPI_IT_ERR                      SPI_CR2_ERRIE
/**
  * @}
  */

/**
  * @}
  */

/* Exported macros -----------------------------------------------------------*/
/** @defgroup SPI_Exported_Macros SPI Exported Macros
  * @{
  */

/** @brief  Check whether the specified SPI flag is set or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @param  __FLAG__ specifies the flag to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_FLAG_RXNE: Receive buffer not empty flag
  *            @arg SPI_FLAG_TXE: Transmit buffer empty flag
  *            @arg SPI_FLAG_CRCERR: CRC error flag
  *            @arg SPI_FLAG_MODF: Mode fault flag
  *            @arg SPI_FLAG_OVR: Overrun flag
  *            @arg SPI_FLAG_BSY: Busy flag
  *            @arg SPI_FLAG_FRE: Frame format error flag
  * @retval The new state of __FLAG__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_FLAG(__HANDLE__, __FLAG__) ((((__HANDLE__)->Instance->SR) & (__FLAG__)) == (__FLAG__))

/** @brief  Clear the SPI OVR pending flag.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_CLEAR_OVRFLAG(__HANDLE__) \
  do{                                       \
    __IO uint32_t tmpreg_ovr = 0x00U;       \
    tmpreg_ovr = (__HANDLE__)->Instance->DR;\
    tmpreg_ovr = (__HANDLE__)->Instance->SR;\
    UNUSED(tmpreg_ovr);                     \
  } while(0U)

/** @brief  Enable the specified SPI interrupt.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to enable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_ENABLE_IT(__HANDLE__, __INTERRUPT__)   SET_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))

/** @brief  Disable the specified SPI interrupt.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the interrupt source to disable.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt enable
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt enable
  *            @arg SPI_IT_ERR: Error interrupt enable
  * @retval None
  */
#define __HAL_SPI_DISABLE_IT(__HANDLE__, __INTERRUPT__)  CLEAR_BIT((__HANDLE__)->Instance->CR2, (__INTERRUPT__))

/** @brief  Check whether the specified SPI interrupt has occurred or not.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @param  __INTERRUPT__ specifies the SPI interrupt source to check.
  *         This parameter can be one of the following values:
  *            @arg SPI_IT_TXE: Tx buffer empty interrupt
  *            @arg SPI_IT_RXNE: RX buffer not empty interrupt
  *            @arg SPI_IT_ERR: Error interrupt
  * @retval The new state of __INTERRUPT__ (TRUE or FALSE).
  */
#define __HAL_SPI_GET_IT_SOURCE(__HANDLE__, __INTERRUPT__) ((((__HANDLE__)->Instance->CR2) & (__INTERRUPT__)) == (__INTERRUPT__))

/** @brief  Reset SPI handle state.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_RESET_HANDLE_STATE(__HANDLE__) ((__HANDLE__)->State = HAL_SPI_STATE_RESET)

/** @brief  Enable the specified SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_ENABLE(__HANDLE__)  SET_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)

/** @brief  Disable the specified SPI peripheral.
  * @param  __HANDLE__ specifies the SPI Handle.
  *         This parameter can be SPI where x: 1, 2 or 3 to select the SPI peripheral.
  * @retval None
  */
#define __HAL_SPI_DISABLE(__HANDLE__) CLEAR_BIT((__HANDLE__)->Instance->CR1, SPI_CR1_SPE)

/**
  * @}
  */

/* Exported functions --------------------------------------------------------*/
/** @addtogroup SPI_Exported_Functions
  * @{
  */

/** @addtogroup SPI_Exported_Functions_Group1
  * @{
  */
/* Initialization/de-initialization functions  ********************************/
HAL_StatusTypeDef HAL_SPI_Init(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DeInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi);
void HAL_SPI_MspDeInit(SPI_HandleTypeDef *hspi);
/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group2
  * @{
  */
/* I/O operation functions  ***************************************************/
HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Receive(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData, uint16_t Size,
                                          uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_Transmit_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_IT(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_IT(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                             uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Transmit_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_Receive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pData, uint16_t Size);
HAL_StatusTypeDef HAL_SPI_TransmitReceive_DMA(SPI_HandleTypeDef *hspi, uint8_t *pTxData, uint8_t *pRxData,
                                              uint16_t Size);
HAL_StatusTypeDef HAL_SPI_DMAPause(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAResume(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_DMAStop(SPI_HandleTypeDef *hspi);
/* Transfer Abort functions */
HAL_StatusTypeDef HAL_SPI_Abort(SPI_HandleTypeDef *hspi);
HAL_StatusTypeDef HAL_SPI_Abort_IT(SPI_HandleTypeDef *hspi);

void HAL_SPI_IRQHandler(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_RxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi);
void HAL_SPI_AbortCpltCallback(SPI_HandleTypeDef *hspi);
/**
  * @}
  */

/** @addtogroup SPI_Exported_Functions_Group3
  * @{
  */
/* Peripheral State and Error functions ***************************************/
HAL_SPI_StateTypeDef HAL_SPI_GetState(SPI_HandleTypeDef *hspi);
uint32_t             HAL_SPI_GetError(SPI_HandleTypeDef *hspi);
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

#ifdef __cplusplus
}
#endif

#endif /* __STM32F1xx_HAL_SPI_H */
