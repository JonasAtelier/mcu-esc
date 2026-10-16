#ifndef STM32_HAL_STUB_H
#define STM32_HAL_STUB_H

/*
 * Host stub for the slice of the STM32 HAL this library uses, standing in
 * for "stm32f4xx.h" through CMSIS_device_header (see test/Makefile). Names,
 * signatures and values match the real F4/F7 HAL; the handles carry only the
 * fields the drivers read. HAL_Delay() moves stub_tick; the tests define the
 * transfer functions their fake chips answer.
 */

#include <stdint.h>

/* stm32f4xx.h defines its family; the DShot port's bidirectional half
 * builds only under it. STUB_NOT_F4 stands for any other family. */
#ifndef STUB_NOT_F4
#define STM32F4
#endif

typedef enum {
	HAL_OK		= 0x00,
	HAL_ERROR	= 0x01,
	HAL_BUSY	= 0x02,
	HAL_TIMEOUT	= 0x03,
} HAL_StatusTypeDef;

/*
 * A port is a register block on silicon. Here it needs an identity, and
 * for bidirectional DShot the mode, pull and input registers.
 */
typedef struct {
	uint32_t id;
	uint32_t MODER;
	uint32_t PUPDR;
	uint32_t IDR;
} GPIO_TypeDef;

typedef enum {
	GPIO_PIN_RESET = 0,
	GPIO_PIN_SET,
} GPIO_PinState;

#define GPIO_PIN_0		((uint16_t)0x0001)
#define GPIO_PIN_1		((uint16_t)0x0002)
#define GPIO_PIN_4		((uint16_t)0x0010)
#define GPIO_PIN_6		((uint16_t)0x0040)
#define GPIO_PIN_7		((uint16_t)0x0080)
#define GPIO_PIN_8		((uint16_t)0x0100)
#define GPIO_PIN_9		((uint16_t)0x0200)
#define GPIO_PIN_14		((uint16_t)0x4000)
#define GPIO_PIN_15		((uint16_t)0x8000)

typedef struct {
	uint32_t id;
} I2C_HandleTypeDef;

#define I2C_MEMADD_SIZE_8BIT	0x00000001U

typedef struct {
	uint32_t DataSize;
	uint32_t CLKPolarity;
	uint32_t CLKPhase;
} SPI_InitTypeDef;

typedef struct {
	SPI_InitTypeDef Init;
} SPI_HandleTypeDef;

typedef struct {
	uint32_t id;
} UART_HandleTypeDef;

/* The F4 values. The drivers compare against the names, never the numbers. */
#define SPI_DATASIZE_8BIT	0x00000000U
#define SPI_DATASIZE_16BIT	0x00000800U
#define SPI_POLARITY_LOW	0x00000000U
#define SPI_POLARITY_HIGH	0x00000002U
#define SPI_PHASE_1EDGE		0x00000000U
#define SPI_PHASE_2EDGE		0x00000001U

/*
 * SysTick, as CMSIS lays it out. Every access through SysTick goes through
 * stub_systick(), which lets time pass first -- stub_systick_step core
 * cycles, VAL counting down and reloading from LOAD -- so a loop that reads
 * VAL sees it move, as on silicon. stub_cycles counts every cycle that
 * passed; SystemCoreClock is the core clock the delay divides by.
 */
typedef struct {
	uint32_t CTRL;
	uint32_t LOAD;
	uint32_t VAL;
	uint32_t CALIB;
} SysTick_Type;

#define SysTick_CTRL_ENABLE_Msk		(1UL << 0)
#define SysTick_CTRL_CLKSOURCE_Msk	(1UL << 2)

extern uint32_t SystemCoreClock;
extern uint32_t stub_systick_step;
extern uint64_t stub_cycles;
extern uint32_t stub_systick_phase;	/* cycles into a count at core / 8 */

SysTick_Type *stub_systick(void);
#define SysTick		(stub_systick())

uint32_t HAL_GetTick(void);
void HAL_Delay(uint32_t Delay);

void HAL_GPIO_WritePin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin,
		       GPIO_PinState PinState);
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin);

HAL_StatusTypeDef HAL_I2C_IsDeviceReady(I2C_HandleTypeDef *hi2c,
					uint16_t DevAddress, uint32_t Trials,
					uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Read(I2C_HandleTypeDef *hi2c,
				   uint16_t DevAddress, uint16_t MemAddress,
				   uint16_t MemAddSize, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c,
				    uint16_t DevAddress, uint16_t MemAddress,
				    uint16_t MemAddSize, uint8_t *pData,
				    uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Transmit(I2C_HandleTypeDef *hi2c,
					  uint16_t DevAddress, uint8_t *pData,
					  uint16_t Size, uint32_t Timeout);
HAL_StatusTypeDef HAL_I2C_Master_Receive(I2C_HandleTypeDef *hi2c,
					 uint16_t DevAddress, uint8_t *pData,
					 uint16_t Size, uint32_t Timeout);

HAL_StatusTypeDef HAL_SPI_Transmit(SPI_HandleTypeDef *hspi,
				   const uint8_t *pData, uint16_t Size,
				   uint32_t Timeout);
HAL_StatusTypeDef HAL_SPI_TransmitReceive(SPI_HandleTypeDef *hspi,
					  const uint8_t *pTxData,
					  uint8_t *pRxData, uint16_t Size,
					  uint32_t Timeout);

HAL_StatusTypeDef HAL_UART_Transmit(UART_HandleTypeDef *huart,
				    const uint8_t *pData, uint16_t Size,
				    uint32_t Timeout);
HAL_StatusTypeDef HAL_UART_Receive(UART_HandleTypeDef *huart, uint8_t *pData,
				   uint16_t Size, uint32_t Timeout);

/*
 * The UART flag macros the port uses, as functions over a fake receive
 * register: RXNE is "a stale byte is waiting", and the flush reads one.
 */
#define UART_FLAG_RXNE				0x20U
#define __HAL_UART_GET_FLAG(__HANDLE__, __FLAG__) \
	stub_uart_get_flag((__HANDLE__), (__FLAG__))
#define __HAL_UART_CLEAR_OREFLAG(__HANDLE__)	stub_uart_clear_ore(__HANDLE__)
#define __HAL_UART_FLUSH_DRREGISTER(__HANDLE__)	stub_uart_flush(__HANDLE__)

int stub_uart_get_flag(UART_HandleTypeDef *huart, uint32_t flag);
void stub_uart_clear_ore(UART_HandleTypeDef *huart);
void stub_uart_flush(UART_HandleTypeDef *huart);

/*
 * TIM: the register block holds what the PWM port touches, ARR and the four
 * compares; the macros are the F4 HAL's (stm32f4xx_hal_tim.h), the
 * functions test-defined.
 */
typedef struct {
	uint32_t DIER;
	uint32_t EGR;
	uint32_t CCER;
	uint32_t ARR;
	uint32_t CCR1, CCR2, CCR3, CCR4;
	uint32_t DCR;
	uint32_t DMAR;
} TIM_TypeDef;

/* The F405 CMSIS header's bits. */
#define TIM_EGR_UG			0x00000001U
#define TIM_CCER_CC1P			0x00000002U

/*
 * DMA: the handle's settings the DShot port checks and its state, values
 * from stm32f4xx_hal_dma.h (F4 HAL v1.8.5) and the F405 CMSIS header's
 * DMA_SxCR bits.
 */
typedef struct {
	uint32_t Direction;
	uint32_t PeriphInc;
	uint32_t MemInc;
	uint32_t PeriphDataAlignment;
	uint32_t MemDataAlignment;
	uint32_t Mode;
} DMA_InitTypeDef;

typedef enum {
	HAL_DMA_STATE_RESET = 0x00U,
	HAL_DMA_STATE_READY = 0x01U,
	HAL_DMA_STATE_BUSY = 0x02U,
} HAL_DMA_StateTypeDef;

/*
 * The streams: an identity and an address each, DMA1's below DMA2's as on
 * the F405 (0x4002 6010 and 0x4002 6410); stub_dma_streams is in
 * hal_stubs.c.
 */
typedef struct {
	uint32_t CR;
} DMA_Stream_TypeDef;

extern DMA_Stream_TypeDef stub_dma_streams[16];
#define DMA1_Stream5		(&stub_dma_streams[5])
#define DMA2_Stream0		(&stub_dma_streams[8])
#define DMA2_Stream1		(&stub_dma_streams[9])

#define HAL_DMA_ERROR_NONE		0x00000000U
#define HAL_DMA_ERROR_TE		0x00000001U
#define HAL_DMA_ERROR_FE		0x00000002U
#define HAL_DMA_ERROR_DME		0x00000004U

typedef struct __DMA_HandleTypeDef {
	DMA_Stream_TypeDef *Instance;
	DMA_InitTypeDef Init;
	HAL_DMA_StateTypeDef State;
	uint32_t ErrorCode;
	void *Parent;
	void (*XferCpltCallback)(struct __DMA_HandleTypeDef *hdma);
	void (*XferHalfCpltCallback)(struct __DMA_HandleTypeDef *hdma);
	void (*XferErrorCallback)(struct __DMA_HandleTypeDef *hdma);
} DMA_HandleTypeDef;

#define DMA_PERIPH_TO_MEMORY		0x00000000U
#define DMA_MEMORY_TO_PERIPH		0x00000040U
#define DMA_PINC_DISABLE		0x00000000U
#define DMA_PINC_ENABLE			0x00000200U
#define DMA_MINC_ENABLE			0x00000400U
#define DMA_PDATAALIGN_HALFWORD		0x00000800U
#define DMA_PDATAALIGN_WORD		0x00001000U
#define DMA_MDATAALIGN_HALFWORD		0x00002000U
#define DMA_MDATAALIGN_WORD		0x00004000U
#define DMA_NORMAL			0x00000000U
#define DMA_CIRCULAR			0x00000100U

HAL_DMA_StateTypeDef HAL_DMA_GetState(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Init(DMA_HandleTypeDef *hdma);
HAL_StatusTypeDef HAL_DMA_Start_IT(DMA_HandleTypeDef *hdma,
				   uint32_t SrcAddress, uint32_t DstAddress,
				   uint32_t DataLength);
HAL_StatusTypeDef HAL_DMA_Abort(DMA_HandleTypeDef *hdma);

typedef struct {
	TIM_TypeDef *Instance;
	DMA_HandleTypeDef *hdma[7];
} TIM_HandleTypeDef;

#define TIM_DMA_ID_UPDATE		((uint16_t)0x0000)
#define TIM_DMABASE_CCR1		0x0000000DU
#define TIM_DMA_UPDATE			0x00000100U	/* TIM_DIER_UDE */
#define TIM_DMABURSTLENGTH_4TRANSFERS	0x00000300U

HAL_StatusTypeDef HAL_TIM_DMABurst_MultiWriteStart(TIM_HandleTypeDef *htim,
						   uint32_t BurstBaseAddress,
						   uint32_t BurstRequestSrc,
						   const uint32_t *BurstBuffer,
						   uint32_t BurstLength,
						   uint32_t DataLength);
HAL_StatusTypeDef HAL_TIM_DMABurst_WriteStop(TIM_HandleTypeDef *htim,
					     uint32_t BurstRequestSrc);

#define TIM_CHANNEL_1		0x00000000U
#define TIM_CHANNEL_2		0x00000004U
#define TIM_CHANNEL_3		0x00000008U
#define TIM_CHANNEL_4		0x0000000CU

#define __HAL_TIM_GET_AUTORELOAD(__HANDLE__)	((__HANDLE__)->Instance->ARR)
#define __HAL_TIM_ENABLE_DMA(__HANDLE__, __DMA__) \
	((__HANDLE__)->Instance->DIER |= (__DMA__))
#define __HAL_TIM_DISABLE_DMA(__HANDLE__, __DMA__) \
	((__HANDLE__)->Instance->DIER &= ~(__DMA__))
#define __HAL_TIM_SET_COMPARE(__HANDLE__, __CHANNEL__, __COMPARE__) \
	(((__CHANNEL__) == TIM_CHANNEL_1) ? \
	 ((__HANDLE__)->Instance->CCR1 = (__COMPARE__)) : \
	 ((__CHANNEL__) == TIM_CHANNEL_2) ? \
	 ((__HANDLE__)->Instance->CCR2 = (__COMPARE__)) : \
	 ((__CHANNEL__) == TIM_CHANNEL_3) ? \
	 ((__HANDLE__)->Instance->CCR3 = (__COMPARE__)) : \
	 ((__HANDLE__)->Instance->CCR4 = (__COMPARE__)))

HAL_StatusTypeDef HAL_TIM_PWM_Start(TIM_HandleTypeDef *htim,
				    uint32_t Channel);
HAL_StatusTypeDef HAL_TIM_PWM_Stop(TIM_HandleTypeDef *htim, uint32_t Channel);

/* --- test hooks, defined in hal_stubs.c --------------------------------- */

/* What HAL_GetTick() returns, in ms; tests set it, HAL_Delay() moves it. */
extern uint32_t stub_tick;

/*
 * The last printf() line, without its newline. The tests link with
 * -Wl,--wrap=printf, so every esc_* log line lands here as well as on
 * stdout.
 */
extern char stub_log_last[256];

#endif
