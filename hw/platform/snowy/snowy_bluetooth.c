/* snowy_bluetooth.c
 * 
 * RebbleOS
 *
 * Author: Barry Carter <barry.carter@gmail.com>
 */

#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"
#include "display.h"
#include "log.h"
#include "vibrate.h"
#include "snowy_display.h"
#include <stm32f4xx_spi.h>
#include <stm32f4xx_tim.h>
#include "stm32_power.h"
#include "btstack_rebble.h"

// Init USART1
// DMA
// IRQ
// usart1 global interrupt?
// dma rx/tx
// clocks
// 

void snowy_bluetooth_init(void)
{
    stm32_power_request(STM32_POWER_APB2, RCC_APB2Periph_SYSCFG);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOB);
    
    //GPIO_InitTypeDef gpio_init_structure;
    GPIO_InitTypeDef gpio_init_bt;
    
    
    // nSHUTD on B12 (??)
    gpio_init_bt.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init_bt.GPIO_Pin =  GPIO_Pin_12;
    gpio_init_bt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init_bt.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_bt.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOB, &gpio_init_bt);

    _init_USART1();
    
    // configure DMA    
    //_snowy_bluetooth_init_dma();
    
    bt_device_init();

    stm32_power_release(STM32_POWER_APB2, RCC_APB2Periph_SYSCFG);
    stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOB);
}

void _init_USART1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    stm32_power_request(STM32_POWER_APB2, RCC_APB2Periph_USART1);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOA);

    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // Configure TX (9) and RTX (12)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);


    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);

    stm32_power_release(STM32_POWER_APB2, RCC_APB2Periph_USART1);
    stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOA);
}

ssize_t _bt_write(const void *buf, size_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
      while (!(USART1->SR & USART_FLAG_TXE));
      USART1->DR =  ((uint8_t *) buf)[i];
    }
}

ssize_t _bt_read(void *buf, size_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
      while (!(USART1->SR & USART_FLAG_RXNE));
      ((uint8_t *) buf)[i] =  USART1->DR & 0xff;
    }
    return i;
}
