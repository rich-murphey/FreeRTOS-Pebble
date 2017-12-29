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

#define BT_SHUTD        GPIO_Pin_12

void hw_bluetooth_init(void)
{
    stm32_power_request(STM32_POWER_APB2, RCC_APB2Periph_SYSCFG);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOB);
    
    // shutdown IS B12
    // USART1
    // slowclock from LSE over MCO1
    
    //GPIO_InitTypeDef gpio_init_structure;
    GPIO_InitTypeDef gpio_init_bt;
    
    
    // nSHUTD on B12
    gpio_init_bt.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init_bt.GPIO_Pin =  BT_SHUTD;
    gpio_init_bt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init_bt.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_bt.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOB, &gpio_init_bt);
    
    // Other random periphs?   
    gpio_init_bt.GPIO_Mode = GPIO_Mode_OUT;
    gpio_init_bt.GPIO_Pin =  GPIO_Pin_4;
    gpio_init_bt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init_bt.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_bt.GPIO_OType = GPIO_OType_OD;
    GPIO_Init(GPIOA, &gpio_init_bt);
    
    // Set MCO as an AF output
    gpio_init_bt.GPIO_Mode = GPIO_Mode_AF;
    gpio_init_bt.GPIO_Pin =  GPIO_Pin_8;
    gpio_init_bt.GPIO_PuPd = GPIO_PuPd_NOPULL;
    gpio_init_bt.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_init_bt.GPIO_OType = GPIO_OType_PP;
    GPIO_Init(GPIOA, &gpio_init_bt);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_MCO);
   
    _init_USART1();
    
    // configure DMA
    //_snowy_bluetooth_init_dma();
    
    // initialise BTStack....For later!
    // bt_device_init();
    
    // all dem clocks (woo remove this)
    stm32_power_request(STM32_POWER_APB2, RCC_APB2Periph_USART1);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOA);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOE);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOF);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOD);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOG);
    stm32_power_request(STM32_POWER_APB1, RCC_APB1Periph_UART8);
    stm32_power_request(STM32_POWER_APB1, RCC_APB1Periph_PWR);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
    // it should go like this
    // reset the chip by pulling nSHUTD high, sleep then low
    // Then we turn on the ext slow clock 32.768khz clock (MCO1)
    // then issue an HCI reset command.
    // The BT module should give a return of 7 bytes to ACK
    // RTS (Our CTS) will also get pulled LOW to ack reset
    
    // Pull RTS High
    // GPIO_SetBits(GPIOA, GPIO_Pin_12);
    
    // Well, lets go for broke. Dunno what this does
    GPIO_SetBits(GPIOA, GPIO_Pin_4);
    
    // allow RTC access so we can play with LSE
    PWR_BackupAccessCmd(ENABLE); 
    
    printf("BT set MCO1 32khz\n");
    // Turn on the MCO1 clock and pump 32khz to bluetooth
    RCC_LSEConfig(RCC_LSE_ON);

    // knock knock
    while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET);
    printf("LSE Ready\n");
    
    // Enable power to the MCO Osc out pin
    RCC_MCO1Config(RCC_MCO1Source_LSE, RCC_MCO1Div_1);

    // datasheet says no more than 2ms for clock stabilisation. Lets go for more
    do_delay_ms(2);
    IWDG_ReloadCounter();
     
    // initialise BTStack....For later!
    //bt_device_init();
    
    // For now lets try a good old fashioned HCI command
    // lets reboot
    
    int a,b = 0;
    a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
    b = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
    printf("Pre: RTS %d, CTS %d\n", a, b);
    //bluetooth_power_cycle();
    printf("BT: Reset...\n");

    // B12 low
    GPIO_ResetBits(GPIOB, BT_SHUTD);
    printf("BT: nS_L\n");
    do_delay_ms(20);
    
    GPIO_SetBits(GPIOB, BT_SHUTD);
    printf("BT: nS_H\n");
    // datasheet says at least 90ms to init
    do_delay_ms(250);

    printf("BT: Reset Done\n");
        
    a = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_12);
    b = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);

    printf("Post: %d %d\n", a, b);

    printf("BT: HCI Reset\n");
    
    // send magic HCI reset
    const uint8_t hci_reset[] = { 0x01, 0x03, 0x0c, 0x00 };
    _bt_write(hci_reset, sizeof(hci_reset));
    
    /// ANNND becuase we don't get a response back uver usart, RTS isn't answered and we lock
    // disabling flow control just lets us shout at the wind
    printf("Wait for toggle\n");
    
    int olda, oldb;
    olda = a;
    oldb = b;
    for(int i = 0; i < 10000; i++)
    {
        a = GPIO_ReadInputDataBit(GPIOA, BT_SHUTD);
        b = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_11);
        
        if (a != olda)
        {
            printf("A!\n");
            break;
        }
        if (b != oldb)
        {
            printf("B!\n");
            break;
        }
        IWDG_ReloadCounter();
        
        if (i % 1000 == 0)
            printf("... still\n");

        do_delay_ms(1);
    }
    printf("::%d %d\n", a, b);
    
    // Read it back. We should have something (7 bytes)
    char buf[100];
    for (int i = 0; i < 10; i++)
    {
        IWDG_ReloadCounter();

        printf("To Wait...\n");
        _bt_read(buf, 1);
        printf("%d\n", buf[0]);
        do_delay_us(10);
        printf("Wait...\n");
    }
    printf("Done...\n");
    stm32_power_release(STM32_POWER_APB2, RCC_APB2Periph_USART1);
    stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOA);
    stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOE);
    stm32_power_release(STM32_POWER_APB1, RCC_APB1Periph_UART8);
    
    stm32_power_release(STM32_POWER_APB2, RCC_APB2Periph_SYSCFG);
    stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOB);
}

void _init_USART1(void)
{
    GPIO_InitTypeDef GPIO_InitStruct;
    USART_InitTypeDef USART_InitStruct;

    stm32_power_request(STM32_POWER_APB2, RCC_APB2Periph_USART1);
    stm32_power_request(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOA);

//     // CTS (11) RX (10)
//     GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
//     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN;
//     GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//     GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;
//     GPIO_Init(GPIOA, &GPIO_InitStruct);
//     
//     // Configure TX (9) and RTS (12)
//     GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9 | BT_SHUTD;
//     GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
//     GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
//     GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
//     GPIO_Init(GPIOA, &GPIO_InitStruct);

    // RX (10)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // CTS (11)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);
    
    // TX (9)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    // RTS (12)
    GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_USART1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_USART1);

    USART_InitStruct.USART_BaudRate = 115200;
    USART_InitStruct.USART_WordLength = USART_WordLength_8b;
    USART_InitStruct.USART_StopBits = USART_StopBits_1;
    USART_InitStruct.USART_Parity = USART_Parity_No;
    USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_RTS_CTS;
    USART_InitStruct.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
    USART_Init(USART1, &USART_InitStruct);
    USART_Cmd(USART1, ENABLE);

    USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);
    // Enable global interrupt
    NVIC_EnableIRQ(USART1_IRQn);

//     stm32_power_release(STM32_POWER_APB2, RCC_APB2Periph_USART1);
//     stm32_power_release(STM32_POWER_AHB1, RCC_AHB1Periph_GPIOA);
}

void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
//         Serial1._rxRingBuffer.write(USART_ReceiveData(USART1));
        printf("RECV! %d\n", USART_ReceiveData(USART1));
    }
    if (USART_GetITStatus(USART1, USART_IT_TXE) != RESET)
    {
        // Transmitter is empty, maybe send another char?
//         if (Serial1._txRingBuffer.isEmpty())
//             USART_ITConfig(USART1, USART_IT_TXE, DISABLE); // No more to send, disable the TX interrupt
//         else
//             USART_SendData(USART1, Serial1._txRingBuffer.read());
    }
}

ssize_t _bt_write(const void *buf, size_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
      while (!(USART1->SR & USART_FLAG_TXE));
//       USART1->DR =  ((uint8_t *) buf)[i];
      USART_SendData(USART1, ((uint8_t *) buf)[i]);
    }
    
    return i;
}

ssize_t _bt_read(void *buf, size_t len)
{
    int i;
    for (i = 0; i < len; i++)
    {
        
      while (!(USART1->SR & USART_FLAG_RXNE));
      ((uint8_t *) buf)[i] = USART_ReceiveData(USART1);
//       ((uint8_t *) buf)[i] =  USART1->DR & 0xff;
    }
    return i;
}

#define DELAY_TIM_FREQUENCY_US 1000000
#define DELAY_TIM_FREQUENCY_MS 1000

// Init and start timer for Milliseconds delays
void _init_timer(uint32_t prescaler);

void _init_timer(uint32_t prescaler)
{
    // Start the timer 2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);

    TIM_TimeBaseInitTypeDef timebase_init;
    TIM_TimeBaseStructInit(&timebase_init);
    timebase_init.TIM_Prescaler = (SystemCoreClock / prescaler) - 1;
    timebase_init.TIM_Period = UINT16_MAX;
    timebase_init.TIM_ClockDivision = 0;
    timebase_init.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM2, &timebase_init);
    
    TIM_Cmd(TIM2, ENABLE);
}

// Stop timer
void _stop_timer()
{
    TIM_Cmd(TIM2, DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, DISABLE);
}

// Do delay for nTime milliseconds
void do_delay_ms(uint32_t ms) {
    // For QEmu which apparently doesn't like my TIM2 timer?!?
    // Init and start timer
    /*for (int i = 0; i < (SystemCoreClock/ 300000) * mSecs; i++)
    {
        asm("nop");
        asm("");
    }
    return;*/
    _init_timer(DELAY_TIM_FREQUENCY_MS);
    
    for (uint32_t i = 0; i < ms; i++)
    {
        volatile uint32_t start = TIM2->CNT;
        while((TIM2->CNT - start) <= 1);
        
        // reset WDG so we don't reboot
        IWDG_ReloadCounter();
    }

    _stop_timer();
}


// Do delay for nTime microseconds
void do_delay_us(uint32_t us)
{
    _init_timer(DELAY_TIM_FREQUENCY_US);

    // Dummy loop with 16 bit count wrap around
    volatile uint32_t start = TIM2->CNT;
    while((TIM2->CNT - start) <= us);

    _stop_timer();
}
