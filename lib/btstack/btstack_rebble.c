#include "btstack_rebble.h"
#include "btstack.h"
// #include "btstack_debug.h"
#include "btstack_chipset_cc256x.h"
// #include "btstack_run_loop_embedded.h"
#include "rebbleos.h"

// UART configuration
static const hci_transport_config_uart_t config = {
    HCI_TRANSPORT_CONFIG_UART,
    115200,
    0,  // main baud rate = initial baud rate
    1,  // use flow control
    NULL
};

int btstack_main(int argc, const char ** argv);

void bt_device_init(void)
{
    // init memory pools
    btstack_memory_init();
    // default run loop for embedded systems - classic while loop
    btstack_run_loop_init(btstack_run_loop_freertos_get_instance());
    // enable packet logging, at least while porting
    hci_dump_open( NULL, HCI_DUMP_STDOUT );
    // init HCI
    hci_init(hci_transport_h4_instance(btstack_uart_block_freertos_instance()), (void*) &config);
    hci_set_chipset(btstack_chipset_cc256x_instance()); // Do I need this ??
    // hand over to BTstack example code (we hope)
    btstack_main(0, NULL);
    // go
    
    //    I think this needs to be threaded...
    // It does.
    // XXX MASSIVE TODO 
    btstack_run_loop_execute();
    
    printf("WOOHOO?\n");
}
// BT stack needs these HAL implementations

#include "hal_time_ms.h"
uint32_t hal_time_ms(void)
{
    TickType_t tick =  xTaskGetTickCount();
    return tick * portTICK_RATE_MS;
}

// hal_cpu.h implementation
// #include "hal_cpu.h"

void hal_cpu_disable_irqs(void)
{
    __disable_irq();
}

void hal_cpu_enable_irqs(void)
{
    __enable_irq();
}

void hal_cpu_enable_irqs_and_sleep(void)
{
    __enable_irq();
    __asm__("wfe"); // go to sleep if event flag isn't set. if set, just clear it. IRQs set event flag
}

// hal_uart_dma.c implementation
#include "hal_uart_dma.h"

static void dummy_handler(void);
static void dummy_handler(void){};
// handlers
static void (*rx_done_handler)(void) = &dummy_handler;
static void (*tx_done_handler)(void) = &dummy_handler;



void hal_uart_dma_set_sleep(uint8_t sleep){
    // later..
}

// reset Bluetooth using nShutdown
void bluetooth_power_cycle(void)
{
//     printf("Bluetooth power cycle\n");
    GPIO_SetBits(GPIOB, GPIO_Pin_12);
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOF, GPIO_Pin_2);
    GPIO_SetBits(GPIOF, GPIO_Pin_3);
    GPIO_SetBits(GPIOD, GPIO_Pin_2);
    GPIO_SetBits(GPIOF, GPIO_Pin_2);
    GPIO_SetBits(GPIOF, GPIO_Pin_3);
    GPIO_SetBits(GPIOF, GPIO_Pin_6);
    GPIO_SetBits(GPIOF, GPIO_Pin_8);
    GPIO_SetBits(GPIOF, GPIO_Pin_9);
     GPIO_SetBits(GPIOA, GPIO_Pin_4);
    GPIO_SetBits(GPIOA, GPIO_Pin_8);
    GPIO_SetBits(GPIOA, GPIO_Pin_13);
    GPIO_SetBits(GPIOG, GPIO_Pin_5);
    delay_us(150);
    delay_us(150);
    delay_us(150);
    GPIO_ResetBits(GPIOB, GPIO_Pin_12);
//     GPIO_ResetBits(GPIOD, GPIO_Pin_2);
//     GPIO_ResetBits(GPIOF, GPIO_Pin_2);
//     GPIO_ResetBits(GPIOF, GPIO_Pin_3);
//      GPIO_ResetBits(GPIOA, GPIO_Pin_4);
//     GPIO_ResetBits(GPIOF, GPIO_Pin_6);
//     GPIO_ResetBits(GPIOF, GPIO_Pin_8);
//     GPIO_ResetBits(GPIOF, GPIO_Pin_9);
//     GPIO_ResetBits(GPIOA, GPIO_Pin_8);
//     GPIO_ResetBits(GPIOA, GPIO_Pin_13);
//     GPIO_ResetBits(GPIOG, GPIO_Pin_5);
    delay_us(80);
    delay_us(80);
    delay_us(80);
}
/*
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart3){
        (*tx_done_handler)();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
    if (huart == &huart3){
        (*rx_done_handler)();
    }
}*/

void hal_uart_dma_init(void){
    bluetooth_power_cycle();
}
void hal_uart_dma_set_block_received( void (*the_block_handler)(void)){
    rx_done_handler = the_block_handler;
}

void hal_uart_dma_set_block_sent( void (*the_block_handler)(void)){
    tx_done_handler = the_block_handler;
}

void hal_uart_dma_set_csr_irq_handler( void (*the_irq_handler)(void)){
    // .. later
}

int  hal_uart_dma_set_baud(uint32_t baud){
    // .. later
    return 0;
}

void hal_uart_dma_send_block(const uint8_t *data, uint16_t size)
{
//     HAL_UART_Transmit_DMA( &huart3, (uint8_t *) data, size);
}

void hal_uart_dma_receive_block(uint8_t *data, uint16_t size)
{
//     HAL_UART_Receive_DMA( &huart3, data, size );
}
