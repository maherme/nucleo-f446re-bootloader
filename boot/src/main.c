/*****************************************************************************************************
* FILENAME :        main.c
*
* DESCRIPTION :
*       File containing the main function.
*
**/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "usart_driver.h"

#define FLASH_SECTOR2_BASE_ADDRESS 0x08008000U

static USART_Handle_t USART1Handle;
static USART_Handle_t USART3Handle;

extern void initialise_monitor_handles(void);

static void LED_GPIOInit(void);
static void Button_GPIOInit(void);
static void USART1_Init(USART_Handle_t* pUSART_Handle);
static void USART1_GPIOInit(void);
static void USART3_Init(USART_Handle_t* pUSART_Handle);
static void USART3_GPIOInit(void);
static void bootloader_uart_read_data(void);
static void bootloader_jump_to_app(void);
__attribute__(( always_inline )) static inline void __set_MSP(uint32_t topOfMainStack);

int main(void){

    uint32_t count = 0;

    initialise_monitor_handles();

    printf("Starting bootloader program!!!\n");

    /* LED configuration */
    LED_GPIOInit();
    /* Button configuration */
    Button_GPIOInit();
    /* USART1 configuration */
    USART1_GPIOInit();
    USART1_Init(&USART1Handle);
    /* USART1 interrupt configuration */
    USART_IRQConfig(IRQ_NO_USART1, ENABLE);
    /* USART3 configuration */
    USART3_GPIOInit();
    USART3_Init(&USART3Handle);
    /* USART3 interrupt configuration */
    USART_IRQConfig(IRQ_NO_USART3, ENABLE);

    USART_Enable(USART1, ENABLE);
    USART_Enable(USART3, ENABLE);

    if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == GPIO_PIN_RESET){
        bootloader_uart_read_data();
    }
    else{
        bootloader_jump_to_app();
    }

    for(;;){
        if(count > 250000){
            /* Blink LED */
            GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
            count = 0;
        }
        else{
            count++;
        }
    }

    return 0;
}

static void LED_GPIOInit(void){

    GPIO_Handle_t GpioLed;

    memset(&GpioLed, 0, sizeof(GpioLed));

    GpioLed.pGPIOx = GPIOA;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PULL;

    GPIO_Init(&GpioLed);
}

static void Button_GPIOInit(void){

    GPIO_Handle_t GpioBtn;

    memset(&GpioBtn, 0, sizeof(GpioBtn));

    GpioBtn.pGPIOx = GPIOC;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_RT;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PD;

    GPIO_Init(&GpioBtn);
}

static void USART1_Init(USART_Handle_t* pUSART_Handle){

    memset(pUSART_Handle, 0, sizeof(*pUSART_Handle));

    pUSART_Handle->pUSARTx = USART1;
    pUSART_Handle->USART_Config.USART_Baud = USART_STD_BAUD_115200;
    pUSART_Handle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    pUSART_Handle->USART_Config.USART_Mode = USART_MODE_TXRX;
    pUSART_Handle->USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    pUSART_Handle->USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    pUSART_Handle->USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(pUSART_Handle);
}

static void USART1_GPIOInit(void){

    GPIO_Handle_t USARTPins;

    memset(&USARTPins, 0, sizeof(USARTPins));

    USARTPins.pGPIOx = GPIOB;
    USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* USART1 TX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&USARTPins);

    /* USART1 RX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&USARTPins);
}

void USART1_Handler(void){

    USART_IRQHandling(&USART1Handle);
}

static void USART3_Init(USART_Handle_t* pUSART_Handle){

    memset(pUSART_Handle, 0, sizeof(*pUSART_Handle));

    pUSART_Handle->pUSARTx = USART3;
    pUSART_Handle->USART_Config.USART_Baud = USART_STD_BAUD_115200;
    pUSART_Handle->USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    pUSART_Handle->USART_Config.USART_Mode = USART_MODE_TXRX;
    pUSART_Handle->USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    pUSART_Handle->USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    pUSART_Handle->USART_Config.USART_ParityControl = USART_PARITY_DISABLE;

    USART_Init(pUSART_Handle);
}

static void USART3_GPIOInit(void){

    GPIO_Handle_t USARTPins;

    memset(&USARTPins, 0, sizeof(USARTPins));

    USARTPins.pGPIOx = GPIOC;
    USARTPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    USARTPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    USARTPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    USARTPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    USARTPins.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    /* USART3 TX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_10;
    GPIO_Init(&USARTPins);

    /* USART3 RX */
    USARTPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_11;
    GPIO_Init(&USARTPins);
}

void USART3_Handler(void){

    USART_IRQHandling(&USART3Handle);
}

static void bootloader_uart_read_data(void){
    printf("UART read data\r\n");
}

static void bootloader_jump_to_app(void){

    printf("Jump to application\r\n");

    void (*app_reset_handler)(void);

    uint32_t msp_value = *(volatile uint32_t *)FLASH_SECTOR2_BASE_ADDRESS;
    __set_MSP(msp_value);

    uint32_t reset_handler_address = *(volatile uint32_t *)(FLASH_SECTOR2_BASE_ADDRESS + 4);

    app_reset_handler = (void*)reset_handler_address;

    app_reset_handler();
}

__attribute__((always_inline)) static inline void __set_MSP(uint32_t topOfMainStack){

    __asm volatile ("MSR msp, %0\n" : : "r" (topOfMainStack));
}

