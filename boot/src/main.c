/********************************************************************************************************//**
* @file main.c
*
* @brief File containing the main function.
*/

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stm32f446xx.h"
#include "gpio_driver.h"
#include "usart_driver.h"
#include "btl_functions.h"
#include "crc_driver.h"

/** @brief Handler struct for managing the USART1 */
static USART_Handle_t USART1Handle;

/** @brief Function needed for enabling semihosting */
extern void initialise_monitor_handles(void);

/***********************************************************************************************************/
/*                                       Static Function Prototypes                                        */
/***********************************************************************************************************/

/**
 * @brief Function for initializing the GPIO peripheral regarding to LED.
 * @return void
 */
static void LED_GPIOInit(void);

/**
 * @brief Function for initializing the GPIO peripheral regarding to button.
 * @return void
 */
static void Button_GPIOInit(void);

/**
 * @brief Function for initializing the USART1 peripheral.
 * @return void
 */
static void USART1_Init(USART_Handle_t* pUSART_Handle);

/**
 * @brief Function for initializing the GPIO peripheral regarding to USART1.
 * @return void
 */
static void USART1_GPIOInit(void);

int main(void){

    uint32_t count = 0;

    initialise_monitor_handles();

    printf("Starting bootloader program!!!\r\n");

    /* CRC initialization */
    CRC_Init();
    /* LED configuration */
    LED_GPIOInit();
    /* Button configuration */
    Button_GPIOInit();
    /* USART1 configuration */
    USART1_GPIOInit();
    USART1_Init(&USART1Handle);
    /* USART1 interrupt configuration */
    USART_IRQConfig(IRQ_NO_USART1, ENABLE);

    USART_Enable(USART1, ENABLE);

    if(GPIO_ReadFromInputPin(GPIOC, GPIO_PIN_NO_13) == GPIO_PIN_RESET){
        for(;;){
            uart_read_command(&USART1Handle);
            if(count > 75000){
                /* Blink LED */
                GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_5);
                count = 0;
            }
            else{
                count++;
            }
        }
    }
    else{
        jump_to_app();
    }

    return 0;
}

/***********************************************************************************************************/
/*                               Weak Function Overwrite Definitions                                       */
/***********************************************************************************************************/

/** @brief Handler function for managing the USART1 interrupts */
void USART1_Handler(void){
    USART_IRQHandling(&USART1Handle);
}

/***********************************************************************************************************/
/*                                       Static Function Definitions                                       */
/***********************************************************************************************************/

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
