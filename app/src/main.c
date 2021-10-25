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

int main(void){

    uint32_t count = 0;

    initialise_monitor_handles();

    printf("Starting application program!!!\n");

    /* LED configuration */
    LED_GPIOInit();

    for(;;){
        if(count > 1000000){
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
