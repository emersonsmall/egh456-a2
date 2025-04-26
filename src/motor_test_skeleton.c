/* Standard includes. */
#include "driverlib/pin_map.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "drivers/rtos_hw_drivers.h"
#include "utils/uartstdio.h"
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"

#include "motorlib.h"

/*-----------------------------------------------------------*/

/*
 * The tasks as described in the comments at the top of this file.
 */
static void prvMotorTask( void *pvParameters );

/*
 * Called by main() to create the Hello print task.
 */
void vCreateMotorTask( void );

/*
 * Hardware interrupt handlers
 */

/*-----------------------------------------------------------*/

void vCreateMotorTask( void )
{
    /* Create the task as described in the comments at the top of this file.
     *
     * The xTaskCreate parameters in order are:
     *  - The function that implements the task.
     *  - The text name Hello task - for debug only as it is
     *    not used by the kernel.
     *  - The size of the stack to allocate to the task.
     *  - No parameter passed to the task
     *  - The priority assigned to the task.
     *  - The task handle is NULL */
    xTaskCreate( prvMotorTask,
                 "Hello",
                 configMINIMAL_STACK_SIZE,
                 NULL,
                 tskIDLE_PRIORITY + 1,
                 NULL );
}
/*-----------------------------------------------------------*/

static void prvMotorTask( void *pvParameters )
{
    uint16_t duty_value = 5;
    uint16_t period_value = 50;

    /* Initialise the motors and set the duty cycle (speed) in microseconds */
    initMotorLib(period_value);
    /* Set at >10% to get it to start */
    setDuty(duty_value);

    /* Kick start the motor */
    // Do an initial read of the hall effect sensor GPIO lines
    // give the read hall effect sensor lines to updateMotor() to move the motor
    // one single phase
    // Recommendation is to use an interrupt on the hall effect sensors GPIO lines 
    // So that the motor continues to be updated every time the GPIO lines change from high to low
    // or low to high
    // Include the updateMotor function call in the ISR to achieve this behaviour.

    /* Motor test - ramp up the duty cycle from 10% to 100%, than stop the motor */
    for (;;)
    {

        if(duty_value>=period_value){
            stopMotor(1);
            continue;
        }

        setDuty(duty_value);
        vTaskDelay(pdMS_TO_TICKS( 250 ));
        duty_value++;

    }
}
/*-----------------------------------------------------------*/


/* Interrupt handlers */

void HallSensorHandler(void)
{
    
    //1. Read hall effect sensors

    //
    //2. call update motor to change to next phase
    //   updateMotor(??, ??, ??);
    
    //3. Clear interrupt
    // GPIOIntClear(??);

    // Could also add speed sensing code here too.

}
