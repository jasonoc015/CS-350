/*
 * Copyright (c) 2015-2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/*
 *  ======== gpiointerrupt.c ========
 */
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <unistd.h>

/* Driver Header files */
#include <ti/drivers/GPIO.h>

/* Driver configuration */
#include "ti_drivers_config.h"
#include <ti/drivers/I2C.h>

#include <ti/drivers/Timer.h>
#include <ti/drivers/UART2.h>
#define DISPLAY(x) UART2_write(uart, &output, x, NULL);

void gpioButtonFxn1(uint_least8_t index);
void gpioButtonFxn2(uint_least8_t index);
void TickButtonCheck();
void TickTemperatureSM();

enum SM_States {SM_start, SM_s0, SM_s1} SM_State;

// Driver Handles - Global variables
Timer_Handle timer0;

// I2C Global Variables
static const struct {
uint8_t address;
uint8_t resultReg;
char *id;
    } sensors[3] = {
    { 0x48, 0x0000, "11X" },
    { 0x49, 0x0000, "116" },
    { 0x41, 0x0001, "006" }
};
uint8_t txBuffer[1];
uint8_t rxBuffer[2];
I2C_Transaction i2cTransaction;

// Driver Handles - Global variables
I2C_Handle i2c;

// UART Global Variables
char output[64];
int bytesToSend;

// Driver Handles - Global variables
UART2_Handle uart;

void initUART(void) {
    UART2_Params uartParams;

    // Initialize the UART driver
    //UART_init();

    // Configure the UART driver
    UART2_Params_init(&uartParams);
    //uartParams.writeDataMode = UART_DATA_BINARY;
    //uartParams.readDataMode = UART_DATA_BINARY;
    uartParams.readReturnMode = UART2_ReadReturnMode_FULL;
    uartParams.baudRate = 115200;

    // Open the UART driver
    uart = UART2_open(CONFIG_UART2_0, &uartParams);
    if (uart == NULL) {
        /* UART_open() failed */
        while (1);
    }
}

void initI2C(void) {
    int8_t i, found;
    I2C_Params i2cParams;

    DISPLAY(snprintf(output, 64, "Initializing I2C Driver - "))

    // Init the driver
    I2C_init();

    // Configure the driver
    I2C_Params_init(&i2cParams);
    i2cParams.bitRate = I2C_400kHz;

    // Open the driver
    i2c = I2C_open(CONFIG_I2C_0, &i2cParams);
    if (i2c == NULL) {
        DISPLAY(snprintf(output, 64, "Failed\n\r"))
        while (1);
    }

    DISPLAY(snprintf(output, 32, "Passed\n\r"))

    // Boards were shipped with different sensors.
    // Welcome to the world of embedded systems.
    // Try to determine which sensor we have.
    // Scan through the possible sensor addresses

    /* Common I2C transaction setup */
    i2cTransaction.writeBuf = txBuffer;
    i2cTransaction.writeCount = 1;
    i2cTransaction.readBuf = rxBuffer;
    i2cTransaction.readCount = 0;

    found = false;
    for (i=0; i<3; ++i) {
        i2cTransaction.targetAddress = sensors[i].address;
        txBuffer[0] = sensors[i].resultReg;
        DISPLAY(snprintf(output, 64, "Is this %s? ", sensors[i].id))

        if (I2C_transfer(i2c, &i2cTransaction)){
            DISPLAY(snprintf(output, 64, "Found\n\r"))
            found = true;
            break;
        }
        DISPLAY(snprintf(output, 64, "No\n\r"))
    }

    if(found) {
        DISPLAY(snprintf(output, 64, "Detected TMP%s I2C address: %x\n\r", sensors[i].id, i2cTransaction.targetAddress))
    }
    else {
        DISPLAY(snprintf(output, 64, "Temperature sensor not found, contact professor\n\r"))
    }
}

int16_t readTemp(void)
{
    int j;
    int16_t temperature = 0;
    i2cTransaction.readCount = 2;
    if (I2C_transfer(i2c, &i2cTransaction))
    {
        /*
        * Extract degrees C from the received data;
        * see TMP sensor datasheet
        */
        temperature = (rxBuffer[0] << 8) | (rxBuffer[1]);
        temperature *= 0.0078125;
        /*
        * If the MSB is set '1', then we have a 2's complement
        * negative value which needs to be sign extended
        */
        if (rxBuffer[0] & 0x80)
        {
            temperature |= 0xF000;
        }
    }
    else
    {
        DISPLAY(snprintf(output, 64, "Error reading temperature sensor (%d)\n\r", i2cTransaction.status))
        DISPLAY(snprintf(output, 64, "Please power cycle your board by unplugging USB and plugging back in.\n\r"))
    }
    return temperature;
}

volatile unsigned char TimerFlag = 0;
void timerCallback(Timer_Handle myHandle, int_fast16_t status)
{
TimerFlag = 1;
}

void initTimer(void)
{
    Timer_Params params;
    // Init the driver
    Timer_init();
    // Configure the driver
    Timer_Params_init(&params);
    params.period = 100000;
    params.periodUnits = Timer_PERIOD_US;
    params.timerMode = Timer_CONTINUOUS_CALLBACK;
    params.timerCallback = timerCallback;
    // Open the driver
    timer0 = Timer_open(CONFIG_TIMER_0, &params);
    if (timer0 == NULL) {
        /* Failed to initialized timer */
        while (1) {}
    }
    if (Timer_start(timer0) == Timer_STATUS_ERROR) {
        /* Failed to start timer */
        while (1) {}
    }
}

volatile bool RaiseTemperatureFlag = false;
/*
 *  ======== gpioButtonFxn2 ========
 *  Interrupt callback function that is executed when the linked button is pressed.
 *
 *  Raises the flag to decrease the set point.
 */
void gpioButtonFxn1(uint_least8_t index)
{
    // raise raiseTemp flag
    RaiseTemperatureFlag = true;
}

volatile bool LowerTemperatureFlag = false;
/*
 *  ======== gpioButtonFxn2 ========
 *  Interrupt callback function that is executed when the linked button is pressed.
 *
 *  Raises the flag to increase the set point.
 */
void gpioButtonFxn2(uint_least8_t index)
{
    // raise lowerTemp flag
    LowerTemperatureFlag = true;
}

volatile int16_t SetPoint = 26;
/*
 *  ======== TickButtonCheck ========
 *  Tick function that executes one flag check tick. This flag check is set to tick once every 200ms.
 */
void TickButtonCheck(){
    // check the button flags and make changes to set point
    if (RaiseTemperatureFlag){
        SetPoint++;
    }
    if (LowerTemperatureFlag){
        SetPoint--;
    }

    // reset the flags
    RaiseTemperatureFlag = false;
    LowerTemperatureFlag = false;
}

volatile int16_t Temperature;
volatile uint16_t Heat;
/*
 *  ======== TickTemperatureSM ========
 *  Tick function that executes one state machine tick. This state machine is set to tick once every 500ms.
 */
void TickTemperatureSM(){
    // read the temperature from the sensor
    Temperature = readTemp();

    // state transitions
    switch (SM_State){
    case SM_start:
        // switch to state 0
        SM_State = SM_s0;
        break;
    case SM_s0:
        if (Temperature < SetPoint){
            // switch to state 1
            SM_State = SM_s1;
        }
        break;
    case SM_s1:
        if (Temperature >= SetPoint){
            // switch to state 0
            SM_State = SM_s0;
        }
        break;
    default:
        // switch to state 0
        SM_State = SM_s0;
        break;
    }

    // state actions
    switch (SM_State){
    case SM_s0:
        // turn led off
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_OFF);
        // indicate heat is off
        Heat = 0;
        break;
    case SM_s1:
        // turn led on
        GPIO_write(CONFIG_GPIO_LED_0, CONFIG_GPIO_LED_ON);
        // indicate heat is on
        Heat = 1;
        break;
    default:
        break;
    }
}

/*
 *  ======== mainThread ========
 */
void *mainThread(void *arg0)
{
    // set initial state for temperature SM
    SM_State = SM_start;

    /* Call driver init functions */
    GPIO_init();

    /* Configure the LED and button pins */
    GPIO_setConfig(CONFIG_GPIO_LED_0, GPIO_CFG_OUT_STD | GPIO_CFG_OUT_LOW);
    GPIO_setConfig(CONFIG_GPIO_BUTTON_0, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

    /* Install Button callback */
    GPIO_setCallback(CONFIG_GPIO_BUTTON_0, gpioButtonFxn1);

    /* Enable interrupts */
    GPIO_enableInt(CONFIG_GPIO_BUTTON_0);

    /*
     *  If more than one input pin is available for your device, interrupts
     *  will be enabled on CONFIG_GPIO_BUTTON1.
     */
    if (CONFIG_GPIO_BUTTON_0 != CONFIG_GPIO_BUTTON_1)
    {
        /* Configure BUTTON1 pin */
        GPIO_setConfig(CONFIG_GPIO_BUTTON_1, GPIO_CFG_IN_PU | GPIO_CFG_IN_INT_FALLING);

        /* Install Button callback */
        GPIO_setCallback(CONFIG_GPIO_BUTTON_1, gpioButtonFxn2);
        GPIO_enableInt(CONFIG_GPIO_BUTTON_1);
    }

    /* Activate timer */
    initTimer();

    /* Activate UART (must be called before I2C)*/
    initUART();

    /* Activate I2C */
    initI2C();

    /* Triggers */
    const unsigned int TIMER_PERIOD = 100;
    const unsigned int BUTTON_CHECK_INTERVAL = 200;
    const unsigned int TEMPERATURE_CHECK_INTERVAL = 500;
    const unsigned int DISPLAY_OUTPUT_INTERVAL = 1000;

    /*Time Variables */
    int seconds = 0;
    unsigned int buttonElapsedTime = 0;
    unsigned int temperatureElapsedTime = 0;
    unsigned int displayElapsedTime = 0;

    while(1){
        while(!TimerFlag){}    // wait for timer period of 100ms to go by
        TimerFlag = 0;         // lower flag raised by timer for next iteration

        // increment the elapsed times by 100ms
        buttonElapsedTime += TIMER_PERIOD;
        temperatureElapsedTime += TIMER_PERIOD;
        displayElapsedTime += TIMER_PERIOD;

        // update the total number of seconds that the program has been running
        seconds+=1;

        // Every 200ms check the button flags
        if (buttonElapsedTime >= BUTTON_CHECK_INTERVAL){
            // execute one tick of button synch SM
            TickButtonCheck();

            // reset the elapsed time
            buttonElapsedTime = 0;
        }
        if (temperatureElapsedTime >= TEMPERATURE_CHECK_INTERVAL){
            // execute 1 tick of the temperature SM
            TickTemperatureSM();

            // reset the elapsed time
            temperatureElapsedTime = 0;
        }

        // Every 500ms check the temp and update the LED
        if(displayElapsedTime >= DISPLAY_OUTPUT_INTERVAL){
            // Every second output the following to the UART
            DISPLAY( snprintf(output, 64, "<%02d,%02d,%d,%04d>\n\r", Temperature, SetPoint, Heat, seconds))

            // reset the elapsed time
            displayElapsedTime = 0;
        }
    }

    return (NULL);
}
