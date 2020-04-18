/**
 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "nrf_sdh_freertos.h"
#include "nrf_sdh.h"

/* Group of FreeRTOS-related includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "semphr.h"

#include "TranslateMorseCode.h"

#define NRF_LOG_MODULE_NAME nrf_sdh_freertos
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define NRF_BLE_FREERTOS_SDH_TASK_STACK 256


static QueueHandle_t buttonQueue = NULL,
		//message Queue: contains char* which must be dynamically allocated and freed as the queue is emptied
		messageQueue = NULL,
		//send message Queue: contains char* which must be dynamically allocated and freed as the queue is emptied
		sendMessageQueue = NULL,
		//notification Queue: contains char* which must be dynamically allocated and freed as the queue is emptied
		notificationQueue = NULL,
		//display queue: used in the timers to turn on and off the buzzer
		displayQueue = NULL,
		//uart1 queue: queues individual characters to be put together and then queued(for notifications)
		uart1Queue = NULL;

static SemaphoreHandle_t  semaphoreButtonPressed = NULL, semaphoreButtonPressActive = NULL,
                          semaphoreButtonReleased = NULL, semaphoreButtonReleaseActive = NULL,
                          semaphoreSendMessage = NULL, semaphoreStopSendMessage = NULL;

static TimerHandle_t buttonReleasedTimer = NULL, buttonPressedTimer = NULL, DisplaySpaceTimer = NULL, DisplayBeepTimer = NULL;

static TaskHandle_t                 m_softdevice_task,              //!< Reference to SoftDevice FreeRTOS task.
                                    RecordButtonPressesTask = NULL;  

static nrf_sdh_freertos_task_hook_t m_task_hook;        //!< A hook function run by the SoftDevice task before entering its loop.

struct ButtonPress{
	uint32_t time;
	uint8_t buttonState;
};

void SD_EVT_IRQHandler(void)
{
    BaseType_t yield_req = pdFALSE;

    vTaskNotifyGiveFromISR(m_softdevice_task, &yield_req);

    /* Switch the task if required. */
    portYIELD_FROM_ISR(yield_req);
}


/* This function gets events from the SoftDevice and processes them. */
static void softdevice_task(void * pvParameter)
{
    NRF_LOG_DEBUG("Enter softdevice_task.");

    if (m_task_hook != NULL)
    {
        m_task_hook(pvParameter);
    }

    while (true)
    {
        nrf_sdh_evts_poll();                    /* let the handlers run first, incase the EVENT occured before creating this task */

        (void) ulTaskNotifyTake(pdTRUE,         /* Clear the notification value before exiting (equivalent to the binary semaphore). */
                                portMAX_DELAY); /* Block indefinitely (INCLUDE_vTaskSuspend has to be enabled).*/
    }
}

static void TranslateMorseCode()
{
    //stop recorButtonPresses task(can i use semaphore, i think beter to do this since is activated by a timer, block time must be zero if using semaphore)
    //vTaskSuspend(RecordButtonPressesTask);

    //try to take semaphores so tasks do not start(could maybe suspend the task until finished)
    xSemaphoreTake(semaphoreButtonPressActive, 0);
    xSemaphoreTake(semaphoreButtonPressed, 0);
    xSemaphoreTake(semaphoreButtonReleaseActive, 0);
    xSemaphoreTake(semaphoreButtonReleased, 0);

    char* message = TranslateSelf();
    NRF_LOG_INFO("translate MorseCode %s", message);
    
    //xQueueSend( messageQueue, &message, 1);

    //start recorButtonPresses task
    //vTaskResume(RecordButtonPressesTask);
    xSemaphoreGive(semaphoreButtonPressActive);
}

static void ButtonReleased_handler( TimerHandle_t xTimer )
{
    uint8_t ulCount = ( uint8_t ) pvTimerGetTimerID( xTimer );
    ulCount++;
    vTimerSetTimerID( xTimer, ( void * ) ulCount );
    if(ulCount > SPACE_UNITS_END_OF_MESSAGE)
    {
        xTimerStop(xTimer, 0);
        TranslateMorseCode();
    }
}

static void ButtonPressed_handler( TimerHandle_t xTimer )
{
    uint8_t ulCount = ( uint8_t ) pvTimerGetTimerID( xTimer );
    ulCount++;
    vTimerSetTimerID( xTimer, ( void * ) ulCount );
    if(ulCount > DASH_UNIT)
    {
        xTimerStop(xTimer, 0);
    }
}

void ISRButtonPressed()
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(xSemaphoreTakeFromISR( semaphoreButtonPressActive, &xHigherPriorityTaskWoken ) == pdTRUE){
        xSemaphoreGiveFromISR( semaphoreButtonPressed, &xHigherPriorityTaskWoken );
    }
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void ISRButtonReleased()
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(xSemaphoreTakeFromISR( semaphoreButtonReleaseActive, &xHigherPriorityTaskWoken ) == pdTRUE){
        xSemaphoreGiveFromISR( semaphoreButtonReleased, &xHigherPriorityTaskWoken );
    }
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

static void PollingTask( void *pvParameters )
{
    TickType_t startTicks = 0;
    struct ButtonPress buttonRecord;
    for(;;)
    {
        //wait for semaphore from interrupt
        if( semaphoreButtonPressed != NULL )
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks(can maybe increase this to max so it waits forever) to see if it becomes free. */
            if( xSemaphoreTake( semaphoreButtonPressed, portMAX_DELAY) == pdTRUE ){
                vTimerSetTimerID(buttonPressedTimer, 0u);
                xTimerStop(buttonReleasedTimer, 0);
                xTimerStart(buttonPressedTimer, 0);
                TickType_t endTicks, difference;

                uint8_t unheldCount = ( uint8_t ) pvTimerGetTimerID( buttonReleasedTimer );
                ButtonPress(unheldCount, 0);
                NRF_LOG_DEBUG("unheld time: %d", unheldCount);
                
                xSemaphoreGive( semaphoreButtonReleaseActive );
                if( xSemaphoreTake( semaphoreButtonReleased, portMAX_DELAY) == pdTRUE ){
                    vTimerSetTimerID(buttonReleasedTimer, 0u);
                    xTimerStop(buttonPressedTimer, 0);
                    xTimerStart(buttonReleasedTimer, 0);

                    uint8_t heldCount = ( uint8_t ) pvTimerGetTimerID( buttonPressedTimer );
                    ButtonPress(heldCount, 1);
                    NRF_LOG_DEBUG("held time: %d", heldCount);
                }

                //continue to block semaphoreButtonPressed in ISR cannot be triggerd for a period of time
                //vTaskDelay(30);
                
                //release semaphoreButtonPressActive (giving the semaphore so ISR can happen and give this task the semaphore it needs)
                xSemaphoreGive( semaphoreButtonPressActive );
            }
        }
    }
}


void nrf_sdh_freertos_init(nrf_sdh_freertos_task_hook_t hook_fn, void * p_context)
{
    NRF_LOG_DEBUG("Creating a SoftDevice task.");

    m_task_hook = hook_fn;

    /* Create the timer(s) */
    buttonReleasedTimer = xTimerCreate( 	"ButtonReleased", 				/* A text name, purely to help debugging. */
                                        100*(1.0/(configTICK_RATE_HZ * (.001f))),/* The timer period, in this case (SPACE_TICK_LENGTH * 10) ms. */
                                        pdTRUE,					/* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                                        ( void * ) 0,				/* The ID is not used, so can be set to anything. */
                                        ButtonReleased_handler);			/* The callback function that switches the LED off. */
    buttonPressedTimer = xTimerCreate( 	"ButtonPressed", 				/* A text name, purely to help debugging. */
                                        100*(1.0/(configTICK_RATE_HZ * (.001f))),/* The timer period, in this case (SPACE_TICK_LENGTH * 10) ms. */
                                        pdTRUE,					/* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                                        ( void * ) 0,				/* The ID is not used, so can be set to anything. */
                                        ButtonPressed_handler);			/* The callback function that switches the LED off. */

    buttonQueue = xQueueCreate( 10, sizeof( struct ButtonPress ) );
    messageQueue = xQueueCreate( 10, sizeof( char* ) );

    semaphoreButtonPressed = xSemaphoreCreateBinary();
    semaphoreButtonPressActive = xSemaphoreCreateBinary();
    semaphoreButtonReleased = xSemaphoreCreateBinary();
    semaphoreButtonReleaseActive = xSemaphoreCreateBinary();

    xSemaphoreGive( semaphoreButtonPressActive );

    int32_t priority = 1;
                           xTaskCreate(PollingTask,
                                       "ButtonPolling",
                                       NRF_BLE_FREERTOS_SDH_TASK_STACK,
                                       p_context,
                                       priority++,
                                       NULL);

    BaseType_t xReturned = xTaskCreate(softdevice_task,
                                       "BLE",
                                       NRF_BLE_FREERTOS_SDH_TASK_STACK,
                                       p_context,
                                       priority++,
                                       &m_softdevice_task);


    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("SoftDevice task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}
