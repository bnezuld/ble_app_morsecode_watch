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
static SemaphoreHandle_t  semaphorePolling = NULL, semaphoreISR = NULL, semaphoreSendMessage = NULL, semaphoreStopSendMessage = NULL;
static TimerHandle_t buttonReleaseTimer = NULL, DisplaySpaceTimer = NULL, DisplayBeepTimer = NULL;

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

static void TranslateMorseCode( TimerHandle_t xTimer )
{
    //stop recorButtonPresses task(can i use semaphore, i think beter to do this since is activated by a timer, block time must be zero if using semaphore)
    //vTaskSuspend(RecordButtonPressesTask);
    xSemaphoreTake(semaphoreISR, 0);//try to clear the semaphore for the ISR
    xSemaphoreTake(semaphorePolling, 0);//try to clear the semaphore for the ISR

    char* message = "i";//TranslateSelf();

    xQueueSend( messageQueue, &message, 1);

    //start recorButtonPresses task
    //vTaskResume(RecordButtonPressesTask);
    xSemaphoreGive(semaphoreISR);//try to clear the semaphore for the ISR
}

void ISR_buttonPressed()
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    if(xSemaphoreTakeFromISR( semaphoreISR, &xHigherPriorityTaskWoken ) == pdTRUE){
        xSemaphoreGiveFromISR( semaphorePolling, &xHigherPriorityTaskWoken );
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
        if( semaphorePolling != NULL )
        {
            /* See if we can obtain the semaphore.  If the semaphore is not
            available wait 10 ticks(can maybe increase this to max so it waits forever) to see if it becomes free. */
            if( xSemaphoreTake( semaphorePolling, ( TickType_t ) 10 ) == pdTRUE ){
                //xTimerStop(buttonReleaseTimer, 0);

                TickType_t endTicks, difference;
                /* Record button press */
                difference = xTaskGetTickCount() - startTicks;

                buttonRecord.buttonState = 0;//time from when it was released
                buttonRecord.time = difference;
                xQueueSend( buttonQueue, &buttonRecord, 0 );

                //turn on buzzer
                //GPIO_BUZZER_PORT->BSRR = (uint32_t)GPIO_BUZZER;

                startTicks = xTaskGetTickCount();

                /* Buton release polling */
                /*while(GPIO_ReadInputDataBit(GPIOA, GPIO_PIN_0) != Bit_RESET){
                        //wait for the button to be unpressed(or maybe can connect same button to a interrupt that can release and it will wait for that semaphore?)
                        TickType_t tmpTicks = xTaskGetTickCount();
                        vTaskDelayUntil(&tmpTicks, mainQUEUE_SEND_FREQUENCY_MS );
                        //no operation(used to keep empty while loop working)
                        asm("nop");
                }*/

                /* Record button release time*/
                //turn buzzer off maybe switch this to after the vTaskDelayUntil
                //GPIO_BUZZER_PORT->BRR = (uint32_t)GPIO_BUZZER;// << 16U;

                endTicks = xTaskGetTickCount();
                difference = endTicks - startTicks;

                buttonRecord.buttonState = 1;
                buttonRecord.time = difference;
                xQueueSend( buttonQueue, &buttonRecord, 1);

                //start Timer, to call the translate task
                //xTimerReset(buttonReleaseTimer, 0);
                //xTimerStart(buttonReleaseTimer, 0);

                //record start ticks
                startTicks = xTaskGetTickCount();

                //block so ISR semaphore in ISR cannot be triggerd for a period of time
                vTaskDelay(50);

                //release semaphoreISR (giving the semaphore so ISR can happen and give this task the semaphore it needs)
                xSemaphoreGive( semaphoreISR );
            }
        }
    }
}


void nrf_sdh_freertos_init(nrf_sdh_freertos_task_hook_t hook_fn, void * p_context)
{
    NRF_LOG_DEBUG("Creating a SoftDevice task.");

    m_task_hook = hook_fn;

    /* Create the timer(s) */
    //buttonReleaseTimer = xTimerCreate( 	"buttonTimer", 				/* A text name, purely to help debugging. */
    //                                    20/(1.0f/configTICK_RATE_HZ),/* The timer period, in this case (SPACE_TICK_LENGTH * 10) ms. */
    //                                    pdFALSE,					/* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
    //                                    ( void * ) 0,				/* The ID is not used, so can be set to anything. */
    //                                    TranslateMorseCode);			/* The callback function that switches the LED off. */

    buttonQueue = xQueueCreate( 10, sizeof( struct ButtonPress ) );
    messageQueue = xQueueCreate( 10, sizeof( char* ) );

    semaphorePolling = xSemaphoreCreateBinary();
    semaphoreISR = xSemaphoreCreateBinary();

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
