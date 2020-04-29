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
                          semaphoreSendMessage = NULL, semaphoreStopSendMessage = NULL,
                          semaphoreCompleteNotificationMsg = NULL;

static TimerHandle_t buttonReleasedTimer = NULL, buttonPressedTimer = NULL, DisplaySpaceTimer = NULL, DisplayBeepTimer = NULL;

static TaskHandle_t                 m_softdevice_task,              //!< Reference to SoftDevice FreeRTOS task.
                                    RecordButtonPressesTask = NULL;  

//diffrent handlers that will be called
static nrf_sdh_freertos_task_hook_t m_task_hook;        //!< A hook function run by the SoftDevice task before entering its loop.
static ble_getNewAlert getNewAlert_hook;        

char* notificationMsg = NULL;
uint8_t notificationMsgLength = 0;
     
struct ButtonPress{
	uint8_t time;
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
    vTaskSuspend(RecordButtonPressesTask);

    //try to take semaphores so tasks do not start(could maybe suspend the task until finished)
    xSemaphoreTake(semaphoreButtonPressActive, 0);
    xSemaphoreTake(semaphoreButtonPressed, 0);
    xSemaphoreTake(semaphoreButtonReleaseActive, 0);
    xSemaphoreTake(semaphoreButtonReleased, 0);

    char* message = TranslateSelf();
    NRF_LOG_INFO("translate MorseCode %s", message);
    
    xQueueSend( messageQueue, &message, 1);

    //start recorButtonPresses task
    vTaskResume(RecordButtonPressesTask);
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

                uint8_t unheldCount = ( uint8_t ) pvTimerGetTimerID( buttonReleasedTimer );
                buttonRecord.buttonState = 0;
                buttonRecord.time = unheldCount;
                xQueueSend( buttonQueue, &buttonRecord, 2 );
                NRF_LOG_DEBUG("unheld time: %d", unheldCount);
                
                xSemaphoreGive( semaphoreButtonReleaseActive );
                if( xSemaphoreTake( semaphoreButtonReleased, portMAX_DELAY) == pdTRUE ){
                    vTimerSetTimerID(buttonReleasedTimer, 0u);
                    xTimerStop(buttonPressedTimer, 0);
                    xTimerStart(buttonReleasedTimer, 0);

                    uint8_t heldCount = ( uint8_t ) pvTimerGetTimerID( buttonPressedTimer );
                    buttonRecord.buttonState = 1;
                    buttonRecord.time = heldCount;
                    xQueueSend( buttonQueue, &buttonRecord, 2 );
                    NRF_LOG_DEBUG("held time: %d", heldCount);
                }
            
                //release semaphoreButtonPressActive (giving the semaphore so ISR can happen and give this task the semaphore it needs)
                xSemaphoreGive( semaphoreButtonPressActive );
            }
        }
    }
}

static void RecordButtonPresses( void *pvParameters )
{
	struct ButtonPress buttonRecord;
	char *message;
	for(;;)
	{
		//wait for something to be in the queue for portMAX_DELAY and record it
		xQueueReceive( buttonQueue, &buttonRecord, portMAX_DELAY );
		ButtonPress(buttonRecord.time,buttonRecord.buttonState);
	}
}

static void Menu( void *pvParameters )
{
	char *message = NULL;
	for(;;){
		xQueueReceive( messageQueue, &message, portMAX_DELAY );
		if(strcmp(message, "T") == 0)
		{
			free(message);
		}else if(strcmp(message, "N") == 0)
		{
			free(message);

			char* test = malloc(3 * sizeof(char));
			test[0] = 'N';
			test[1] = '\0';
			//try to queue test
                        NRF_LOG_DEBUG("N menu");
			if(xQueueSend(sendMessageQueue, &test, 10) == pdTRUE)
			{
                                NRF_LOG_DEBUG("N menu1");
                                getNewAlert_hook();
                                if(xSemaphoreTake(semaphoreCompleteNotificationMsg, portMAX_DELAY) == pdTRUE)//TODO - error happening here semaphoreCompleteNotificationMsg is not being given if the notification is empty
                                {
                                    xQueueSend(sendMessageQueue, &notificationMsg, 10); 
                                    notificationMsg = NULL;
                                    notificationMsgLength = 0;
                                }
			}else
			{
				free(test);
			}
		}else if(strcmp(message, "I") == 0)
		{
			free(message);
			xSemaphoreGive(semaphoreStopSendMessage);
			xQueueReset(displayQueue);
		}else if(strcmp(message, "R") == 0)
		{
			free(message);
			//reply to previous Notification?
		}else
		{
			free(message);
		}
	}
}

#pragma region name



#pragma endregion name

static void DisplayOn( TimerHandle_t xTimer )
{
	int8_t ulCount = ( int32_t ) pvTimerGetTimerID( xTimer );

	if(ulCount == 0 )
	{
		if(xQueueReceive( displayQueue, &ulCount, 0) == pdTRUE)
		{
			vTimerSetTimerID( DisplaySpaceTimer, ( void * ) ulCount );
			nrf_drv_gpiote_out_set(18);
			xTimerReset(DisplaySpaceTimer, 0);
			return;
		}else{
			nrf_drv_gpiote_out_set(18);
			xSemaphoreGive(semaphoreSendMessage);
		}
	}else{
		ulCount--;
		vTimerSetTimerID( xTimer, ( void * ) ulCount );
		xTimerReset(DisplayBeepTimer, 0);
	}
}

static void DisplayOff( TimerHandle_t xTimer )
{
	int8_t ulCount = ( int32_t ) pvTimerGetTimerID( xTimer );

	if(ulCount == 0 )
	{
		if(xQueueReceive( displayQueue, &ulCount, 0) == pdTRUE)
		{
			vTimerSetTimerID( DisplayBeepTimer, ( void * ) ulCount );
			nrf_drv_gpiote_out_clear(18);
			xTimerReset(DisplayBeepTimer, 0);
			return;
		}else{
			nrf_drv_gpiote_out_set(18);
			xSemaphoreGive(semaphoreSendMessage);
		}
	}else{
		ulCount--;
		vTimerSetTimerID( xTimer, ( void * ) ulCount );
		xTimerReset(DisplaySpaceTimer, 0);
	}
}

static void SendMessage(void *pvParameters )
{
	char* message;
	for(;;){
		//wait for previous write to finish
		if(xSemaphoreTake( semaphoreSendMessage, portMAX_DELAY ) == pdTRUE)
		{
			//check to see if there is a message to be sent
			if(xQueuePeek( sendMessageQueue, &message, portMAX_DELAY ) == pdTRUE)
			{
                                NRF_LOG_DEBUG("SendMessage: \"%s\"", message);
				//try to take the semaphore if it was given when not sending a message
				xSemaphoreTake( semaphoreStopSendMessage, 0 );
				char* tmpMsg = message;
				int resetTimer = 1;//only reset timer once, although there is still a possible race condition if the queue empties before it finishes processing this message
				while(*tmpMsg != '\0')
				{
                                        NRF_LOG_DEBUG("SendMessage char: %c", *tmpMsg);
					//translate message
					//queue up message for timers to use
					char* c =  TranslateCharToMorseCode(*tmpMsg);
					uint8_t validNextChar = *(tmpMsg + 1) != ' ';
					while(*c != '\0')
					{
						if(xSemaphoreTake( semaphoreStopSendMessage, 0 ) == pdTRUE)
						{
							*(tmpMsg + 1) = '\0';
							break;
						}
						int8_t val = -1;
						if(*c == '.')
							val = 0;
						else if(*c == '-')
							val = 2;

						if(val >= 0){
							xQueueSend( displayQueue, &val, portMAX_DELAY);//beep
							val = 0;
							if(*(c + 1) == '\0')
							{
								if(validNextChar == 1){
									val = SPACE_UNITS_LETTERS;
									xQueueSend( displayQueue, &val, portMAX_DELAY);//space
								}
								else{
									val = SPACE_UNITS_SPACE;
									xQueueSend( displayQueue, &val, portMAX_DELAY);//space
								}
							}else
							{
								xQueueSend( displayQueue, &val, portMAX_DELAY);//space
							}
							if(resetTimer == 1){
								xTimerReset(DisplaySpaceTimer, 0);
								resetTimer = 0;
							}
						}
						c++;
					}
					tmpMsg++;
				}
                                //timers were not set, and must give semaphoreSendMessage since the timers would normally do that
                                if(resetTimer == 1)
                                {
                                    xSemaphoreGive(semaphoreSendMessage);
                                }
				free(message);
				xQueueReceive( sendMessageQueue, &message, portMAX_DELAY );
                                NRF_LOG_DEBUG("SendMessage free message");
			}
		}
	}
}

void AddToNotificationMsg(char* str, uint8_t length)
{
    char* tmpNotificationMsg = notificationMsg;
    char* tmp2notificationMsg = malloc((notificationMsgLength + length) * sizeof(char));
    notificationMsg = tmp2notificationMsg;

    if(tmpNotificationMsg != NULL)
    {
        memcpy(notificationMsg,
               tmpNotificationMsg,
               notificationMsgLength);

        free(tmpNotificationMsg);
    }

    memcpy(&notificationMsg[notificationMsgLength],
           str,
           length);
    notificationMsgLength += length;
    NRF_LOG_DEBUG("str: %s", str);
    NRF_LOG_DEBUG("notification add %d: %s", notificationMsgLength, notificationMsg);
}

void CompleteNotificationMsg()
{
    //give semahpore to signal notification is completely received
        NRF_LOG_DEBUG("CompleteNotificationMsg %d: %s", notificationMsgLength, notificationMsg);
    xSemaphoreGive(semaphoreCompleteNotificationMsg);
}

void nrf_sdh_freertos_init(nrf_sdh_freertos_task_hook_t hook_fn, void * p_context, sdhfreertos_init const* freertos_init)
{
    NRF_LOG_DEBUG("Creating a SoftDevice task.");

    m_task_hook = hook_fn;
    getNewAlert_hook = freertos_init->GetNewAlert;

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
    DisplaySpaceTimer = xTimerCreate( 	"DisplayOff", 				/* A text name, purely to help debugging. */
                                        100*(1.0/(configTICK_RATE_HZ * (.001f))),/* The timer period, in this case (SPACE_TICK_LENGTH * 10) ms. */
                                        pdFALSE,					/* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                                        ( void * ) 0,				/* The ID is not used, so can be set to anything. */
                                        DisplayOff);			/* The callback function that switches the LED off. */
    DisplayBeepTimer = xTimerCreate( 	"DisplayOn", 				/* A text name, purely to help debugging. */
                                        100*(1.0/(configTICK_RATE_HZ * (.001f))),/* The timer period, in this case (SPACE_TICK_LENGTH * 10) ms. */
                                        pdFALSE,					/* This is a one-shot timer, so xAutoReload is set to pdFALSE. */
                                        ( void * ) 0,				/* The ID is not used, so can be set to anything. */
                                        DisplayOn);			/* The callback function that switches the LED off. */

    buttonQueue = xQueueCreate( 10, sizeof( struct ButtonPress ) );
    messageQueue = xQueueCreate( 10, sizeof( char* ) );
    sendMessageQueue = xQueueCreate( 1, sizeof( char* ) );
    displayQueue = xQueueCreate( 10, sizeof( int8_t ) );

    semaphoreButtonPressed = xSemaphoreCreateBinary();
    semaphoreButtonPressActive = xSemaphoreCreateBinary();
    semaphoreButtonReleased = xSemaphoreCreateBinary();
    semaphoreButtonReleaseActive = xSemaphoreCreateBinary();
    semaphoreSendMessage = xSemaphoreCreateBinary();
    semaphoreStopSendMessage = xSemaphoreCreateBinary();
    semaphoreCompleteNotificationMsg = xSemaphoreCreateBinary(); 

    xSemaphoreGive( semaphoreButtonPressActive );
    xSemaphoreGive( semaphoreSendMessage );

    int32_t priority = 1;

                          xTaskCreate(SendMessage,
                                       "SendMessage",
                                       configMINIMAL_STACK_SIZE,
                                       p_context,
                                       priority++,
                                       NULL);

                          xTaskCreate(Menu,
                                       "Menu",
                                       200,
                                       p_context,
                                       priority++,
                                       NULL);

                          xTaskCreate(RecordButtonPresses,
                                       "RecordBP",
                                       configMINIMAL_STACK_SIZE,
                                       p_context,
                                       priority++,
                                       &RecordButtonPressesTask);

                           xTaskCreate(PollingTask,
                                       "ButtonPolling",
                                       configMINIMAL_STACK_SIZE,
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
