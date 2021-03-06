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
                          semaphoreCompleteNotificationMsg = NULL,
                          semaphoreSendMessageComplete = NULL,
                          semaphoreCurrentTimeComplete = NULL;

static TimerHandle_t buttonReleasedTimer = NULL, buttonPressedTimer = NULL, DisplaySpaceTimer = NULL, DisplayBeepTimer = NULL;

static TaskHandle_t                 m_softdevice_task,              //!< Reference to SoftDevice FreeRTOS task.
                                    RecordButtonPressesTask = NULL,
                                    polling_task = NULL,
                                    idle_task = NULL;  
#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

//diffrent handlers that will be called
static nrf_sdh_freertos_task_hook_t m_task_hook;        //!< A hook function run by the SoftDevice task before entering its loop.
static ble_getNewAlert getNewAlert_hook;   
static ble_replyToNotification replyToNotification_hook;
static def_writeSensorData writeSensorData_hook;
static def_readSensorData readSensorData_hook;   
static def_freertos_event_handler eventHandler;    

char* notificationMsg = NULL;
uint8_t notificationMsgLength = 0;

time_t m_time = 0;
static struct tm time_struct; 

uint8_t displayOut = 0;
     
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
    if(ulCount > GetSpaceUnit(SPACE_UNITS_END_OF_MESSAGE))
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
    TurnOnDisplay(1);
    if(xSemaphoreTakeFromISR( semaphoreButtonPressActive, &xHigherPriorityTaskWoken ) == pdTRUE){
        xSemaphoreGiveFromISR( semaphoreButtonPressed, &xHigherPriorityTaskWoken );
    }
    portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}

void ISRButtonReleased()
{
    portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
    TurnOffDisplay(1);
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
                        if(eventHandler(GET_CURRENT_TIME))
                        {
                                if(xSemaphoreTake(semaphoreCurrentTimeComplete, 10000*(1.0/(configTICK_RATE_HZ * (.001f)))) == pdTRUE)
                                {
                                        //update m_time is complete
                                }
                        }
                        
                        time_struct = *localtime(&m_time);
                        char* test = malloc(6 * sizeof(char));
                        test[0] = time_struct.tm_hour/10 + 48;
                        test[1] = time_struct.tm_hour%10 + 48;
                        test[2] = ' ';
                        test[3] = time_struct.tm_min/10 + 48;
                        test[4] = time_struct.tm_min%10 + 48;
                        test[5] = '\0';
                        if(xQueueSend(sendMessageQueue, &test, 10) != pdTRUE)
                        {
                                free(test);
                        }
                
		}else if(strcmp(message, "N") == 0)
		{
			free(message);

			char* test = malloc(2 * sizeof(char));
			test[0] = 'N';
			test[1] = '\0';
			//try to queue test
                        NRF_LOG_DEBUG("N menu");
			if(xQueueSend(sendMessageQueue, &test, 10) == pdTRUE)
			{
                                NRF_LOG_DEBUG("N menu1");
                                getNewAlert_hook();
                                if(xSemaphoreTake(semaphoreCompleteNotificationMsg, 10000*(1.0/(configTICK_RATE_HZ * (.001f)))) == pdTRUE)//TODO - error happening here semaphoreCompleteNotificationMsg is not being given if the notification is empty
                                {
                                    NRF_LOG_DEBUG("N menu2");

                                    xQueueSend(sendMessageQueue, &notificationMsg, portMAX_DELAY); 
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
			xSemaphoreGive(semaphoreStopSendMessage);
			xQueueReset(displayQueue);
                        
                        char* test = malloc(2 * sizeof(char));
			test[0] = 'R';
			test[1] = '\0';
			//try to queue test
                        NRF_LOG_DEBUG("R menu");
			if(xQueueSend(sendMessageQueue, &test, 10) == pdTRUE)
			{
                            NRF_LOG_DEBUG("R menu 1");
                            xQueueReceive( messageQueue, &message, portMAX_DELAY );
                            replyToNotification_hook(message);
                            free(message);
			}else
			{
                            free(test);
			}
			//reply to previous Notification?
		}else if(strcmp(message, "S") == 0)
		{
                        free(message);

                        char* test = malloc(2 * sizeof(char));
			test[0] = 'S';
			test[1] = '\0';
                        if(xQueueSend(sendMessageQueue, &test, portMAX_DELAY) == pdTRUE)
			{
                            NRF_LOG_DEBUG("wait for message");
                            if(xSemaphoreTake(semaphoreSendMessageComplete, portMAX_DELAY) == pdTRUE)
                            {
                                //save current settings
                                uint8_t originalSpaceMod = GetSpaceUnitModifier(), updateSpeed = 0, msgSize = 0;
                                bool saveChange = false;

                                xQueueReceive( messageQueue, &message, portMAX_DELAY );
                                while(message[msgSize] != '\0' && msgSize <= 3)
                                {
                                    msgSize++;
                                };
                                if(msgSize < 3)
                                {
                                      for(int position = 0; position < msgSize; position++)
                                      {
                                          uint8_t messageNumber = message[position] - '0';
                                          if(messageNumber >= 0 && messageNumber < 10)
                                          {
                                              updateSpeed += messageNumber * ((msgSize - position) == 2 ? 10 : 1);
                                              saveChange = true;
                                          }else
                                          {
                                              saveChange = false;
                                              break;
                                          }
                                      }
                                }
                                
                                free(message);
                                
                                if(!saveChange)
                                {
                                    SetSpaceUnitModifier(originalSpaceMod);

                                    char* display = malloc(2 * sizeof(char));
                                    display[0] = 'I';
                                    display[1] = '\0';
                                    if(xQueueSend(sendMessageQueue, &display, portMAX_DELAY) != pdTRUE)
                                    {
                                        free(display);
                                    }
                                }else 
                                {
                                    SetSpaceUnitModifier(updateSpeed);
                                    writeSensorData_hook(0x50,0x0080,updateSpeed);

                                    char* display = malloc(2 * sizeof(char));
                                    display[0] = 'E';
                                    display[1] = '\0';
                                    if(xQueueSend(sendMessageQueue, &display, portMAX_DELAY) != pdTRUE)
                                    {
                                        free(display);
                                    }
                                }
                            }else
                            {
                                free(test);
                            }
                        }
		}else if(strcmp(message, "D") == 0)
		{
			free(message);
                        if(eventHandler(GET_CURRENT_TIME))
                        {
                                if(xSemaphoreTake(semaphoreCurrentTimeComplete, 10000*(1.0/(configTICK_RATE_HZ * (.001f)))) == pdTRUE)
                                {
                                        //update m_time is complete
                                }
                        }

                        time_struct = *localtime(&m_time);
                        char* test = malloc(11 * sizeof(char));
                        test[0] = time_struct.tm_mon/10 + 48;
                        test[1] = time_struct.tm_mon%10 + 48;
                        test[2] = ' ';
                        test[3] = time_struct.tm_mday/10 + 48;
                        test[4] = time_struct.tm_mday%10 + 48;
                        test[5] = ' ';
                        uint16_t year = time_struct.tm_year + 1900;
                        test[9] = year%10 + 48;
                        year -= year%10;
                        test[8] = (year%100)/10 + 48;
                        year -= year%100;
                        test[7] = (year%1000)/100 + 48;
                        year -= year%1000;
                        test[6] = year/1000 + 48;
                        test[10] = '\0';
                        if(xQueueSend(sendMessageQueue, &test, 10) != pdTRUE)
                        {
                                free(test);
                        }
                }else
		{
			free(message);
                        char* test = malloc(2 * sizeof(char));
			test[0] = 'I';
			test[1] = '\0';
			//try to queue test
			if(xQueueSend(sendMessageQueue, &test, 10) == pdTRUE)
			{

			}else
			{
                            free(test);
			}
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
                        TurnOffDisplay(1 << 1);
			xTimerReset(DisplaySpaceTimer, 0);
			return;
		}else{
                        TurnOffDisplay(1 << 1);
			xSemaphoreGive(semaphoreSendMessage);
                        xSemaphoreGive(semaphoreSendMessageComplete);
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
                        TurnOnDisplay(1 << 1);
			xTimerReset(DisplayBeepTimer, 0);
			return;
		}else{
                        TurnOffDisplay(1 << 1);
			xSemaphoreGive(semaphoreSendMessage);
                        xSemaphoreGive(semaphoreSendMessageComplete);
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
                        //try to free sendMessageQueue doing this at start so i know i can only queue a message once the previous one is finished
                        xQueueReceive( sendMessageQueue, &message, 10 );
                        NRF_LOG_DEBUG("SendMessage free message");
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
					char* c =  TranslateCharToMorseCode(*tmpMsg);//TODO - change morse code translate to return a uint8_t[] which contains the . as 0 and - as 1 ex. N = 00000110b = 6
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
									val = GetSpaceUnit(SPACE_UNITS_LETTERS);
                                                                        NRF_LOG_DEBUG("SendMessage val1: %i", val);
									xQueueSend( displayQueue, &val, portMAX_DELAY);//space
								}
								else{
									val = GetSpaceUnit(SPACE_UNITS_SPACE);
                                                                        NRF_LOG_DEBUG("SendMessage val2: %i", val);
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

void UpdateCurrentTime(time_t updateTime)
{
    m_time = updateTime - NRF_RTC2->COUNTER/8;
    xSemaphoreGive(semaphoreCurrentTimeComplete);
}

/**@brief A function which is hooked to idle task.
 * @note Idle hook must be enabled in FreeRTOS configuration (configUSE_IDLE_HOOK).
 */
void vApplicationIdleHook( void )
{
#if NRF_LOG_ENABLED
     if(!buffer_is_empty())
     {
        vTaskResume(m_logger_thread);
     }
#endif
}

void PRE_SLEEP_PROCESSING(TickType_t ticks)
{
    nrf_drv_gpiote_out_clear(PIN_OUT_MOTOR_SLEEP);
    nrf_drv_gpiote_out_clear(PIN_OUT_MOTOR_MODE);
    eventHandler(DISABLE_UART);
}

void POST_SLEEP_PROCESSING(TickType_t ticks)
{
    nrf_drv_gpiote_out_set(PIN_OUT_MOTOR_SLEEP);
    nrf_drv_gpiote_out_set(PIN_OUT_MOTOR_MODE);
    eventHandler(ENABLE_UART);
}

void RTC2_IRQHandler(void)
{
    if(NRF_RTC2->EVENTS_COMPARE[0])
    {
        NRF_RTC2->EVENTS_COMPARE[0] = 0;
        
        NRF_RTC2->TASKS_CLEAR = 1;
        
        m_time += 60;
        //if(cal_event_callback) cal_event_callback();
    }
}

void TurnOnDisplay(uint8_t displayPort)
{
    displayOut |= displayPort;
    eventHandler(ENABLE_DISPLAY_OUT);
}

void TurnOffDisplay(uint8_t displayPort)
{
    displayOut &= ~displayPort;
    if(displayOut == 0)
    {
        eventHandler(DISABLE_DISPLAY_OUT);
    }
}

#if configUSE_TICKLESS_IDLE == 2
disable_interrupts()
{
    //xTimerStop(ButtonReleased_handler,0);
    //xTimerStop(ButtonPressed_handler,0);
    //xTimerStop(DisplayOff,0);
    //xTimerStop(DisplayOn,0);
    //eventHandler(EVENT_DISCONNECT);
    //__disable_irq();
    //eventHandler(DISABLE_INTERRUPTS);
    eventHandler(DISABLE_UART);
    vTaskSuspend(m_logger_thread);
    vTaskSuspend(polling_task);
}

enable_interrupts()
{
    //__enable_irq();
    //eventHandler(ENABLE_INTERRUPTS);
    eventHandler(ENABLE_UART);
    //vTaskResume(m_logger_thread);
    vTaskResume(polling_task);
}

prvSleep()
{
    //shuts off and needs to restart?
    sd_power_system_off();
    //sd_app_evt_wait();

    //goes to sleep
    //nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}

prvStopTickInterruptTimer()
{
    //vTaskSuspend(idle_task);

    //nrf_rtc_int_disable(portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);

    /* Configure CTC interrupt */
    //nrf_rtc_cc_set(portNRF_RTC_REG, 0, wakeupTime);
    //nrf_rtc_event_clear(portNRF_RTC_REG, NRF_RTC_EVENT_COMPARE_0);
    //nrf_rtc_int_enable(portNRF_RTC_REG, NRF_RTC_INT_COMPARE0_MASK);
}

prvStartTickInterruptTimer()
{
    //vTaskResume(idle_task);
    //TickISR()
    //nrf_rtc_int_enable(portNRF_RTC_REG, NRF_RTC_INT_TICK_MASK);
}

/* First define the portSUPPRESS_TICKS_AND_SLEEP() macro.  The parameter is the
time, in ticks, until the kernel next needs to execute. */
//#define portSUPPRESS_TICKS_AND_SLEEP( xIdleTime ) vApplicationSleep( xIdleTime )

/* Define the function that is called by portSUPPRESS_TICKS_AND_SLEEP(). */
void portSUPPRESS_TICKS_AND_SLEEP( TickType_t xExpectedIdleTime )
{
//if first sleep since wake set timer to give user time to press a button


//unsigned long ulLowPowerTimeBeforeSleep, ulLowPowerTimeAfterSleep;
eSleepModeStatus eSleepStatus;

    /* Read the current time from a time source that will remain operational
    while the microcontroller is in a low power state. */
    //ulLowPowerTimeBeforeSleep = ulGetExternalTime();

    /* Stop the timer that is generating the tick interrupt. */
   
    prvStopTickInterruptTimer();

    /* Enter a critical section that will not effect interrupts bringing the MCU
    out of sleep mode. */
    disable_interrupts();

    nrf_gpio_cfg_sense_input(PIN_IN, NRF_GPIO_PIN_PULLUP, NRF_GPIO_PIN_SENSE_HIGH);

    /* Ensure it is still ok to enter the sleep mode. */
    eSleepStatus = eTaskConfirmSleepModeStatus();

    if( eSleepStatus == eAbortSleep )
    {
        /* A task has been moved out of the Blocked state since this macro was
        executed, or a context siwth is being held pending.  Do not enter a
        sleep state.  Restart the tick and exit the critical section. */
        prvStartTickInterruptTimer();
        enable_interrupts();
    }
    else
    {
        if( eSleepStatus == eNoTasksWaitingTimeout )
        {
            /* It is not necessary to configure an interrupt to bring the
            microcontroller out of its low power state at a fixed time in the
            future. */
            //NRF_LOG_DEBUG("long sleep");
            prvSleep();
        }
        else
        {
            //NRF_LOG_DEBUG("short sleep");
            /* Configure an interrupt to bring the microcontroller out of its low
            power state at the time the kernel next needs to execute.  The
            interrupt must be generated from a source that remains operational
            when the microcontroller is in a low power state. */
            //vSetWakeTimeInterrupt( xExpectedIdleTime );//2634 2638 2608

            /* Enter the low power state. */
            //prvSleep();

            /* Determine how long the microcontroller was actually in a low power
            state for, which will be less than xExpectedIdleTime if the
            microcontroller was brought out of low power mode by an interrupt
            other than that configured by the vSetWakeTimeInterrupt() call.
            Note that the scheduler is suspended before
            portSUPPRESS_TICKS_AND_SLEEP() is called, and resumed when
            portSUPPRESS_TICKS_AND_SLEEP() returns.  Therefore no other tasks will
            execute until this function completes. */
            //ulLowPowerTimeAfterSleep = ulGetExternalTime();

            /* Correct the kernels tick count to account for the time the
            microcontroller spent in its low power state. */
            //vTaskStepTick( ulLowPowerTimeAfterSleep - ulLowPowerTimeBeforeSleep );
        }

        /* Exit the critical section - it might be possible to do this immediately
        after the prvSleep() calls. */
        enable_interrupts();

        /* Restart the timer that is generating the tick interrupt. */
        prvStartTickInterruptTimer();
    }
}
#endif


nrf_sdh_freertos_TaskStratScheduler()
{
    vTaskStartScheduler();
    idle_task = xTaskGetIdleTaskHandle();
}

void nrf_sdh_freertos_init(nrf_sdh_freertos_task_hook_t hook_fn, void * p_context, sdhfreertos_init const* freertos_init)
{
    NRF_LOG_DEBUG("Creating a SoftDevice task.");

    m_task_hook = hook_fn;
    getNewAlert_hook = freertos_init->GetNewAlert;
    replyToNotification_hook = freertos_init->ReplyToNotification;
    writeSensorData_hook = freertos_init->writeSensorData;
    readSensorData_hook = freertos_init->readSensorData;
#if NRF_LOG_ENABLED
    m_logger_thread = freertos_init->m_logger_thread;
#endif
    eventHandler = freertos_init->eventHandler;

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
    semaphoreSendMessageComplete = xSemaphoreCreateBinary(); 
    semaphoreCurrentTimeComplete = xSemaphoreCreateBinary(); 

    xSemaphoreGive( semaphoreButtonPressActive );
    xSemaphoreGive( semaphoreSendMessage );

    int32_t priority = 2;

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
                                       &polling_task);

    BaseType_t xReturned = xTaskCreate(softdevice_task,
                                       "BLE",
                                       NRF_BLE_FREERTOS_SDH_TASK_STACK,
                                       p_context,
                                       priority++,
                                       &m_softdevice_task);

    SetSpaceUnitModifier(readSensorData_hook(0x50,0x0080));

    if (xReturned != pdPASS)
    {
        NRF_LOG_ERROR("SoftDevice task not created.");
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
}
