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

#ifndef NRF_SDH_FREERTOS_H__
#define NRF_SDH_FREERTOS_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "nrf_sdh.h"
#include "nrf_drv_gpiote.h"
#include "ble_ans_c.h"

typedef void (*ble_getNewAlert) ();
typedef void (*ble_replyToNotification) (char*);
typedef void (*def_writeSensorData) (uint8_t deviceAddress, uint16_t address, uint8_t data);
typedef uint8_t (*def_readSensorData) (uint8_t deviceAddress, uint16_t address);

typedef struct{
    ble_getNewAlert  GetNewAlert;
    ble_replyToNotification ReplyToNotification;
    def_writeSensorData writeSensorData;
    def_readSensorData readSensorData;
} sdhfreertos_init;

/**
 * @name FreeRTOS implementation of SoftDevice Handler
 * @{
 * @ingroup  nrf_sdh
 */

typedef void (*nrf_sdh_freertos_task_hook_t)(void * p_context);

/**@brief   Function for creating a task to retrieve SoftDevice events.
 * @param[in]   hook        Function to run in the SoftDevice FreeRTOS task,
 *                          before entering the task loop.
 * @param[in]   p_context   Parameter for the function @p hook.
 */
void nrf_sdh_freertos_init(nrf_sdh_freertos_task_hook_t hook, void * p_context, sdhfreertos_init const *freertos_init);

/**@brief   Function for dealing with button pressed event
 * @}
 */
void ISR_buttonPressed();

void AddToNotificationMsg(char* str, uint8_t length);

void CompleteNotificationMsg();


#ifdef __cplusplus
}
#endif

#endif /* NRF_SDH_FREERTOS_H__ */
