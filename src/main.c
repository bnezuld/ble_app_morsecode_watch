/**
 * Copyright (c) 2014 - 2019, Nordic Semiconductor ASA
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
// Board/nrf6310/ble/ble_app_hrs_rtx/main.c
/**
 *
 * @brief Heart Rate Service Sample Application with RTX main file.
 *
 * This file contains the source code for a sample application using RTX and the
 * Heart Rate service (and also Battery and Device Information services).
 * This application uses the @ref srvlib_conn_params module.
 */

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_twi.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_hrs.h"
#include "ble_dis.h"
#include "ble_ans_c.h"
#include "ble_conn_params.h"
#include "sensorsim.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_freertos.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
//#include "bsp_btn_ble.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "fds.h"
#include "ble_conn_state.h"
#include "nrf_drv_clock.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_gap.h"
#include "ble_err.h"


#include "ble_db_discovery.h"

#include "nrf_pwr_mgmt.h"

#include "nrf_delay.h"

#include "nrf_drv_rtc.h"
#include "nrf_drv_clock.h"

#include "ble_cts_c.h"
#include "ble_date_time.h"




#define DEVICE_NAME                         "Nordic_MorseCode_Smartwatch"           /**< Name of device. Will be included in the advertising data. */
#define MANUFACTURER_NAME                   "NordicSemiconductor"                   /**< Manufacturer. Will be passed to Device Information Service. */

#define APP_BLE_OBSERVER_PRIO               3                                       /**< Application's BLE observer priority. You shouldn't need to modify this value. */
#define APP_BLE_CONN_CFG_TAG                1                                       /**< A tag identifying the SoftDevice BLE configuration. */

#define APP_ADV_INTERVAL                    1600                                     /**< The advertising interval (in units of 0.625 ms. This value corresponds to 1000 ms). */
#define APP_ADV_DURATION                    0                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds.or 0 for advertising forever */

#define BATTERY_LEVEL_MEAS_INTERVAL         2000                                    /**< Battery level measurement interval (ms). */
#define MIN_BATTERY_LEVEL                   81                                      /**< Minimum simulated battery level. */
#define MAX_BATTERY_LEVEL                   100                                     /**< Maximum simulated battery level. */
#define BATTERY_LEVEL_INCREMENT             1                                       /**< Increment between each simulated battery level measurement. */

#define HEART_RATE_MEAS_INTERVAL            1000                                    /**< Heart rate measurement interval (ms). */
#define MIN_HEART_RATE                      140                                     /**< Minimum heart rate as returned by the simulated measurement function. */
#define MAX_HEART_RATE                      300                                     /**< Maximum heart rate as returned by the simulated measurement function. */
#define HEART_RATE_INCREMENT                10                                      /**< Value by which the heart rate is incremented/decremented for each call to the simulated measurement function. */

#define RR_INTERVAL_INTERVAL                320                                     /**< RR interval interval (ms). */
#define MIN_RR_INTERVAL                     100                                     /**< Minimum RR interval as returned by the simulated measurement function. */
#define MAX_RR_INTERVAL                     500                                     /**< Maximum RR interval as returned by the simulated measurement function. */
#define RR_INTERVAL_INCREMENT               1                                       /**< Value by which the RR interval is incremented/decremented for each call to the simulated measurement function. */

#define SENSOR_CONTACT_DETECTED_INTERVAL    5000                                    /**< Sensor Contact Detected toggle interval (ms). */

#define MIN_CONN_INTERVAL                   MSEC_TO_UNITS(400, UNIT_1_25_MS)        /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL                   MSEC_TO_UNITS(650, UNIT_1_25_MS)        /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY                       0                                       /**< Slave latency. */
#define CONN_SUP_TIMEOUT                    MSEC_TO_UNITS(4000, UNIT_10_MS)         /**< Connection supervisory time-out (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY      5000                                    /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY       30000                                   /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT        3                                       /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_BOND                      1                                       /**< Perform bonding. */
#define SEC_PARAM_MITM                      0                                       /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                      0                                       /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS                  0                                       /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES           BLE_GAP_IO_CAPS_NONE                    /**< No I/O capabilities. */
#define SEC_PARAM_OOB                       0                                       /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE              7                                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE              16                                      /**< Maximum encryption key size. */

#define DEAD_BEEF                           0xDEADBEEF                              /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define OSTIMER_WAIT_FOR_QUEUE              2                                       /**< Number of ticks to wait for the timer queue to be ready */

#define MESSAGE_BUFFER_SIZE             17                                          /**< Size of buffer holding optional messages in notifications. */
#define BLE_ANS_NB_OF_CATEGORY_ID       10                                          /**< Number of categories. */

/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

typedef enum 
{
    ALERT_NOTIFICATION_DISABLED, /**< Alert Notifications has been disabled. */
    ALERT_NOTIFICATION_ENABLED,  /**< Alert Notifications has been enabled. */
    ALERT_NOTIFICATION_ON,       /**< Alert State is on. */
} ble_ans_c_alert_state_t;

BLE_BAS_DEF(m_bas);                                                 /**< Battery service instance. */
BLE_HRS_DEF(m_hrs);                                                 /**< Heart rate service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                 /**< Advertising module instance. */
BLE_ANS_C_DEF(m_ans_c);  
BLE_CTS_C_DEF(m_cts_c);                                                             /**< Current Time service instance. */
BLE_DB_DISCOVERY_DEF(m_ble_db_discovery);  
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                                    /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_PERIPHERAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);      

/* Indicates if operation on TWI has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample[2] = {0, 0};
/* Buffer for samples write to temperature sensor. */
static uint8_t m_write_sample[1] = {7};

static volatile bool readDone = true;

static uint16_t m_conn_handle         = BLE_CONN_HANDLE_INVALID;    /**< Handle of the current connection. */
static bool     m_rr_interval_enabled = true;                       /**< Flag for enabling and disabling the registration of new RR interval measurements (the purpose of disabling this is just to test sending HRM without RR interval data. */


static uint8_t m_alert_message_buffer[MESSAGE_BUFFER_SIZE];                         /**< Message buffer for optional notify messages. */
static ble_ans_c_alert_state_t m_new_alert_state    = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of New Alert Notifications, i.e. Enabled, Alert On, Disabled. */
static ble_ans_c_alert_state_t m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of Unread Alert Notifications, i.e. Enabled, Alert On, Disabled. */

static sensorsim_cfg_t   m_battery_sim_cfg;                         /**< Battery Level sensor simulator configuration. */
static sensorsim_state_t m_battery_sim_state;                       /**< Battery Level sensor simulator state. */
static sensorsim_cfg_t   m_heart_rate_sim_cfg;                      /**< Heart Rate sensor simulator configuration. */
static sensorsim_state_t m_heart_rate_sim_state;                    /**< Heart Rate sensor simulator state. */
static sensorsim_cfg_t   m_rr_interval_sim_cfg;                     /**< RR Interval sensor simulator configuration. */
static sensorsim_state_t m_rr_interval_sim_state;                   /**< RR Interval sensor simulator state. */

static ble_uuid_t m_adv_uuids[] =                                   /**< Universally unique service identifiers. */
{
    {BLE_UUID_HEART_RATE_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
    {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}
};

/**@brief String literals for the iOS notification categories. used then printing to UART. */
static char const * lit_catid[BLE_ANS_NB_OF_CATEGORY_ID] =
{
    "Simple alert",
    "Email",
    "News",
    "Incoming call",
    "Missed call",
    "SMS/MMS",
    "Voice mail",
    "Schedule",
    "High prioritized alert",
    "Instant message"
};

static TimerHandle_t m_battery_timer;                               /**< Definition of battery timer. */
static TimerHandle_t m_heart_rate_timer;                            /**< Definition of heart rate timer. */
static TimerHandle_t m_rr_interval_timer;                           /**< Definition of RR interval timer. */
static TimerHandle_t m_sensor_contact_timer;                        /**< Definition of sensor contact detected timer. */

#if NRF_LOG_ENABLED
static TaskHandle_t m_logger_thread;                                /**< Definition of Logger thread. */
#endif

static void advertising_start(void * p_erase_bonds);

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    bool delete_bonds = false;

    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);

    switch (p_evt->evt_id)
    {
        case PM_EVT_PEERS_DELETE_SUCCEEDED:
            advertising_start(&delete_bonds);
            break;

        default:
            break;
    }
}


/**@brief Function for performing battery measurement and updating the Battery Level characteristic
 *        in Battery Service.
 */
static void battery_level_update(void)
{
    ret_code_t err_code;
    uint8_t  battery_level;

    battery_level = (uint8_t)sensorsim_measure(&m_battery_sim_state, &m_battery_sim_cfg);

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}


/**@brief Function for handling the Battery measurement timer time-out.
 *
 * @details This function will be called each time the battery level measurement timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void battery_level_meas_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);
    battery_level_update();
}


/**@brief Function for handling the Heart rate measurement timer time-out.
 *
 * @details This function will be called each time the heart rate measurement timer expires.
 *          It will exclude RR Interval data from every third measurement.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void heart_rate_meas_timeout_handler(TimerHandle_t xTimer)
{
    static uint32_t cnt = 0;
    ret_code_t      err_code;
    uint16_t        heart_rate;

    UNUSED_PARAMETER(xTimer);

    heart_rate = (uint16_t)sensorsim_measure(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    cnt++;
    err_code = ble_hrs_heart_rate_measurement_send(&m_hrs, heart_rate);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }

    // Disable RR Interval recording every third heart rate measurement.
    // NOTE: An application will normally not do this. It is done here just for testing generation
    // of messages without RR Interval measurements.
    m_rr_interval_enabled = ((cnt % 3) != 0);
}


/**@brief Function for handling the RR interval timer time-out.
 *
 * @details This function will be called each time the RR interval timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void rr_interval_timeout_handler(TimerHandle_t xTimer)
{
    UNUSED_PARAMETER(xTimer);

    if (m_rr_interval_enabled)
    {
        uint16_t rr_interval;

        rr_interval = (uint16_t)sensorsim_measure(&m_rr_interval_sim_state,
                                                  &m_rr_interval_sim_cfg);
        ble_hrs_rr_interval_add(&m_hrs, rr_interval);
    }
}


/**@brief Function for handling the Sensor Contact Detected timer time-out.
 *
 * @details This function will be called each time the Sensor Contact Detected timer expires.
 *
 * @param[in] xTimer Handler to the timer that called this function.
 *                   You may get identifier given to the function xTimerCreate using pvTimerGetTimerID.
 */
static void sensor_contact_detected_timeout_handler(TimerHandle_t xTimer)
{
    static bool sensor_contact_detected = false;

    UNUSED_PARAMETER(xTimer);

    sensor_contact_detected = !sensor_contact_detected;
    ble_hrs_sensor_contact_detected_update(&m_hrs, sensor_contact_detected);
}


/**@brief Function for the Timer initialization.
 *
 * @details Initializes the timer module. This creates and starts application timers.
 */
static void timers_init(void)
{
    // Initialize timer module.
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);

    // Create timers.
    /*m_battery_timer = xTimerCreate("BATT",
                                   BATTERY_LEVEL_MEAS_INTERVAL,
                                   pdTRUE,
                                   NULL,
                                   battery_level_meas_timeout_handler);
    m_heart_rate_timer = xTimerCreate("HRT",
                                      HEART_RATE_MEAS_INTERVAL,
                                      pdTRUE,
                                      NULL,
                                      heart_rate_meas_timeout_handler);
    m_rr_interval_timer = xTimerCreate("RRT",
                                       RR_INTERVAL_INTERVAL,
                                       pdTRUE,
                                       NULL,
                                       rr_interval_timeout_handler);
    m_sensor_contact_timer = xTimerCreate("SCT",
                                          SENSOR_CONTACT_DETECTED_INTERVAL,
                                          pdTRUE,
                                          NULL,
                                          sensor_contact_detected_timeout_handler);*/

    /* Error checking */
    /*if ( (NULL == m_battery_timer)
         || (NULL == m_heart_rate_timer)
         || (NULL == m_rr_interval_timer)
         || (NULL == m_sensor_contact_timer) )
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }*/
}


/**@brief Function for the GAP initialization.
 *
 * @details This function sets up all the necessary GAP (Generic Access Profile) parameters of the
 *          device including the device name, appearance, and the preferred connection parameters.
 */
static void gap_params_init(void)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_appearance_set(BLE_APPEARANCE_GENERIC_WATCH);
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the GATT module. */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for handling database discovery events.
 *
 * @details This function is callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function should forward the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    //if(p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_ALERT_NOTIFICATION_SERVICE)
    {
        ble_ans_c_on_db_disc_evt(&m_ans_c, p_evt);
    }
    //if(p_evt->params.discovered_db.srv_uuid.uuid == BLE_UUID_CURRENT_TIME_SERVICE)
    {
        ble_cts_c_on_db_disc_evt(&m_cts_c, p_evt);
    }
}

/** @brief Database discovery module initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t err_code = ble_db_discovery_init(&db_init);

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for setup of alert notifications in central.
 *
 * @details This function will be called when a successful connection has been established.
 */
static void alert_notification_setup(void)
{
    ret_code_t err_code;

    err_code = ble_ans_c_enable_notif_new_alert(&m_ans_c);
    APP_ERROR_CHECK(err_code);

    m_new_alert_state = ALERT_NOTIFICATION_ENABLED;
    NRF_LOG_INFO("New Alert State: Enabled.");

    err_code = ble_ans_c_enable_notif_unread_alert(&m_ans_c);
    APP_ERROR_CHECK(err_code);

    m_unread_alert_state = ALERT_NOTIFICATION_ENABLED;
    NRF_LOG_INFO("Unread Alert State: Enabled.");

    NRF_LOG_DEBUG("Notifications enabled.");
}

static void GetNewAlert()
{
    //m_ans_c->message_buffer_size;
    //m_ans_c->p_message_buffer;
    if(m_ans_c.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ret_code_t err_code;

        err_code = ble_ans_c_new_alert_notify(&m_ans_c, ANS_TYPE_ALL_ALERTS);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("GetNewAlert.");
    }
    //err_code = ble_ans_c_new_alert_read(&m_ans_c);
    //APP_ERROR_CHECK(err_code);
    //return msg;
    //uint8_t *
    /*memcpy(p_alert->p_alert_msg_buf,
           &p_notification->data[NOTIFICATION_DATA_LENGTH],
           p_alert->alert_msg_length); //lint !e416

    //these are both used in 
    m_ans_c->p_gatt_queue//the gatt queue
    m_ans_c->conn_handle//conn handler
ee
    NRF_LOG_DEBUG("Notifications Enabled.");
    m_ans_c
    ble_ans_c_enable_notif_new_alert(&m_ans_c);
    m_new_alert_state = ALERT_NOTIFICATION_ENABLED;*/
}

static void ReplyToNotification(char * msg)
{
    //m_ans_c->message_buffer_size;
    //m_ans_c->p_message_buffer;
    if(m_ans_c.conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        ret_code_t err_code;

        err_code = ble_ans_c_reply_new_alert(&m_ans_c, ANS_TYPE_ALL_ALERTS, msg);
        APP_ERROR_CHECK(err_code);

        NRF_LOG_DEBUG("Reply To Notification.");
    }
    //err_code = ble_ans_c_new_alert_read(&m_ans_c);
    //APP_ERROR_CHECK(err_code);
    //return msg;
    //uint8_t *
    /*memcpy(p_alert->p_alert_msg_buf,
           &p_notification->data[NOTIFICATION_DATA_LENGTH],
           p_alert->alert_msg_length); //lint !e416

    //these are both used in 
    m_ans_c->p_gatt_queue//the gatt queue
    m_ans_c->conn_handle//conn handler
ee
    NRF_LOG_DEBUG("Notifications Enabled.");
    m_ans_c
    ble_ans_c_enable_notif_new_alert(&m_ans_c);
    m_new_alert_state = ALERT_NOTIFICATION_ENABLED;*/
}

/**@brief Function for lighting up the LED corresponding to the notification received.
 *
 * @param[in]   p_evt   Event containing the notification.
 */
static void handle_alert_notification(ble_ans_c_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->uuid.uuid == BLE_UUID_UNREAD_ALERT_CHAR)
    {
        if (m_unread_alert_state == ALERT_NOTIFICATION_ENABLED)
        {
            //err_code = bsp_indication_set(BSP_INDICATE_ALERT_1);
            //APP_ERROR_CHECK(err_code);
            m_unread_alert_state = ALERT_NOTIFICATION_ON;
            NRF_LOG_INFO("Unread Alert state: On.");
            NRF_LOG_INFO("  Category:                 %s",
                         (uint32_t)lit_catid[p_evt->data.alert.alert_category]);
            NRF_LOG_INFO("  Number of unread alerts:  %d",
                         p_evt->data.alert.alert_category_count);
        }
    }
    else if (p_evt->uuid.uuid == BLE_UUID_NEW_ALERT_CHAR)
    {
        NRF_LOG_INFO("m_new_alert_state: %d", m_new_alert_state);
        //if (m_new_alert_state == ALERT_NOTIFICATION_ENABLED)
        {
            //err_code = bsp_indication_set(BSP_INDICATE_ALERT_0);
            //APP_ERROR_CHECK(err_code);
            //m_new_alert_state = ALERT_NOTIFICATION_ON;
            NRF_LOG_INFO("New Alert state: On.");
            NRF_LOG_INFO("  Category:                 %s",
                         (uint32_t)lit_catid[p_evt->data.alert.alert_category]);
            NRF_LOG_INFO("  Number of new alerts:     %d",
                         p_evt->data.alert.alert_category_count);
            NRF_LOG_INFO("  Text String Length:  %d",
                         (uint32_t)p_evt->data.alert.alert_msg_length);
            NRF_LOG_INFO("  Text String Information:  %s",
                         (uint32_t)p_evt->data.alert.p_alert_msg_buf);
            NRF_LOG_INFO("  Text String Information:  %s",
                         (uint32_t)p_evt->data.alert.alert_msg_continued);
            if(p_evt->data.alert.p_alert_msg_buf != NULL)
            {
                if(p_evt->data.alert.alert_msg_continued == 1)
                {
                    AddToNotificationMsg(p_evt->data.alert.p_alert_msg_buf, p_evt->data.alert.alert_msg_length);

                    err_code = ble_ans_c_new_alert_notify(&m_ans_c, ANS_TYPE_ALL_ALERTS);
                    APP_ERROR_CHECK(err_code);
                
                }else
                {
                    AddToNotificationMsg(p_evt->data.alert.p_alert_msg_buf, p_evt->data.alert.alert_msg_length);
                    CompleteNotificationMsg();
                }
            }else
            {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                             
                CompleteNotificationMsg();
            }
        }
    }
    else
    {
        // Only Unread and New Alerts exists, thus do nothing.
    }
}

/**@brief Function for reading supported alert notifications in central.
 *
 * @details This function will be called when a connection has been established.
 */
static void supported_alert_notification_read(void)
{
    NRF_LOG_DEBUG("Read supported Alert Notification characteristics on the connected peer.");

    ret_code_t err_code;

    err_code = ble_ans_c_new_alert_cat_read(&m_ans_c);
    APP_ERROR_CHECK(err_code);

    err_code = ble_ans_c_unread_alert_read(&m_ans_c);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for setup of alert notifications in central.
 *
 * @details This function will be called when supported alert notification and
 *          supported unread alert notifications has been fetched.
 *
 * @param[in] p_evt  Event containing the response with supported alert types.
 */
static void control_point_setup(ble_ans_c_evt_t * p_evt)
{
    uint32_t                err_code;
    ble_ans_control_point_t setting;

    if (p_evt->uuid.uuid == BLE_UUID_SUPPORTED_UNREAD_ALERT_CATEGORY_CHAR)
    {
        setting.command  = ANS_ENABLE_UNREAD_CATEGORY_STATUS_NOTIFICATION;
        setting.category = (ble_ans_category_id_t)p_evt->data.alert.alert_category;
        NRF_LOG_DEBUG("Unread status notification enabled for received categories.");
    }
    else if (p_evt->uuid.uuid == BLE_UUID_SUPPORTED_NEW_ALERT_CATEGORY_CHAR)
    {
        setting.command  = ANS_ENABLE_NEW_INCOMING_ALERT_NOTIFICATION;
        setting.category = (ble_ans_category_id_t)p_evt->data.alert.alert_category;
        NRF_LOG_DEBUG("New incoming notification enabled for received categories.");
    }
    else
    {
        return;
    }

    err_code = ble_ans_c_control_point_write(&m_ans_c, &setting, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the Alert Notification Service Client.
 *
 * @details This function will be called for all events in the Alert Notification Client which
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Alert Notification Service Client.
 */
static void on_ans_c_evt(ble_ans_c_evt_t * p_evt)
{
    ret_code_t err_code;
    NRF_LOG_DEBUG("on_ans_c_evt %d", p_evt->evt_type);
    switch (p_evt->evt_type)
    {
        case BLE_ANS_C_EVT_NOTIFICATION:
            handle_alert_notification(p_evt);
            NRF_LOG_DEBUG("Alert Notification received from server, UUID: %X.", p_evt->uuid.uuid);
            break; // BLE_ANS_C_EVT_NOTIFICATION

        case BLE_ANS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_DEBUG("Alert Notification Service discovered on the server.");
            err_code = ble_ans_c_handles_assign(&m_ans_c,
                                                p_evt->conn_handle,
                                                &p_evt->data.service);
            APP_ERROR_CHECK(err_code);
            supported_alert_notification_read();
            alert_notification_setup();
            break; // BLE_ANS_C_EVT_DISCOVERY_COMPLETE

        case BLE_ANS_C_EVT_READ_RESP:
            NRF_LOG_DEBUG("Alert Setup received from server, UUID: %X.", p_evt->uuid.uuid);
            control_point_setup(p_evt);
            break; // BLE_ANS_C_EVT_READ_RESP

        case BLE_ANS_C_EVT_DISCONN_COMPLETE:
            m_new_alert_state    = ALERT_NOTIFICATION_DISABLED;
            m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;

            //err_code = bsp_indication_set(BSP_INDICATE_ALERT_OFF);
            //APP_ERROR_CHECK(err_code);
            break; // BLE_ANS_C_EVT_DISCONN_COMPLETE

        default:
            NRF_LOG_DEBUG("not implemented evt_type");
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the Alert Notification Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void alert_notification_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for handling the Current Time Service client events.
 *
 * @details This function will be called for all events in the Current Time Service client that
 *          are passed to the application.
 *
 * @param[in] p_evt Event received from the Current Time Service client.
 */
static void on_cts_c_evt(ble_cts_c_t * p_cts, ble_cts_c_evt_t * p_evt)
{
    ret_code_t err_code;

    switch (p_evt->evt_type)
    {
        case BLE_CTS_C_EVT_DISCOVERY_COMPLETE:
            NRF_LOG_INFO("Current Time Service discovered on server.");
            err_code = ble_cts_c_handles_assign(&m_cts_c,
                                                p_evt->conn_handle,
                                                &p_evt->params.char_handles);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_CTS_C_EVT_DISCOVERY_FAILED:
            NRF_LOG_INFO("Current Time Service not found on server. ");
            // CTS not found in this case we just disconnect. There is no reason to stay
            // in the connection for this simple app since it all wants is to interact with CT
            if (p_evt->conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = sd_ble_gap_disconnect(p_evt->conn_handle,
                                                 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BLE_CTS_C_EVT_DISCONN_COMPLETE:
            NRF_LOG_INFO("Disconnect Complete.");
            break;

        case BLE_CTS_C_EVT_CURRENT_TIME:
            NRF_LOG_INFO("Current Time received.");
            //current_time_print(p_evt);
            time_t updateTime = 0;
            static struct tm time_struct;
            time_struct.tm_year = p_evt->params.current_time.exact_time_256.day_date_time.date_time.year - 1900;
            time_struct.tm_mon = p_evt->params.current_time.exact_time_256.day_date_time.date_time.month;
            time_struct.tm_mday = p_evt->params.current_time.exact_time_256.day_date_time.date_time.day;
            time_struct.tm_hour = p_evt->params.current_time.exact_time_256.day_date_time.date_time.hours;
            time_struct.tm_min = p_evt->params.current_time.exact_time_256.day_date_time.date_time.minutes;
            time_struct.tm_sec = p_evt->params.current_time.exact_time_256.day_date_time.date_time.seconds;   
            updateTime = mktime(&time_struct);
            UpdateCurrentTime(updateTime);
            break;

        case BLE_CTS_C_EVT_INVALID_TIME:
            NRF_LOG_INFO("Invalid Time received.");
            break;

        default:
            break;
    }
} 


/**@brief Function for handling the Current Time Service errors.
 *
 * @param[in]  nrf_error  Error code containing information about what went wrong.
 */
static void current_time_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void)
{
    ret_code_t         err_code;
    ble_hrs_init_t     hrs_init;
    ble_bas_init_t     bas_init;
    ble_dis_init_t     dis_init;
    ble_ans_c_init_t     ans_c_init;
    ble_cts_c_init_t   cts_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    uint8_t            body_sensor_location;

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Heart Rate Service.
    body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

    memset(&hrs_init, 0, sizeof(hrs_init));

    hrs_init.evt_handler                 = NULL;
    hrs_init.is_sensor_contact_supported = true;
    hrs_init.p_body_sensor_location      = &body_sensor_location;

    // Here the sec level for the Heart Rate Service can be changed/increased.
    hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
    hrs_init.bsl_rd_sec      = SEC_OPEN;

    err_code = ble_hrs_init(&m_hrs, &hrs_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);

    // Initialize Device Information Service.
    memset(&dis_init, 0, sizeof(dis_init));

    ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

    dis_init.dis_char_rd_sec = SEC_OPEN;

    err_code = ble_dis_init(&dis_init);
    APP_ERROR_CHECK(err_code);

    // Initialize the Alert Notification Service Client.
    memset(&ans_c_init, 0, sizeof(ans_c_init));
    memset(m_alert_message_buffer, 0, MESSAGE_BUFFER_SIZE);

    ans_c_init.evt_handler         = on_ans_c_evt;
    ans_c_init.message_buffer_size = MESSAGE_BUFFER_SIZE;
    ans_c_init.p_message_buffer    = m_alert_message_buffer;
    ans_c_init.error_handler       = alert_notification_error_handler;
    ans_c_init.p_gatt_queue        = &m_ble_gatt_queue;

    err_code = ble_ans_c_init(&m_ans_c, &ans_c_init);
    APP_ERROR_CHECK(err_code);

    // Initialize CTS.
    cts_init.evt_handler   = on_cts_c_evt;
    cts_init.error_handler = current_time_error_handler;
    cts_init.p_gatt_queue  = &m_ble_gatt_queue;
    err_code               = ble_cts_c_init(&m_cts_c, &cts_init);
    APP_ERROR_CHECK(err_code);
}



/**@brief Function for initializing the sensor simulators. */
static void sensor_simulator_init(void)
{
    m_battery_sim_cfg.min          = MIN_BATTERY_LEVEL;
    m_battery_sim_cfg.max          = MAX_BATTERY_LEVEL;
    m_battery_sim_cfg.incr         = BATTERY_LEVEL_INCREMENT;
    m_battery_sim_cfg.start_at_max = true;

    sensorsim_init(&m_battery_sim_state, &m_battery_sim_cfg);

    m_heart_rate_sim_cfg.min          = MIN_HEART_RATE;
    m_heart_rate_sim_cfg.max          = MAX_HEART_RATE;
    m_heart_rate_sim_cfg.incr         = HEART_RATE_INCREMENT;
    m_heart_rate_sim_cfg.start_at_max = false;

    sensorsim_init(&m_heart_rate_sim_state, &m_heart_rate_sim_cfg);

    m_rr_interval_sim_cfg.min          = MIN_RR_INTERVAL;
    m_rr_interval_sim_cfg.max          = MAX_RR_INTERVAL;
    m_rr_interval_sim_cfg.incr         = RR_INTERVAL_INCREMENT;
    m_rr_interval_sim_cfg.start_at_max = false;

    sensorsim_init(&m_rr_interval_sim_state, &m_rr_interval_sim_cfg);
}


/**@brief   Function for starting application timers.
 * @details Timers are run after the scheduler has started.
 */
static void application_timers_start(void)
{
    // Start application timers.
    /*if (pdPASS != xTimerStart(m_battery_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTimerStart(m_heart_rate_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTimerStart(m_rr_interval_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
    if (pdPASS != xTimerStart(m_sensor_contact_timer, OSTIMER_WAIT_FOR_QUEUE))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }*/
}


/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in]   p_evt   Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    ret_code_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module. */
static void conn_params_init(void)
{
    ret_code_t             err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = m_hrs.hrm_handles.cccd_handle;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_ble_timeout_enter(void)
{
    ret_code_t err_code;

    //err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    //APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    //err_code = bsp_btn_ble_sleep_mode_prepare();
    //APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("sleep mode zzz...");

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    sd_nvic_SystemReset();
    APP_ERROR_CHECK(err_code);
}

static void sleep_mode_enter()
{
    nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_INFO("Fast advertising.");
            //err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            //APP_ERROR_CHECK(err_code);
            break;

        case BLE_ADV_EVT_IDLE:
            sleep_mode_ble_timeout_enter();
            break;

        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            //err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            //APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = ble_db_discovery_start(&m_ble_db_discovery, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
            if (p_ble_evt->evt.gap_evt.conn_handle == m_cts_c.conn_handle)
            {
                m_cts_c.conn_handle = BLE_CONN_HANDLE_INVALID;
            }
            if (p_ble_evt->evt.gap_evt.conn_handle == m_ans_c.conn_handle)
            {
                m_ans_c.conn_handle = BLE_CONN_HANDLE_INVALID;
            }

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            NRF_LOG_DEBUG("GATT Client Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            NRF_LOG_DEBUG("GATT Server Timeout.");
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
    bool is_set = nrf_drv_gpiote_in_is_set(PIN_IN);
    if(!is_set)
    {
      ISRButtonReleased();
    }else
    {
      ISRButtonPressed();
    }
}
/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
static void gpio_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_gpiote_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
        nrf_drv_gpiote_out_config_t out_config_low = GPIOTE_CONFIG_OUT_SIMPLE(true);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT_MOTOR_SLEEP, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(PIN_OUT_MOTOR_SLEEP);

    err_code = nrf_drv_gpiote_out_init(PIN_OUT_MOTOR_MODE, &out_config);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_set(PIN_OUT_MOTOR_MODE); 

    err_code = nrf_drv_gpiote_out_init(PIN_OUT, &out_config_low);
    APP_ERROR_CHECK(err_code);
    nrf_drv_gpiote_out_clear(PIN_OUT);

    //err_code = nrf_drv_gpiote_out_init(PIN_OUT_2, &out_config_low);
    //APP_ERROR_CHECK(err_code);
    //nrf_drv_gpiote_out_clear(PIN_OUT_2);

    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(PIN_IN, true);
}


/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
static bool freertos_event_handler(uint8_t event)
{
    ret_code_t err_code;
    switch (event)
    {
        case EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }else
            {
                return false;
            }
            break;

        case EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }else
                {
                    return false;
                }
            }else
            {
                return false;
            }
            break;

        case DISABLE_UART:
            nrf_drv_twi_disable(&m_twi);
            break;
        case ENABLE_UART:
            nrf_drv_twi_enable(&m_twi);
            break;
        case DISABLE_INTERRUPTS:
            nrf_drv_gpiote_in_event_disable(PIN_IN);
            break;
        case ENABLE_INTERRUPTS:  ;
            nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);
            in_config.pull = NRF_GPIO_PIN_PULLUP;

            err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
            APP_ERROR_CHECK(err_code);

            nrf_drv_gpiote_in_event_enable(PIN_IN, true);
            break;
        case GET_CURRENT_TIME:
            if (m_cts_c.conn_handle != BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_cts_c_current_time_read(&m_cts_c);
                if (err_code == NRF_ERROR_NOT_FOUND)
                {
                    NRF_LOG_INFO("Current Time Service is not discovered.");
                    return false;
                }
            }else
            {
                return false;
            }
            break;
        case ENABLE_DISPLAY_OUT:
            nrf_drv_gpiote_out_set(PIN_OUT);
            break;
        case DISABLE_DISPLAY_OUT:
            nrf_drv_gpiote_out_clear(PIN_OUT);
            break;
        default:
            return false;
            break;
    }
    return true;
}


/**@brief Function for the Peer Manager initialization. */
static void peer_manager_init(void)
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/**@brief Clear bond information from persistent storage. */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the Advertising functionality. */
static void advertising_init(void)
{
    ret_code_t             err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    init.advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance      = true;
    init.advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
    init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    init.advdata.uuids_complete.p_uuids  = m_adv_uuids;

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;

    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
/*static void buttons_leds_init(bool * p_erase_bonds)
{
    ret_code_t err_code;
    bsp_event_t startup_event;

    err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}*/


/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;

    if (erase_bonds)
    {
        delete_bonds();
        // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
    }
    else
    {
        ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
        APP_ERROR_CHECK(err_code);
    }
}


#if NRF_LOG_ENABLED
/**@brief Thread for handling the logger.
 *
 * @details This thread is responsible for processing log entries if logs are deferred.
 *          Thread flushes all log entries and suspends. It is resumed by idle task hook.
 *
 * @param[in]   arg   Pointer used for passing some arbitrary information (context) from the
 *                    osThreadCreate() call to the thread.
 */
static void logger_thread(void * arg)
{
    UNUSED_PARAMETER(arg);

    while (1)
    {
        NRF_LOG_FLUSH();

        vTaskSuspend(NULL); // Suspend myself
    }
}
#endif //NRF_LOG_ENABLED

/**@brief Function for initializing the clock.
 */
static void clock_init(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint8_t temp)
{
    NRF_LOG_INFO("Temperature: %d Celsius degrees.", temp);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            readDone = true;
            NRF_LOG_INFO("read done true");
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = 27,
       .sda                = 26,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_LOW,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&m_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&m_twi);
}


/**
 * @brief Function for writing data to temperature sensor.
 * @param[in] p_instance Pointer to the driver instance structure.
 * @param[in] address    Address of a specific slave device (only 7 LSB).
 * @param[in] p_data     Pointer to a transmit buffer.
 * @param[in] length     Number of bytes to send.
 * @param[in] no_stop    If set, the stop condition is not generated on the bus
 *                       after the transfer has completed successfully (allowing
 *                       for a repeated start in the next transfer).
 */
void writeSensorData(uint8_t deviceAddress, uint16_t address, uint8_t data)
{

    uint8_t outgoing_data[3] = {address >> 8, address & 0xFF, data};

    ret_code_t err_code; 
    readDone = false;

    do{
    err_code = nrf_drv_twi_tx(&m_twi, deviceAddress, outgoing_data, sizeof(outgoing_data), false);
    APP_ERROR_CHECK(err_code);
    nrf_delay_us(500);
    }while(readDone == false);
}

uint8_t readSensorData(uint8_t deviceAddress, uint16_t address)
{

    readDone = false;
    uint8_t outgoing_data[2] = {address >> 8, address & 0xFF};

    ret_code_t err_code; 
    
    do{
    err_code = nrf_drv_twi_tx(&m_twi, deviceAddress, outgoing_data, sizeof(outgoing_data), false);
    APP_ERROR_CHECK(err_code);
    nrf_delay_us(500);
    }while(readDone == false);
    /*{
        __WFE();
    }*/

    readDone = false;
    NRF_LOG_INFO("read done false");
    uint8_t resultsWhoAmI[1];
    do{
    err_code = nrf_drv_twi_rx(&m_twi, 0x50, resultsWhoAmI, sizeof(resultsWhoAmI));
    APP_ERROR_CHECK(err_code);
    nrf_delay_us(500);
    }while(readDone == false);
    /*{
        __WFE();
    }*/

    return resultsWhoAmI[0];
}

//const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(2);//


/** @brief Function initialization and configuration of RTC driver instance.
 */
static void rtc_config(void)
{
    /* Request LF clock */
    nrf_drv_clock_lfclk_request(NULL);

    // Configure the RTC for 1 minute wakeup (default)
    NRF_RTC2->PRESCALER = 0xFFF;
    NRF_RTC2->EVTENSET = RTC_EVTENSET_COMPARE0_Msk;
    NRF_RTC2->INTENSET = RTC_INTENSET_COMPARE0_Msk;
    NRF_RTC2->CC[0] = 60 * 8;
    NRF_RTC2->TASKS_START = 1;


    NVIC_SetPriority(RTC2_IRQn, configKERNEL_INTERRUPT_PRIORITY);
    NVIC_EnableIRQ(RTC2_IRQn);
}

/**@brief Function for application main entry.
 */
int main(void)
 {
    bool erase_bonds;

    // Initialize modules.
    log_init();
    clock_init();

    // Do not start any interrupt that uses system functions before system initialisation.
    // The best solution is to start the OS before any other initalisation.

#if NRF_LOG_ENABLED
    // Start execution.
    if (pdPASS != xTaskCreate(logger_thread, "LOGGER", 256, NULL, 1, &m_logger_thread))
    {
        APP_ERROR_HANDLER(NRF_ERROR_NO_MEM);
    }
#endif

    // Activate deep sleep mode.
    SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;

    // Configure and initialize the BLE stack.
    ble_stack_init();

    // Initialize modules.
    timers_init();
    //buttons_leds_init(&erase_bonds);
    gap_params_init();
    gatt_init();
    advertising_init();
    db_discovery_init();
    services_init();
    sensor_simulator_init();
    conn_params_init();
    peer_manager_init();
    gpio_init();
    twi_init();
    application_timers_start();
    rtc_config();



//used to test the clock frequency
    ret_code_t err_code = sd_clock_hfclk_request();
    
    APP_ERROR_CHECK(err_code);
    uint32_t hfclk_is_running = 0;
    while (!hfclk_is_running) {
	APP_ERROR_CHECK(sd_clock_hfclk_is_running(&hfclk_is_running));
    }  // */ 

    // Create a FreeRTOS task for the BLE stack.
    // The task will run advertising_start() before entering its loop.
    sdhfreertos_init freertos_init;
    memset(&freertos_init, 0, sizeof(freertos_init));

    //read_sensor_data();
    //while(readDone == false)
    //{
    //    __WFE();
    //}

    freertos_init.GetNewAlert = GetNewAlert;
    freertos_init.ReplyToNotification = ReplyToNotification;
    freertos_init.writeSensorData = writeSensorData;
    freertos_init.readSensorData = readSensorData;
#if NRF_LOG_ENABLED
    freertos_init.m_logger_thread = m_logger_thread;
#endif
    freertos_init.eventHandler = freertos_event_handler;
    nrf_sdh_freertos_init(advertising_start, &erase_bonds, &freertos_init);

    //write_sensor_data(0x50, 0x0080, 0xaa);
    //read_sensor_data();
    NRF_LOG_INFO("HRS FreeRTOS example started.");
    // Start FreeRTOS scheduler.
    nrf_sdh_freertos_TaskStratScheduler();

    for (;;)
    {
        APP_ERROR_HANDLER(NRF_ERROR_FORBIDDEN);
    }
}


