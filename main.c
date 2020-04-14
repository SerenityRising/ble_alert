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
/** @file
 *
 * @defgroup ble_sdk_app_beacon_main main.c
 * @{
 * @ingroup ble_sdk_app_beacon
 * @brief Beacon Transmitter Sample Application main file.
 *
 * This file contains the source code for an Beacon transmitter sample application.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//ADDED START
#include "nrfx_lpcomp.h"
#include "nrf_lpcomp.h"
#include "boards.h"
#include "nrfx_timer.h"
//ADDED END

#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

//ADDED START
static void nrfx_lpcomp_event_handler(nrf_lpcomp_event_t event);
//declare out timer instance as being Timer 1.
//static scope so LPCOMP can access and start timer
static const nrfx_timer_t nrfx_timer_1 = NRFX_TIMER_INSTANCE(1);
static void nrfx_timer_event_handler(nrf_timer_event_t event_type, void * p_context);
static uint8_t tone_burst_count = 0;
static void lpcomp_init(void);
static void timer1_init(void);
//ADDED END

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};


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

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;

    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
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
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

/**@brief Function for initializing the LPCOMP for smoke detector sensing.
 * ADDED
 *@details 
 */
static void lpcomp_init(void)
{
  //make config struct for LPCOMP for initialization later
    nrfx_lpcomp_config_t nrfx_lpcomp_config = {
                                               //NRFX_LPCOMP_CONFIG_REFERENCE is set to <3=> Supply 4/8 
                                               //NRFX_LPCOMP_CONFIG_DETECTION is set to <1=> Up
                                               //NRFX_LPCOMP_CONFIG_HYST is set to 1. It looks like this is automatically
                                               //     set to 50mV in nrf_lpcomp.h
                                              .hal                = { 
                                                                      (nrf_lpcomp_ref_t) 3,
                                                                      (nrf_lpcomp_detect_t) 1,
                                                                      (nrf_lpcomp_hysteresis_t) 1 
                                                                    },
                                              //NRFX_LPCOMP_CONFIG_INPUT set to <7=> 7 (Pin 0.31)
                                              .input              = (nrf_lpcomp_input_t) 7,
                                              //NRFX_LPCOMP_CONFIG_IRQ_PRIORITY set to <2=> 2
                                              .interrupt_priority = 2
                                              };
    //initialize LPCOMP
    nrfx_err_t err_code = nrfx_lpcomp_init(&nrfx_lpcomp_config,
                                           nrfx_lpcomp_event_handler);

    //have to override work done in SDK init function
    //because they went too far with abstraction and
    //made this impossible through their preferred interface
    //nrf_lpcomp_int_enable(LPCOMP_INTENSET_UP_Msk | LPCOMP_INTENSET_DOWN_Msk); 
}

/**@brief This function initialized Timer1 for smoke detector sensing.
 * ADDED
 * @details Expects a global or static variable "static const nrfx_timer_t nrfx_timer_1 = NRFX_TIMER_INSTANCE(1);" to 
 *          be in the file. Expects a handler function "static void nrfx_timer_event_handler(nrf_timer_event_t event_type, void * p_context)"
 *          to be in the file.
 */
static void timer1_init(void)
{
    //Note that that Timer 1 instance was created as static outside of main.
    //Note that if frequency is set to 31.25KHz max CC time is ~2 seconds with default
    //    bit width of 16. To get more time increase bit width.

    //make timer 1 config struct for init function later
    nrfx_timer_config_t nrfx_timer_config_1 = {
                                                //frequency set to <9=> 31.25 kHz 
                                               .frequency          = (nrf_timer_frequency_t) 9,
                                                //mode set to <0=> Timer 
                                               .mode               = (nrf_timer_mode_t) 0,
                                                //bit_width set to <2=> 24 bit
                                               .bit_width          = (nrf_timer_bit_width_t) 2,
                                                //interrupt_priority set to <3=> 3
                                               .interrupt_priority = 3,                    
                                               .p_context          = NULL                                                       
                                              };

    //find out how many ticks we need for our counter times:
    //1.25 seconds worth of ticks at 31.25KHz
    uint32_t timer_1_CC0_ticks = 39063;
                                 //nrfx_timer_ms_to_ticks(&nrfx_timer_1, 
                                 //      (uint32_t) 1250);
    //0.75 seconds worth of ticks at 31.25KHz
    uint32_t timer_1_CC1_ticks = 23438;
                                 //nrfx_timer_ms_to_ticks(&nrfx_timer_1, 
                                 //      (uint32_t) 750);

    //3 seconds worth of ticks at 31.25KHz
    uint32_t timer_1_CC2_ticks = 93750;
                                 //nrfx_timer_ms_to_ticks(&nrfx_timer_1, 
                                 //      (uint32_t) 3000);

    //initialize timer 1
    nrfx_err_t timer_init_err = nrfx_timer_init(&nrfx_timer_1,
                                                &nrfx_timer_config_1,
                                                nrfx_timer_event_handler);

    //set up CC0 for 1.25 sec and enable interrupts
    nrfx_timer_compare(&nrfx_timer_1,
                       (nrf_timer_cc_channel_t) 0,
                       timer_1_CC0_ticks,
                       1);

    //set up CC1 for 0.75 sec and enable interrupts
    nrfx_timer_compare(&nrfx_timer_1,
                       (nrf_timer_cc_channel_t) 1,
                       timer_1_CC1_ticks,
                       1);

    //set up CC2 for 3 sec and enable interrupts
    nrfx_timer_compare(&nrfx_timer_1,
                       (nrf_timer_cc_channel_t) 2,
                       timer_1_CC2_ticks,
                       1);
    
    //enabling automatically starts the timer, so we need to
    //    stop it and reset after enabling
    nrfx_timer_enable(&nrfx_timer_1);
    nrfx_timer_pause(&nrfx_timer_1);
    nrfx_timer_clear(&nrfx_timer_1);
}

/**ADDED
 * @brief LPCOMP event handler is called when LPCOMP detects voltage drop.
 *
 * This function is called from interrupt context so it is very important
 * to return quickly. Don't put busy loops or any other CPU intensive actions here.
 * It is also not allowed to call soft device functions from it (if LPCOMP IRQ
 * priority is set to APP_IRQ_PRIORITY_HIGH).
 */
static void nrfx_lpcomp_event_handler(nrf_lpcomp_event_t event)
{
    if (event == NRF_LPCOMP_EVENT_UP)
    {
      //TODO LPCOMP RISING EVENT CODE HERE

      //disable LPCOMP interrupts so we only trigger once at
      //    start of PWM (timer will turn them back on later)
      //nrf_lpcomp_int_disable(LPCOMP_INTENSET_UP_Msk);
      nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_STOP);
      //turn on LED3
      bsp_board_led_on(BSP_BOARD_LED_2);
      //increment tone_burst_count
      tone_burst_count++;
      //clear Timer 1
      nrfx_timer_clear(&nrfx_timer_1);
      //start Timer 1
      nrfx_timer_resume(&nrfx_timer_1);
    }

    //if you want to use NRF_LPCOMP_EVENT_DOWN and UP  at the same time you have to have
    //  nrf_lpcomp_int_enable(LPCOMP_INTENSET_UP_Msk | LPCOMP_INTENSET_DOWN_Msk)
    //  after LPCOMP is initialized
    //else if (event == NRF_LPCOMP_EVENT_DOWN)
    //{
      //TODO LPCOMP FALLING EVENT CODE HERE
    //  bsp_board_led_off(BSP_BOARD_LED_2);
    //}
}

/**ADDED
 * @brief Timer driver event handler type.
 *
 * @param[in] event_type Timer event.
 * @param[in] p_context  General purpose parameter set during initialization of
 *                       the timer. This parameter can be used to pass
 *                       additional information to the handler function, for
 *                       example, the timer ID.
 */
static void nrfx_timer_event_handler(nrf_timer_event_t event_type, void * p_context)
{
    //TODO TIMER1 COMPARE 0 EVENT CODE HERE
    //the mid-burst inter-tone timeout has been reached
    if(event_type == NRF_TIMER_EVENT_COMPARE0)
    {
      //if we got here because a tone 3xburst finished...
      if(tone_burst_count == 3)
      {
        //turn on LED4
        bsp_board_led_on(BSP_BOARD_LED_3);
      }
      //if we got here but three tones weren't counted...
      else
      {
        //turn off LED4
        bsp_board_led_off(BSP_BOARD_LED_3);
      }
      tone_burst_count = 0;
    }
    //TODO TIMER1 COMPARE 1 EVENT CODE HERE
    //the LPCOMP interrupt pause is done
    else if(event_type == NRF_TIMER_EVENT_COMPARE1)
    {
      //turn off LED3
      bsp_board_led_off(BSP_BOARD_LED_2);
      //nrf_lpcomp_int_enable(LPCOMP_INTENSET_UP_Msk);
      nrf_lpcomp_task_trigger(NRF_LPCOMP_TASK_START);
    }
    //TODO TIMER1 COMPARE 2 EVENT CODE HERE
    //the inter-burst timeout has been reached
    else if(event_type == NRF_TIMER_EVENT_COMPARE2)
    {
      //turn off LED4
      bsp_board_led_off(BSP_BOARD_LED_3);
      //pause timers
      nrfx_timer_pause(&nrfx_timer_1);
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    log_init();
    timers_init();
    leds_init();
    power_management_init();
    ble_stack_init();
    advertising_init();
    
    //ADDED START
    bsp_board_init(BSP_INIT_LEDS);
    lpcomp_init();
    timer1_init();
    nrfx_lpcomp_enable();
    //ADDED END

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        idle_state_handle();
    }
}


/**
 * @}
 */
