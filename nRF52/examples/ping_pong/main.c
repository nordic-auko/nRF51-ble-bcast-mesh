/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include "rbc_mesh.h"
#include "cmd_if.h"
#include "timeslot_handler.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define MESH_ACCESS_ADDR        (0x8E89BED6)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (37)
#define MESH_CLOCK_SRC_PPM      (75)

#define CENTRAL_LINK_COUNT              0                                          /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                          /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define INVALID_HANDLE          (RBC_MESH_INVALID_HANDLE)

static rbc_mesh_value_handle_t m_handle = INVALID_HANDLE;
static uint8_t m_data[RBC_MESH_VALUE_MAX_LEN];

extern void UART0_IRQHandler(void);

static void print_usage(void)
{
    _LOG("To configure: transmit the handle number this device responds to, \r\n"
    "or 0 to respond to all handles.\r\n");
}

/** 
* @brief Handle an incoming command, and act accordingly.
*/
static void cmd_rx(uint8_t* cmd, uint32_t len)
{
    if (len <= 1)
        return;
    m_handle = atoi((char*) cmd);
    if (m_handle == 0)
    {
        m_data[6]++;
        _LOG("Responding to all\r\n");
    }
    else
    {
        m_data[6]++;
        rbc_mesh_value_set(m_handle, m_data, RBC_MESH_VALUE_MAX_LEN);
        rbc_mesh_persistence_set(m_handle, true);
        _LOG("Responding to handle %d\r\n", (int) m_handle);
    }
}

/**
* @brief General error handler.
*/
static void error_loop(void)
{
    nrf_gpio_pin_clear(LED_START + 1);
    nrf_gpio_pin_set(LED_START + 2);
    __disable_irq();
    while (1)
    {
        UART0_IRQHandler();
    }
}

/**
* @brief Softdevice crash handler, never returns
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    _LOG("SD ERROR: %s:L%d\r\n", (const char*) p_file_name, (int) line_num);
    SEGGER_RTT_printf(0, "SD ERROR: %s:L%d\r\n", (const char*) p_file_name, (int) line_num);
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
//void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
//{
//    _LOG("APP ERROR: %s:L%d - E:%X\r\n", p_file_name, (int) line_num, (int) error_code);
//    SEGGER_RTT_printf(0, "APP ERROR: %s:L%d - E:%X\r\n", p_file_name, (int) line_num, (int) error_code);
//    error_loop();
//}

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    /* static error variables - in order to prevent removal by optimizers */
    static volatile struct
    {
        uint32_t        fault_id;
        uint32_t        pc;
        uint32_t        error_info;
        assert_info_t * p_assert_info;
        error_info_t  * p_error_info;
        ret_code_t      err_code;
        uint32_t        line_num;
        const uint8_t * p_file_name;
    } m_error_data = {0};

    // The following variable helps Keil keep the call stack visible, in addition, it can be set to
    // 0 in the debugger to continue executing code after the error check.
    volatile bool loop = true;
    UNUSED_VARIABLE(loop);

    m_error_data.fault_id   = id;
    m_error_data.pc         = pc;
    m_error_data.error_info = info;

    switch (id)
    {
        case NRF_FAULT_ID_SDK_ASSERT:
            m_error_data.p_assert_info = (assert_info_t *)info;
            m_error_data.line_num      = m_error_data.p_assert_info->line_num;
            m_error_data.p_file_name   = m_error_data.p_assert_info->p_file_name;
            break;

        case NRF_FAULT_ID_SDK_ERROR:
            m_error_data.p_error_info = (error_info_t *)info;
            m_error_data.err_code     = m_error_data.p_error_info->err_code;
            m_error_data.line_num     = m_error_data.p_error_info->line_num;
            m_error_data.p_file_name  = m_error_data.p_error_info->p_file_name;
            break;
    }
    
    _LOG("APP ERROR: %s:L%d - E:%X\r\n", m_error_data.p_file_name, (int) m_error_data.line_num, (int) m_error_data.err_code);
    SEGGER_RTT_printf(0, "APP ERROR: %s:L%d - E:%X\r\n", m_error_data.p_file_name, (int) m_error_data.line_num, (int) m_error_data.err_code);
    error_loop();
}



void HardFault_Handler(void)
{
    _LOG("HARDFAULT\r\n");
    SEGGER_RTT_printf(0, "HARDFAULT\r\n");
    error_loop();
}

/**
* @brief RBC_MESH framework event handler. Handles events coming from the mesh. 
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{ 
    static const char cmd[] = {'U', 'C', 'N', 'I', 'T'};
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:  
            if (evt->value_handle == m_handle || m_handle == 0)
            {
                nrf_gpio_pin_toggle(LED_START);
                if (m_handle == 0)
                {
                    rbc_mesh_value_set(evt->value_handle, m_data, 1); /* short ack */
                    _LOG("%c[%d] \r\n", cmd[evt->event_type], evt->value_handle);
                }
                else
                {
                    m_data[6]++;
                    rbc_mesh_value_set(evt->value_handle, m_data, RBC_MESH_VALUE_MAX_LEN);
                }
            }
            else
            {
                //rbc_mesh_value_disable(evt->value_handle);
            }
            break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED: break;
        case RBC_MESH_EVENT_TYPE_TX: break;
    }
}

/** @brief main function */
int main(void)
{   
    uint32_t err_code;
    
    SEGGER_RTT_Init();
    SEGGER_RTT_WriteString(0, "START\n");
    
    cmd_init(cmd_rx);

    /* Enable Softdevice (including sd_ble before framework) */
    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;
    
    clock_lf_cfg.xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM;
    
    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);
    
    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    
    // Enable BLE stack.
    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_ble_evt_handler_set(rbc_mesh_ble_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    /* Init the rbc_mesh */
    rbc_mesh_init_params_t init_params;

    init_params.access_addr     = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel         = MESH_CHANNEL;
    init_params.lfclksrc_ppm    = MESH_CLOCK_SRC_PPM;
    
    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    ble_gap_addr_t addr;
    sd_ble_gap_address_get(&addr);
    memcpy(m_data, addr.addr, 6);
    
    nrf_gpio_range_cfg_output(0, 32);
    for (uint32_t i = LED_START; i <= LED_STOP; ++i)
    {
        nrf_gpio_pin_set(i);
    }
    
    

    _LOG("START\r\n");
    print_usage();
    
    rbc_mesh_event_t evt;
    while (true)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_packet_release(evt.data);
        }
        
        sd_app_evt_wait();
    }
}

