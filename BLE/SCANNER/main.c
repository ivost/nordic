
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
//#include "nrf_sdh.h"
//#include "nrf_sdh_ble.h"
#include "app_timer.h"
//#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_bas_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
//#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_m.h"

#include "pca10056.h"
//
////////////
void ble_m_init(void);
void scan_start(void);
//void buttons_init(void);
//void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
///////////

/**@brief Function for handling asserts in the SoftDevice.
 *
 * @details This function is called in case of an assert in the SoftDevice.
 *
 * @warning This handler is only an example and is not meant for the final product. You need to analyze
 *          how your product is supposed to react in case of an assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num     Line number of the failing assert call.
 * @param[in] p_file_name  File name of the failing assert call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(0xDEADBEEF, line_num, p_file_name);
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
 * @details This function handles any pending log operations, then sleeps until the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/** @brief Function for initializing the log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/** @brief Function for initializing the timer.
 */
static void timer_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


int main(void)
{
    log_init();
    timer_init();
    power_management_init();

    ble_m_init();

    // Start execution.
    NRF_LOG_INFO("Scanner v.0.5.29.0 starting.");
    scan_start();

    for (;;)
    {
        idle_state_handle();
    }
}
