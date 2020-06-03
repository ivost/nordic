#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nordic_common.h"

#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//#include "boards.h"
#include "pca10056.h"

#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that writes to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */


/**@brief Function for handling events from the button handler module.
 *
 * @param[in] pin_no        The pin that the event applies to.
 * @param[in] button_action The button action (press or release).
 */
static void button_event_handler(uint8_t pin_no, uint8_t button_action)
{
    ret_code_t err_code;

    NRF_LOG_INFO("=== button_event_handler Pin %d, action %d", pin_no, button_action);

//    switch (pin_no)
//    {
//        case LEDBUTTON_BUTTON:
//            err_code = led_status_send_to_all(button_action);
//            if (err_code == NRF_SUCCESS)
//            {
//                NRF_LOG_INFO("Pin %d, action %d", pin_no, button_action);
//            }
//            break;
//
//        default:
//            APP_ERROR_HANDLER(pin_no);
//            break;
//    }

}


/**@brief Function for initializing the button handler module.
 */
void buttons_init(void)
{
    ret_code_t err_code;

   // The array must be static because a pointer to it is saved in the 
   // button handler module.
    static app_button_cfg_t buttons[] =
    {
        {LEDBUTTON_BUTTON, false, BUTTON_PULL, button_event_handler}
    };

    err_code = app_button_init(buttons, ARRAY_SIZE(buttons), BUTTON_DETECTION_DELAY);
    APP_ERROR_CHECK(err_code);
}
