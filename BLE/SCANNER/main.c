
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "bsp_btn_ble.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "ble_db_discovery.h"
#include "ble_bas_c.h"
#include "ble_conn_state.h"
#include "nrf_ble_gatt.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_ble_scan.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "ble_m.h"

#include "pca10056.h"


#define APP_BLE_CONN_CFG_TAG      1                                     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< BLE observer priority of the application. There is no need to modify this value. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that writes to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

#define DEV_NAME_LEN                ((BLE_GAP_ADV_SET_DATA_SIZE_MAX + 1) - \
                                    AD_DATA_OFFSET)                     /**< Determines the device name length. */

////////////
void buttons_init(void);

///////////

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */

BLE_BAS_C_ARRAY_DEF(m_bas_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< LED button client instances. */

BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */
NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

static char const m_target_periph_name[] = "Thing";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */

int16_t state;

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


/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}

/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
{
    ret_code_t  err_code;
    uint8_t   * p_adv_data;
    uint16_t    data_len;
    uint16_t    field_len;
    uint16_t    dev_name_offset = 0;
    char        dev_name[DEV_NAME_LEN];

    // Initialize advertisement report for parsing.
    p_adv_data = (uint8_t *)p_adv_report->data.p_data;
    data_len   = p_adv_report->data.len;

    // Search for advertising names.
    field_len = ble_advdata_search(p_adv_data,
                                   data_len,
                                   &dev_name_offset,
                                   BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
    if (field_len == 0)
    {
        // Look for the short local name if it was not found as complete.
        field_len = ble_advdata_search(p_adv_data,
                                       data_len,
                                       &dev_name_offset,
                                       BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME);
        if (field_len == 0)
        {
            // Exit if the data cannot be parsed.
            return;
        }
    }

    NRF_LOG_DEBUG("Found advertising device");
    //NRF_LOG_HEXDUMP_DEBUG(p_adv_data, data_len);

    memcpy(dev_name, &p_adv_data[dev_name_offset], field_len);
    dev_name[field_len] = 0;
    NRF_LOG_HEXDUMP_DEBUG(dev_name, field_len);

    //NRF_LOG_DEBUG("Found advertising device: %s", nrf_log_push((char *)dev_name));
    //NRF_LOG_DEBUG("Found advertising device: %s", (char *)dev_name);

//    // Check if the device address is the same as address taken from the NFC tag.
//    if (nfc_oob_pairing_tag_match(&p_adv_report->peer_addr))
//    {
//        // If the address is correct, stop scanning and initiate a connection with the peripheral device.
//        nrf_ble_scan_stop();
//
//        err_code = sd_ble_gap_connect(&p_adv_report->peer_addr,
//                                      &m_scan.scan_params,
//                                      &m_scan.conn_params,
//                                      APP_BLE_CONN_CFG_TAG);
//        APP_ERROR_CHECK(err_code);
//    }
}


static void scan_evt_handler(scan_evt_t const * p_scan_evt)
{
    ret_code_t err_code;

    //NRF_LOG_INFO("scan_evt_handler - event: %d", p_scan_evt->scan_evt_id);
//    NRF_BLE_SCAN_EVT_FILTER_MATCH,         /**< A filter is matched or all filters are matched in the multifilter mode. */
//    NRF_BLE_SCAN_EVT_WHITELIST_REQUEST,    /**< Request the whitelist from the main application. For whitelist scanning to work, the whitelist must be set when this event occurs. */
//    NRF_BLE_SCAN_EVT_WHITELIST_ADV_REPORT, /**< Send notification to the main application when a device from the whitelist is found. */
//    NRF_BLE_SCAN_EVT_NOT_FOUND,            /**< The filter was not matched for the scan data. */
//    NRF_BLE_SCAN_EVT_SCAN_TIMEOUT,         /**< Scan timeout. */
//    NRF_BLE_SCAN_EVT_SCAN_REQ_REPORT,      /**< Scan request report. */
//    NRF_BLE_SCAN_EVT_CONNECTING_ERROR,     /**< Error occurred when establishing the connection. In this event, an error is passed from the function call @ref sd_ble_gap_connect. */
//    NRF_BLE_SCAN_EVT_CONNECTED             /**< Connected to device. */

    switch(p_scan_evt->scan_evt_id)
    {
        case NRF_BLE_SCAN_EVT_FILTER_MATCH:
        {
            NRF_LOG_INFO("FILTER MATCH");
        }  break;

        case NRF_BLE_SCAN_EVT_CONNECTED:
        {
            NRF_LOG_INFO("CONNECTED");
            state = 1;
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
        {
            NRF_LOG_INFO("SCAN_TIMEOUT");
        } break;

        case NRF_BLE_SCAN_EVT_SCAN_REQ_REPORT:
        {
            NRF_LOG_INFO("SCAN_REQ_REPORT");
        } break;

        case NRF_BLE_SCAN_EVT_CONNECTING_ERROR:
        {
            NRF_LOG_INFO("CONNECT ERROR");
            err_code = p_scan_evt->params.connecting_err.err_code;
            APP_ERROR_CHECK(err_code);
        } break;

        default:
            break;
    }
}


/**@brief Function for initializing the scanning and setting the filters.
 */
static void scan_init(void)
{
    ret_code_t          rc;
    nrf_ble_scan_init_t init_scan;
    // testing with clion 5/27
    NRF_LOG_INFO("scan init");

    memset(&init_scan, 0, sizeof(init_scan));

    init_scan.connect_if_match = true;
    init_scan.conn_cfg_tag     = APP_BLE_CONN_CFG_TAG;

    rc = nrf_ble_scan_init(&m_scan, &init_scan, scan_evt_handler);
    APP_ERROR_CHECK(rc);

    rc = nrf_ble_scan_filter_set(&m_scan, SCAN_NAME_FILTER, m_target_periph_name);
    APP_ERROR_CHECK(rc);

    rc = nrf_ble_scan_filters_enable(&m_scan, NRF_BLE_SCAN_NAME_FILTER, false);
    APP_ERROR_CHECK(rc);
}


/**@brief Function for starting scanning. */
void scan_start_(void)
{
    ret_code_t ret;

    NRF_LOG_INFO("Start scanning for device (name): %s", m_target_periph_name);
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

    // For readability.
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    uint16_t evt = p_ble_evt->header.evt_id;
    switch (evt)
    {
        // Upon connection, check which peripheral is connected, initiate DB
        // discovery, update LEDs status, and resume scanning, if necessary.
        case BLE_GAP_EVT_CONNECTED:
        {
            uint16_t handle = p_gap_evt->conn_handle;
            NRF_LOG_INFO("Connection 0x%x established, state %x, starting DB discovery.",
                         handle, state);
            APP_ERROR_CHECK_BOOL(handle < NRF_SDH_BLE_CENTRAL_LINK_COUNT);

            // Update LEDs status and check whether it is needed to look for more
            // peripherals to connect to.
            bsp_board_led_on(CENTRAL_CONNECTED_LED);
            if (ble_conn_state_central_conn_count() == NRF_SDH_BLE_CENTRAL_LINK_COUNT)
            {
                bsp_board_led_off(CENTRAL_SCANNING_LED);
                break;
            }
            // Resume scanning if not connected
            if (state == 0) 
            {
              bsp_board_led_on(CENTRAL_SCANNING_LED);
              scan_start_();
              break;
            }

            err_code = ble_bas_c_handles_assign(&m_bas_c[handle], handle, NULL);
            APP_ERROR_CHECK(err_code);

            err_code = ble_db_discovery_start(&m_db_disc[handle], handle);
            APP_ERROR_CHECK(err_code);


        } break; // BLE_GAP_EVT_CONNECTED

        // Upon disconnection, reset the connection handle of the peer that disconnected, update
        // the LEDs status and start scanning again.
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("central link 0x%x disconnected (reason: 0x%x)",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            state = 0;

            if (ble_conn_state_central_conn_count() == 0)
            {
                err_code = app_button_disable();
                APP_ERROR_CHECK(err_code);

                // Turn off the LED that indicates the connection.
                bsp_board_led_off(CENTRAL_CONNECTED_LED);
            }

            // Start scanning.
            scan_start_();

            // Turn on the LED for indicating scanning.
            bsp_board_led_on(CENTRAL_SCANNING_LED);

        } break;

        case BLE_GAP_EVT_TIMEOUT:
        {
            // Timeout for scanning is not specified, so only the connection requests can time out.
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_DEBUG("Connection request timed out.");
            }
        } break;

//        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
//        {
//            NRF_LOG_DEBUG("BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST.");
//            // Accept parameters requested by peer.
//            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
//                                        &p_gap_evt->params.conn_param_update_request.conn_params);
//            APP_ERROR_CHECK(err_code);
//        } break;

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
        {
            // Disconnect on GATT client timeout event.
            NRF_LOG_DEBUG("GATT client timeout.");
            state = 0;
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GATTS_EVT_TIMEOUT:
        {
            // Disconnect on GATT server timeout event.
            NRF_LOG_DEBUG("GATT server timeout.");
            state = 0;
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
        } break;

        // 29 BLE_GAP_EVT_ADV_REPORT
        case BLE_GAP_EVT_ADV_REPORT:
           //NRF_LOG_DEBUG("BLE_GAP_EVT_ADV_REPORT - got advertisement");
           on_adv_report(&p_gap_evt->params.adv_report);

            break;

        // 30
        case BLE_GAP_EVT_SCAN_REQ_REPORT:
            NRF_LOG_DEBUG("BLE_GAP_EVT_SCAN_REQ_REPORT");
            break;
        // 32
        case BLE_GAP_EVT_PHY_UPDATE:
            NRF_LOG_DEBUG("BLE_GAP_EVT_PHY_UPDATE");
            break;
        // 48
        case BLE_GAP_EVT_ADV_SET_TERMINATED :
            NRF_LOG_DEBUG("BLE_GAP_EVT_ADV_SET_TERMINATED");
            break;

        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
            NRF_LOG_DEBUG("BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP");
            break;

//    BLE_GATTC_EVT_REL_DISC_RSP,                             /**< Relationship Discovery Response event.             \n See @ref ble_gattc_evt_rel_disc_rsp_t.                */
//    BLE_GATTC_EVT_CHAR_DISC_RSP,                            /**< Characteristic Discovery Response event.           \n See @ref ble_gattc_evt_char_disc_rsp_t.               */
//    BLE_GATTC_EVT_DESC_DISC_RSP,                            /**< Descriptor Discovery Response event.               \n See @ref ble_gattc_evt_desc_disc_rsp_t.               */

        case BLE_GATTC_EVT_REL_DISC_RSP:
            NRF_LOG_DEBUG("BLE_GATTC_EVT_REL_DISC_RSP");
            break;

        case BLE_GATTC_EVT_CHAR_DISC_RSP:
            NRF_LOG_DEBUG("BLE_GATTC_EVT_CHAR_DISC_RSP");
            on_characteristics_discovery_rsp(&(p_ble_evt->evt.gattc_evt));
            break;

        case BLE_GATTC_EVT_DESC_DISC_RSP:
            NRF_LOG_DEBUG("BLE_GATTC_EVT_DESC_DISC_RSP");
            break;

        case BLE_GAP_EVT_AUTH_STATUS:
            NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS");
            if (p_ble_evt->evt.gap_evt.params.auth_status.auth_status == BLE_GAP_SEC_STATUS_SUCCESS)
            {
                NRF_LOG_INFO("Authorization succeeded!");
            }
            else
            {
                NRF_LOG_INFO("Authorization failed with code: %u!",
                             p_ble_evt->evt.gap_evt.params.auth_status.auth_status);
            }
            break;

        case BLE_GAP_EVT_CONN_SEC_UPDATE:
            NRF_LOG_INFO("BLE_GAP_EVT_CONN_SEC_UPDATE");
            NRF_LOG_INFO("Security mode: %u. Security level: %u",
                         p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.sm,
                         p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv);
            break;


//        case :
//            NRF_LOG_DEBUG("");
//            break;

//        case :
//            NRF_LOG_DEBUG("");
//            break;

//        case BLE_GAP_EVT_
//            break;

        default:
            NRF_LOG_DEBUG("BLE EVENT id: %d (%x)", evt, evt);

            // No implementation needed.
            break;
    }
}


//#define BLE_SVC_BASE           0x60       /**< Common BLE SVC base. */
//#define BLE_SVC_LAST           0x6B       /**< Common BLE SVC last. */
//
//#define BLE_GAP_SVC_BASE       0x6C       /**< GAP BLE SVC base. */
//#define BLE_GAP_SVC_LAST       0x9A       /**< GAP BLE SVC last. */
//
//#define BLE_GATTC_SVC_BASE     0x9B       /**< GATTC BLE SVC base. */
//#define BLE_GATTC_SVC_LAST     0xA7       /**< GATTC BLE SVC last. */
//
//#define BLE_GATTS_SVC_BASE     0xA8       /**< GATTS BLE SVC base. */
//#define BLE_GATTS_SVC_LAST     0xB7       /**< GATTS BLE SVC last. */
//
//#define BLE_L2CAP_SVC_BASE     0xB8       /**< L2CAP BLE SVC base. */
//#define BLE_L2CAP_SVC_LAST     0xBF       /**< L2CAP BLE SVC last. */
//
//
//#define BLE_EVT_INVALID        0x00       /**< Invalid BLE Event. */
//
//#define BLE_EVT_BASE           0x01       /**< Common BLE Event base. */
//#define BLE_EVT_LAST           0x0F       /**< Common BLE Event last. */
//
//#define BLE_GAP_EVT_BASE       0x10       /**< GAP BLE Event base. */
//#define BLE_GAP_EVT_LAST       0x2F       /**< GAP BLE Event last. */
//
//#define BLE_GATTC_EVT_BASE     0x30       /**< GATTC BLE Event base. */
//#define BLE_GATTC_EVT_LAST     0x4F       /**< GATTC BLE Event last. */
//
//#define BLE_GATTS_EVT_BASE     0x50       /**< GATTS BLE Event base. */
//#define BLE_GATTS_EVT_LAST     0x6F       /**< GATTS BLE Event last. */
//
//#define BLE_L2CAP_EVT_BASE     0x70       /**< L2CAP BLE Event base. */
//#define BLE_L2CAP_EVT_LAST     0x8F       /**< L2CAP BLE Event last. */
//
//
//#define BLE_OPT_INVALID        0x00       /**< Invalid BLE Option. */
//
//#define BLE_OPT_BASE           0x01       /**< Common BLE Option base. */
//#define BLE_OPT_LAST           0x1F       /**< Common BLE Option last. */
//
//#define BLE_GAP_OPT_BASE       0x20       /**< GAP BLE Option base. */
//#define BLE_GAP_OPT_LAST       0x3F       /**< GAP BLE Option last. */
//
//#define BLE_GATT_OPT_BASE      0x40       /**< GATT BLE Option base. */
//#define BLE_GATT_OPT_LAST      0x5F       /**< GATT BLE Option last. */
//
//#define BLE_GATTC_OPT_BASE     0x60       /**< GATTC BLE Option base. */
//#define BLE_GATTC_OPT_LAST     0x7F       /**< GATTC BLE Option last. */
//
//#define BLE_GATTS_OPT_BASE     0x80       /**< GATTS BLE Option base. */
//#define BLE_GATTS_OPT_LAST     0x9F       /**< GATTS BLE Option last. */
//
//#define BLE_L2CAP_OPT_BASE     0xA0       /**< L2CAP BLE Option base. */
//#define BLE_L2CAP_OPT_LAST     0xBF       /**< L2CAP BLE Option last. */
//
//
//#define BLE_CFG_INVALID        0x00       /**< Invalid BLE configuration. */
//
//#define BLE_CFG_BASE           0x01       /**< Common BLE configuration base. */
//#define BLE_CFG_LAST           0x1F       /**< Common BLE configuration last. */
//
//#define BLE_CONN_CFG_BASE      0x20       /**< BLE connection configuration base. */
//#define BLE_CONN_CFG_LAST      0x3F       /**< BLE connection configuration last. */
//
//#define BLE_GAP_CFG_BASE       0x40       /**< GAP BLE configuration base. */
//#define BLE_GAP_CFG_LAST       0x5F       /**< GAP BLE configuration last. */
//
//#define BLE_GATT_CFG_BASE      0x60       /**< GATT BLE configuration base. */
//#define BLE_GATT_CFG_LAST      0x7F       /**< GATT BLE configuration last. */
//
//#define BLE_GATTC_CFG_BASE     0x80       /**< GATTC BLE configuration base. */
//#define BLE_GATTC_CFG_LAST     0x9F       /**< GATTC BLE configuration last. */
//
//#define BLE_GATTS_CFG_BASE     0xA0       /**< GATTS BLE configuration base. */
//#define BLE_GATTS_CFG_LAST     0xBF       /**< GATTS BLE configuration last. */
//
//#define BLE_L2CAP_CFG_BASE     0xC0       /**< L2CAP BLE configuration base. */
//#define BLE_L2CAP_CFG_LAST     0xDF       /**< L2CAP BLE configuration last. */



// base is 0x10

//  BLE_GAP_EVT_CONNECTED                   = BLE_GAP_EVT_BASE,       /**< Connected to peer.                              \n See @ref ble_gap_evt_connected_t             */
//  BLE_GAP_EVT_DISCONNECTED                = BLE_GAP_EVT_BASE + 1,   /**< Disconnected from peer.                         \n See @ref ble_gap_evt_disconnected_t.         */
//  BLE_GAP_EVT_CONN_PARAM_UPDATE           = BLE_GAP_EVT_BASE + 2,   /**< Connection Parameters updated.                  \n See @ref ble_gap_evt_conn_param_update_t.    */
//  BLE_GAP_EVT_SEC_PARAMS_REQUEST          = BLE_GAP_EVT_BASE + 3,   /**< Request to provide security parameters.         \n Reply with @ref sd_ble_gap_sec_params_reply.  \n See @ref ble_gap_evt_sec_params_request_t. */
//  BLE_GAP_EVT_SEC_INFO_REQUEST            = BLE_GAP_EVT_BASE + 4,   /**< Request to provide security information.        \n Reply with @ref sd_ble_gap_sec_info_reply.    \n See @ref ble_gap_evt_sec_info_request_t.   */
//  BLE_GAP_EVT_PASSKEY_DISPLAY             = BLE_GAP_EVT_BASE + 5,   /**< Request to display a passkey to the user.       \n In LESC Numeric Comparison, reply with @ref sd_ble_gap_auth_key_reply. \n See @ref ble_gap_evt_passkey_display_t. */
//  BLE_GAP_EVT_KEY_PRESSED                 = BLE_GAP_EVT_BASE + 6,   /**< Notification of a keypress on the remote device.\n See @ref ble_gap_evt_key_pressed_t           */
//  BLE_GAP_EVT_AUTH_KEY_REQUEST            = BLE_GAP_EVT_BASE + 7,   /**< Request to provide an authentication key.       \n Reply with @ref sd_ble_gap_auth_key_reply.    \n See @ref ble_gap_evt_auth_key_request_t.   */
//  BLE_GAP_EVT_LESC_DHKEY_REQUEST          = BLE_GAP_EVT_BASE + 8,   /**< Request to calculate an LE Secure Connections DHKey. \n Reply with @ref sd_ble_gap_lesc_dhkey_reply.  \n See @ref ble_gap_evt_lesc_dhkey_request_t */
//  BLE_GAP_EVT_AUTH_STATUS                 = BLE_GAP_EVT_BASE + 9,   /**< Authentication procedure completed with status. \n See @ref ble_gap_evt_auth_status_t.          */
//  BLE_GAP_EVT_CONN_SEC_UPDATE             = BLE_GAP_EVT_BASE + 10,  /**< Connection security updated.                    \n See @ref ble_gap_evt_conn_sec_update_t.      */
//  BLE_GAP_EVT_TIMEOUT                     = BLE_GAP_EVT_BASE + 11,  /**< Timeout expired.                                \n See @ref ble_gap_evt_timeout_t.              */
//  BLE_GAP_EVT_RSSI_CHANGED                = BLE_GAP_EVT_BASE + 12,  /**< RSSI report.                                    \n See @ref ble_gap_evt_rssi_changed_t.         */
//  BLE_GAP_EVT_ADV_REPORT                  = BLE_GAP_EVT_BASE + 13,  /**< Advertising report.                             \n See @ref ble_gap_evt_adv_report_t.           */
//  BLE_GAP_EVT_SEC_REQUEST                 = BLE_GAP_EVT_BASE + 14,  /**< Security Request.                               \n See @ref ble_gap_evt_sec_request_t.          */
//  BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST   = BLE_GAP_EVT_BASE + 15,  /**< Connection Parameter Update Request.            \n Reply with @ref sd_ble_gap_conn_param_update. \n See @ref ble_gap_evt_conn_param_update_request_t. */
//  BLE_GAP_EVT_SCAN_REQ_REPORT             = BLE_GAP_EVT_BASE + 16,  /**< Scan request report.                            \n See @ref ble_gap_evt_scan_req_report_t. */
//  BLE_GAP_EVT_PHY_UPDATE_REQUEST          = BLE_GAP_EVT_BASE + 17,  /**< PHY Update Request.                             \n Reply with @ref sd_ble_gap_phy_update. \n See @ref ble_gap_evt_phy_update_request_t. */
//  BLE_GAP_EVT_PHY_UPDATE                  = BLE_GAP_EVT_BASE + 18,  /**< PHY Update Procedure is complete.               \n See @ref ble_gap_evt_phy_update_t.           */
//  BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST = BLE_GAP_EVT_BASE + 19,   /**< Data Length Update Request.                     \n Reply with @ref sd_ble_gap_data_length_update. \n See @ref ble_gap_evt_data_length_update_request_t. */
//  BLE_GAP_EVT_DATA_LENGTH_UPDATE         = BLE_GAP_EVT_BASE + 20,   /**< LL Data Channel PDU payload length updated.     \n See @ref ble_gap_evt_data_length_update_t. */
//  BLE_GAP_EVT_QOS_CHANNEL_SURVEY_REPORT  = BLE_GAP_EVT_BASE + 21,   /**< Channel survey report.                          \n See @ref ble_gap_evt_qos_channel_survey_report_t. */
//  BLE_GAP_EVT_ADV_SET_TERMINATED         = BLE_GAP_EVT_BASE + 22,   /**< Advertising set terminated.                     \n See @ref ble_gap_evt_adv_set_terminated_t. */
// 38

///**
// * @brief GATT Client Event IDs.
// base  0x30
// */
//enum BLE_GATTC_EVTS
//{
//    BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP = BLE_GATTC_EVT_BASE,  /**< Primary Service Discovery Response event.          \n See @ref ble_gattc_evt_prim_srvc_disc_rsp_t.          */
//    BLE_GATTC_EVT_REL_DISC_RSP,                             /**< Relationship Discovery Response event.             \n See @ref ble_gattc_evt_rel_disc_rsp_t.                */
//    BLE_GATTC_EVT_CHAR_DISC_RSP,                            /**< Characteristic Discovery Response event.           \n See @ref ble_gattc_evt_char_disc_rsp_t.               */
//    BLE_GATTC_EVT_DESC_DISC_RSP,                            /**< Descriptor Discovery Response event.               \n See @ref ble_gattc_evt_desc_disc_rsp_t.               */
//    BLE_GATTC_EVT_ATTR_INFO_DISC_RSP,                       /**< Attribute Information Response event.              \n See @ref ble_gattc_evt_attr_info_disc_rsp_t. */
//    BLE_GATTC_EVT_CHAR_VAL_BY_UUID_READ_RSP,                /**< Read By UUID Response event.                       \n See @ref ble_gattc_evt_char_val_by_uuid_read_rsp_t.   */
//    BLE_GATTC_EVT_READ_RSP,                                 /**< Read Response event.                               \n See @ref ble_gattc_evt_read_rsp_t.                    */
//    BLE_GATTC_EVT_CHAR_VALS_READ_RSP,                       /**< Read multiple Response event.                      \n See @ref ble_gattc_evt_char_vals_read_rsp_t.          */
//    BLE_GATTC_EVT_WRITE_RSP,                                /**< Write Response event.                              \n See @ref ble_gattc_evt_write_rsp_t.                   */
//    BLE_GATTC_EVT_HVX,                                      /**< Handle Value Notification or Indication event.     \n Confirm indication with @ref sd_ble_gattc_hv_confirm.  \n See @ref ble_gattc_evt_hvx_t. */
//    BLE_GATTC_EVT_EXCHANGE_MTU_RSP,                         /**< Exchange MTU Response event.                       \n See @ref ble_gattc_evt_exchange_mtu_rsp_t.            */
//    BLE_GATTC_EVT_TIMEOUT,                                  /**< Timeout event.                                     \n See @ref ble_gattc_evt_timeout_t.                     */
//    BLE_GATTC_EVT_WRITE_CMD_TX_COMPLETE                     /**< Write without Response transmission complete.      \n See @ref ble_gattc_evt_write_cmd_tx_complete_t.       */
//};




/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupts.
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



/**@brief Handles events coming from the LED Button central module.
 *
 * @param[in] p_bas_c     The instance of bas_c that triggered the event.
 * @param[in] p_bas_c_evt The bas_c event.
 */
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
    switch (p_bas_c_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
        {
            ret_code_t err_code;

            NRF_LOG_INFO("BAS Service discovered on conn_handle 0x%x",
                         p_bas_c_evt->conn_handle);

            err_code = app_button_enable();
            APP_ERROR_CHECK(err_code);

            // Service discovered. Enable notification of Button.
            //err_code = ble_bas_c_button_notif_enable(p_bas_c);
            //APP_ERROR_CHECK(err_code);
        } break; // BLE_bas_c_EVT_DISCOVERY_COMPLETE

//        case BLE_ba_c_EVT_BUTTON_NOTIFICATION:
//        {
//            NRF_LOG_INFO("Link 0x%x, Button state changed on peer to 0x%x",
//                         p_bas_c_evt->conn_handle,
//                         p_bas_c_evt->params.button.button_state);
//
//            if (p_bas_c_evt->params.button.button_state)
//            {
//                bsp_board_led_on(LEDBUTTON_LED);
//            }
//            else
//            {
//                bsp_board_led_off(LEDBUTTON_LED);
//            }
//        } break; // BLE_bas_c_EVT_BUTTON_NOTIFICATION

        default:
            // No implementation needed.
            break;
    }
}

static void bas_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Batt.service (BAS) collector initialization. */
static void bas_c_init(void)
{
    ret_code_t       err_code;
    ble_bas_c_init_t bas_c_init_obj;

    bas_c_init_obj.evt_handler   = bas_c_evt_handler;
    bas_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;
    bas_c_init_obj.error_handler = bas_error_handler;

    for (uint32_t i = 0; i < NRF_SDH_BLE_CENTRAL_LINK_COUNT; i++)
    {
        err_code = ble_bas_c_init(&m_bas_c[i], &bas_c_init_obj);
        APP_ERROR_CHECK(err_code);
    }
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


/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling database discovery events.
 *
 * @details This function is a callback function to handle events from the database discovery module.
 *          Depending on the UUIDs that are discovered, this function forwards the events
 *          to their respective services.
 *
 * @param[in] p_event  Pointer to the database discovery event.
 */
static void db_disc_handler(ble_db_discovery_evt_t * p_evt)
{
    NRF_LOG_INFO("db_disc_handler for instance %d and link 0x%x!",
                  p_evt->conn_handle,
                  p_evt->conn_handle);

    //ble_bas_on_db_disc_evt(&m_bas_c[p_evt->conn_handle], p_evt);
}


/** @brief Database discovery initialization.
 */
static void db_discovery_init(void)
{
    ble_db_discovery_init_t db_init;

    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));

    db_init.evt_handler  = db_disc_handler;
    db_init.p_gatt_queue = &m_ble_gatt_queue;

    ret_code_t rc = ble_db_discovery_init(&db_init);
    NRF_LOG_INFO("ble_db_discovery_init rc %d", rc);

    APP_ERROR_CHECK(rc);
}


int main(void)
{
    log_init();
    timer_init();
    leds_init();
    buttons_init();
    power_management_init();
    ble_stack_init();
    gatt_init();
    db_discovery_init();
    bas_c_init();
    ble_conn_state_init();
    scan_init();

    // Start execution.
    NRF_LOG_INFO("Scanner v.%s0.5.27.0 starting.");
    scan_start_();

    for (;;)
    {
        idle_state_handle();
    }
}
