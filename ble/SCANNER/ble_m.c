#include "ble_m.h"
#include "nordic_common.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "peer_manager.h"
//#include "fds.h"
//#include "nrf_fstorage.h"
#include "ble_db_discovery.h"
//#include "ble_hrs_c.h"
#include "ble_bas_c.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_scan.h"
#include "bsp_btn_ble.h"

#include "pca10056.h"


#define NRF_LOG_MODULE_NAME BLE_M
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#define APP_BLE_CONN_CFG_TAG        1                                   /**< Tag for the configuration of the BLE stack. */
#define APP_BLE_OBSERVER_PRIO       3                                   /**< BLE observer priority of the application. There is no need to modify this value. */
#define APP_SOC_OBSERVER_PRIO       1                                   /**< SoC observer priority of the application. There is no need to modify this value. */

#define DEV_NAME_LEN                ((BLE_GAP_ADV_SET_DATA_SIZE_MAX + 1) - \
                                    AD_DATA_OFFSET)                     /**< Determines the device name length. */


#define CCCD_DESCRIPTOR_UUID           BLE_UUID_DESCRIPTOR_CLIENT_CHAR_CONFIG /**< UUID of characteristic CCCD descriptors. */
#define L2CAP_HDR_LEN                  4                                      /**< Length of a L2CAP header, in bytes. */
#define BATTERY_INITIAL_LVL            100                                    /**< Battery initial level. */
#define UUID_STRING_LEN                5                                      /**< UUID uint16_t string length. */
#define UUID_16_POS                    12                                     /**< Position of 2 - bytes which identifies 16 UUID inside of 128 - bit UUID. */
#define UUID_128_OFFSET                3                                      /**< Offset of 128 - bit UUID in declarative handle read response data. */
#define APP_BLE_CONN_CFG_TAG      1                                     /**< Tag that refers to the BLE stack configuration that is set with @ref sd_ble_cfg_set. The default tag is @ref APP_BLE_CONN_CFG_TAG. */
#define APP_BLE_OBSERVER_PRIO     3                                     /**< BLE observer priority of the application. There is no need to modify this value. */

#define CENTRAL_SCANNING_LED      BSP_BOARD_LED_0
#define CENTRAL_CONNECTED_LED     BSP_BOARD_LED_1
#define LEDBUTTON_LED             BSP_BOARD_LED_2                       /**< LED to indicate a change of state of the Button characteristic on the peer. */

#define LEDBUTTON_BUTTON          BSP_BUTTON_0                          /**< Button that writes to the LED characteristic of the peer. */
#define BUTTON_DETECTION_DELAY    APP_TIMER_TICKS(50)                   /**< Delay from a GPIOTE event until a button is reported as pushed (in number of timer ticks). */

// Structure storing data of all discovered services.
typedef struct
{
    ble_gattc_service_t services[MAX_SERVICE_COUNT]; /**< Data of the services found. */
    uint8_t             count;                       /**< Count of the services found. */
} device_srv_t;


typedef struct
{
    ble_uuid_t            uuid;              /**< UUID of the characteristic. */
    uint16_t              decl_handle;       /**< Handle of the characteristic declaration. */
    uint16_t              value_handle;      /**< Handle of the characteristic value. */
    uint16_t              cccd_desc_handle;  /**< Handle of the CCCD descriptors. */
    ble_gatt_char_props_t char_props;        /**< GATT Characteristic Properties. */
} char_data_t;

// Structure storing the data of all discovered characteristics.
typedef struct
{
    char_data_t char_data[MAX_CHARACTERISTIC_COUNT]; /**< Characteristics data. */
    uint8_t     count;                               /**< Characteristics count. */
} srv_char_t;

scanned_device_t   m_device[DEVICE_TO_FIND_MAX];                 /**< Stores device info from scan data. */
char               m_addr_str_for_connection[ADDR_STRING_LEN];   /**< Stores device address as string for establishing a connection. */
srv_char_t         m_srv_char;                                   /**< Stores all the characteristics data from one service to allow further operations. */
uint16_t           m_desc_handle;                                /**< The CCCD descriptor handle found. */
device_srv_t     * mp_device_srv[NRF_BLE_LINK_COUNT];            /**< Pointers to the allocated memory needed to discover the services on the server. */
static bool        m_numeric_match_requested = false;            /**< Numeric match request. */
static uint16_t    m_num_comp_conn_handle;                       /**< Numeric comparison connection handle. */
//static conn_peer_t m_connected_peers[NRF_BLE_LINK_COUNT];        /**< Connected devices data. */
bool               m_scanning = false;                           /**< Variable that informs about the ongoing scanning. True when scan is ON. */
bool               m_vendor_uuid_read = false;                   /**< Variable that informs about the read request for a 128-bit service UUID. */
bool               m_vendor_char_uuid_read = false;              /**< Variable that informs about the read request for a 128-bit characteristic UUID. */
uint8_t            m_uuid_attr_handle;

NRF_BALLOC_DEF(m_srv_pool, sizeof(device_srv_t), NRF_BLE_LINK_COUNT);

NRF_BLE_GATT_DEF(m_gatt);                                               /**< GATT module instance. */
BLE_DB_DISCOVERY_DEF(m_db_discovery);                                   /**< Database Discovery module instance. */
NRF_BLE_SCAN_DEF(m_scan);                                               /**< Scanning Module instance. */

static bool     m_is_connected              = false;                    /**< Flag to keep track of the BLE connections with peripheral devices. */
static uint16_t m_conn_handle               = BLE_CONN_HANDLE_INVALID;  /**< Current connection handle. */
static bool     m_memory_access_in_progress = false;                    /**< Flag to keep track of the ongoing operations on persistent memory. */
static bool     m_hrs_notif_enabled         = false;                    /**< Flag indicating that HRS notification has been enabled. */


BLE_BAS_C_ARRAY_DEF(m_bas_c, NRF_SDH_BLE_CENTRAL_LINK_COUNT);           /**< client instances. */

BLE_DB_DISCOVERY_ARRAY_DEF(m_db_disc, NRF_SDH_BLE_CENTRAL_LINK_COUNT);  /**< Database discovery module instances. */

NRF_BLE_GQ_DEF(m_ble_gatt_queue,                                        /**< BLE GATT Queue instance. */
               NRF_SDH_BLE_CENTRAL_LINK_COUNT,
               NRF_BLE_GQ_QUEUE_SIZE);

//static char const m_target_periph_name[] = "SensorTag";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */
static char const m_target_periph_name[] = "Thing";             /**< Name of the device to try to connect to. This name is searched for in the scanning report data. */

/**@brief Function for handling the Heart Rate Service Client and Battery Service Client errors.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void service_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}



bool ble_is_connected(void)
{
    return m_is_connected;
}


uint16_t ble_get_conn_handle(void)
{
    return m_conn_handle;
}


void ble_disconnect(void)
{
    ret_code_t err_code;

    if (m_is_connected)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        if (err_code != NRF_ERROR_INVALID_STATE)
        {
            APP_ERROR_CHECK(err_code);
        }
    }
}


char const * phy_str(ble_gap_phys_t const * const p_phys)
{
    ASSERT(p_phys);

    static char const * p_str[] =
    {
        "1 Mbps",
        "2 Mbps",
        "Coded",
        "Unknown"
    };

    switch (p_phys->tx_phys)
    {
        case BLE_GAP_PHY_1MBPS:
            return p_str[0];

        case BLE_GAP_PHY_2MBPS:
        case (BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS):
        case (BLE_GAP_PHY_2MBPS | BLE_GAP_PHY_1MBPS | BLE_GAP_PHY_CODED):
            return p_str[1];

        case BLE_GAP_PHY_CODED:
            return p_str[2];

        default:
            return p_str[3];
    }
}


void int_addr_to_hex_str(char * p_result, uint8_t result_len, uint8_t const * const p_addr)
{
    ASSERT(p_result);
    ASSERT(p_addr);

    if (result_len > BLE_GAP_ADDR_LEN)
    {
        return;
    }

    char buffer[BLE_GAP_ADDR_LEN] = {0};

    memset(p_result, 0, result_len);

    for (uint8_t i = 0; i < result_len; ++i)
    {
        sprintf(buffer, "%.2X", p_addr[result_len - (i + 1)]);
        strcat(p_result, buffer);

        if (i < (result_len - 1))
        {
            strcat(p_result, ":");
        }
    }
}


/**@brief Function for printing the UUID for each service.
 *
 * @param[in] conn_handle    The connection handle identifying the connection to perform this procedure on.
 * @param[in] service_p      Pointer to ble_gattc_service_t.
 */
static void uuid_print(uint16_t conn_handle, ble_gattc_service_t const * p_service)
{
    NRF_LOG_INFO("Found service UUIDs: \r\n");

    for (uint8_t i = 0; i < mp_device_srv[conn_handle]->count; i++)
    {
        NRF_LOG_INFO("%s: %X %s: 0x%X\r\n", 
                         "UUID", 
                         p_service[i].uuid.uuid,
                         "type", 
                         p_service[i].uuid.type);
    }
}

static void characteristics_print(void)
{
    for (uint8_t i = 0; i < m_srv_char.count; i++)
    {
           ble_gatt_char_props_t const * p_char_props = 
                                 &m_srv_char.char_data[i].char_props;
           NRF_LOG_INFO("Characteristic UUID: %X\r\n",
                            m_srv_char.char_data[i].uuid.uuid);
           NRF_LOG_INFO("Parameters:\r\n");
           NRF_LOG_INFO("broadcast: %d ", p_char_props->broadcast);
           NRF_LOG_INFO("read: %d ", p_char_props->read);
           NRF_LOG_INFO("write_wo_resp: %d ", p_char_props->write_wo_resp);
           NRF_LOG_INFO("write: %d ", p_char_props->write);
           NRF_LOG_INFO("notify: %d\r\n", p_char_props->notify);
           NRF_LOG_INFO("indicate: %d ", p_char_props->indicate);
           NRF_LOG_INFO("auth_signed_wr: %d\r\n", p_char_props->auth_signed_wr);
    }

    NRF_LOG_INFO("Number of characteristics: %d\r\n", m_srv_char.count);

}


bool is_address_compare(ble_gap_addr_t const * const p_connected_addr, char const * const p_addr)
{
    ASSERT(p_connected_addr);
    ASSERT(p_addr);

    char string_addr_buf[ADDR_STRING_LEN] = {0};

    int_addr_to_hex_str(string_addr_buf,
                        BLE_GAP_ADDR_LEN,
                        p_connected_addr->addr);

    return (memcmp(string_addr_buf, p_addr, sizeof(string_addr_buf))) ? false : true;
}


//uint16_t addr_string_to_conn_handle(char const * const p_addr)
//{
//    ASSERT(p_addr);
//
//    uint16_t conn_handle = 0;
//    uint8_t  idx;
//
//    for (idx = 0; idx < NRF_BLE_LINK_COUNT; idx++)
//    {
//        if (is_address_compare(&m_connected_peers[idx].address, p_addr))
//        {
//            conn_handle = idx;
//            return conn_handle;
//        }
//    }
//
//    return BLE_CONN_HANDLE_INVALID;
//}


/**@brief Function for handling primary service discovery response.
 *
 * @details This function will handle the primary service discovery response.
 *
 * @param[in] p_ble_gattc_evt   Pointer to the GATT Client event.
 */
static void on_primary_srv_discovery_rsp(ble_gattc_evt_t const * p_ble_gattc_evt)
{
    uint16_t        count;
    uint16_t        bytes_to_copy;
    uint16_t        conn_handle;
    static uint16_t offset = 0;
    ret_code_t      err_code;

    // For readability.
    ble_gattc_evt_prim_srvc_disc_rsp_t const * p_prim_serv =
        &(p_ble_gattc_evt->params.prim_srvc_disc_rsp);

    ASSERT(p_prim_serv != NULL);

    // Number of discovered services.
    count = p_prim_serv->count;

    conn_handle = p_ble_gattc_evt->conn_handle;

    ble_gattc_service_t * const p_service = mp_device_srv[conn_handle]->services;

    NRF_LOG_INFO("---  conn_handle: %x, count: %d, gatt status: %x, p_service: %x",  conn_handle, count, p_ble_gattc_evt->gatt_status, p_service);
    ASSERT(p_service != NULL);

    // If no more services are found.
    if ((count != 0) && 
        (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS))
    {
        if ((count + offset) > MAX_SERVICE_COUNT)
        {
            bytes_to_copy = MAX_SERVICE_COUNT - offset;
        }
        else
        {
            bytes_to_copy = count;
        }

        // Save services data.
        memcpy((p_service + offset),
               p_prim_serv->services,
               bytes_to_copy * sizeof(ble_gattc_service_t));

        offset += count;
        uint16_t start = p_prim_serv->services[count - 1].handle_range.end_handle + 1;
        NRF_LOG_INFO("--- sd_ble_gattc_primary_services_discover: %x", start);
        // If the last service has not been reached, this function must be called again with a new start handle.
        err_code = sd_ble_gattc_primary_services_discover(
            conn_handle,
            start,     
            NULL);
        APP_ERROR_CHECK(err_code);
    }
    else
    {
        mp_device_srv[conn_handle]->count = offset;

        // If service UUID type is unknown, then look for a 128-bit UUID.
        // Only the first one is searched for here, the rest is searched for in the @ref on_read_rsp.
        for (uint8_t i = 0; i < offset; i++)
        {
            if (p_service[i].uuid.type == BLE_UUID_TYPE_UNKNOWN)
            {
                m_vendor_uuid_read = true;
                // Read service 128-bit UUID.
                err_code = sd_ble_gattc_read(conn_handle, p_service[i].handle_range.start_handle, 0);
                APP_ERROR_CHECK(err_code);
                offset = 0;

                return;
            }
        }

        NRF_LOG_INFO("Services count: %d", offset);
        uuid_print(p_ble_gattc_evt->conn_handle, p_service);

        offset = 0;
    }
}


/**@brief Function for handling the advertising report BLE event.
 *
 * @param[in] p_adv_report  Advertising report from the SoftDevice.
 */
static void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
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

    memcpy(dev_name, &p_adv_data[dev_name_offset], field_len);
    dev_name[field_len] = 0;

    if (dev_name[0] > ' ') 
    {
      NRF_LOG_DEBUG("Found advertising device: %s", nrf_log_push((char *)dev_name));
    }
}

/**@brief Function for displaying an address in HEX format.
 */
static void hex_addr_display(uint8_t const * p_addr, size_t size)
{
    NRF_LOG_RAW_INFO("Connected to address: ");

    for (uint8_t i = 0; i < size; ++i)
    {
        NRF_LOG_RAW_INFO("%.2X ", p_addr[size - (i + 1)]);
    }

    NRF_LOG_RAW_INFO("\r\n");
}


/**@brief Function for searching for the CCCD of a characteristic.
 *
 * @param[in] char_uuid       UUID of the characteristic.
 * @param[in] conn_handle     The connection handle that identifies the connection to perform this procedure on.
 */

static void cccd_descriptors_search(uint16_t char_uuid, uint16_t conn_handle)
{
    uint8_t                  i;
    ret_code_t               err_code;
    ble_gattc_handle_range_t handle_range;
    uint16_t                 start_handle = 0;
    uint16_t                 end_handle   = 0;

 
    NRF_LOG_INFO("=== cccd_descriptors_search, uuid %x", char_uuid);

    // Searching for the handle of the initial characteristic.
    for (i = 0; i < m_srv_char.count; i++)
    {
        if (m_srv_char.char_data[i].uuid.uuid == char_uuid)
        {
            start_handle = m_srv_char.char_data[i].value_handle + 1;
            break;
        }
    }

    // Searching for the handle of the final characteristic if the characteristic is not the last one in the service.
    if ((i < (m_srv_char.count - 1)) && (m_srv_char.char_data[i + 1].uuid.uuid != 0))
    {
        // If the characteristic is not the last one, start a handle equal to the declaration handle of the next characteristic's declaration handle.
        end_handle = m_srv_char.char_data[i + 1].decl_handle - 1;
    }
    // Searching for the handle of the final characteristic. If the characteristic is the last one in the service, the end handle is equal to the service end handle.
    else
    {
        for (uint8_t j = 0; j < mp_device_srv[conn_handle]->count; j++)
        {
            if ((start_handle >
                 mp_device_srv[conn_handle]->services[j].handle_range.start_handle) &&
                (start_handle <= mp_device_srv[conn_handle]->services[j].handle_range.end_handle))
            {
                end_handle = mp_device_srv[conn_handle]->services[j].handle_range.end_handle;
                break;
            }
        }
    }

    NRF_LOG_INFO("=== start_handle %x, end_handle %x", start_handle, end_handle);

    handle_range.start_handle = start_handle;
    handle_range.end_handle   = end_handle;

    // Discovery descriptors inside, the found handle range.
    err_code = sd_ble_gattc_descriptors_discover(conn_handle, &handle_range);
    APP_ERROR_CHECK(err_code);
}


uint16_t handle_value_search(char const * const p_service_uuid_str)
{
    ASSERT(p_service_uuid_str);

    char buffer[UUID_STRING_LEN] = {0};

    for (uint8_t i = 0; i < m_srv_char.count; i++)
    {
        sprintf(buffer, "%X", m_srv_char.char_data[i].uuid.uuid);

        if (!strcmp(buffer, p_service_uuid_str))
        {
            return m_srv_char.char_data[i].value_handle;
        }
    }

    return BLE_GATT_HANDLE_INVALID;
}


ble_gattc_handle_range_t * handle_range_search(char const * const p_service_uuid_str,
                                               uint16_t           conn_handle)
{
    ASSERT(p_service_uuid_str);

    char buffer[UUID_STRING_LEN] = {0};

    for (uint8_t i = 0; i < mp_device_srv[conn_handle]->count; i++)
    {

        sprintf(buffer, "%X", mp_device_srv[conn_handle]->services[i].uuid.uuid);
        if (!strcmp(buffer, p_service_uuid_str))
        {
            return &mp_device_srv[conn_handle]->services[i].handle_range;
        }
    }

    return NULL;
}


/**@brief Function for starting a discovery of CCCD descriptors.
 *
 * @details If characteristics can be notified, then look for CCCD descriptors in all 
 *          characteristics inside the service.
 *
 * @param[in] p_ble_gattc_evt 
 */
static void cccd_descriptors_discovery(ble_gattc_evt_t const * p_ble_gattc_evt)
{
    for (uint8_t i = 0; i < m_srv_char.count; i++)
    {
        // If it is possible to enable notification.
        if ((m_srv_char.char_data[i].char_props.notify ||
             m_srv_char.char_data[i].char_props.indicate) &&
            (m_srv_char.char_data[i].cccd_desc_handle == 0))
        {
            // Search for CCCD descriptor handle
            cccd_descriptors_search(m_srv_char.char_data[i].uuid.uuid, p_ble_gattc_evt->conn_handle);

            break;
        }
    }
}


/**@brief Function for handling a characteristic discovery response.
 *
 * @param[in] p_ble_gattc_evt   Pointer to the GATT Client event.
 */
void on_characteristics_discovery_rsp(ble_gattc_evt_t const * p_ble_gattc_evt)
{
    uint16_t        count;
    static uint16_t offset = 0;
    uint16_t        bytes_to_copy;
    ret_code_t      err_code;
    uint16_t        conn_handle = p_ble_gattc_evt->conn_handle;

    // For readability.
    count = p_ble_gattc_evt->params.char_disc_rsp.count;

    NRF_LOG_INFO("*** on_characteristics_discovery_rsp status: %x, count %d", p_ble_gattc_evt->gatt_status, count);

    if (p_ble_gattc_evt->gatt_status != BLE_GATT_STATUS_SUCCESS) 
    {
      return;
    }   

    if (p_ble_gattc_evt->gatt_status == BLE_GATT_STATUS_SUCCESS)
    {
        if ((count + offset) > MAX_CHARACTERISTIC_COUNT)
        {
            bytes_to_copy = MAX_CHARACTERISTIC_COUNT - offset;
            NRF_LOG_INFO("Too many characteristics discovered\r\n");
        }
        else
        {
            bytes_to_copy = count;
        }

        ble_gattc_evt_char_disc_rsp_t const * p_char_disc_rsp_evt = &p_ble_gattc_evt->params.char_disc_rsp;

        // Save characteristics data.
       for (uint8_t i = 0; i < bytes_to_copy; i++)
       {
           //NRF_LOG_INFO("idx %d", i);
           m_srv_char.char_data[i + offset].decl_handle      = p_char_disc_rsp_evt->chars[i].handle_decl;
           m_srv_char.char_data[i + offset].value_handle     = p_char_disc_rsp_evt->chars[i].handle_value;
           m_srv_char.char_data[i + offset].uuid             = p_char_disc_rsp_evt->chars[i].uuid;
           m_srv_char.char_data[i + offset].char_props       = p_char_disc_rsp_evt->chars[i].char_props;
           m_srv_char.char_data[i + offset].cccd_desc_handle = 0;
           //NRF_LOG_INFO("decl_handle  %x", m_srv_char.char_data[i + offset].decl_handle);
           //NRF_LOG_INFO("value_handle  %x", m_srv_char.char_data[i + offset].value_handle);
           //NRF_LOG_INFO("uuid  %x", m_srv_char.char_data[i + offset].uuid);
       }

       offset += bytes_to_copy;
    }


    // If the last characteristic has not been reached, look for a new handle range.
    ble_gattc_handle_range_t handle_range;
    handle_range.start_handle = m_srv_char.char_data[offset - 1].value_handle + 1;
    uint8_t c = mp_device_srv[conn_handle]->count;
    NRF_LOG_INFO("handle_range.start_handle: %x, count  %d", handle_range.start_handle, c);


    // Search for end handle.
    for (uint8_t j = 0; j < mp_device_srv[conn_handle]->count; j++)
    {
        if ((handle_range.start_handle > mp_device_srv[conn_handle]->services[j].handle_range.start_handle) &&
            (handle_range.start_handle < mp_device_srv[conn_handle]->services[j].handle_range.end_handle))
        {
            handle_range.end_handle = mp_device_srv[conn_handle]->services[j].handle_range.end_handle;
            NRF_LOG_INFO("handle_range.end_handle: %x", handle_range.end_handle);
            break;
        }
    }

    // Handle value of the characteristic being discovered is less than the end handle of
    // the service being discovered. There is no possibility of more characteristics being
    // present.
    if ((m_srv_char.char_data[offset - 1].value_handle >= handle_range.end_handle) ||
        (offset == MAX_CHARACTERISTIC_COUNT) ||
        (p_ble_gattc_evt->gatt_status != BLE_GATT_STATUS_SUCCESS))
    {
        m_srv_char.count = offset;
        offset           = 0;
        NRF_LOG_INFO("VENDOR UUID");

        for (uint8_t i = 0; i < m_srv_char.count; i++)
        {
            if (m_srv_char.char_data[i].uuid.type == BLE_UUID_TYPE_UNKNOWN)
            {
                NRF_LOG_INFO("VENDOR UUID");
                m_vendor_char_uuid_read = true;
                // Read char 128-bit UUID.
                err_code = sd_ble_gattc_read(conn_handle, m_srv_char.char_data[i].decl_handle, 0);
                APP_ERROR_CHECK(err_code);
                return;
            }
        }

        // Print characteristic data.
        characteristics_print();

        // Search for the CCCD descriptors.
        cccd_descriptors_discovery(p_ble_gattc_evt);

        return;
    }

    // If the last Characteristic has not been reached, this function must be called again with new handle range.
    err_code = sd_ble_gattc_characteristics_discover(p_ble_gattc_evt->conn_handle, &handle_range);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the LEDs initialization.
 *
 * @details Initializes all LEDs used by the application.
 */
static void leds_init(void)
{
    bsp_board_init(BSP_INIT_LEDS);
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;
    ble_gap_evt_t const * p_gap_evt = &p_ble_evt->evt.gap_evt;
    m_conn_handle  = p_ble_evt->evt.gap_evt.conn_handle;

    switch (p_ble_evt->header.evt_id)
    {
        // Upon connection, initiate secure bonding.
        case BLE_GAP_EVT_CONNECTED:
            m_is_connected = true;
            NRF_LOG_INFO("Connected, handle %x", m_conn_handle);
            // Display device address.
            hex_addr_display(p_ble_evt->evt.gap_evt.params.connected.peer_addr.addr,
                             BLE_GAP_ADDR_LEN);

            // Allocate memory for services data.
            mp_device_srv[m_conn_handle] = (device_srv_t *)nrf_balloc_alloc(&m_srv_pool);
            NRF_LOG_INFO("Alloc memory: %x", mp_device_srv[m_conn_handle]);
            ASSERT(mp_device_srv[m_conn_handle] != NULL);

            // Discover the peer services.
            err_code = ble_db_discovery_start(&m_db_discovery,
                                              p_ble_evt->evt.gap_evt.conn_handle);
            APP_ERROR_CHECK(err_code);

            break;

        // Upon disconnection, reset the connection handle of the peer that disconnected
        // and invalidate data taken from the NFC tag.
        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected. conn_handle: 0x%x, reason: 0x%x",
                         p_gap_evt->conn_handle,
                         p_gap_evt->params.disconnected.reason);

            // Release of allocated memory.
            nrf_balloc_free(&m_srv_pool, mp_device_srv[m_conn_handle]);

            m_conn_handle       = BLE_CONN_HANDLE_INVALID;
            m_is_connected      = false;
            m_hrs_notif_enabled = false;
            //nfc_oob_pairing_tag_invalidate();
            break;

        // 29 BLE_GAP_EVT_ADV_REPORT
        case BLE_GAP_EVT_ADV_REPORT:
            //NRF_LOG_DEBUG("BLE_GAP_EVT_ADV_REPORT");
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

        case BLE_GAP_EVT_CONN_PARAM_UPDATE_REQUEST:
            // Accept parameters requested by the the peer.
            err_code = sd_ble_gap_conn_param_update(p_gap_evt->conn_handle,
                                        &p_gap_evt->params.conn_param_update_request.conn_params);
            APP_ERROR_CHECK(err_code);
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

        case BLE_GAP_EVT_TIMEOUT:
            if (p_gap_evt->params.timeout.src == BLE_GAP_TIMEOUT_SRC_CONN)
            {
                NRF_LOG_INFO("Connection Request timed out.");
            }
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
//            if (p_ble_evt->evt.gap_evt.params.conn_sec_update.conn_sec.sec_mode.lv >= SECURITY_LEVEL_THR)
//            {
//                NRF_LOG_INFO("Security level high enough to enable HRS notifications.");
//            }
//            else
//            {
//                NRF_LOG_INFO("Security level too low to enable HRS notifications.");
//            }
            break;


//        case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
//        {
//            NRF_LOG_DEBUG("BLE_GATTC_EVT_PRIM_DISC_RSP");
//            on_primary_srv_discovery_rsp(&(p_ble_evt->evt.gattc_evt));
//        }
//        break;


//        case BLE_GATTC_EVT_DESC_DISC_RSP:
//        {
//            on_descriptor_discovery_rsp(&(p_ble_evt->evt.gattc_evt));
//        }
//        break;

//        case BLE_GATTC_EVT_READ_RSP:
//        {
//            on_read_rsp(&(p_ble_evt->evt.gattc_evt));
//        }
//        break;
//
//        case BLE_GATTC_EVT_WRITE_RSP:
//        {
//            on_write_rsp(&(p_ble_evt->evt.gattc_evt));
//        }
//        break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for handling the system events of the application.
 *
 * @param[in]   sys_evt   System event.
 */
static void soc_evt_handler(uint32_t sys_evt, void * p_context)
{
    switch (sys_evt)
    {
        case NRF_EVT_FLASH_OPERATION_SUCCESS:
            /* fall through */

        case NRF_EVT_FLASH_OPERATION_ERROR:
            if (m_memory_access_in_progress)
            {
                m_memory_access_in_progress = false;
                scan_start();
            }
            break;

        default:
            // No implementation needed.
            break;
    }
}



/**@brief Function for handling Battery Level Collector events.
 *
 * @param[in] p_bas_c       Pointer to Battery Service Client structure.
 * @param[in] p_bas_c_evt   Pointer to event structure.
 */
static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
{
    ret_code_t err_code;

    switch (p_bas_c_evt->evt_type)
    {
        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
            err_code = ble_bas_c_handles_assign(p_bas_c,
                                                p_bas_c_evt->conn_handle,
                                                &p_bas_c_evt->params.bas_db);
            APP_ERROR_CHECK(err_code);

            // Battery Service discovered. Enable notification of Battery Level.
            NRF_LOG_DEBUG("Battery Service discovered. Reading Battery Level.");

            err_code = ble_bas_c_bl_read(p_bas_c);
            APP_ERROR_CHECK(err_code);

            NRF_LOG_DEBUG("Enabling Battery Level Notification. ");
            err_code = ble_bas_c_bl_notif_enable(p_bas_c);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_BAS_C_EVT_BATT_NOTIFICATION:
            NRF_LOG_DEBUG("Battery Level received %d %%", p_bas_c_evt->params.battery_level);
//            if (!m_hrs_notif_enabled)
//            {
//                // Enable notifications of Heart Rate Measurement.
//                err_code = ble_hrs_c_hrm_notif_enable(&m_hrs_c);
//                APP_ERROR_CHECK(err_code);
//            }
            break;

        case BLE_BAS_C_EVT_BATT_READ_RESP:
            NRF_LOG_INFO("Battery Level read as %d %%", p_bas_c_evt->params.battery_level);
            break;

        default:
            break;
    }
}


//static void scan_evt_handler(scan_evt_t const * p_scan_evt)
//{
//    switch(p_scan_evt->scan_evt_id)
//    {
//        case NRF_BLE_SCAN_EVT_SCAN_TIMEOUT:
//            NRF_LOG_DEBUG("Scan timed out.");
//            scan_start();
//
//            break;
//
//        default:
//            break;
//    }
//
//}


///**@brief Function for handling the advertising report BLE event.
// *
// * @param[in] p_adv_report  Advertising report from the SoftDevice.
// */
//void on_adv_report(ble_gap_evt_adv_report_t const * p_adv_report)
//{
//    ret_code_t  err_code;
//    uint8_t   * p_adv_data;
//    uint16_t    data_len;
//    uint16_t    field_len;
//    uint16_t    dev_name_offset = 0;
//    char        dev_name[DEV_NAME_LEN];
//
//    // Initialize advertisement report for parsing.
//    p_adv_data = (uint8_t *)p_adv_report->data.p_data;
//    data_len   = p_adv_report->data.len;
//
//    // Search for advertising names.
//    field_len = ble_advdata_search(p_adv_data,
//                                   data_len,
//                                   &dev_name_offset,
//                                   BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME);
//    if (field_len == 0)
//    {
//        // Look for the short local name if it was not found as complete.
//        field_len = ble_advdata_search(p_adv_data,
//                                       data_len,
//                                       &dev_name_offset,
//                                       BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME);
//        if (field_len == 0)
//        {
//            // Exit if the data cannot be parsed.
//            return;
//        }
//    }
//
//    NRF_LOG_DEBUG("Found advertising device");
//    //NRF_LOG_HEXDUMP_DEBUG(p_adv_data, data_len);
//
//    memcpy(dev_name, &p_adv_data[dev_name_offset], field_len);
//    dev_name[field_len] = 0;
//    NRF_LOG_HEXDUMP_DEBUG(dev_name, field_len);

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
//}


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
            //state = 1;
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
void scan_start(void)
{
    ret_code_t ret;

    NRF_LOG_INFO("Start scanning for device (name): %s", m_target_periph_name);
    ret = nrf_ble_scan_start(&m_scan);
    APP_ERROR_CHECK(ret);
    // Turn on the LED to signal scanning.
    bsp_board_led_on(CENTRAL_SCANNING_LED);
}

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



///**
// * @brief Function for initializing the Battery Level Collector.
// */
//static void bas_c_init(void)
//{
//    ble_bas_c_init_t bas_c_init_obj;
//
//    bas_c_init_obj.evt_handler   = bas_c_evt_handler;
//    bas_c_init_obj.error_handler = service_error_handler;
//    bas_c_init_obj.p_gatt_queue  = &m_ble_gatt_queue;
//
//    ret_code_t err_code = ble_bas_c_init(&m_bas_c, &bas_c_init_obj);
//    APP_ERROR_CHECK(err_code);
//}


/**
 * @brief Function for initializing the Database Discovery Collector.
 */
//static void db_discovery_init(void)
//{
//    ble_db_discovery_init_t db_init;
//
//    memset(&db_init, 0, sizeof(ble_db_discovery_init_t));
//
//    db_init.evt_handler  = db_disc_handler;
//    db_init.p_gatt_queue = &m_ble_gatt_queue;
//
//    ret_code_t err_code = ble_db_discovery_init(&db_init);
//    APP_ERROR_CHECK(err_code);
//}


/**@brief Function for initializing the GATT module.
 */
//static void gatt_init(void)
//{
//    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
//    APP_ERROR_CHECK(err_code);
//}


///**@brief Function for initializing scanning.
// */
//static void scan_init(void)
//{
//    ret_code_t err_code;
//
//    err_code = nrf_ble_scan_init(&m_scan, NULL, scan_evt_handler);
//    APP_ERROR_CHECK(err_code);
//}


//void ble_stack_init(void)
//{
//    ret_code_t err_code;
//
//    err_code = nrf_sdh_enable_request();
//    APP_ERROR_CHECK(err_code);
//
//    // Configure the BLE stack using the default settings.
//    // Fetch the start address of the application RAM.
//    uint32_t ram_start = 0;
//    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
//    APP_ERROR_CHECK(err_code);
//
//    // Enable BLE stack.
//    err_code = nrf_sdh_ble_enable(&ram_start);
//    APP_ERROR_CHECK(err_code);
//
//    // Register handlers for BLE and SoC events.
////    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
////    NRF_SDH_SOC_OBSERVER(m_soc_observer, APP_SOC_OBSERVER_PRIO, soc_evt_handler, NULL);
//
////    gatt_init();
////    db_discovery_init();
////    hrs_c_init();
////    bas_c_init();
////    scan_init();
//}

/**
 * @brief Function for initializing a block allocator.
 */
static inline void balloc_init(void)
{
    ret_code_t err_code;

    err_code = nrf_balloc_init(&m_srv_pool);
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

    ble_bas_on_db_disc_evt(&m_bas_c[p_evt->conn_handle], p_evt);
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

//
///**@brief Handles events coming from the LED Button central module.
// *
// * @param[in] p_bas_c     The instance of bas_c that triggered the event.
// * @param[in] p_bas_c_evt The bas_c event.
// */
//static void bas_c_evt_handler(ble_bas_c_t * p_bas_c, ble_bas_c_evt_t * p_bas_c_evt)
//{
//    switch (p_bas_c_evt->evt_type)
//    {
//        case BLE_BAS_C_EVT_DISCOVERY_COMPLETE:
//        {
//            ret_code_t err_code;
//
//            NRF_LOG_INFO("BAS Service discovered on conn_handle 0x%x",
//                         p_bas_c_evt->conn_handle);
//
//            err_code = app_button_enable();
//            APP_ERROR_CHECK(err_code);
//
//            // Service discovered. Enable notification of Button.
//            //err_code = ble_bas_c_button_notif_enable(p_bas_c);
//            //APP_ERROR_CHECK(err_code);
//        } break; // BLE_bas_c_EVT_DISCOVERY_COMPLETE

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

//        default:
//            // No implementation needed.
//            break;
//    }
//}


void ble_m_init(void)
{
    // Initialization of required BLE components.
    NRF_LOG_DEBUG("ble_m_init");
    balloc_init();

    ble_stack_init();
    gatt_init();
    db_discovery_init();
    bas_c_init();
    //ble_conn_state_init();
    scan_init();

////    buttons_leds_init();
//    ble_stack_init();
//    scan_init();
//    //gap_params_init();
//    gatt_init();
////    bas_init();
////    conn_params_init();
////    advertising_init();
}


