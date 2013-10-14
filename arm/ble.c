/*
 * Erl's Bluetooth Low Energy stuff
 */
 
#include "erl_ble.h"
#include "app_timer.h"

#include "ble_gap.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "ble_error_log.h"
#include "ble_stack_handler.h"
// #include "ble_srv_common.h"
#include "ble_bondmngr.h"
// #include "ble_radio_notification.h"
#include "ble_flash.h"
#include "ble_debug_assert_handler.h"

/*
 * Static function prototypes
 */
// static void ble_init( const char *deviceName );
static void ble_evt_dispatch(ble_evt_t * p_ble_evt);
static void gap_params_init(const char *deviceName );
static void advertising_init(void);
static void conn_params_init(void);
static void conn_params_error_handler(uint32_t nrf_error);

static void bond_manager_error_handler(uint32_t nrf_error);
static void bond_manager_init(void);
static void on_bond_mgmr_evt(ble_bondmngr_evt_t * p_evt);

static void sec_params_init(void);
static void on_ble_evt(ble_evt_t * p_ble_evt);
static void advertising_start(void);
static void timers_init(void);

/*
 * Constants
 */
#define APP_ADV_INTERVAL                40                                                   /**< The advertising interval (in units of 0.625 ms. This value corresponds to 25 ms). */
#define APP_ADV_INTERVAL_SLOW           3200                                                 /**< Slow advertising interval (in units of 0.625 ms. This value corresponds to 2 seconds). */
#define APP_ADV_TIMEOUT_IN_SECONDS      180                                                  /**< The advertising timeout in units of seconds. */
#define ADV_INTERVAL_FAST_PERIOD        30                                                   /**< The duration of the fast advertising period (in seconds). */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(500, UNIT_1_25_MS)                     /**< Minimum acceptable connection interval (0.5 seconds). */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(1000, UNIT_1_25_MS)                    /**< Maximum acceptable connection interval (1 second). */
#define SLAVE_LATENCY                   0                                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)                      /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(20 * 1000, APP_TIMER_PRESCALER)      /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (20 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(5 * 1000, APP_TIMER_PRESCALER)       /**< Time between each call to sd_ble_gap_conn_param_update after the first (5 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                                    /**< Number of attempts before giving up the connection parameter negotiation. */

#define SEC_PARAM_TIMEOUT               30                                                   /**< Timeout for Pairing Request or Security Request (in seconds). */
#define SEC_PARAM_BOND                  1                                                    /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                                    /**< Man In The Middle protection not required. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                                 /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                                    /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                                    /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                                   /**< Maximum encryption key size. */

#define FLASH_PAGE_SYS_ATTR             (BLE_FLASH_PAGE_END - 3)                             /**< Flash page used for bond manager system attribute information. */
#define FLASH_PAGE_BOND                 (BLE_FLASH_PAGE_END - 1)                             /**< Flash page used for bond manager bonding information. */
#define ANS_FLASH_PAGE                  (BLE_FLASH_PAGE_END - 4)                             /**< Flash page used for Alert Notification Client. */

#define DEAD_BEEF                       0xDEADBEEF                                           /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

typedef enum
{
    BLE_NO_ADVERTISING,                                                                      /**< No advertising running. */
    BLE_SLOW_ADVERTISING,                                                                    /**< Slow advertising running. */
    BLE_FAST_ADVERTISING                                                                     /**< Fast advertising running. */
} ble_advertising_mode_t;

/*
 * Global variables
 */
// static ble_ans_c_t                      m_ans_c;                                             /**< Structure used to identify the Alert Notification Service Client. */
// static uint8_t                          m_alert_message_buffer[MESSAGE_BUFFER_SIZE];         /**< Message buffer for optional notify messages. */

static ble_gap_adv_params_t             m_adv_params;                                        /**< Parameters to be passed to the stack when starting advertising. */
static ble_advertising_mode_t           m_advertising_mode;                                  /**< Variable to keep track of when we are advertising. */

// static ble_ans_c_alert_state_t          m_new_alert_state    = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of New Alert Notifications, i.e. Enabled, Alert On, Disabled. */
// static ble_ans_c_alert_state_t          m_unread_alert_state = ALERT_NOTIFICATION_DISABLED;  /**< State that holds the current state of Unread Alert Notifications, i.e. Enabled, Alert On, Disabled. */

static ble_gap_sec_params_t             m_sec_params;                                        /**< Security requirements for this application. */

/*
 * Start of code
 */
void erl_ble_init( const char *deviceName )
{
	// timers_init();
   bond_manager_init();
	
    BLE_STACK_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_20_PPM,
                           BLE_L2CAP_MTU_DEF,
                           ble_evt_dispatch,
                           false);
	// disable DC/DC to save power
	sd_power_dcdc_mode_set( NRF_POWER_DCDC_MODE_AUTOMATIC  );
	
  ble_error_log_init();
  gap_params_init( deviceName );
	advertising_init();
	// alert_notification_init();
	conn_params_init();
	sec_params_init();
	// radio_notification_init();
	
	// Start execution.
	advertising_start();
}


/**@brief Function for error handling, which is called when an error has occurred. 
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze 
 *          how your product is supposed to react in case of error.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name. 
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    // nrf_gpio_pin_set(ASSERT_LED_PIN_NO);

    // This call can be used for debug purposes during development of an application.
    // @note CAUTION: Activating this code will write the stack to flash on an error.
    //                This function should NOT be used in a final product.
    //                It is intended STRICTLY for development/debugging purposes.
    //                The flash write will happen EVEN if the radio is active, thus interrupting
    //                any communication.
    //                Use with care. Un-comment the line below to use.
    // ble_debug_assert_handler(error_code, line_num, p_file_name);

    // On assert, the system can only recover on reset.
    NVIC_SystemReset();
}


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
    app_error_handler(0xdeadbeef, line_num, p_file_name);
}

/**@brief Function for dispatching a BLE stack event to all modules with a BLE stack event handler.
 *
 * @details This function is called from the BLE Stack event interrupt handler after a BLE stack
 *          event has been received.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void ble_evt_dispatch(ble_evt_t * p_ble_evt)
{
    ble_bondmngr_on_ble_evt(p_ble_evt);
    ble_conn_params_on_ble_evt(p_ble_evt);
    // ble_ans_c_on_ble_evt(&m_ans_c, p_ble_evt);
    on_ble_evt(p_ble_evt);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function shall be used to setup all the necessary GAP (Generic Access Profile)
 *          parameters of the device. It also sets the permissions and appearance.
 */
static void gap_params_init( const char *deviceName )
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) deviceName, 
                                          strlen(deviceName));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
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
    uint8_t       flags = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;
    
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));
    
    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.include_appearance      = true;
    advdata.flags.size              = sizeof(flags);
    advdata.flags.p_data            = &flags;
    advdata.uuids_complete.uuid_cnt = 0;
    advdata.uuids_complete.p_uuids  = NULL;
    
    err_code = ble_advdata_set(&advdata, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = true;
    cp_init.evt_handler                    = NULL;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing security parameters.
 */
static void sec_params_init(void)
{
    m_sec_params.timeout      = SEC_PARAM_TIMEOUT;
    m_sec_params.bond         = SEC_PARAM_BOND;
    m_sec_params.mitm         = SEC_PARAM_MITM;
    m_sec_params.io_caps      = SEC_PARAM_IO_CAPABILITIES;
    m_sec_params.oob          = SEC_PARAM_OOB;  
    m_sec_params.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
    m_sec_params.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
}


/**@brief Function for handling a Bond Manager error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void bond_manager_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Bond Manager.
 */
static void bond_manager_init(void)
{
    uint32_t            err_code;
    ble_bondmngr_init_t bond_init_data;
    bool                bonds_delete;

    // Clear all bonded masters if the Bonds Delete button is pushed.
    // err_code = app_button_is_pushed(BONDMNGR_DELETE_BUTTON_PIN_NO, &bonds_delete);
    // APP_ERROR_CHECK(err_code);

    // Initialize the Bond Manager.
    bond_init_data.flash_page_num_bond     = FLASH_PAGE_BOND;
    bond_init_data.flash_page_num_sys_attr = FLASH_PAGE_SYS_ATTR;
    bond_init_data.evt_handler             = on_bond_mgmr_evt;
    bond_init_data.error_handler           = bond_manager_error_handler;
    bond_init_data.bonds_delete            = bonds_delete;
    
    err_code = ble_bondmngr_init(&bond_init_data);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the Bond Manager.
 *
 * @details This function will be called for all events in the Bond Manager which
 *          are passed to the application.
 *
 * @param[in]   p_evt   Event received from the Bond Manager.
 */
static void on_bond_mgmr_evt(ble_bondmngr_evt_t * p_evt)
{
    // ble_ans_c_on_bondmgmr_evt(&m_ans_c, p_evt);
}

/**@brief Function for handling the Application's BLE Stack events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 */
static void on_ble_evt(ble_evt_t * p_ble_evt)
{
    uint32_t        err_code = NRF_SUCCESS;
    static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID;
    
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
//            nrf_gpio_pin_set(CONNECTED_LED_PIN_NO);
//            nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
//            err_code = app_button_enable();
            m_advertising_mode = BLE_NO_ADVERTISING;
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            break;
            
        case BLE_GAP_EVT_DISCONNECTED:
//            nrf_gpio_pin_clear(CONNECTED_LED_PIN_NO);
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Since we are not in a connection and have not start to advertise either, store bonds.
            err_code = ble_bondmngr_bonded_masters_store();
            APP_ERROR_CHECK(err_code);

            // Stop detecting button presses when not connected.
//            err_code = app_button_disable();
//            APP_ERROR_CHECK(err_code);

            // Stop detecting button presses when not connected.
//            err_code = app_button_disable();
//            APP_ERROR_CHECK(err_code);

//            err_code = ble_ans_c_service_store();

            advertising_start();
            break;
            
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, 
                                                   BLE_GAP_SEC_STATUS_SUCCESS, 
                                                   &m_sec_params);
            break;

        case BLE_GAP_EVT_TIMEOUT:
            if (p_ble_evt->evt.gap_evt.params.timeout.src == BLE_GAP_TIMEOUT_SRC_ADVERTISEMENT)
            { 
                if (m_advertising_mode == BLE_FAST_ADVERTISING)
                {
                    advertising_start();
                }
                else
                {
//                    nrf_gpio_pin_clear(ADVERTISING_LED_PIN_NO);
                    m_advertising_mode = BLE_NO_ADVERTISING;

                    // Go to system-off mode (function will not return; wakeup will cause a reset).
										// erl: WTF?
//									GPIO_WAKEUP_BUTTON_CONFIG(WAKEUP_BUTTON_PIN);
//                    GPIO_WAKEUP_BUTTON_CONFIG(BONDMNGR_DELETE_BUTTON_PIN_NO);
                    err_code = sd_power_system_off();
                }
            }
            break;
            
        case BLE_GATTC_EVT_TIMEOUT:
        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server and Client timeout events.
            err_code = sd_ble_gap_disconnect(m_conn_handle, 
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            break;

        default:
            break;
    }

    APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code;

    if (m_advertising_mode == BLE_NO_ADVERTISING)
    {
        m_advertising_mode = BLE_FAST_ADVERTISING;
    }
    else
    {
        m_advertising_mode = BLE_SLOW_ADVERTISING;
    }

    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));
    
    m_adv_params.type        = BLE_GAP_ADV_TYPE_ADV_IND;
    m_adv_params.p_peer_addr = NULL;                           // Undirected advertisement.
    m_adv_params.fp          = BLE_GAP_ADV_FP_ANY;

    if (m_advertising_mode == BLE_FAST_ADVERTISING)
    {
        m_adv_params.interval = APP_ADV_INTERVAL;
        m_adv_params.timeout  = ADV_INTERVAL_FAST_PERIOD;
    }
    else
    {
        m_adv_params.interval = APP_ADV_INTERVAL_SLOW;
        m_adv_params.timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
    }

    err_code = sd_ble_gap_adv_start(&m_adv_params);
    APP_ERROR_CHECK(err_code);
    
//    nrf_gpio_pin_set(ADVERTISING_LED_PIN_NO);
}
