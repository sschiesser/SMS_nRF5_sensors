#ifndef BLE_APS_H__
#define BLE_APS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APS_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define APS_UUID_SERVICE     0x1723
#define APS_UUID_VALUE_CHAR	 0x1724
//#define APS_UUID_LED_CHAR    0x1725
							  
// Forward declaration of the ble_aps_t type.
typedef struct ble_aps_s ble_aps_t;

typedef void (*ble_aps_val_write_handler_t) (ble_aps_t * p_aps, uint8_t value);

/** @brief Air Pressure Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
	ble_aps_val_write_handler_t val_write_handler; /**< Event handler to be called when the Air Pressure Characteristic is written. */
} ble_aps_init_t;

/**@brief Air Pressure Service structure. This structure contains various status information for the service. */
struct ble_aps_s
{
    uint16_t                    service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    val_char_handles;    /**< Handles related to the LED Characteristic. */
//    ble_gatts_char_handles_t    button_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_aps_val_write_handler_t val_write_handler;   /**< Event handler to be called when the pressure Air Pressure Characteristic is written. */
};


/**@brief Function for initializing the Air Pressure Service.
 *
 * @param[out] p_aps      Air Pressure Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_aps_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_aps_init(ble_aps_t * p_aps, const ble_aps_init_t * p_aps_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_aps      Air Pressure Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_aps_on_ble_evt(ble_aps_t * p_aps, ble_evt_t * p_ble_evt);

/**@brief Function for sending a air pressure value notification.
 *
 * @param[in] p_aps      Air Pressure Service structure.
 * @param[in] value  New pressure value.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_aps_on_new_value(ble_aps_t * p_aps, uint8_t value);


#ifdef __cplusplus
}
#endif


#endif // SMS_BLE_H__
