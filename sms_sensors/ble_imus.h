#ifndef BLE_IMUS_H__
#define BLE_IMUS_H__

#include <stdint.h>
#include <stdbool.h>
#include "ble.h"
#include "ble_srv_common.h"

#ifdef __cplusplus
extern "C" {
#endif

#define IMUS_UUID_BASE        {0x23, 0xD1, 0xBC, 0xEA, 0x5F, 0x78, 0x23, 0x15, \
                              0xDE, 0xEF, 0x12, 0x12, 0x00, 0x00, 0x00, 0x00}
#define IMUS_UUID_SERVICE     0x1623
#define IMUS_UUID_BUTTON_CHAR 0x1624
#define IMUS_UUID_LED_CHAR    0x1625

// Forward declaration of the ble_imus_t type.
typedef struct ble_imus_s ble_imus_t;

typedef void (*ble_imus_val_write_handler_t) (ble_imus_t * p_imus, uint8_t imu_value);

/** @brief IMU Service init structure. This structure contains all options and data needed for
 *        initialization of the service.*/
typedef struct
{
    ble_imus_val_write_handler_t val_write_handler; /**< Event handler to be called when the IMU Characteristic is written. */
} ble_imus_init_t;

/**@brief IMU Service structure. This structure contains various status information for the service. */
struct ble_imus_s
{
    uint16_t                    service_handle;      /**< Handle of LED Button Service (as provided by the BLE stack). */
    ble_gatts_char_handles_t    led_char_handles;    /**< Handles related to the LED Characteristic. */
    ble_gatts_char_handles_t    button_char_handles; /**< Handles related to the Button Characteristic. */
    uint8_t                     uuid_type;           /**< UUID type for the LED Button Service. */
    uint16_t                    conn_handle;         /**< Handle of the current connection (as provided by the BLE stack). BLE_CONN_HANDLE_INVALID if not in a connection. */
    ble_imus_val_write_handler_t val_write_handler;   /**< Event handler to be called when the LED Characteristic is written. */
};


/**@brief Function for initializing the IMU Service.
 *
 * @param[out] p_imus      IMU Service structure. This structure must be supplied by
 *                        the application. It is initialized by this function and will later
 *                        be used to identify this particular service instance.
 * @param[in] p_imus_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was initialized successfully. Otherwise, an error code is returned.
 */
uint32_t ble_imus_init(ble_imus_t * p_imus, const ble_imus_init_t * p_imus_init);

/**@brief Function for handling the application's BLE stack events.
 *
 * @details This function handles all events from the BLE stack that are of interest to the LED Button Service.
 *
 * @param[in] p_imus      IMU Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
void ble_imus_on_ble_evt(ble_imus_t * p_imus, ble_evt_t * p_ble_evt);

/**@brief Function for sending a imu value notification.
 *
 * @param[in] p_imus      IMU Service structure.
 * @param[in] imu_value  New IMU value.
 *
 * @retval NRF_SUCCESS If the notification was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_imus_on_new_value(ble_imus_t * p_imus, uint8_t imu_value);


#ifdef __cplusplus
}
#endif


#endif // BLE_IMUS_H__
