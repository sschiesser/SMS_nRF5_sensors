#include "sdk_common.h"
#include "ble_aps.h"
#include "ble_srv_common.h"

uint32_t ble_aps_init(ble_aps_t * p_aps, const ble_aps_init_t * p_aps_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_aps->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_aps->val_write_handler = p_aps_init->val_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {APS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_aps->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_aps->uuid_type;
    ble_uuid.uuid = APS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_aps->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
//    err_code = button_char_add(p_lbs, p_lbs_init);
//    VERIFY_SUCCESS(err_code);

//    err_code = led_char_add(p_lbs, p_lbs_init);
//    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
