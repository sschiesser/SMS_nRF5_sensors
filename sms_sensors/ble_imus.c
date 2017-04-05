#include "sdk_common.h"
#include "ble_imus.h"
#include "ble_srv_common.h"

/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t value_char_add(ble_imus_t * p_imus, const ble_imus_init_t * p_imus_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_md_t cccd_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.notify = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = &cccd_md;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_imus->uuid_type;
    ble_uuid.uuid = IMUS_UUID_VALUE_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc       = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth    = 0;
    attr_md.wr_auth    = 0;
    attr_md.vlen       = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid       = &ble_uuid;
    attr_char_value.p_attr_md    = &attr_md;
    attr_char_value.init_len     = sizeof(uint8_t);
    attr_char_value.init_offs    = 0;
    attr_char_value.max_len      = sizeof(uint8_t);
    attr_char_value.p_value      = NULL;

    return sd_ble_gatts_characteristic_add(p_imus->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_imus->val_char_handles);
}

uint32_t ble_imus_init(ble_imus_t * p_imus, const ble_imus_init_t * p_imus_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_imus->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_imus->val_write_handler = p_imus_init->val_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {IMUS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_imus->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_imus->uuid_type;
    ble_uuid.uuid = IMUS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_imus->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = value_char_add(p_imus, p_imus_init);
    VERIFY_SUCCESS(err_code);

//    err_code = led_char_add(p_lbs, p_lbs_init);
//    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}
