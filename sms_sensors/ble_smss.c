/* Copyright (c) 2013 Nordic Semiconductor. All Rights Reserved.
 *
 * Use of this source code is governed by a BSD-style license that can be
 * found in the license.txt file.
 */
#include "sdk_common.h"
#include "ble_smss.h"
#include "ble_srv_common.h"
#include "SEGGER_RTT.h"


/**@brief Function for handling the Connect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_connect(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    p_smss->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
}


/**@brief Function for handling the Disconnect event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_disconnect(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    UNUSED_PARAMETER(p_ble_evt);
    p_smss->conn_handle = BLE_CONN_HANDLE_INVALID;
}


/**@brief Function for handling the Write event.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_ble_evt  Event received from the BLE stack.
 */
static void on_write(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    ble_gatts_evt_write_t * p_evt_write = &p_ble_evt->evt.gatts_evt.params.write;

    if ((p_evt_write->handle == p_smss->led_char_handles.value_handle) &&
        (p_evt_write->len == 1) &&
        (p_smss->led_write_handler != NULL))
    {
        p_smss->led_write_handler(p_smss, p_evt_write->data[0]);
    }
}


void ble_smss_on_ble_evt(ble_smss_t * p_smss, ble_evt_t * p_ble_evt)
{
    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            on_connect(p_smss, p_ble_evt);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            on_disconnect(p_smss, p_ble_evt);
            break;

        case BLE_GATTS_EVT_WRITE:
            on_write(p_smss, p_ble_evt);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for adding the LED Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t led_char_add(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init)
{
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t    attr_char_value;
    ble_uuid_t          ble_uuid;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read   = 1;
    char_md.char_props.write  = 1;
    char_md.p_char_user_desc  = NULL;
    char_md.p_char_pf         = NULL;
    char_md.p_user_desc_md    = NULL;
    char_md.p_cccd_md         = NULL;
    char_md.p_sccd_md         = NULL;

    ble_uuid.type = p_smss->uuid_type;
    ble_uuid.uuid = SMSS_UUID_LED_CHAR;

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
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

    return sd_ble_gatts_characteristic_add(p_smss->service_handle,
                                           &char_md,
                                           &attr_char_value,
                                           &p_smss->led_char_handles);
}


static uint32_t press_char_add(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init)
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

    ble_uuid.type = p_smss->uuid_type;
    ble_uuid.uuid = SMSS_UUID_PRESS_CHAR;

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

    return sd_ble_gatts_characteristic_add(p_smss->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_smss->press_char_handles);
}

static uint32_t imu_char_add(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init)
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

    ble_uuid.type = p_smss->uuid_type;
    ble_uuid.uuid = SMSS_UUID_BUTTON_CHAR;

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

    return sd_ble_gatts_characteristic_add(p_smss->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_smss->imu_char_handles);
}



/**@brief Function for adding the Button Characteristic.
 *
 * @param[in] p_lbs      LED Button Service structure.
 * @param[in] p_lbs_init LED Button Service initialization structure.
 *
 * @retval NRF_SUCCESS on success, else an error value from the SoftDevice
 */
static uint32_t button_char_add(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init)
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

    ble_uuid.type = p_smss->uuid_type;
    ble_uuid.uuid = SMSS_UUID_IMU_CHAR;

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

    return sd_ble_gatts_characteristic_add(p_smss->service_handle,
                                               &char_md,
                                               &attr_char_value,
                                               &p_smss->button_char_handles);
}

uint32_t ble_smss_init(ble_smss_t * p_smss, const ble_smss_init_t * p_smss_init)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;

    // Initialize service structure.
    p_smss->conn_handle       = BLE_CONN_HANDLE_INVALID;
    p_smss->led_write_handler = p_smss_init->led_write_handler;
//	p_smss->button_write_handler = p_smss_init->button_write_handler;
//	p_smss->press_write_handler = p_smss_init->press_write_handler;
//	p_smss->imu_write_handler = p_smss_init->imu_write_handler;

    // Add service.
    ble_uuid128_t base_uuid = {SMSS_UUID_BASE};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_smss->uuid_type);
    VERIFY_SUCCESS(err_code);

    ble_uuid.type = p_smss->uuid_type;
    ble_uuid.uuid = SMSS_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY, &ble_uuid, &p_smss->service_handle);
    VERIFY_SUCCESS(err_code);

    // Add characteristics.
    err_code = button_char_add(p_smss, p_smss_init);
    VERIFY_SUCCESS(err_code);
	
	err_code = led_char_add(p_smss, p_smss_init);
	VERIFY_SUCCESS(err_code);
	
	err_code = press_char_add(p_smss, p_smss_init);
	VERIFY_SUCCESS(err_code);

	err_code = imu_char_add(p_smss, p_smss_init);
	VERIFY_SUCCESS(err_code);
	
	SEGGER_RTT_WriteString(0, "SMS Service initialized\n");
	SEGGER_RTT_printf(0, "service uuid: 0x%#04x\n", ble_uuid.uuid);
	SEGGER_RTT_printf(0, "service type: 0x%#02x\n", ble_uuid.type);
	SEGGER_RTT_printf(0, "service handle: 0x%#04x\n", p_smss->conn_handle);
	SEGGER_RTT_printf(0, "Button char...\n-value handle 0x%04x\n-cccd handle 0x%04x\n- user desc handle 0x%04x\n", 
				p_smss->button_char_handles.value_handle, 
				p_smss->button_char_handles.cccd_handle,
				p_smss->button_char_handles.user_desc_handle);
	SEGGER_RTT_printf(0, "Pressure char...\n-value handle 0x%04x\n-cccd handle 0x%04x\n- user desc handle 0x%04x\n", 
				p_smss->press_char_handles.value_handle, 
				p_smss->press_char_handles.cccd_handle,
				p_smss->press_char_handles.user_desc_handle);
	SEGGER_RTT_printf(0, "IMU char...\n-value handle 0x%04x\n-cccd handle 0x%04x\n- user desc handle 0x%04x\n", 
				p_smss->imu_char_handles.value_handle, 
				p_smss->imu_char_handles.cccd_handle,
				p_smss->imu_char_handles.user_desc_handle);
	
    return NRF_SUCCESS;
}

uint32_t ble_smss_on_button_change(ble_smss_t * p_smss, uint8_t button_state)
{
	SEGGER_RTT_printf(0, "on button change...\r\n");
    ble_gatts_hvx_params_t params;
    uint16_t len = sizeof(button_state);

    memset(&params, 0, sizeof(params));
    params.type = BLE_GATT_HVX_NOTIFICATION;
    params.handle = p_smss->button_char_handles.value_handle;
//    params.handle = p_smss->press_char_handles.value_handle;
    params.p_data = &button_state;
    params.p_len = &len;

	SEGGER_RTT_printf(0, "Sending: 0x%02x to 0x%04x->0x%04x\n", params.p_data[0], p_smss->conn_handle, params.handle);
    return sd_ble_gatts_hvx(p_smss->conn_handle, &params);
}

uint32_t ble_smss_on_press_value(ble_smss_t * p_smss, uint8_t value)
{
	SEGGER_RTT_printf(0, "on press value...\r\n");
	ble_gatts_hvx_params_t params;
	uint16_t len = sizeof(value);
//	uint8_t content = value;
	
	memset(&params, 0, sizeof(params));
	params.type = BLE_GATT_HVX_NOTIFICATION;
	params.handle = p_smss->press_char_handles.value_handle;
	params.p_data = &value;
	params.p_len = &len;
	
	SEGGER_RTT_printf(0, "Sending: 0x%02x to 0x%04x->0x%04x\n", params.p_data[0], p_smss->conn_handle, params.handle);
//	return NRF_SUCCESS;
	return sd_ble_gatts_hvx(p_smss->conn_handle, &params);
}
