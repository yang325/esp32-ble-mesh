/* main.c - Application main entry point */

/*
 * Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Includes ------------------------------------------------------------------*/

#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "nvs_flash.h"

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"

/* Private function prototypes -----------------------------------------------*/

static esp_err_t bluetooth_init(void);
static esp_err_t ble_mesh_init(void);
static void esp_ble_mesh_prov_cb(esp_ble_mesh_prov_cb_event_t event,
                                    esp_ble_mesh_prov_cb_param_t *param);
static void esp_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                    esp_ble_mesh_cfg_client_cb_param_t *param);
static void esp_ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                    esp_ble_mesh_generic_client_cb_param_t *param);
static void recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[ESP_BD_ADDR_LEN],
                                    esp_ble_addr_type_t addr_type, uint16_t oob_info,
                                    uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer);
static void prov_complete(int node_idx, const esp_ble_mesh_octet16_t uuid,
                                    uint16_t unicast, uint8_t elem_num, uint16_t net_idx);
static void cfg_client_get_state(esp_ble_mesh_cfg_client_cb_param_t *param);
static void cfg_client_set_state(esp_ble_mesh_cfg_client_cb_param_t *param);
static void generic_client_get_state(esp_ble_mesh_generic_client_cb_param_t *param);

/* Private define ------------------------------------------------------------*/

#define TAG                 "mesh_provisioner"
#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12
#define NET_KEY_OCTET       0x34
#define PROV_OWN_ADDR       0x0001
#define CID_ESP             0x02E5
#define CID_NVAL            0xFFFF
#define MSG_SEND_TTL        3

/* Private variables ---------------------------------------------------------*/

static uint8_t dev_uuid[16];
static esp_ble_mesh_client_t config_client;
static esp_ble_mesh_client_t onoff_client;

static esp_ble_mesh_cfg_srv_t config_server = {
    .relay = ESP_BLE_MESH_RELAY_DISABLED,
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_GATT_PROXY)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
    .default_ttl = MSG_SEND_TTL,
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
};

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
    ESP_BLE_MESH_MODEL_CFG_CLI(&config_client),
    ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(NULL, &onoff_client),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .elements = elements,
    .element_count = ARRAY_SIZE(elements),
};

static esp_ble_mesh_prov_t provision = {
    .prov_uuid           = dev_uuid,
    .prov_unicast_addr   = PROV_OWN_ADDR,
    .prov_start_address  = 0x0005,
    .prov_attention      = 0x00,
    .prov_algorithm      = 0x00,
    .prov_pub_key_oob    = 0x00,
    .prov_static_oob_val = NULL,
    .prov_static_oob_len = 0x00,
    .flags               = 0x00,
    .iv_index            = 0x00,
};

static struct esp_ble_mesh_key {
    uint16_t net_idx;
    uint16_t app_idx;
    uint8_t  net_key[16];
    uint8_t  app_key[16];
} prov_key;

/* Exported functions --------------------------------------------------------*/

void app_main(void)
{
    int err;

    ESP_LOGI(TAG, "Initializing ...");

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
        return;
    }

    ESP_LOGI(TAG, "Have initialized");
}

static esp_err_t bluetooth_init(void)
{
    esp_err_t ret;

    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(TAG, "%s initialize controller failed", __func__);
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(TAG, "%s enable controller failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return ret;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return ret;
    }

    return ret;
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = 0;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));
    memset(prov_key.net_key, NET_KEY_OCTET, sizeof(prov_key.net_key));

    memcpy(dev_uuid, esp_bt_dev_get_address(), ESP_BD_ADDR_LEN);

    esp_ble_mesh_register_prov_callback(esp_ble_mesh_prov_cb);
    esp_ble_mesh_register_config_client_callback(esp_ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(esp_ble_mesh_generic_client_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err) {
        ESP_LOGE(TAG, "Initializing mesh failed (err %d)", err);
        return err;
    }

    esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV);
    esp_ble_mesh_provisioner_add_local_net_key(prov_key.net_key, prov_key.net_idx);
    esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);
    esp_ble_mesh_provisioner_bind_app_key_to_local_model(PROV_OWN_ADDR, prov_key.app_idx,
                                                    ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI, CID_NVAL);

    ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return ESP_OK;
}

static void esp_ble_mesh_prov_cb(esp_ble_mesh_prov_cb_event_t event,
                                    esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
        case ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_PROV_ENABLE_COMP_EVT, err_code %d", param->provisioner_prov_enable_comp.err_code);
            break;
        case ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_RECV_UNPROV_ADV_PKT_EVT");
            recv_unprov_adv_pkt(param->provisioner_recv_unprov_adv_pkt.dev_uuid, param->provisioner_recv_unprov_adv_pkt.addr,
                            param->provisioner_recv_unprov_adv_pkt.addr_type, param->provisioner_recv_unprov_adv_pkt.oob_info,
                            param->provisioner_recv_unprov_adv_pkt.adv_type, param->provisioner_recv_unprov_adv_pkt.bearer);
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_LINK_OPEN_EVT:
            ESP_LOGI(TAG, "%s link open", param->provisioner_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_LINK_CLOSE_EVT:
            ESP_LOGI(TAG, "%s link close, reason 0x%02x",
                            param->provisioner_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT",
                            param->provisioner_prov_link_close.reason);
            break;
        case ESP_BLE_MESH_PROVISIONER_PROV_COMPLETE_EVT:
            prov_complete(param->provisioner_prov_complete.node_idx, param->provisioner_prov_complete.device_uuid,
                            param->provisioner_prov_complete.unicast_addr, param->provisioner_prov_complete.element_num,
                            param->provisioner_prov_complete.netkey_idx);
            break;
        case ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT:
            ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_UNPROV_DEV_COMP_EVT, err_code %d", param->provisioner_add_unprov_dev_comp.err_code);
            break;
        default:
            break;
    }
}

static void esp_ble_mesh_config_client_cb(esp_ble_mesh_cfg_client_cb_event_t event,
                                            esp_ble_mesh_cfg_client_cb_param_t *param)
{
    ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
            __func__, param->error_code, event, param->params->ctx.addr, param->params->opcode);

    switch (event) {
        case ESP_BLE_MESH_CFG_CLIENT_GET_STATE_EVT:
            cfg_client_get_state(param);
            break;
        case ESP_BLE_MESH_CFG_CLIENT_SET_STATE_EVT:
            cfg_client_set_state(param);
            break;
        case ESP_BLE_MESH_CFG_CLIENT_PUBLISH_EVT:
            break;
        case ESP_BLE_MESH_CFG_CLIENT_TIMEOUT_EVT:
            break;
        default:
            break;
    }
}

static void esp_ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                            esp_ble_mesh_generic_client_cb_param_t *param)
{
    ESP_LOGI(TAG, "%s, error_code = 0x%02x, event = 0x%02x, addr: 0x%04x, opcode: 0x%04x",
            __func__, param->error_code, event, param->params->ctx.addr, param->params->opcode);

    switch (event) {
        case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
            generic_client_get_state(param);
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
            break;
        case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
            break;
        default:
            break;
    }
}

static void recv_unprov_adv_pkt(uint8_t dev_uuid[16], uint8_t addr[ESP_BD_ADDR_LEN],
                                esp_ble_addr_type_t addr_type, uint16_t oob_info,
                                uint8_t adv_type, esp_ble_mesh_prov_bearer_t bearer)
{
    esp_err_t err;
    esp_ble_mesh_unprov_dev_add_t add_dev = {0};

    ESP_LOGI(TAG, "address: %s, address type: %d, adv type: %d", bt_hex(addr, ESP_BD_ADDR_LEN), addr_type, adv_type);
    ESP_LOGI(TAG, "device uuid: %s", bt_hex(dev_uuid, 16));
    ESP_LOGI(TAG, "oob info: %d, bearer: %s", oob_info, (bearer & ESP_BLE_MESH_PROV_ADV) ? "PB-ADV" : "PB-GATT");

    if (ESP_BLE_MESH_PROV_ADV & bearer) {
        memcpy(add_dev.addr, addr, ESP_BD_ADDR_LEN);
        add_dev.addr_type = addr_type;
        memcpy(add_dev.uuid, dev_uuid, 16);
        add_dev.oob_info = oob_info;
        add_dev.bearer = ESP_BLE_MESH_PROV_ADV;
        err = esp_ble_mesh_provisioner_add_unprov_dev(&add_dev, ADD_DEV_RM_AFTER_PROV_FLAG | ADD_DEV_START_PROV_NOW_FLAG);
        if (err) {
            ESP_LOGE(TAG, "Adding device into list failed (err %d)", err);
        }
    }
}

static void prov_complete(int node_idx, const esp_ble_mesh_octet16_t uuid,
                               uint16_t unicast, uint8_t elem_num, uint16_t net_idx)
{
    esp_err_t err;
    char name[10];
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_get_state_t get_state = {0};

    ESP_LOGI(TAG, "node index: 0x%x, unicast address: 0x%02x, element num: %d, netkey index: 0x%02x",
                node_idx, unicast, elem_num, net_idx);
    ESP_LOGI(TAG, "device uuid: %s", bt_hex(uuid, 16));

    sprintf(name, "NODE-%d", node_idx);
    err = esp_ble_mesh_provisioner_set_node_name(node_idx, name);
    if (err) {
        ESP_LOGE(TAG, "%s: Send node name set failed", __func__);
        return;
    }

    common.opcode = ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET;
    common.model = config_client.model;
    common.ctx.net_idx = prov_key.net_idx;
    common.ctx.app_idx = prov_key.app_idx;
    common.ctx.addr = unicast;
    common.ctx.send_ttl = MSG_SEND_TTL;
    common.ctx.send_rel = false;
    common.msg_timeout = 0;
    common.msg_role = ROLE_PROVISIONER;
    get_state.comp_data_get.page = 0x00;

    err = esp_ble_mesh_config_client_get_state(&common, &get_state);
    if (err) {
        ESP_LOGE(TAG, "%s: Send config comp data get failed", __func__);
        return;
    }
}

static void cfg_client_get_state(esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_err_t err;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};

    switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_COMPOSITION_DATA_GET:
            ESP_LOGI(TAG, "composition data %s", bt_hex(param->status_cb.comp_data_status.composition_data->data,
                            param->status_cb.comp_data_status.composition_data->len));
            common.opcode = ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD;
            common.model = config_client.model;
            common.ctx.net_idx = prov_key.net_idx;
            common.ctx.app_idx = prov_key.app_idx;
            common.ctx.addr = param->params->ctx.addr;
            common.ctx.send_ttl = MSG_SEND_TTL;
            common.ctx.send_rel = false;
            common.msg_timeout = 0;
            common.msg_role = ROLE_PROVISIONER;
            set_state.app_key_add.net_idx = prov_key.net_idx;
            set_state.app_key_add.app_idx = prov_key.app_idx;
            memcpy(set_state.app_key_add.app_key, prov_key.app_key, sizeof(prov_key.app_key));
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err) {
                ESP_LOGE(TAG, "%s: Config AppKey Add failed", __func__);
            }
            break;
        default:
            break;
    }
}

static void cfg_client_set_state(esp_ble_mesh_cfg_client_cb_param_t *param)
{
    esp_err_t err;
    esp_ble_mesh_client_common_param_t common = {0};
    esp_ble_mesh_cfg_client_set_state_t set_state = {0};
    esp_ble_mesh_generic_client_get_state_t get_state = {0};

    switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            common.opcode = ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND;
            common.model = config_client.model;
            common.ctx.net_idx = prov_key.net_idx;
            common.ctx.app_idx = prov_key.app_idx;
            common.ctx.addr = param->params->ctx.addr;
            common.ctx.send_ttl = MSG_SEND_TTL;
            common.ctx.send_rel = false;
            common.msg_timeout = 0;
            common.msg_role = ROLE_PROVISIONER;
            set_state.model_app_bind.element_addr = param->params->ctx.addr;
            set_state.model_app_bind.model_app_idx = prov_key.app_idx;
            set_state.model_app_bind.model_id = ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_SRV;
            set_state.model_app_bind.company_id = CID_NVAL;
            err = esp_ble_mesh_config_client_set_state(&common, &set_state);
            if (err) {
                ESP_LOGE(TAG, "%s: Config Model App Bind failed", __func__);
            }
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET;
            common.model = onoff_client.model;
            common.ctx.net_idx = prov_key.net_idx;
            common.ctx.app_idx = prov_key.app_idx;
            common.ctx.addr = param->params->ctx.addr;
            common.ctx.send_ttl = MSG_SEND_TTL;
            common.ctx.send_rel = false;
            common.msg_timeout = 0;
            common.msg_role = ROLE_PROVISIONER;
            err = esp_ble_mesh_generic_client_get_state(&common, &get_state);
            if (err) {
                ESP_LOGE(TAG, "%s: Generic OnOff Get failed", __func__);
            }
            break;
        default:
            break;
    }
}

static void generic_client_get_state(esp_ble_mesh_generic_client_cb_param_t *param)
{
    switch (param->params->opcode) {
        case ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET onoff: 0x%02x",
                            param->status_cb.onoff_status.present_onoff);
            break;
        default:
            break;
    }
}
