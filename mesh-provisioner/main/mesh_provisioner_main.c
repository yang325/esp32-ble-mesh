/* main.c - Application main entry point */

/*
 * Copyright (c) 2018 Espressif Systems (Shanghai) PTE LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

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

#define TAG "ble_mesh_provisioner"

void app_main(void)
{
    int err;

    ESP_LOGI(TAG, "Initializing...");

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
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

static int ble_mesh_init(void)
{
    uint8_t match[2] = {0xdd, 0xdd};
    int err = 0;

    prov_key.net_idx = ESP_BLE_MESH_KEY_PRIMARY;
    prov_key.app_idx = APP_KEY_IDX;
    memset(prov_key.app_key, APP_KEY_OCTET, sizeof(prov_key.app_key));

    memcpy(dev_uuid, esp_bt_dev_get_address(), ESP_BD_ADDR_LEN);

    esp_ble_mesh_register_prov_callback(esp_ble_mesh_prov_cb);
    esp_ble_mesh_register_custom_model_callback(esp_ble_mesh_model_cb);
    esp_ble_mesh_register_config_client_callback(esp_ble_mesh_config_client_cb);
    esp_ble_mesh_register_generic_client_callback(esp_ble_mesh_generic_client_cb);

    esp_ble_mesh_provisioner_set_dev_uuid_match(match, sizeof(match), 0x0, false);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err) {
        ESP_LOGE(TAG, "Initializing mesh failed (err %d)", err);
        return err;
    }

    esp_ble_mesh_provisioner_prov_enable(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT);

    esp_ble_mesh_provisioner_add_local_app_key(prov_key.app_key, prov_key.net_idx, prov_key.app_idx);

    ESP_LOGI(TAG, "BLE Mesh Provisioner initialized");

    return err;
}