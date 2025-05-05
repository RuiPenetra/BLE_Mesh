#include "esp_log.h"
#include "esp_bt.h"
#include "nvs_flash.h"

#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "ble_mesh_example_init.h"

#define TAG "RELAY_NODE"

static esp_ble_mesh_gen_onoff_srv_t onoff_server = {
    .state.onoff = 0,
};

ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub, 2 + 3, ROLE_NODE);

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub, &onoff_server),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
};

static esp_ble_mesh_comp_t composition = {
    .cid = 0x02E5,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

static esp_ble_mesh_prov_t provision = {
    .uuid = { 0xdd, 0xdd },
};

void app_main(void) {
    ESP_LOGI(TAG, "Inicializando nó relay com OnOff Server...");

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    ble_mesh_get_dev_uuid(provision.uuid);
    ESP_ERROR_CHECK(ble_mesh_init(&provision, &composition));
}