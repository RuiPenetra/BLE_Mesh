/* main.c - Ponto de entrada principal da aplicação */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

 #include <stdio.h>
 #include <string.h>
 #include <inttypes.h>
 
 #include "esp_log.h"
 #include "nvs_flash.h"
 
 #include "esp_ble_mesh_common_api.h"
 #include "esp_ble_mesh_provisioning_api.h"
 #include "esp_ble_mesh_networking_api.h"
 #include "esp_ble_mesh_config_model_api.h"
 #include "esp_ble_mesh_generic_model_api.h"
 
 #include "board.h"
 #include "ble_mesh_example_init.h"
 #include "ble_mesh_example_nvs.h"
 
 #define TAG "EXAMPLE" // Tag para logs
 #define BUTTON_GPIO 27  // Pino GPIO conectado ao botão físico
 
 #define CID_ESP 0x02E5 // Identificador de empresa da Espressif
 
 // UUID do dispositivo (16 bytes)
 static uint8_t dev_uuid[16] = { 0xdd, 0xdd };
 
 // Estrutura para armazenar informações do nó Mesh
 static struct example_info_store {
     uint16_t net_idx;   // Índice da NetKey
     uint16_t app_idx;   // Índice da AppKey  
     uint8_t  onoff;     // Estado atual do LED (ON/OFF)
     uint8_t  tid;       // Transaction ID (para sequenciamento de mensagens)
 } __attribute__((packed)) store = {
     .net_idx = ESP_BLE_MESH_KEY_UNUSED, // Valor inicial
     .app_idx = ESP_BLE_MESH_KEY_UNUSED,
     .onoff = LED_OFF,
     .tid = 0x0,
 };
 
 // Handles para armazenamento NVS
 static nvs_handle_t NVS_HANDLE;
 static const char * NVS_KEY = "onoff_client";
 
 // Cliente BLE Mesh
 static esp_ble_mesh_client_t onoff_client;
 
 // Configuração do servidor BLE Mesh
 static esp_ble_mesh_cfg_srv_t config_server = {
     .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20), // 3 transmissões com intervalo de 20ms
     .relay = ESP_BLE_MESH_RELAY_DISABLED, // Retransmissão desativada
     .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
     .beacon = ESP_BLE_MESH_BEACON_ENABLED, // Beacon ativado
 #if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
     .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
 #else
     .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
 #endif
 #if defined(CONFIG_BLE_MESH_FRIEND)
     .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
 #else
     .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
 #endif
     .default_ttl = 7, // Time-to-Live padrão
 };
 
 // Definição de publicação para o modelo cliente OnOff
 //ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 2 + 1, ROLE_NODE);
 ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_cli_pub, 5, ROLE_NODE);
 
 // Modelos raiz (Configuration Server + Generic OnOff Client)
 static esp_ble_mesh_model_t root_models[] = {
     ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
     ESP_BLE_MESH_MODEL_GEN_ONOFF_CLI(&onoff_cli_pub, &onoff_client),
 };
 
 // Elementos da composição do dispositivo
 static esp_ble_mesh_elem_t elements[] = {
     ESP_BLE_MESH_ELEMENT(0, root_models, ESP_BLE_MESH_MODEL_NONE),
 };
 
 // Composição completa do dispositivo BLE Mesh
 static esp_ble_mesh_comp_t composition = {
     .cid = CID_ESP,
     .element_count = ARRAY_SIZE(elements),
     .elements = elements,
 };
 
 // Configuração de provisionamento (sem OOB)
 static esp_ble_mesh_prov_t provision = {
     .uuid = dev_uuid,
     .output_size = 0,
     .output_actions = 0
 };
 
 /* Armazena informações do nó no NVS */
 static void mesh_example_info_store(void)
 {
     ble_mesh_nvs_store(NVS_HANDLE, NVS_KEY, &store, sizeof(store));
 }
 
 /* Recupera informações do nó do NVS */
 static void mesh_example_info_restore(void)
 {
     esp_err_t err = ESP_OK;
     bool exist = false;
 
     err = ble_mesh_nvs_restore(NVS_HANDLE, NVS_KEY, &store, sizeof(store), &exist);
     if (err != ESP_OK) {
         return;
     }
 
     if (exist) {
         ESP_LOGI(TAG, "Restaurado: net_idx 0x%04x, app_idx 0x%04x, onoff %u, tid 0x%02x",
             store.net_idx, store.app_idx, store.onoff, store.tid);
     }
 }
 
 /* Callback chamado quando o provisionamento é completado */
 static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
 {
     ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
     ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
     ESP_LOGI(TAG, "Provisionamento completado! NetIdx: 0x%04x, Addr: 0x%04x", net_idx, addr);
     board_led_operation(LED_G, LED_OFF);
     store.net_idx = net_idx;
 }
 
 /* Callback de eventos de provisionamento */
 static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                              esp_ble_mesh_prov_cb_param_t *param)
 {
     switch (event) {
     case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
         ESP_LOGI(TAG, "Registro de provisionamento completado, código: %d", param->prov_register_comp.err_code);
         mesh_example_info_restore(); // Restaura informações salvas
         break;
     case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
         ESP_LOGI(TAG, "Nó de provisionamento ativado, código: %d", param->node_prov_enable_comp.err_code);
         break;
     case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
         ESP_LOGI(TAG, "Link de provisionamento aberto, bearer: %s",
             param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
         break;
     case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
         ESP_LOGI(TAG, "Link de provisionamento fechado, bearer: %s",
             param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
         break;
     case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
         ESP_LOGI(TAG, "Provisionamento completado");
         prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
             param->node_prov_complete.flags, param->node_prov_complete.iv_index);
         break;
     default:
         break;
     }
 }
 
 /* Envia comando SET para o servidor OnOff */
 void example_ble_mesh_send_gen_onoff_set(void)
 {
     // Verifica se a rede está configurada
     if (store.net_idx == ESP_BLE_MESH_KEY_UNUSED || store.app_idx == ESP_BLE_MESH_KEY_UNUSED) {
         ESP_LOGE(TAG, "Rede não configurada! NetIdx: 0x%04x, AppIdx: 0x%04x", 
                 store.net_idx, store.app_idx);
         return;
     }
 
     esp_ble_mesh_generic_client_set_state_t set = {0};
     //esp_ble_mesh_client_common_param_t common = {0};
     
     // Configuração da mensagem
     //common.opcode = ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK;
     //common.model = onoff_client.model;
     //common.ctx.net_idx = store.net_idx;    // NetKey Index
     //common.ctx.app_idx = store.app_idx;    // AppKey Index
     //common.ctx.addr = 0x003A;             // Endereço do nó destino
     //common.ctx.send_ttl = 3;              // Time-to-Live
     //common.msg_timeout = 0;               // Timeout padrão
 
     // Configura o estado OnOff
     //set.onoff_set.op_en = false;
     //set.onoff_set.onoff = store.onoff;
     //set.onoff_set.tid = store.tid++;
 
     uint8_t msg[2];
     msg[0] = store.onoff;
     msg[1] = store.tid++;

     // Envia a mensagem
     esp_err_t err = esp_ble_mesh_model_publish(
        onoff_client.model,
        ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET_UNACK,
        sizeof(msg),
        msg,
        ROLE_NODE
    );
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Falha ao enviar comando: 0x%04X", err);
         return;
     }
 
     // Alterna o estado e armazena
     store.onoff = !store.onoff;
     mesh_example_info_store();
     
     ESP_LOGI(TAG, "Comando enviado! Estado: %s", store.onoff ? "ON" : "OFF");
 }
 
 /* Callback para eventos do cliente genérico */
 static void example_ble_mesh_generic_client_cb(esp_ble_mesh_generic_client_cb_event_t event,
                                                esp_ble_mesh_generic_client_cb_param_t *param)
 {
     ESP_LOGI(TAG, "Evento de cliente genérico: %u, código: %d, opcode: 0x%04" PRIx32,
         event, param->error_code, param->params->opcode);
 
     switch (event) {
     case ESP_BLE_MESH_GENERIC_CLIENT_GET_STATE_EVT:
         ESP_LOGI(TAG, "Evento GET_STATE");
         if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_GET) {
             ESP_LOGI(TAG, "Estado OnOff: %d", param->status_cb.onoff_status.present_onoff);
         }
         break;
     case ESP_BLE_MESH_GENERIC_CLIENT_SET_STATE_EVT:
         ESP_LOGI(TAG, "Evento SET_STATE");
         if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
             ESP_LOGI(TAG, "Estado OnOff definido: %d", param->status_cb.onoff_status.present_onoff);
         }
         break;
     case ESP_BLE_MESH_GENERIC_CLIENT_PUBLISH_EVT:
         ESP_LOGI(TAG, "Evento de publicação");
         break;
     case ESP_BLE_MESH_GENERIC_CLIENT_TIMEOUT_EVT:
         ESP_LOGI(TAG, "Timeout");
         if (param->params->opcode == ESP_BLE_MESH_MODEL_OP_GEN_ONOFF_SET) {
             // Reenvia o comando em caso de timeout
             example_ble_mesh_send_gen_onoff_set();
         }
         break;
     default:
         break;
     }
 }
 
 /* Callback para eventos do servidor de configuração */
 static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                               esp_ble_mesh_cfg_server_cb_param_t *param)
 {
     if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
         switch (param->ctx.recv_op) {
         case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
             ESP_LOGI(TAG, "AppKey adicionada");
             ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                 param->value.state_change.appkey_add.net_idx,
                 param->value.state_change.appkey_add.app_idx);
             break;
         case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
             ESP_LOGI(TAG, "Modelo vinculado à AppKey");
             if (param->value.state_change.mod_app_bind.model_id == ESP_BLE_MESH_MODEL_ID_GEN_ONOFF_CLI) {
                 store.app_idx = param->value.state_change.mod_app_bind.app_idx;
                 ESP_LOGI(TAG, "AppKey vinculada ao OnOff Client: AppIdx 0x%04x", store.app_idx);
                 mesh_example_info_store();
             }
             break;
         default:
             break;
         }
     }
 }
 
 /* Inicializa o BLE Mesh */
 static esp_err_t ble_mesh_init(void)
 {
     esp_err_t err = ESP_OK;
 
     // Registra callbacks
     esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
     esp_ble_mesh_register_generic_client_callback(example_ble_mesh_generic_client_cb);
     esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
 
     // Inicializa a stack Mesh
     err = esp_ble_mesh_init(&provision, &composition);
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Falha ao inicializar a stack Mesh (erro %d)", err);
         return err;
     }
 
     // Ativa o nó
     err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
     if (err != ESP_OK) {
         ESP_LOGE(TAG, "Falha ao ativar o nó Mesh (erro %d)", err);
         return err;
     }
 
     ESP_LOGI(TAG, "Nó BLE Mesh inicializado");
     board_led_operation(LED_G, LED_ON);
 
     return err;
 }
 
 // Variável para controle do botão
 static volatile bool button_pressed = false;
 
 /* Handler de interrupção do botão */
 static void IRAM_ATTR button_isr_handler(void *arg) {
     button_pressed = true;
 }
 
 /* Inicializa o botão */
 void init_button(void) {
     gpio_config_t btn_config = {
         .pin_bit_mask = (1ULL << BUTTON_GPIO), // Máscara do pino
         .mode = GPIO_MODE_INPUT,               // Modo de entrada
         .pull_up_en = GPIO_PULLUP_ENABLE,      // Ativa pull-up interno
         .intr_type = GPIO_INTR_POSEDGE,        // Interrupção na borda de subida (quando solta o botão)
     };
     gpio_config(&btn_config);
     gpio_install_isr_service(0);               // Instala serviço de ISR
     gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL); // Adiciona handler
 }
 
 /* Função principal */
 void app_main(void)
 {
     esp_err_t err;
 
     ESP_LOGI(TAG, "Inicializando...");
 
     // Inicializa a placa
     board_init();
 
     // Inicializa NVS
     err = nvs_flash_init();
     if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
         ESP_ERROR_CHECK(nvs_flash_erase());
         err = nvs_flash_init();
     }
     ESP_ERROR_CHECK(err);
 
     // Inicializa Bluetooth
     err = bluetooth_init();
     if (err) {
         ESP_LOGE(TAG, "Falha na inicialização do Bluetooth (erro %d)", err);
         return;
     }
 
     // Abre namespace NVS
     err = ble_mesh_nvs_open(&NVS_HANDLE);
     if (err) {
         return;
     }
 
     // Obtém UUID do dispositivo
     ble_mesh_get_dev_uuid(dev_uuid);
 
     // Inicializa BLE Mesh
     err = ble_mesh_init();
     if (err) {
         ESP_LOGE(TAG, "Falha na inicialização do BLE Mesh (erro %d)", err);
     }
 
     // Inicializa botão
     init_button();
 
     // Loop principal
     while (1) {
         if (button_pressed) {
             button_pressed = false;  // Reseta flag
             example_ble_mesh_send_gen_onoff_set();  // Envia comando
             vTaskDelay(pdMS_TO_TICKS(20));  // Debounce de 20ms
         }
         vTaskDelay(pdMS_TO_TICKS(10));  // Delay para reduzir uso de CPU
     }
 }