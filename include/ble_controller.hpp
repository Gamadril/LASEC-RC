#pragma once

#include "common.hpp"
#include "config_handler.hpp"
#include "esp_log.h"
#include "esp_system.h"
#include "host/ble_att.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "rgb_controller.hpp"
#include "servo_controller.hpp"
#include <algorithm>
#include <cstring>
#include <string>
#include <vector>

// Message types for BLE protocol
enum MessageType : uint8_t {
  // Config messages (JSON UTF-8 payload, may be fragmented)
  CONFIG_GET = 0x01,
  CONFIG_SET = 0x02,
  // State messages (binary packed State)
  STATE_GET = 0x11,
  STATE_SET = 0x12,
  // Ping/Pong
  PING = 0x20,
  PONG = 0x21,
  // Servo control
  GET_SHIFTING_SERVO = 0x30,
  SET_SHIFTING_SERVO = 0x31,
  GET_STEERING_SERVO = 0x32,
  SET_STEERING_SERVO = 0x33,
  // RGB LED (colour = uint32 LE 0x00RRGGBB, brightness = uint8)
  GET_RGB_COLOUR = 0x34,
  SET_RGB_COLOUR = 0x35,
  GET_RGB_BRIGHTNESS = 0x36,
  SET_RGB_BRIGHTNESS = 0x37,
  // Config management
  SAVE_CONFIG = 0x40,
  // System control
  REBOOT = 0x50,
  // Error
  CONFIG_ERROR = 0xFF
};

// Fragmentable message header
// size  = payload bytes in THIS packet
// total = full payload length across all fragments
// offset = byte offset of this fragment within total
struct __attribute__((packed)) BLEMessageHeader {
  uint8_t type;
  uint16_t size;
  uint16_t total;
  uint16_t offset;
};

// Nordic UART Service (NUS) UUIDs
// 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
static const ble_uuid128_t NUS_SVC_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x01, 0x00, 0x40, 0x6e);

static const ble_uuid128_t NUS_RX_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x02, 0x00, 0x40, 0x6e);

static const ble_uuid128_t NUS_TX_UUID =
    BLE_UUID128_INIT(0x9e, 0xca, 0xdc, 0x24, 0x0e, 0xe5, 0xa9, 0xe0, 0x93, 0xf3, 0xa3, 0xb5, 0x03, 0x00, 0x40, 0x6e);

class BLEController {
public:
  static inline uint16_t tx_handle = 0;
  static inline uint16_t conn_handle = BLE_HS_CONN_HANDLE_NONE;
  static inline uint8_t own_addr_type;

  static void init(State *state, ServoController *servoController, ConfigHandler *configHandler,
                   RgbController *rgbController = nullptr) {
    _configHandler = configHandler;
    _config = configHandler->getConfig();
    _state = state;
    _servoController = servoController;
    _rgbController = rgbController;

    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gap_device_name_set(_config->model_name);

    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_svcs);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_gatts_count_cfg failed: %d", rc);
      return;
    }

    rc = ble_gatts_add_svcs(gatt_svcs);
    if (rc != 0) {
      ESP_LOGE(TAG, "ble_gatts_add_svcs failed: %d", rc);
      return;
    }

    ble_hs_cfg.sync_cb = ble_app_on_sync;
    ble_hs_cfg.reset_cb = ble_app_on_reset;
    ble_hs_cfg.gatts_register_cb = gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    nimble_port_freertos_init(ble_host_task);
  }

  static const char *getMessageTypeName(MessageType type) {
    switch (type) {
      case CONFIG_GET:
        return "CONFIG_GET";
      case CONFIG_SET:
        return "CONFIG_SET";
      case STATE_GET:
        return "STATE_GET";
      case STATE_SET:
        return "STATE_SET";
      case PING:
        return "PING";
      case PONG:
        return "PONG";
      case GET_SHIFTING_SERVO:
        return "GET_SHIFTING_SERVO";
      case SET_SHIFTING_SERVO:
        return "SET_SHIFTING_SERVO";
      case GET_STEERING_SERVO:
        return "GET_STEERING_SERVO";
      case SET_STEERING_SERVO:
        return "SET_STEERING_SERVO";
      case GET_RGB_COLOUR:
        return "GET_RGB_COLOUR";
      case SET_RGB_COLOUR:
        return "SET_RGB_COLOUR";
      case GET_RGB_BRIGHTNESS:
        return "GET_RGB_BRIGHTNESS";
      case SET_RGB_BRIGHTNESS:
        return "SET_RGB_BRIGHTNESS";
      case SAVE_CONFIG:
        return "SAVE_CONFIG";
      case REBOOT:
        return "REBOOT";
      case CONFIG_ERROR:
        return "CONFIG_ERROR";
      default:
        return "UNKNOWN";
    }
  }

  static void send(const uint8_t *data, size_t len) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
      return;
    }

    if (len >= sizeof(BLEMessageHeader)) {
      auto *header = reinterpret_cast<const BLEMessageHeader *>(data);
      ESP_LOGI(TAG, "TX: type=%s (0x%02X), size=%u total=%u offset=%u",
               getMessageTypeName(static_cast<MessageType>(header->type)), header->type, header->size, header->total,
               header->offset);
    } else {
      ESP_LOGD(TAG, "TX: raw data, len=%zu", len);
    }

    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (!om) {
      ESP_LOGE(TAG, "Failed to allocate mbuf for TX");
      return;
    }
    int rc = ble_gatts_notify_custom(conn_handle, tx_handle, om);
    if (rc != 0) {
      ESP_LOGW(TAG, "Notify failed: %d", rc);
    }
  }

  static void sendMessage(uint8_t type, const void *payload, size_t len) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE) {
      return;
    }
    if (len > 0xFFFF) {
      sendError("Payload too large");
      return;
    }

    const auto *p = static_cast<const uint8_t *>(payload);
    const size_t max_chunk = maxChunkPayload();

    if (len == 0) {
      BLEMessageHeader header = {.type = type, .size = 0, .total = 0, .offset = 0};
      send(reinterpret_cast<const uint8_t *>(&header), sizeof(header));
      return;
    }

    for (size_t offset = 0; offset < len;) {
      size_t chunk = std::min(len - offset, max_chunk);
      std::vector<uint8_t> packet(sizeof(BLEMessageHeader) + chunk);
      BLEMessageHeader header = {.type = type,
                                 .size = static_cast<uint16_t>(chunk),
                                 .total = static_cast<uint16_t>(len),
                                 .offset = static_cast<uint16_t>(offset)};
      memcpy(packet.data(), &header, sizeof(header));
      memcpy(packet.data() + sizeof(header), p + offset, chunk);
      send(packet.data(), packet.size());
      offset += chunk;
      if (offset < len) {
        vTaskDelay(pdMS_TO_TICKS(20));
      }
    }
  }

  static void sendConfig() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !_configHandler) {
      return;
    }

    std::string json = _configHandler->toJson(false);
    if (json.empty()) {
      sendError("Config serialize failed");
      return;
    }

    ESP_LOGI(TAG, "Sending config JSON (%zu bytes)", json.size());
    sendMessage(CONFIG_GET, json.data(), json.size());
  }

  static void sendState() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !_state) {
      return;
    }

    ESP_LOGI(TAG, "Sending state (size=%zu bytes)", sizeof(State));
    sendMessage(STATE_GET, _state, sizeof(State));
  }

private:
  static inline const char *TAG = "BLE";
  static inline Config *_config = nullptr;
  static inline State *_state = nullptr;
  static inline ServoController *_servoController = nullptr;
  static inline ConfigHandler *_configHandler = nullptr;
  static inline RgbController *_rgbController = nullptr;

  static inline std::vector<uint8_t> _rx_buf;
  static inline uint8_t _rx_type = 0;
  static inline uint16_t _rx_total = 0;
  static inline uint16_t _rx_received = 0;

  static const struct ble_gatt_svc_def gatt_svcs[];

  static size_t maxChunkPayload() {
    uint16_t mtu = ble_att_mtu(conn_handle);
    // ATT notify overhead: 3 bytes (opcode + handle)
    if (mtu <= 3 + sizeof(BLEMessageHeader)) {
      return 20;
    }
    return mtu - 3 - sizeof(BLEMessageHeader);
  }

  static void ble_host_task(void *param) {
    ESP_LOGI(TAG, "BLE Host Task Started");
    nimble_port_run();
    nimble_port_freertos_deinit();
  }

  static void ble_app_on_reset(int reason) {
    ESP_LOGE(TAG, "Resetting state; reason=%d", reason);
  }

  static void ble_app_on_sync(void) {
    int rc = ble_hs_util_ensure_addr(0);
    if (rc != 0) {
      ESP_LOGE(TAG, "Failed to ensure address: %d", rc);
      return;
    }

    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0) {
      ESP_LOGE(TAG, "Failed to infer address type: %d", rc);
      return;
    }

    ble_app_advertise();
  }

  static void ble_app_advertise(void) {
    struct ble_gap_adv_params adv_params;
    struct ble_hs_adv_fields fields;
    struct ble_hs_adv_fields rsp_fields;
    const char *name;
    int rc;

    memset(&fields, 0, sizeof fields);
    memset(&rsp_fields, 0, sizeof rsp_fields);

    name = ble_svc_gap_device_name();
    fields.name = (uint8_t *)name;
    fields.name_len = strlen(name);
    fields.name_is_complete = 1;
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    rsp_fields.uuids128 = &NUS_SVC_UUID;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;

    rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
      ESP_LOGE(TAG, "Error setting adv fields: %d", rc);
      return;
    }

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
      ESP_LOGE(TAG, "Error setting scan rsp fields: %d", rc);
      return;
    }

    memset(&adv_params, 0, sizeof adv_params);
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;

    rc = ble_gap_adv_start(own_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
    if (rc != 0) {
      ESP_LOGE(TAG, "Error enabling advertisement: %d", rc);
      return;
    }
    ESP_LOGI(TAG, "Advertising started");
  }

  static int ble_gap_event(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
      case BLE_GAP_EVENT_CONNECT:
        ESP_LOGI(TAG, "Connection %s; status=%d", event->connect.status == 0 ? "established" : "failed",
                 event->connect.status);
        if (event->connect.status == 0) {
          conn_handle = event->connect.conn_handle;
          resetReassembly();
        } else {
          ble_app_advertise();
        }
        return 0;

      case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
        conn_handle = BLE_HS_CONN_HANDLE_NONE;
        resetReassembly();
        ble_app_advertise();
        return 0;

      case BLE_GAP_EVENT_CONN_UPDATE:
        ESP_LOGI(TAG, "Connection updated");
        return 0;

      case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGI(TAG, "Advertise complete; reason=%d", event->adv_complete.reason);
        ble_app_advertise();
        return 0;

      case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update event; conn_handle=%d mtu=%d", event->mtu.conn_handle, event->mtu.value);
        return 0;
    }
    return 0;
  }

  static void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    char buf[BLE_UUID_STR_LEN];
    switch (ctxt->op) {
      case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "Registered service %s with handle=%d", ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;
      case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "Registered characteristic %s with def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf), ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
      case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGI(TAG, "Registered descriptor %s with handle=%d", ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;
    }
  }

  static int nus_access_cb(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
      uint16_t om_len = OS_MBUF_PKTLEN(ctxt->om);
      std::vector<uint8_t> data(om_len);
      int rc = ble_hs_mbuf_to_flat(ctxt->om, data.data(), om_len, nullptr);
      if (rc != 0) {
        ESP_LOGE(TAG, "mbuf flatten failed: %d", rc);
        return BLE_ATT_ERR_UNLIKELY;
      }

      if (om_len >= sizeof(BLEMessageHeader)) {
        auto *header = reinterpret_cast<BLEMessageHeader *>(data.data());
        ESP_LOGI(TAG, "RX: type=%s (0x%02X), size=%u total=%u offset=%u",
                 getMessageTypeName(static_cast<MessageType>(header->type)), header->type, header->size, header->total,
                 header->offset);
      }

      handleRxPacket(data.data(), data.size());
      return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
  }

  static void resetReassembly() {
    _rx_buf.clear();
    _rx_type = 0;
    _rx_total = 0;
    _rx_received = 0;
  }

  static void handleRxPacket(const uint8_t *data, size_t len) {
    if (len < sizeof(BLEMessageHeader)) {
      ESP_LOGW(TAG, "Received message too short: %zu bytes", len);
      return;
    }

    auto *header = reinterpret_cast<const BLEMessageHeader *>(data);
    const uint8_t *payload = data + sizeof(BLEMessageHeader);
    size_t payload_len = len - sizeof(BLEMessageHeader);

    if (payload_len != header->size) {
      ESP_LOGW(TAG, "Chunk length mismatch: got %zu expected %u", payload_len, header->size);
      resetReassembly();
      return;
    }

    // Empty / request-only messages
    if (header->total == 0 && header->size == 0) {
      dispatchMessage(header->type, nullptr, 0);
      return;
    }

    // Single fragment
    if (header->offset == 0 && header->size == header->total) {
      dispatchMessage(header->type, payload, header->size);
      return;
    }

    // Multi-fragment reassembly
    if (header->offset == 0) {
      _rx_type = header->type;
      _rx_total = header->total;
      _rx_received = 0;
      _rx_buf.assign(header->total, 0);
    } else if (header->type != _rx_type || header->total != _rx_total || _rx_buf.empty()) {
      ESP_LOGW(TAG, "Fragment out of sequence, resetting");
      resetReassembly();
      return;
    }

    if (static_cast<size_t>(header->offset) + header->size > _rx_total) {
      ESP_LOGW(TAG, "Fragment overflow");
      resetReassembly();
      return;
    }

    memcpy(_rx_buf.data() + header->offset, payload, header->size);
    _rx_received = static_cast<uint16_t>(std::max<uint32_t>(_rx_received, header->offset + header->size));

    if (_rx_received >= _rx_total) {
      dispatchMessage(_rx_type, _rx_buf.data(), _rx_total);
      resetReassembly();
    }
  }

  static void dispatchMessage(uint8_t type, const uint8_t *payload, size_t payload_len) {
    switch (type) {
      case CONFIG_GET:
        ESP_LOGI(TAG, "Handling CONFIG_GET request");
        sendConfig();
        break;
      case CONFIG_SET:
        ESP_LOGI(TAG, "Handling CONFIG_SET (%zu bytes JSON)", payload_len);
        handleConfigSet(payload, payload_len);
        break;
      case STATE_GET:
        ESP_LOGI(TAG, "Handling STATE_GET request");
        sendState();
        break;
      case STATE_SET:
        ESP_LOGI(TAG, "Handling STATE_SET request");
        handleStateSet(payload, payload_len);
        break;
      case PING:
        ESP_LOGI(TAG, "Handling PING");
        sendPong();
        break;
      case GET_SHIFTING_SERVO:
        ESP_LOGI(TAG, "Handling GET_SHIFTING_SERVO");
        sendShiftingServoValue();
        break;
      case SET_SHIFTING_SERVO:
        ESP_LOGI(TAG, "Handling SET_SHIFTING_SERVO");
        handleSetShiftingServo(payload, payload_len);
        break;
      case GET_STEERING_SERVO:
        ESP_LOGI(TAG, "Handling GET_STEERING_SERVO");
        sendSteeringServo();
        break;
      case SET_STEERING_SERVO:
        ESP_LOGI(TAG, "Handling SET_STEERING_SERVO");
        handleSetSteeringServo(payload, payload_len);
        break;
      case GET_RGB_COLOUR:
        ESP_LOGI(TAG, "Handling GET_RGB_COLOUR");
        sendRgbColour();
        break;
      case SET_RGB_COLOUR:
        ESP_LOGI(TAG, "Handling SET_RGB_COLOUR");
        handleSetRgbColour(payload, payload_len);
        break;
      case GET_RGB_BRIGHTNESS:
        ESP_LOGI(TAG, "Handling GET_RGB_BRIGHTNESS");
        sendRgbBrightness();
        break;
      case SET_RGB_BRIGHTNESS:
        ESP_LOGI(TAG, "Handling SET_RGB_BRIGHTNESS");
        handleSetRgbBrightness(payload, payload_len);
        break;
      case SAVE_CONFIG:
        ESP_LOGI(TAG, "Handling SAVE_CONFIG");
        handleSaveConfig();
        break;
      case REBOOT:
        ESP_LOGI(TAG, "Handling REBOOT");
        handleReboot();
        break;
      default:
        ESP_LOGW(TAG, "Unknown message type: 0x%02X", type);
        break;
    }
  }

  static void sendError(const char *message) {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
      return;

    ESP_LOGW(TAG, "Sending error message: %s", message);
    sendMessage(CONFIG_ERROR, message, strlen(message));
  }

  static void handleConfigSet(const uint8_t *payload, size_t payload_len) {
    if (!_configHandler || !_config) {
      sendError("Config not ready");
      return;
    }
    if (payload_len == 0 || payload == nullptr) {
      sendError("Empty config");
      return;
    }

    std::string json(reinterpret_cast<const char *>(payload), payload_len);
    Config backup = *_config;

    if (!_configHandler->fromJson(json)) {
      ESP_LOGW(TAG, "Config JSON parse failed");
      sendError("Invalid JSON");
      return;
    }

    if (!validateConfig(_config)) {
      ESP_LOGW(TAG, "Config validation failed");
      *_config = backup;
      sendError("Invalid config data");
      return;
    }

    // Apply config RGB to the live strip (config is source of truth after SET)
    if (_rgbController && _configHandler->hasRgbLed()) {
      _rgbController->setBrightness(_configHandler->getRGBBrightness());
      _rgbController->setColour(0, _configHandler->getRGBColour());
      _rgbController->show();
    }

    ESP_LOGI(TAG, "Config JSON applied (%zu bytes)", payload_len);
    sendMessage(CONFIG_SET, nullptr, 0);
  }

  static void handleStateSet(const uint8_t *payload, size_t payload_len) {
    if (payload_len != sizeof(State)) {
      ESP_LOGW(TAG, "Invalid state size: %zu (expected %zu)", payload_len, sizeof(State));
      sendError("Invalid state size");
      return;
    }

    ESP_LOGI(TAG, "Received state: %zu bytes", payload_len);
    if (_state) {
      memcpy(_state, payload, sizeof(State));
    }
    sendMessage(STATE_SET, nullptr, 0);
  }

  static bool validateConfig(const Config *config) {
    if (!config)
      return false;
    if (config->escConfig.min >= config->escConfig.max)
      return false;
    if (strnlen(config->model_name, CONFIG_NAME_LEN) >= CONFIG_NAME_LEN)
      return false;
    return true;
  }

  static void sendPong() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE)
      return;
    ESP_LOGI(TAG, "Sending PONG response");
    sendMessage(PONG, nullptr, 0);
  }

  static void sendShiftingServoValue() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !_state)
      return;
    uint16_t value = _servoController->get_shifting_us();
    ESP_LOGI(TAG, "Sending shifting servo value: %u µs", value);
    sendMessage(GET_SHIFTING_SERVO, &value, sizeof(value));
  }

  static void handleSetShiftingServo(const uint8_t *payload, size_t payload_len) {
    if (payload_len != sizeof(uint16_t)) {
      ESP_LOGW(TAG, "Invalid shifting servo value size: %zu (expected %zu)", payload_len, sizeof(uint16_t));
      sendError("Invalid servo value size");
      return;
    }
    uint16_t value;
    memcpy(&value, payload, sizeof(uint16_t));
    ESP_LOGI(TAG, "Setting shifting servo to: %u µs", value);
    _servoController->set_shifting_us(value);
    sendMessage(SET_SHIFTING_SERVO, nullptr, 0);
  }

  static void sendSteeringServo() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !_state)
      return;
    uint16_t value = _servoController->get_steering_us();
    ESP_LOGI(TAG, "Sending steering servo value: %u µs", value);
    sendMessage(GET_STEERING_SERVO, &value, sizeof(value));
  }

  static void handleSetSteeringServo(const uint8_t *payload, size_t payload_len) {
    if (payload_len != sizeof(uint16_t)) {
      ESP_LOGW(TAG, "Invalid steering servo value size: %zu (expected %zu)", payload_len, sizeof(uint16_t));
      sendError("Invalid servo value size");
      return;
    }

    uint16_t value;
    memcpy(&value, payload, sizeof(uint16_t));
    ESP_LOGI(TAG, "Setting steering servo to: %u µs", value);
    _servoController->set_steering_us(value);
    sendMessage(SET_STEERING_SERVO, nullptr, 0);
  }

  static bool ensureRgb() {
    if (!_rgbController || !_configHandler) {
      sendError("RGB not available");
      return false;
    }
    if (!_configHandler->hasRgbLed()) {
      sendError("RGB disabled in config");
      return false;
    }
    return true;
  }

  // Live tester control — does NOT mutate saved/config RGB values
  static void sendRgbColour() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !ensureRgb()) {
      return;
    }
    uint32_t colour = _rgbController->getColourCode(0);
    ESP_LOGI(TAG, "Sending live RGB colour: 0x%06lX", (unsigned long)(colour & 0xFFFFFF));
    sendMessage(GET_RGB_COLOUR, &colour, sizeof(colour));
  }

  static void handleSetRgbColour(const uint8_t *payload, size_t payload_len) {
    if (!ensureRgb()) {
      return;
    }
    if (payload_len != sizeof(uint32_t)) {
      ESP_LOGW(TAG, "Invalid RGB colour size: %zu (expected %zu)", payload_len, sizeof(uint32_t));
      sendError("Invalid RGB colour size");
      return;
    }
    uint32_t colour = 0;
    memcpy(&colour, payload, sizeof(colour));
    colour &= 0x00FFFFFFu;
    ESP_LOGI(TAG, "Live RGB colour: 0x%06lX", (unsigned long)colour);
    _rgbController->setColour(0, colour);
    _rgbController->show();
    sendMessage(SET_RGB_COLOUR, nullptr, 0);
  }

  static void sendRgbBrightness() {
    if (conn_handle == BLE_HS_CONN_HANDLE_NONE || !ensureRgb()) {
      return;
    }
    uint8_t brightness = _rgbController->getBrightness();
    ESP_LOGI(TAG, "Sending live RGB brightness: %u", brightness);
    sendMessage(GET_RGB_BRIGHTNESS, &brightness, sizeof(brightness));
  }

  static void handleSetRgbBrightness(const uint8_t *payload, size_t payload_len) {
    if (!ensureRgb()) {
      return;
    }
    if (payload_len != sizeof(uint8_t)) {
      ESP_LOGW(TAG, "Invalid RGB brightness size: %zu (expected %zu)", payload_len, sizeof(uint8_t));
      sendError("Invalid RGB brightness size");
      return;
    }
    uint8_t brightness = payload[0];
    ESP_LOGI(TAG, "Live RGB brightness: %u", brightness);
    _rgbController->setBrightness(brightness);
    _rgbController->show();
    sendMessage(SET_RGB_BRIGHTNESS, nullptr, 0);
  }

  static void handleSaveConfig() {
    ESP_LOGI(TAG, "Save config requested");
    if (!_configHandler->save()) {
      sendError("Save failed");
      return;
    }
    sendMessage(SAVE_CONFIG, nullptr, 0);
    ESP_LOGI(TAG, "Config saved successfully");
  }

  static void handleReboot() {
    ESP_LOGI(TAG, "Reboot requested via BLE");
    sendMessage(REBOOT, nullptr, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_LOGI(TAG, "Rebooting device...");
    esp_restart();
  }
};

const struct ble_gatt_svc_def BLEController::gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &NUS_SVC_UUID.u,
        .includes = NULL,
        .characteristics =
            (struct ble_gatt_chr_def[]){
                {
                    .uuid = &NUS_RX_UUID.u,
                    .access_cb = BLEController::nus_access_cb,
                    .arg = NULL,
                    .descriptors = NULL,
                    .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                    .min_key_size = 0,
                    .val_handle = NULL,
                    .cpfd = NULL,
                },
                {
                    .uuid = &NUS_TX_UUID.u,
                    .access_cb = BLEController::nus_access_cb,
                    .arg = NULL,
                    .descriptors = NULL,
                    .flags = BLE_GATT_CHR_F_NOTIFY,
                    .min_key_size = 0,
                    .val_handle = &BLEController::tx_handle,
                    .cpfd = NULL,
                },
                {
                    0,
                    NULL,
                    NULL,
                    NULL,
                    0,
                    0,
                    NULL,
                    NULL,
                },
            },
    },
    {
        0,
        NULL,
        NULL,
        NULL,
    },
};
