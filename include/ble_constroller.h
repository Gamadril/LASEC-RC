#pragma once

#include "config_handler.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "host/ble_hs.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include <functional>
#include <cstring>

// Message types for Config protocol
enum ConfigMessageType : uint8_t {
  CONFIG_GET_REQUEST = 0x01,
  CONFIG_GET_RESPONSE = 0x02,
  CONFIG_SET_REQUEST = 0x03,
  CONFIG_SET_RESPONSE = 0x04,
  CONFIG_CHUNK_DATA = 0x05,
  CONFIG_CHUNK_ACK = 0x06,
  CONFIG_ERROR = 0xFF
};

// Message header for all BLE messages
struct __attribute__((packed)) BLEMessageHeader {
  uint8_t type;
  uint8_t sequence;
  uint16_t total_size;
  uint16_t chunk_offset;
  uint16_t chunk_size;
};

class BLEController {
public:
  static inline const char *TAG = "BLE";
  static inline uint16_t tx_handle = 0;
  static inline uint8_t addr_type = 0;
  static inline uint16_t conn_handle = 0;

  // Callback for config operations
  static inline std::function<void(const Config *)> onConfigReceived = nullptr;
  static inline std::function<Config *()> onConfigRequested = nullptr;

  // Transfer state with proper management
  static inline uint8_t *rx_buffer = nullptr;
  static inline size_t rx_buffer_size = 0;
  static inline size_t rx_received = 0;
  static inline uint8_t rx_sequence = 0;
  static inline bool transfer_in_progress = false;
  static inline uint32_t last_chunk_time = 0;
  static inline uint32_t transfer_timeout_ms = 5000; // 5 second timeout

  // GATT service definitions
  static struct ble_gatt_svc_def gatt_svr_svcs[];

  static void init(const char *device_name = "ESP32-NUS") {
    esp_nimble_hci_init();
    nimble_port_init();

    ble_svc_gap_init();
    ble_svc_gatt_init();
    ble_svc_gap_device_name_set(device_name);

    ble_hs_cfg.sync_cb = onSync;
    ble_hs_cfg.gatts_register_cb = nullptr; // Use default

    nimble_port_freertos_init([](void *) {
      nimble_port_run();
      vTaskDelete(NULL);
    });
  }

  // Send raw data (legacy method)
  static void send(const uint8_t *data, size_t len) {
    if (conn_handle == 0)
      return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    ble_gatts_notify_custom(conn_handle, tx_handle, om);
  }

  // Send Config struct to Chrome browser with proper MTU handling
  static void sendConfig(const Config *config) {
    if (conn_handle == 0) {
      ESP_LOGE(TAG, "No BLE connection");
      return;
    }

    const uint8_t *data = reinterpret_cast<const uint8_t *>(config);
    size_t total_size = sizeof(Config);

    // Get negotiated MTU, fallback to minimum
    int mtu = ble_att_mtu(conn_handle);
    if (mtu <= 0) {
      mtu = 20; // BLE minimum MTU
      ESP_LOGW(TAG, "Using default MTU: %d", mtu);
    }

    const size_t max_chunk = mtu - sizeof(BLEMessageHeader) - 3; // Reserve 3 bytes for safety
    uint8_t sequence = 0;

    ESP_LOGI(TAG, "Sending Config struct, size: %d bytes, MTU: %d, max chunk: %d",
             total_size, mtu, max_chunk);

    if (max_chunk < 1) {
      ESP_LOGE(TAG, "MTU too small for config transfer: %d", mtu);
      sendError("MTU too small");
      return;
    }

    for (size_t offset = 0; offset < total_size; offset += max_chunk) {
      size_t chunk_size = std::min(max_chunk, total_size - offset);

      BLEMessageHeader header = {.type = CONFIG_CHUNK_DATA,
                                 .sequence = sequence++,
                                 .total_size = static_cast<uint16_t>(total_size),
                                 .chunk_offset = static_cast<uint16_t>(offset),
                                 .chunk_size = static_cast<uint16_t>(chunk_size)};

      // Create packet with proper size
      uint8_t packet[mtu];
      memcpy(packet, &header, sizeof(header));
      memcpy(packet + sizeof(header), data + offset, chunk_size);

      send(packet, sizeof(header) + chunk_size);

      // Small delay between chunks for Web Bluetooth
      vTaskDelay(pdMS_TO_TICKS(5)); // Reduced delay for better performance
    }

    ESP_LOGI(TAG, "Config sent in %d chunks", sequence);
  }

  // Request Config from application
  static void requestConfig() {
    if (onConfigRequested) {
      Config *config = onConfigRequested();
      if (config) {
        sendConfig(config);
      } else {
        sendError("Config not available");
      }
    } else {
      sendError("Config handler not set");
    }
  }

  // Cleanup transfer state
  static void cleanupTransfer() {
    if (rx_buffer) {
      free(rx_buffer);
      rx_buffer = nullptr;
    }
    rx_buffer_size = 0;
    rx_received = 0;
    rx_sequence = 0;
    transfer_in_progress = false;
    last_chunk_time = 0;
  }

  // Validate received config structure
  static bool validateReceivedConfig(uint8_t *buffer) {
    if (!buffer) {
      ESP_LOGE(TAG, "Null buffer in config validation");
      return false;
    }

    Config *config = reinterpret_cast<Config *>(buffer);

    // Basic structure validation
    if (config->escConfig.min >= config->escConfig.max) {
      ESP_LOGE(TAG, "Invalid ESC config: min >= max");
      return false;
    }

    if (config->steeringServo.min >= config->steeringServo.max) {
      ESP_LOGE(TAG, "Invalid steering servo config: min >= max");
      return false;
    }

    if (config->shiftingServo.min >= config->shiftingServo.max) {
      ESP_LOGE(TAG, "Invalid shifting servo config: min >= max");
      return false;
    }

    // Validate string lengths (prevent buffer overflow)
    if (strlen(config->model_name) >= CONFIG_NAME_LEN) {
      ESP_LOGE(TAG, "Model name too long");
      return false;
    }

    // Validate volume levels
    if (config->volume > 100) {
      ESP_LOGW(TAG, "Volume > 100%%: %d", config->volume);
      // Allow but warn
    }

    // Validate servo frequencies (uint8_t range: 0-255)
    if (config->steeringServo.frequency < 50) {
      ESP_LOGE(TAG, "Invalid steering servo frequency: %d", config->steeringServo.frequency);
      return false;
    }

    if (config->shiftingServo.frequency < 50) {
      ESP_LOGE(TAG, "Invalid shifting servo frequency: %d", config->shiftingServo.frequency);
      return false;
    }

    ESP_LOGI(TAG, "Config validation passed");
    return true;
  }

  // Send error message
  static void sendError(const char *message) {
    if (conn_handle == 0)
      return;

    BLEMessageHeader header = {.type = CONFIG_ERROR,
                               .sequence = 0,
                               .total_size = static_cast<uint16_t>(strlen(message)),
                               .chunk_offset = 0,
                               .chunk_size = static_cast<uint16_t>(strlen(message))};

    // Fixed buffer size with proper bounds checking
    constexpr size_t PACKET_BUFFER_SIZE = 20;
    uint8_t packet[PACKET_BUFFER_SIZE];
    memcpy(packet, &header, sizeof(header));

    size_t msg_len = std::min(strlen(message), PACKET_BUFFER_SIZE - sizeof(header) - 1); // -1 for null terminator
    memcpy(packet + sizeof(header), message, msg_len);
    packet[sizeof(header) + msg_len] = '\0'; // Ensure null termination

    send(packet, sizeof(header) + msg_len);
  }

private:
  static inline ble_uuid128_t nus_svc_uuid =
      BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24,
                       0xdc, 0xca, 0x9e);

  static inline ble_uuid128_t nus_rx_uuid =
      BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24,
                       0xdc, 0xca, 0x9e);

  static inline ble_uuid128_t nus_tx_uuid =
      BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24,
                       0xdc, 0xca, 0x9e);

  // RX callback - handles incoming Config data
public:
  static int rxCallback(uint16_t conn_handle_param, uint16_t attr_handle,
                        struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
      uint8_t *data = ctxt->om->om_data;
      size_t len = ctxt->om->om_len;

      ESP_LOGI(TAG, "RX %d bytes", len);

      if (len < sizeof(BLEMessageHeader)) {
        ESP_LOGW(TAG, "Packet too small for header");
        return 0;
      }

      BLEMessageHeader *header = reinterpret_cast<BLEMessageHeader *>(data);
      uint8_t *payload = data + sizeof(BLEMessageHeader);
      size_t payload_len = len - sizeof(BLEMessageHeader);

      switch (header->type) {
      case CONFIG_GET_REQUEST:
        ESP_LOGI(TAG, "Config GET request");
        requestConfig();
        break;

      case CONFIG_SET_REQUEST:
        ESP_LOGI(TAG, "Config SET request, total size: %d", header->total_size);
        handleConfigSetStart(header);
        break;

      case CONFIG_CHUNK_DATA:
        ESP_LOGI(TAG, "Config chunk: seq=%d, offset=%d, size=%d", header->sequence,
                 header->chunk_offset, header->chunk_size);
        handleConfigChunk(header, payload, payload_len);
        break;

      default:
        ESP_LOGW(TAG, "Unknown message type: 0x%02X", header->type);
        break;
      }
    }
    return 0;
  }

  // Handle start of Config SET operation
  static void handleConfigSetStart(BLEMessageHeader *header) {
    // Validate config size
    if (header->total_size != sizeof(Config)) {
      ESP_LOGE(TAG, "Invalid config size: %d, expected: %d", header->total_size, sizeof(Config));
      sendError("Invalid config size");
      return;
    }

    // Check for buffer size limits to prevent DoS
    if (header->total_size > 8192) { // 8KB limit
      ESP_LOGE(TAG, "Config size too large: %d bytes", header->total_size);
      sendError("Config too large");
      return;
    }

    // Cleanup any existing transfer
    if (transfer_in_progress) {
      cleanupTransfer();
    }

    // Allocate receive buffer
    rx_buffer = static_cast<uint8_t *>(malloc(header->total_size));
    if (!rx_buffer) {
      ESP_LOGE(TAG, "Failed to allocate RX buffer of %d bytes", header->total_size);
      sendError("Memory allocation failed");
      return;
    }

    rx_buffer_size = header->total_size;
    rx_received = 0;
    rx_sequence = 0;
    transfer_in_progress = true;
    last_chunk_time = esp_timer_get_time() / 1000ULL;

    ESP_LOGI(TAG, "Ready to receive config, size: %d bytes", rx_buffer_size);
  }

  // Handle Config data chunks
  static void handleConfigChunk(BLEMessageHeader *header, uint8_t *payload, size_t payload_len) {
    // Check if transfer is in progress
    if (!transfer_in_progress || !rx_buffer) {
      ESP_LOGE(TAG, "No transfer in progress");
      sendError("No transfer in progress");
      return;
    }

    // Check for timeout
    uint32_t current_time = esp_timer_get_time() / 1000ULL;
    if (current_time - last_chunk_time > transfer_timeout_ms) {
      ESP_LOGE(TAG, "Transfer timeout after %dms", transfer_timeout_ms);
      cleanupTransfer();
      sendError("Transfer timeout");
      return;
    }
    last_chunk_time = current_time;

    // Validate sequence number
    if (header->sequence != rx_sequence) {
      ESP_LOGW(TAG, "Sequence mismatch: got %d, expected %d", header->sequence, rx_sequence);
      // Don't fail immediately, but log it
    }

    // Validate chunk bounds
    if (header->chunk_offset + header->chunk_size > rx_buffer_size) {
      ESP_LOGE(TAG, "Chunk exceeds buffer size: offset=%d, size=%d, buffer=%d",
               header->chunk_offset, header->chunk_size, rx_buffer_size);
      cleanupTransfer();
      sendError("Chunk out of bounds");
      return;
    }

    // Validate payload size
    if (payload_len != header->chunk_size) {
      ESP_LOGE(TAG, "Payload size mismatch: got %d, expected %d", payload_len, header->chunk_size);
      sendError("Payload size error");
      return;
    }

    // Validate payload data exists
    if (!payload && payload_len > 0) {
      ESP_LOGE(TAG, "Null payload with non-zero length");
      sendError("Invalid payload");
      return;
    }

    // Copy chunk data safely
    memcpy(rx_buffer + header->chunk_offset, payload, payload_len);
    rx_received += payload_len;
    rx_sequence++;

    ESP_LOGI(TAG, "Received chunk: seq=%d, offset=%d, size=%d, total=%d/%d",
             header->sequence, header->chunk_offset, payload_len, rx_received, rx_buffer_size);

    // Check if transfer complete
    if (rx_received >= rx_buffer_size) {
      ESP_LOGI(TAG, "Config transfer complete: %d bytes", rx_received);

      // Validate complete config structure
      if (validateReceivedConfig(rx_buffer)) {
        // Parse received config
        Config *received_config = reinterpret_cast<Config *>(rx_buffer);

        // Notify application
        if (onConfigReceived) {
          onConfigReceived(received_config);
        }

        // Send acknowledgment
        BLEMessageHeader ack_header = {.type = CONFIG_SET_RESPONSE,
                                       .sequence = 0,
                                       .total_size = 0,
                                       .chunk_offset = 0,
                                       .chunk_size = 0};
        send(reinterpret_cast<uint8_t *>(&ack_header), sizeof(ack_header));
      } else {
        ESP_LOGE(TAG, "Received config validation failed");
        sendError("Invalid config data");
      }

      // Cleanup
      cleanupTransfer();
    }
  }

  static void onSync(void) {
    ble_hs_id_infer_auto(0, &addr_type);

    // Register Nordic UART Service using the simpler approach
    int rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to count GATT configuration: %d", rc);
        return;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        ESP_LOGE(TAG, "Failed to add GATT services: %d", rc);
        return;
    }

    ESP_LOGI(TAG, "Nordic UART Service registered successfully");

    startAdvertising();
  }

  // Handle connection events
  static int onGapEvent(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
      case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
          conn_handle = event->connect.conn_handle;
          ESP_LOGI(TAG, "Connected, handle: %d", conn_handle);
        } else {
          ESP_LOGE(TAG, "Connection failed, status: %d", event->connect.status);
        }
        break;

      case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "Disconnected, handle: %d, reason: %d", event->disconnect.conn.conn_handle, event->disconnect.reason);

        // Cleanup on disconnect
        if (transfer_in_progress) {
          ESP_LOGW(TAG, "Transfer interrupted by disconnection");
          cleanupTransfer();
        }

        conn_handle = 0;
        break;

      case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU updated: %d", event->mtu.value);
        break;

      default:
        break;
    }
    return 0;
  }

  static void startAdvertising() {
    // Set up advertising data (31 bytes max)
    struct ble_hs_adv_fields fields{};
    memset(&fields, 0, sizeof(fields));

    // Set up gap event listener
    struct ble_gap_event_listener event_listener;
    memset(&event_listener, 0, sizeof(event_listener));
    
    // Device name
    const char* device_name = "LASEC-RC";
    fields.name = reinterpret_cast<const uint8_t*>(device_name);
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    
    // Flags
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    
    // Set advertising data
    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
      ESP_LOGE(TAG, "Failed to set advertising data: %d", rc);
    }

    // Set up scan response data with service UUID (31 bytes max)
    struct ble_hs_adv_fields rsp_fields{};
    memset(&rsp_fields, 0, sizeof(rsp_fields));
    
    // Add Nordic UART Service UUID in scan response
    static ble_uuid128_t service_uuid = nus_svc_uuid;
    rsp_fields.uuids128 = &service_uuid;
    rsp_fields.num_uuids128 = 1;
    rsp_fields.uuids128_is_complete = 1;
    
    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
      ESP_LOGE(TAG, "Failed to set scan response data: %d", rc);
    }
    
    // Set up advertising parameters
    struct ble_gap_adv_params adv_params{};
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    adv_params.itvl_min = BLE_GAP_ADV_FAST_INTERVAL1_MIN;
    adv_params.itvl_max = BLE_GAP_ADV_FAST_INTERVAL1_MAX;
    adv_params.channel_map = 0;
    adv_params.filter_policy = 0;
    adv_params.high_duty_cycle = 0;

    // Register gap event handler
    rc = ble_gap_event_listener_register(&event_listener, onGapEvent, nullptr);
    if (rc != 0) {
      ESP_LOGE(TAG, "Failed to register gap event listener: %d", rc);
    }

    // Start advertising
    rc = ble_gap_adv_start(addr_type, nullptr, BLE_HS_FOREVER, &adv_params, nullptr, nullptr);
    if (rc != 0) {
      ESP_LOGE(TAG, "Failed to start advertising: %d", rc);
      return;
    }

    ESP_LOGI(TAG, "Started advertising as %s with NUS service", device_name);
  }
};

// Define static UUIDs to avoid rvalue issues
static ble_uuid128_t static_nus_svc_uuid = BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x01, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e);
static ble_uuid128_t static_nus_rx_uuid = BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x02, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e);
static ble_uuid128_t static_nus_tx_uuid = BLE_UUID128_INIT(0x6e, 0x40, 0x00, 0x03, 0xb5, 0xa3, 0xf3, 0x93, 0xe0, 0xa9, 0xe5, 0x0e, 0x24, 0xdc, 0xca, 0x9e);

// Define characteristics array separately with all fields
static struct ble_gatt_chr_def nus_characteristics[] = {
    {
        .uuid = &static_nus_rx_uuid.u,
        .access_cb = BLEController::rxCallback,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL
    },
    {
        .uuid = &static_nus_tx_uuid.u,
        .access_cb = NULL,
        .arg = NULL,
        .descriptors = NULL,
        .flags = BLE_GATT_CHR_F_NOTIFY,
        .min_key_size = 0,
        .val_handle = &BLEController::tx_handle,
        .cpfd = NULL
    },
    {
        .uuid = NULL,
        .access_cb = NULL,
        .arg = NULL,
        .descriptors = NULL,
        .flags = 0,
        .min_key_size = 0,
        .val_handle = NULL,
        .cpfd = NULL
    } // Null terminator with all fields
};

// Define the GATT service structure with all fields
struct ble_gatt_svc_def BLEController::gatt_svr_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &static_nus_svc_uuid.u,
        .includes = NULL,
        .characteristics = nus_characteristics
    },
    {
        .type = 0,
        .uuid = NULL,
        .includes = NULL,
        .characteristics = NULL
    } // Null terminator with all fields
};
