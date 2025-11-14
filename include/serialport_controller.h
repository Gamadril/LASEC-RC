#pragma once

#include "config_handler.h"
#include "driver/uart.h"
#include "esp_log.h"

extern unsigned long micros();
extern unsigned long millis();

#define MSG_PING 0x01
#define MSG_LOG 0x02
#define MSG_CONFIG_GET 0x03
#define MSG_CONFIG_DATA 0x04
#define MSG_CONFIG_SET 0x05
#define MSG_RGB_BRIGHTNESS_GET 0x06
#define MSG_RGB_BRIGHTNESS_DATA 0x07
#define MSG_RGB_BRIGHTNESS_SET 0x08
#define MSG_RGB_COLOUR_GET 0x09
#define MSG_RGB_COLOUR_DATA 0x0A
#define MSG_RGB_COLOUR_SET 0x0B
#define MSG_VEHICLE_SET 0x0C
#define MSG_SERVO_STEERING_GET 0x0D
#define MSG_SERVO_STEERING_DATA 0x0E
#define MSG_SERVO_STEERING_SET 0x0F
#define MSG_SERVO_SHIFTING_GET 0x10
#define MSG_SERVO_SHIFTING_DATA 0x11
#define MSG_SERVO_SHIFTING_SET 0x12
#define MSG_STATE_GET 0x13
#define MSG_STATE_DATA 0x14
#define MSG_ESC_GET 0x15
#define MSG_ESC_DATA 0x16
#define MSG_ESC_SET 0x17
#define MSG_CONFIG_SAVE 0xE0

struct SerialMessage {
  uint8_t type;
  uint16_t payload_size;
  uint8_t *payload;
};

class SerialPortController {
public:
  typedef void (*serial_msg_callback_t)(SerialMessage &msg);

  SerialPortController() {
  }

  void init(int baud, serial_msg_callback_t serial_message_callback = NULL) {
    _msg_callback = serial_message_callback;

    uart_config_t uart_config = {};
    uart_config.baud_rate = baud;
    uart_config.data_bits = UART_DATA_8_BITS;
    uart_config.parity = UART_PARITY_DISABLE;
    uart_config.stop_bits = UART_STOP_BITS_1;
    uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_0, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_0, &uart_config));

    _startTime = millis();
    _serialEnabled = true;
    _hasReceivedData = false;
  }

  void deinit() {
    _msg_callback = NULL;
  }

  void processMessages() {
    // Check for initial data within first 5 seconds
    if (_serialEnabled && !_hasReceivedData) {
      size_t available = 0;
      uart_get_buffered_data_len(UART_NUM_0, &available);
      if (available > 0) {
        _hasReceivedData = true; // We got data, keep serial enabled
      } else if (millis() - _startTime > 5000) {
        // No data received within 5 seconds, disable serial
        // uart_driver_delete(UART_NUM_0);
        //_serialEnabled = false;
        return;
      }
    }

    // Only process messages if serial is still enabled
    if (!_serialEnabled)
      return;

    size_t available = 0;
    uart_get_buffered_data_len(UART_NUM_0, &available);

    while (available > 0) {
      if (_parseState == WAIT_HEADER) {
        // Read header bytes
        while (_headerBytesRead < 3 && available > 0) {
          uint8_t byte;
          int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(1));
          if (len > 0) {
            _headerBuffer[_headerBytesRead++] = byte;
            available--;
          } else {
            break;
          }
        }
        if (_headerBytesRead < 3) {
          // Not enough header bytes yet
          return;
        }
        // Got full header
        _expectedPayloadSize = (_headerBuffer[2] << 8) | _headerBuffer[1];
        if (_expectedPayloadSize > MAX_PAYLOAD_SIZE) {
          // Invalid, resync: shift header by one and try again
          _headerBuffer[0] = _headerBuffer[1];
          _headerBuffer[1] = _headerBuffer[2];
          _headerBytesRead = 2;
          return;
        }
        _payloadBytesRead = 0;
        _parseState = (_expectedPayloadSize == 0) ? WAIT_HEADER : WAIT_PAYLOAD;
        if (_expectedPayloadSize == 0) {
          // No payload, process immediately
          SerialMessage message;
          message.type = _headerBuffer[0];
          message.payload_size = 0;
          message.payload = NULL;
          if (_msg_callback != NULL) {
            _msg_callback(message);
          }
          _headerBytesRead = 0;
        }
      }
      if (_parseState == WAIT_PAYLOAD && _expectedPayloadSize > 0) {
        // Read payload bytes
        while (_payloadBytesRead < _expectedPayloadSize && available > 0) {
          uint8_t byte;
          int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(1));
          if (len > 0) {
            _payloadBuffer[_payloadBytesRead++] = byte;
            available--;
          } else {
            break;
          }
        }
        if (_payloadBytesRead < _expectedPayloadSize) {
          // Not enough payload yet
          return;
        }
        // Got full payload
        SerialMessage message;
        message.type = _headerBuffer[0];
        message.payload_size = _expectedPayloadSize;
        message.payload = _payloadBuffer;
        if (_msg_callback != NULL) {
          _msg_callback(message);
        }
        // Reset for next message
        _headerBytesRead = 0;
        _payloadBytesRead = 0;
        _parseState = WAIT_HEADER;
      }

      // Update available count
      uart_get_buffered_data_len(UART_NUM_0, &available);
    }
  }

  void send(SerialMessage &message) {
    size_t total_len = message.payload_size + 3;
    if (total_len > MAX_SEND_BUFFER_SIZE) {
      return;
    }

    _sendBuffer[0] = message.type;
    _sendBuffer[1] = message.payload_size & 0xFF;
    _sendBuffer[2] = message.payload_size >> 8;

    if (message.payload_size > 0 && message.payload != NULL) {
      memcpy(_sendBuffer + 3, message.payload, message.payload_size);
    }

    uart_write_bytes(UART_NUM_0, _sendBuffer, total_len);
    uart_wait_tx_done(UART_NUM_0, pdMS_TO_TICKS(100));
  }

private:
  static const size_t MAX_PAYLOAD_SIZE = 512;
  static const size_t MAX_SEND_BUFFER_SIZE = 512 + 3;

  enum MessageParseState { WAIT_HEADER, WAIT_PAYLOAD } _parseState = WAIT_HEADER;
  uint8_t _headerBuffer[3];
  size_t _headerBytesRead = 0;
  uint16_t _expectedPayloadSize = 0;
  size_t _payloadBytesRead = 0;

  serial_msg_callback_t _msg_callback = NULL;
  unsigned long _startTime;
  bool _serialEnabled;
  bool _hasReceivedData;

  uint8_t _payloadBuffer[MAX_PAYLOAD_SIZE];
  uint8_t _sendBuffer[MAX_SEND_BUFFER_SIZE];
};
