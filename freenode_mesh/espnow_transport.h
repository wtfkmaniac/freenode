/*
 * espnow_transport.h — ESP-NOW Transport Plugin for ESP32
 * Version: 0.3.0 — March 2026
 *
 * ИЗМЕНЕНИЯ v0.3:
 *   - Совместимость с новым FNPacket (magic + version + flags)
 *   - sendSize считается корректно через FN_HEADER_SIZE
 *   - Обратная совместимость с v0.2 пакетами
 */

#ifndef ESPNOW_TRANSPORT_H
#define ESPNOW_TRANSPORT_H

#include "transport.h"
#include <esp_now.h>
#include <WiFi.h>

#define RX_BUF_SIZE 16

static FNPacket rxBuf[RX_BUF_SIZE];
static volatile uint8_t rxHead = 0;
static volatile uint8_t rxTail = 0;

static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  // silent
}

static void onDataRecv(const esp_now_recv_info_t *info,
                       const uint8_t *data, int len) {
  uint8_t next = (rxHead + 1) % RX_BUF_SIZE;
  if (next == rxTail) return;

  if (len >= (int)FN_HEADER_SIZE) {
    memset(&rxBuf[rxHead], 0, sizeof(FNPacket));
    memcpy(&rxBuf[rxHead], data, min((size_t)len, sizeof(FNPacket)));
    // Обратная совместимость v0.2
    if (rxBuf[rxHead].magic != FN_MAGIC) {
      rxBuf[rxHead].magic   = FN_MAGIC;
      rxBuf[rxHead].version = 0x02;
    }
    rxHead = next;
  }
}

class EspNowTransport : public Transport {
  bool _ready = false;

public:
  bool init() override {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() != ESP_OK) {
      Serial.println("[ESP-NOW] Init FAILED");
      return false;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t broadcastPeer;
    memset(&broadcastPeer, 0, sizeof(broadcastPeer));
    memset(broadcastPeer.peer_addr, 0xFF, 6);
    broadcastPeer.channel = 0;
    broadcastPeer.encrypt = false;
    broadcastPeer.ifidx = WIFI_IF_STA;

    if (esp_now_add_peer(&broadcastPeer) != ESP_OK) {
      Serial.println("[ESP-NOW] Failed to add broadcast peer");
      return false;
    }

    _ready = true;
    Serial.println("[ESP-NOW] Init OK (v0.3)");
    return true;
  }

  bool send(FNPacket& pkt) override {
    if (!_ready) return false;
    size_t sendSize = FN_HEADER_SIZE + pkt.payloadLen;
    if (sendSize > 250) sendSize = 250;
    esp_err_t result = esp_now_send(pkt.dst, (uint8_t*)&pkt, sendSize);
    return (result == ESP_OK);
  }

  bool receive(FNPacket& pkt) override {
    if (rxHead == rxTail) return false;
    memcpy(&pkt, &rxBuf[rxTail], sizeof(FNPacket));
    rxTail = (rxTail + 1) % RX_BUF_SIZE;
    return true;
  }

  TransportMetrics metrics() override {
    return { 1000000, 5, 90 };
  }

  const char* name() override { return "ESP-NOW"; }
};

#endif // ESPNOW_TRANSPORT_H
