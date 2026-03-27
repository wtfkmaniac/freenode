/*
 * espnow_transport.h — ESP-NOW Transport Plugin for ESP32
 * Version: 0.2.2 — March 2026
 * Совместимость: Arduino ESP32 Core v3.3.7+
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

// v3.3.7: send callback — wifi_tx_info_t*, not uint8_t*
static void onDataSent(const wifi_tx_info_t *tx_info, esp_now_send_status_t status) {
  Serial.printf("[ESP-NOW TX] %s\n",
    status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// v3.3.7: recv callback — esp_now_recv_info_t*
static void onDataRecv(const esp_now_recv_info_t *info,
                       const uint8_t *data, int len) {
  const uint8_t *mac = info->src_addr;
  Serial.printf("[ESP-NOW RX] %d bytes from %02X:%02X:%02X:%02X:%02X:%02X\n",
    len, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

  uint8_t next = (rxHead + 1) % RX_BUF_SIZE;
  if (next == rxTail) return;

  memset(&rxBuf[rxHead], 0, sizeof(FNPacket));
  memcpy(&rxBuf[rxHead], data, min((size_t)len, sizeof(FNPacket)));
  rxHead = next;
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
    Serial.println("[ESP-NOW] Init OK");
    return true;
  }

  bool send(FNPacket& pkt) override {
    if (!_ready) return false;
    // Размер = всё до payload + сам payload
    size_t sendSize = offsetof(FNPacket, payload) + pkt.payloadLen;
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
    return { 250000, 5, 90 };
  }

  const char* name() override { return "ESP-NOW"; }
};

#endif
