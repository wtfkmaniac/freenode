/*
 * ble_transport.h — BLE Transport Plugin for FreeNode
 * Version: 0.5.0 — March 2026
 * Library: NimBLE-Arduino by h2zero
 *
 * Advertising-based (connectionless) BLE transport.
 * Данные в manufacturer-specific data рекламных пакетов.
 * Scan + Advertise чередуются: send() → ADV burst → scan.
 *
 * Manufacturer data format:
 *   [0..1]  Company ID = 0xFFFF (testing)
 *   [2]     Magic 0xFE
 *   [3..]   FNPacket (header + payload)
 *
 * Legacy ADV limit:
 *   31 - 2(AD header) - 2(company ID) - 1(magic) = 26 байт
 *   FNPacket header = 17 байт → 9 байт на текст
 *   Extended ADV (BLE 5, ESP32-S3) — до ~250 байт
 *
 * Install: Library Manager → "NimBLE-Arduino"
 */

#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include "transport.h"
#include <NimBLEDevice.h>

#define BLE_FN_COMPANY_ID    0xFFFF
#define BLE_FN_MAGIC         0xFE
#define BLE_ADV_INTERVAL_MS  250
#define BLE_SCAN_INTERVAL    80       // *0.625ms = 50ms
#define BLE_SCAN_WINDOW      40       // *0.625ms = 25ms
#define BLE_RX_BUF_SIZE      8
#define BLE_ADV_DURATION_SEC 1
#define BLE_MFG_OVERHEAD     3        // company_id(2) + magic(1)
#define BLE_MAX_PKT_DATA     26       // Legacy ADV limit

// ── Буфер приёма ────────────────────────────────────────────────
static FNPacket         bleRxBuf[BLE_RX_BUF_SIZE];
static volatile uint8_t bleRxHead = 0;
static volatile uint8_t bleRxTail = 0;

// ── Состояние ───────────────────────────────────────────────────
static volatile bool     bleIsAdv    = false;
static volatile uint32_t bleAdvStart = 0;

// ── Scan Callback ───────────────────────────────────────────────
class FNBLEScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!dev->haveManufacturerData()) return;

    std::string raw = dev->getManufacturerData().toString();
    size_t len = raw.length();

    if (len < BLE_MFG_OVERHEAD + offsetof(FNPacket, payload)) return;

    const uint8_t* d = (const uint8_t*)raw.data();

    uint16_t cid = d[0] | (d[1] << 8);
    if (cid != BLE_FN_COMPANY_ID) return;
    if (d[2] != BLE_FN_MAGIC) return;

    const uint8_t* pktData = d + BLE_MFG_OVERHEAD;
    size_t pktLen = len - BLE_MFG_OVERHEAD;

    uint8_t next = (bleRxHead + 1) % BLE_RX_BUF_SIZE;
    if (next == bleRxTail) return;

    memset(&bleRxBuf[bleRxHead], 0, sizeof(FNPacket));
    memcpy(&bleRxBuf[bleRxHead], pktData, min(pktLen, sizeof(FNPacket)));
    bleRxHead = next;

    Serial.printf("[BLE RX] %d bytes RSSI:%d\n", (int)pktLen, dev->getRSSI());
  }

  void onDiscoveryComplete(const NimBLEScanResults& results) override {
    NimBLEDevice::getScan()->start(0, false);
  }
};

static FNBLEScanCB fnBleScanCb;

// ═══════════════════════════════════════════════════════════════
class BleTransport : public Transport {
  bool               _ready = false;
  NimBLEAdvertising* _adv   = nullptr;
  NimBLEScan*        _scan  = nullptr;

public:
  bool init() override {
    NimBLEDevice::init("FN");
    NimBLEDevice::setPower(ESP_PWR_LVL_P9);

    _scan = NimBLEDevice::getScan();
    _scan->setScanCallbacks(&fnBleScanCb, false);
    _scan->setActiveScan(false);
    _scan->setInterval(BLE_SCAN_INTERVAL);
    _scan->setWindow(BLE_SCAN_WINDOW);
    _scan->setMaxResults(0);

    if (!_scan->start(0, false)) {
      Serial.println("[BLE] Scan FAILED");
      return false;
    }

    _adv = NimBLEDevice::getAdvertising();
    uint16_t units = (BLE_ADV_INTERVAL_MS * 1000) / 625;
    _adv->setMinInterval(units);
    _adv->setMaxInterval(units);
    _adv->setConnectableMode(BLE_GAP_CONN_MODE_NON);
    _adv->setScannable(false);

    bleRxHead = bleRxTail = 0;
    _ready = true;
    Serial.println("[BLE] Init OK (NimBLE ADV-based)");
    return true;
  }

  bool send(FNPacket& pkt) override {
    if (!_ready) return false;

    size_t pktSize = offsetof(FNPacket, payload) + pkt.payloadLen;
    if (pktSize > BLE_MAX_PKT_DATA) {
      pkt.payloadLen = BLE_MAX_PKT_DATA - offsetof(FNPacket, payload);
      pktSize = BLE_MAX_PKT_DATA;
      Serial.printf("[BLE TX] Truncated to %d bytes payload\n", pkt.payloadLen);
    }

    uint8_t mfg[31];
    mfg[0] = BLE_FN_COMPANY_ID & 0xFF;
    mfg[1] = (BLE_FN_COMPANY_ID >> 8) & 0xFF;
    mfg[2] = BLE_FN_MAGIC;
    memcpy(mfg + BLE_MFG_OVERHEAD, &pkt, pktSize);
    size_t total = BLE_MFG_OVERHEAD + pktSize;

    _scan->stop();

    NimBLEAdvertisementData advData;
    advData.setManufacturerData(std::string((char*)mfg, total));
    _adv->setAdvertisementData(advData);
    _adv->start(BLE_ADV_DURATION_SEC);

    bleIsAdv = true;
    bleAdvStart = millis();

    Serial.printf("[BLE TX] %d bytes (%d payload)\n", (int)total, pkt.payloadLen);
    return true;
  }

  bool receive(FNPacket& pkt) override {
    if (bleRxHead == bleRxTail) return false;
    memcpy(&pkt, &bleRxBuf[bleRxTail], sizeof(FNPacket));
    bleRxTail = (bleRxTail + 1) % BLE_RX_BUF_SIZE;
    return true;
  }

  TransportMetrics metrics() override {
    return { 2000, 250, 70 };
  }

  const char* name() override { return "BLE"; }

  // Вызывать из loop()!
  void update() {
    if (!_ready) return;
    if (bleIsAdv && millis() - bleAdvStart > (BLE_ADV_DURATION_SEC * 1000 + 100)) {
      _adv->stop();
      bleIsAdv = false;
      _scan->start(0, false);
    }
  }
};

#endif
