/*
 * ble_transport.h — BLE Advertising Transport Plugin for ESP32
 * Version: 0.5.0 — March 2026
 * Library: NimBLE-Arduino by h2zero
 *
 * Принцип: connectionless broadcast через BLE Advertising.
 * - TX: пишем FNPacket в manufacturer data, обновляем advertising
 * - RX: сканируем эфир, парсим manufacturer data из чужих advertisments
 *
 * Manufacturer Data формат (после company ID):
 *   [0]     Magic 0xFE (FreeNode)
 *   [1]     Version
 *   [2..7]  src MAC (6 байт)
 *   [8..13] dst MAC (6 байт)
 *   [14]    TTL
 *   [15..16] ID (uint16)
 *   [17]    Type
 *   [18]    Flags
 *   [19]    PayloadLen
 *   [20..]  Payload
 *
 * FN_HEADER_SIZE = 20 байт
 * Max manufacturer data = 29 байт (legacy adv: 31 - 2 flags)
 * Company ID = 2 байт → 29 - 2 = 27 байт для наших данных
 * Payload max = 27 - 20 = 7 байт
 *
 * Для PING/PONG (payload 0-4 байт) — достаточно.
 * Для текста — короткие слова ("hello", "ok", "SOS").
 * TODO v0.6: BLE 5 Extended Advertising → до 255 байт
 *
 * Зависимость: NimBLE-Arduino (Library Manager → "NimBLE-Arduino" by h2zero)
 *
 * ВАЖНО: вызывать ble.update() из loop() для управления advertising lifecycle!
 */

#ifndef BLE_TRANSPORT_H
#define BLE_TRANSPORT_H

#include "transport.h"
#include <NimBLEDevice.h>

// ── Настройки ───────────────────────────────────────────────────
#define BLE_ADV_INTERVAL_MS   250
#define BLE_SCAN_INTERVAL     80     // units of 0.625ms = 50ms
#define BLE_SCAN_WINDOW       40     // units of 0.625ms = 25ms
#define BLE_COMPANY_ID        0xFFFF // Тестовый (незарегистрированный)
#define BLE_ADV_HOLD_MS       600    // Держим advertising после TX
#define BLE_RX_BUF_SIZE       8

// Max данных в manufacturer data (legacy adv):
// 31 (max adv) - 2 (AD flags) = 29 для manufacturer data AD
// manufacturer data AD: 1(len) + 1(type 0xFF) + 2(company_id) + data
// → data max = 29 - 1 - 1 - 2 = 25 байт
// Но NimBLE выделяет ещё пару байт → реально ~25 байт
#define BLE_MAX_DATA          25
#define BLE_MAX_PAYLOAD       (BLE_MAX_DATA - FN_HEADER_SIZE)  // 25-20=5

// ── RX буфер ────────────────────────────────────────────────────
static FNPacket bleRxBuf[BLE_RX_BUF_SIZE];
static volatile uint8_t bleRxHead = 0;
static volatile uint8_t bleRxTail = 0;

// ── Scan callback ───────────────────────────────────────────────
class FNBLEScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice* device) override {
    if (!device->haveManufacturerData()) return;

    NimBLEManufacturerData mfData = device->getManufacturerData();
    // getData() возвращает данные БЕЗ company ID
    std::string data = mfData.getData();

    // Минимальная длина: FN_HEADER_SIZE (20 байт)
    if (data.length() < FN_HEADER_SIZE) return;

    const uint8_t* d = (const uint8_t*)data.data();

    // Проверяем magic
    if (d[0] != FN_MAGIC) return;

    // Собираем FNPacket из BLE данных
    uint8_t next = (bleRxHead + 1) % BLE_RX_BUF_SIZE;
    if (next == bleRxTail) return;  // буфер полон

    FNPacket& pkt = bleRxBuf[bleRxHead];
    memset(&pkt, 0, sizeof(FNPacket));

    pkt.magic   = d[0];
    pkt.version = d[1];
    memcpy(pkt.src, d + 2, 6);
    memcpy(pkt.dst, d + 8, 6);
    pkt.ttl = d[14];
    memcpy(&pkt.id, d + 15, 2);
    pkt.type = d[17];
    pkt.flags = d[18];
    pkt.payloadLen = d[19];

    // Валидация payload
    if (pkt.payloadLen > 200) return;
    if (data.length() < (size_t)(FN_HEADER_SIZE + pkt.payloadLen)) return;

    if (pkt.payloadLen > 0) {
      memcpy(pkt.payload, d + FN_HEADER_SIZE, pkt.payloadLen);
    }

    bleRxHead = next;
  }
};

static FNBLEScanCallbacks _fnBleScanCb;

// ── BLE Transport Plugin ────────────────────────────────────────
class BleTransport : public Transport {
  bool _ready = false;
  NimBLEScan* _pScan = nullptr;
  NimBLEAdvertising* _pAdv = nullptr;
  uint32_t _advExpireMs = 0;

public:
  bool init() override {
    NimBLEDevice::init("FreeNode");
    NimBLEDevice::setPower(ESP_PWR_LVL_P3);  // +3 dBm

    // Scanner
    _pScan = NimBLEDevice::getScan();
    _pScan->setScanCallbacks(&_fnBleScanCb, true);  // wantDuplicates=true
    _pScan->setActiveScan(false);
    _pScan->setInterval(BLE_SCAN_INTERVAL);
    _pScan->setWindow(BLE_SCAN_WINDOW);

    if (!_pScan->start(0, false)) {
      Serial.println("[BLE] Scan start FAILED");
      return false;
    }

    // Advertiser
    _pAdv = NimBLEDevice::getAdvertising();
    uint16_t intervalUnits = BLE_ADV_INTERVAL_MS * 1000 / 625;
    _pAdv->setMinInterval(intervalUnits);
    _pAdv->setMaxInterval(intervalUnits);

    _ready = true;
    Serial.println("[BLE] Init OK (NimBLE advertising)");
    Serial.printf("[BLE] Adv:%dms Scan:passive MaxPayload:%d\n",
      BLE_ADV_INTERVAL_MS, BLE_MAX_PAYLOAD);
    return true;
  }

  bool send(FNPacket& pkt) override {
    if (!_ready) return false;

    // Ограничиваем payload тем что влезет в BLE adv
    uint8_t payloadLen = pkt.payloadLen;
    if (payloadLen > BLE_MAX_PAYLOAD) {
      // Не влезает — пропускаем BLE для этого пакета
      Serial.printf("[BLE TX] skip — payload %d > max %d\n",
        payloadLen, BLE_MAX_PAYLOAD);
      return false;
    }

    // Сериализуем FNPacket в BLE manufacturer data
    uint8_t buf[BLE_MAX_DATA];
    uint8_t pos = 0;
    buf[pos++] = pkt.magic;
    buf[pos++] = pkt.version;
    memcpy(buf + pos, pkt.src, 6);  pos += 6;
    memcpy(buf + pos, pkt.dst, 6);  pos += 6;
    buf[pos++] = pkt.ttl;
    memcpy(buf + pos, &pkt.id, 2);  pos += 2;
    buf[pos++] = pkt.type;
    buf[pos++] = pkt.flags;
    buf[pos++] = payloadLen;
    if (payloadLen > 0) {
      memcpy(buf + pos, pkt.payload, payloadLen);
      pos += payloadLen;
    }

    // Останавливаем scan на время TX
    if (_pScan && _pScan->isScanning()) {
      _pScan->stop();
    }

    // Формируем advertising data
    _pAdv->stop();

    NimBLEAdvertisementData advData;
    advData.setFlags(BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP);

    // setManufacturerData(string) — первые 2 байта = company ID (LE)
    std::string manufPayload;
    manufPayload += (char)(BLE_COMPANY_ID & 0xFF);
    manufPayload += (char)((BLE_COMPANY_ID >> 8) & 0xFF);
    manufPayload += std::string((char*)buf, pos);
    advData.setManufacturerData(manufPayload);

    _pAdv->setAdvertisementData(advData);
    _pAdv->start();

    _advExpireMs = millis() + BLE_ADV_HOLD_MS;

    Serial.printf("[BLE TX] type=0x%02X payload=%d total=%d bytes\n",
      pkt.type, payloadLen, pos);
    return true;
  }

  bool receive(FNPacket& pkt) override {
    if (bleRxHead == bleRxTail) return false;
    memcpy(&pkt, &bleRxBuf[bleRxTail], sizeof(FNPacket));
    bleRxTail = (bleRxTail + 1) % BLE_RX_BUF_SIZE;
    return true;
  }

  TransportMetrics metrics() override {
    return { 1000, 10, 80 };  // ~1kbps, 10ms lat, 80% reliable
  }

  const char* name() override { return "BLE"; }

  // Вызывать из loop()!
  void update() {
    if (!_ready) return;
    if (_advExpireMs && millis() > _advExpireMs) {
      _pAdv->stop();
      _advExpireMs = 0;
      if (_pScan && !_pScan->isScanning()) {
        _pScan->start(0, false);
      }
    }
  }
};

#endif // BLE_TRANSPORT_H
