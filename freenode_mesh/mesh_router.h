/*
 * mesh_router.h — FreeNode Mesh Router (Flooding)
 * Platform: ESP32 (NodeMCU 32S)
 * Version: 0.2 — March 2026
 *
 * Первая версия маршрутизации — flooding с TTL и дедупликацией.
 * Просто, надёжно, не масштабируется — но для прототипа
 * из 5-10 узлов идеально.
 *
 * Логика:
 * 1. Получаем пакет от транспорта
 * 2. Проверяем: уже видели этот ID? → дропаем
 * 3. Запоминаем ID
 * 4. Доставляем локальному обработчику (callback)
 * 5. Если TTL > 1 → декремент, ретранслируем
 */

#ifndef MESH_ROUTER_H
#define MESH_ROUTER_H

#include "transport.h"
#include <esp_mac.h>

#define MAX_TRANSPORTS 4    // макс. число одновременных транспортов
#define MAX_SEEN       128  // размер таблицы дедупликации
#define DEFAULT_TTL    5    // TTL по умолчанию

class MeshRouter {
  // Массив транспортных плагинов
  Transport* _transports[MAX_TRANSPORTS];
  uint8_t    _numTransports = 0;

  // Таблица дедупликации (кольцевой буфер ID)
  uint16_t _seenIds[MAX_SEEN];
  uint8_t  _seenIdx = 0;

  // MAC-адрес этого узла
  uint8_t _myMac[6];

  // Callback для входящих сообщений
  void (*_onMessage)(FNPacket& pkt) = nullptr;

  // Статистика
  uint32_t _pktReceived  = 0;
  uint32_t _pktForwarded = 0;
  uint32_t _pktDropped   = 0;
  uint32_t _pktSent      = 0;

  // Проверка: видели ли мы уже этот пакет?
  bool alreadySeen(uint16_t id) {
    for (int i = 0; i < MAX_SEEN; i++) {
      if (_seenIds[i] == id) return true;
    }
    return false;
  }

  // Запомнить ID пакета
  void markSeen(uint16_t id) {
    _seenIds[_seenIdx] = id;
    _seenIdx = (_seenIdx + 1) % MAX_SEEN;
  }

public:
  // ── Инициализация ─────────────────────────────────────────────
  void begin() {
    // Получаем MAC-адрес ESP32 (esp_read_mac работает во всех версиях)
    esp_read_mac(_myMac, ESP_MAC_WIFI_STA);
    memset(_seenIds, 0, sizeof(_seenIds));
  }

  // ── Добавление транспортного плагина ──────────────────────────
  bool addTransport(Transport* t) {
    if (_numTransports >= MAX_TRANSPORTS) return false;
    if (!t->init()) return false;
    _transports[_numTransports++] = t;
    Serial.printf("[MESH] Transport added: %s (total: %d)\n",
                  t->name(), _numTransports);
    return true;
  }

  // ── Callback для входящих сообщений ───────────────────────────
  void onMessage(void (*cb)(FNPacket&)) { _onMessage = cb; }

  // ── Отправка текстового сообщения ─────────────────────────────
  bool sendText(const char* text) {
    FNPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.src, _myMac, 6);
    memset(pkt.dst, 0xFF, 6); // broadcast
    pkt.ttl = DEFAULT_TTL;
    pkt.id  = (uint16_t)esp_random(); // аппаратный RNG ESP32
    pkt.type = 0; // TEXT
    pkt.payloadLen = min(strlen(text), (size_t)200);
    memcpy(pkt.payload, text, pkt.payloadLen);

    markSeen(pkt.id);

    // Отправляем через все транспорты
    bool ok = false;
    for (uint8_t i = 0; i < _numTransports; i++) {
      if (_transports[i]->send(pkt)) ok = true;
    }
    if (ok) _pktSent++;
    return ok;
  }

  // ── Отправка PING ────────────────────────────────────────────
  bool sendPing() {
    FNPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(pkt.src, _myMac, 6);
    memset(pkt.dst, 0xFF, 6);
    pkt.ttl = DEFAULT_TTL;
    pkt.id  = (uint16_t)esp_random();
    pkt.type = 1; // PING
    // В payload кладём timestamp для измерения RTT
    uint32_t now = millis();
    memcpy(pkt.payload, &now, 4);
    pkt.payloadLen = 4;

    markSeen(pkt.id);

    bool ok = false;
    for (uint8_t i = 0; i < _numTransports; i++) {
      if (_transports[i]->send(pkt)) ok = true;
    }
    if (ok) _pktSent++;
    return ok;
  }

  // ── Главный цикл: приём и ретрансляция ────────────────────────
  void loop() {
    FNPacket pkt;

    for (uint8_t t = 0; t < _numTransports; t++) {
      while (_transports[t]->receive(pkt)) {
        Serial.printf("[LOOP] id=%u type=%d ttl=%d payloadLen=%d\n",
          pkt.id, pkt.type, pkt.ttl, pkt.payloadLen);

        if (alreadySeen(pkt.id)) {
          Serial.printf("[LOOP] DROPPED (already seen id=%u)\n", pkt.id);
          _pktDropped++;
          continue;
        }
        markSeen(pkt.id);
        _pktReceived++;

        if (_onMessage) _onMessage(pkt);

        if (pkt.ttl > 1) {
          pkt.ttl--;
          for (uint8_t i = 0; i < _numTransports; i++) {
            _transports[i]->send(pkt);
          }
          _pktForwarded++;
        }
      }
    }
  }

  // ── Геттеры ───────────────────────────────────────────────────
  const uint8_t* mac() const { return _myMac; }
  uint8_t numTransports() const { return _numTransports; }

  // Статистика
  uint32_t pktReceived()  const { return _pktReceived; }
  uint32_t pktForwarded() const { return _pktForwarded; }
  uint32_t pktDropped()   const { return _pktDropped; }
  uint32_t pktSent()      const { return _pktSent; }

  // Имена подключённых транспортов
  void printTransports() {
    Serial.printf("[MESH] Active transports: %d\n", _numTransports);
    for (uint8_t i = 0; i < _numTransports; i++) {
      TransportMetrics m = _transports[i]->metrics();
      Serial.printf("  [%d] %s — %u bps, %u ms, %u%% reliable\n",
                    i, _transports[i]->name(),
                    m.bandwidth, m.latency, m.reliability);
    }
  }

  // Вывод статистики
  void printStats() {
    Serial.printf("[MESH] Stats: sent=%u recv=%u fwd=%u drop=%u\n",
                  _pktSent, _pktReceived, _pktForwarded, _pktDropped);
  }
};

#endif // MESH_ROUTER_H
