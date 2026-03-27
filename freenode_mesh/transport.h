/*
 * transport.h — FreeNode Transport Plugin Interface
 * Version: 0.2.4 — March 2026
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <Arduino.h>

// ── Пакет FreeNode (packed — без padding!) ──────────────────────
#pragma pack(push, 1)
struct FNPacket {
  uint8_t  src[6];        // MAC отправителя
  uint8_t  dst[6];        // MAC назначения (FF:FF:FF:FF:FF:FF = broadcast)
  uint8_t  ttl;           // Time To Live
  uint16_t id;            // Уникальный ID пакета
  uint8_t  type;          // 0=TEXT, 1=PING, 2=PONG, 3=ROUTE, 4=ACK
  uint8_t  payloadLen;    // Длина payload
  uint8_t  payload[200];  // Полезная нагрузка
};
#pragma pack(pop)

// ── Метрики транспорта ──────────────────────────────────────────
struct TransportMetrics {
  uint32_t bandwidth;     // bps
  uint16_t latency;       // ms
  uint8_t  reliability;   // 0-100%
};

// ── Абстрактный класс транспорта ────────────────────────────────
class Transport {
public:
  virtual bool init() = 0;
  virtual bool send(FNPacket& pkt) = 0;
  virtual bool receive(FNPacket& pkt) = 0;
  virtual TransportMetrics metrics() = 0;
  virtual const char* name() = 0;
  virtual ~Transport() {}
};

#endif // TRANSPORT_H
