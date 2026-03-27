/*
 * transport.h — FreeNode Transport Plugin Interface
 * Version: 0.3.0 — March 2026
 *
 * ИЗМЕНЕНИЯ v0.3:
 *   - Добавлен magic + version в структуру пакета
 *     (кросс-совместимость freenode_mesh ↔ freenode_lora)
 *   - Добавлен байт flags: бит 0 = RELAYED (пакет уже ретранслировался)
 *   - Это позволяет cross-transport relay без петель
 */

#ifndef TRANSPORT_H
#define TRANSPORT_H

#include <Arduino.h>

// ── Протокол FreeNode v0.3 ──────────────────────────────────────
#define FN_MAGIC        0xFE
#define FN_PROTO_VER    0x03

// Типы пакетов
#define FN_TYPE_TEXT      0x00
#define FN_TYPE_PING      0x01
#define FN_TYPE_PONG      0x02
#define FN_TYPE_ROUTE     0x03
#define FN_TYPE_ACK       0x04
#define FN_TYPE_HEARTBEAT 0x05

// Флаги пакета
#define FN_FLAG_RELAYED   0x01  // Пакет уже прошёл через relay-узел

// ── Пакет FreeNode (packed — без padding!) ──────────────────────
#pragma pack(push, 1)
struct FNPacket {
  uint8_t  magic;         // FN_MAGIC = 0xFE
  uint8_t  version;       // FN_PROTO_VER = 0x03
  uint8_t  src[6];        // MAC отправителя
  uint8_t  dst[6];        // MAC назначения (FF:FF:FF:FF:FF:FF = broadcast)
  uint8_t  ttl;           // Time To Live
  uint16_t id;            // Уникальный ID пакета
  uint8_t  type;          // FN_TYPE_*
  uint8_t  flags;         // FN_FLAG_* (битовые флаги)
  uint8_t  payloadLen;    // Длина payload
  uint8_t  payload[200];  // Полезная нагрузка
};
#pragma pack(pop)

// Размер заголовка без payload
#define FN_HEADER_SIZE (sizeof(FNPacket) - 200)

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

// ── Хелперы для работы с пакетами ──────────────────────────────

// Инициализировать пакет с нужными полями
inline void fnPacketInit(FNPacket& pkt, const uint8_t* srcMac,
                          uint8_t type, uint8_t ttl, uint16_t id) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic   = FN_MAGIC;
  pkt.version = FN_PROTO_VER;
  memcpy(pkt.src, srcMac, 6);
  memset(pkt.dst, 0xFF, 6);  // broadcast по умолчанию
  pkt.ttl     = ttl;
  pkt.id      = id;
  pkt.type    = type;
  pkt.flags   = 0;
}

// Проверить валидность пакета
inline bool fnPacketValid(const FNPacket& pkt) {
  return (pkt.magic == FN_MAGIC); // version-agnostic: принимаем 0x02 и 0x03
}

// Проверить что пакет от нас самих (по MAC)
inline bool fnPacketFromSelf(const FNPacket& pkt, const uint8_t* myMac) {
  return memcmp(pkt.src, myMac, 6) == 0;
}

#endif // TRANSPORT_H
