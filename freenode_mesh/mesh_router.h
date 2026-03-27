/*
 * mesh_router.h — FreeNode Mesh Router (Flooding + Cross-Transport Relay)
 * Version: 0.3.0 — March 2026
 *
 * ИЗМЕНЕНИЯ v0.3:
 *   - Cross-transport relay: пакет ретранслируется через ВСЕ транспорты,
 *     кроме того, с которого пришёл
 *   - Флаг FN_FLAG_RELAYED выставляется при первой ретрансляции
 *   - sendText/sendPing используют fnPacketInit из transport.h
 *   - magic + version проставляются автоматически
 */

#ifndef MESH_ROUTER_H
#define MESH_ROUTER_H

#include "transport.h"
#include <esp_mac.h>

#define MAX_TRANSPORTS 4
#define MAX_SEEN       128
#define DEFAULT_TTL    5

class MeshRouter {
  Transport* _transports[MAX_TRANSPORTS];
  uint8_t    _numTransports = 0;

  uint16_t _seenIds[MAX_SEEN];
  uint8_t  _seenIdx = 0;

  uint8_t _myMac[6];

  void (*_onMessage)(FNPacket& pkt) = nullptr;

  uint32_t _pktReceived  = 0;
  uint32_t _pktForwarded = 0;
  uint32_t _pktDropped   = 0;
  uint32_t _pktSent      = 0;

  bool alreadySeen(uint16_t id) {
    for (int i = 0; i < MAX_SEEN; i++) {
      if (_seenIds[i] == id) return true;
    }
    return false;
  }

  void markSeen(uint16_t id) {
    _seenIds[_seenIdx] = id;
    _seenIdx = (_seenIdx + 1) % MAX_SEEN;
  }

  void relayPacket(FNPacket& pkt, int8_t sourceTransportIdx) {
    pkt.ttl--;
    pkt.flags |= FN_FLAG_RELAYED;
    for (uint8_t i = 0; i < _numTransports; i++) {
      if (i == (uint8_t)sourceTransportIdx) continue;
      _transports[i]->send(pkt);
    }
    _pktForwarded++;
  }

public:
  void begin() {
    esp_read_mac(_myMac, ESP_MAC_WIFI_STA);
    memset(_seenIds, 0, sizeof(_seenIds));
  }

  bool addTransport(Transport* t) {
    if (_numTransports >= MAX_TRANSPORTS) return false;
    if (!t->init()) return false;
    _transports[_numTransports++] = t;
    Serial.printf("[MESH] Transport added: %s (total: %d)\n",
                  t->name(), _numTransports);
    return true;
  }

  void onMessage(void (*cb)(FNPacket&)) { _onMessage = cb; }

  bool sendText(const char* text) {
    FNPacket pkt;
    fnPacketInit(pkt, _myMac, FN_TYPE_TEXT, DEFAULT_TTL, (uint16_t)esp_random());
    pkt.payloadLen = min(strlen(text), (size_t)200);
    memcpy(pkt.payload, text, pkt.payloadLen);
    markSeen(pkt.id);
    bool ok = false;
    for (uint8_t i = 0; i < _numTransports; i++) {
      if (_transports[i]->send(pkt)) ok = true;
    }
    if (ok) _pktSent++;
    return ok;
  }

  bool sendPing() {
    FNPacket pkt;
    fnPacketInit(pkt, _myMac, FN_TYPE_PING, DEFAULT_TTL, (uint16_t)esp_random());
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

  void processPacket(FNPacket& pkt, uint8_t sourceIdx) {
    if (!fnPacketValid(pkt)) {
      _pktDropped++;
      return;
    }
    if (fnPacketFromSelf(pkt, _myMac)) return;
    if (alreadySeen(pkt.id)) {
      _pktDropped++;
      return;
    }
    markSeen(pkt.id);
    _pktReceived++;
    if (_onMessage) _onMessage(pkt);
    if (pkt.ttl > 1) {
      relayPacket(pkt, (int8_t)sourceIdx);
      Serial.printf("[MESH] Relayed id=%u src=%s\n",
        pkt.id, _transports[sourceIdx]->name());
    }
  }

  void loop() {
    FNPacket pkt;
    for (uint8_t t = 0; t < _numTransports; t++) {
      while (_transports[t]->receive(pkt)) {
        processPacket(pkt, t);
      }
    }
  }

  const uint8_t* mac() const { return _myMac; }
  uint8_t numTransports() const { return _numTransports; }
  uint32_t pktReceived()  const { return _pktReceived; }
  uint32_t pktForwarded() const { return _pktForwarded; }
  uint32_t pktDropped()   const { return _pktDropped; }
  uint32_t pktSent()      const { return _pktSent; }

  void printTransports() {
    Serial.printf("[MESH] Active transports: %d\n", _numTransports);
    for (uint8_t i = 0; i < _numTransports; i++) {
      TransportMetrics m = _transports[i]->metrics();
      Serial.printf("  [%d] %s — %u bps, %u ms, %u%% reliable\n",
                    i, _transports[i]->name(),
                    m.bandwidth, m.latency, m.reliability);
    }
  }

  void printStats() {
    Serial.printf("[MESH] Stats: sent=%u recv=%u fwd=%u drop=%u\n",
                  _pktSent, _pktReceived, _pktForwarded, _pktDropped);
  }
};

#endif // MESH_ROUTER_H
