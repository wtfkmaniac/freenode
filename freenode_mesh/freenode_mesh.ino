/*
 * freenode_mesh.ino — FreeNode Mesh Node
 * Platform: ESP32 (NodeMCU 32S)
 * Version: 0.5 — March 2026
 *
 * Транспорты: ESP-NOW + BLE (NimBLE advertising)
 *
 * ИЗМЕНЕНИЯ v0.5:
 *   - Добавлен BLE transport (NimBLE advertising-based)
 *   - MeshRouter relay работает через все транспорты: ESP-NOW ↔ BLE
 *   - BLE payload ограничен ~5 байт (legacy advertising)
 *
 * Подготовка:
 * 1. Arduino IDE → Library Manager → установить "NimBLE-Arduino" by h2zero
 * 2. Tools → Board → "ESP32 Dev Module"
 * 3. Tools → Upload Speed → 921600
 * 4. Tools → Partition Scheme → "Default 4MB with spiffs"
 *
 * Команды в Serial Monitor (115200 baud, NL+CR):
 *   /ping     — отправить PING, замерить RTT
 *   /stats    — показать статистику
 *   /info     — показать информацию об узле
 *   /help     — список команд
 *   любой текст — отправить в mesh
 */

#include "transport.h"
#include "espnow_transport.h"
#include "ble_transport.h"
#include "mesh_router.h"
#include <esp_wifi.h>

// ── Объекты ─────────────────────────────────────────────────────
EspNowTransport espnow;
BleTransport ble;
MeshRouter mesh;

// ── Светодиод для индикации ─────────────────────────────────────
#define LED_PIN 2
uint32_t ledOffTime = 0;

void blinkLed(uint16_t ms = 50) {
  digitalWrite(LED_PIN, HIGH);
  ledOffTime = millis() + ms;
}

// ── Вспомогательные функции ─────────────────────────────────────
String macToStr(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

const char* pktTypeName(uint8_t type) {
  switch (type) {
    case FN_TYPE_TEXT:      return "TEXT";
    case FN_TYPE_PING:      return "PING";
    case FN_TYPE_PONG:      return "PONG";
    case FN_TYPE_ROUTE:     return "ROUTE";
    case FN_TYPE_ACK:       return "ACK";
    case FN_TYPE_HEARTBEAT: return "HEARTBEAT";
    default: return "???";
  }
}

// ── Callback: входящее сообщение ────────────────────────────────
void onMsg(FNPacket& pkt) {
  blinkLed(100);

  Serial.printf("[MSG] type=%s ttl=%d flags=0x%02X payloadLen=%d src=%s\n",
    pktTypeName(pkt.type), pkt.ttl, pkt.flags, pkt.payloadLen,
    macToStr(pkt.src).c_str());

  switch (pkt.type) {
    case FN_TYPE_TEXT: {
      char text[201];
      memcpy(text, pkt.payload, pkt.payloadLen);
      text[pkt.payloadLen] = 0;
      Serial.printf("\n\033[1;32m[%s]\033[0m ", macToStr(pkt.src).c_str());
      Serial.printf("TTL:%d ", pkt.ttl);
      Serial.println(text);
      break;
    }
    case FN_TYPE_PING: {
      FNPacket pong;
      fnPacketInit(pong, mesh.mac(), FN_TYPE_PONG, DEFAULT_TTL,
                   (uint16_t)esp_random());
      memcpy(pong.dst, pkt.src, 6);
      memcpy(pong.payload, pkt.payload, pkt.payloadLen);
      pong.payloadLen = pkt.payloadLen;
      Serial.printf("[PING from %s] → sending PONG\n",
                    macToStr(pkt.src).c_str());
      // Broadcast PONG (flooding — mesh_router разошлёт через все транспорты)
      memset(pong.dst, 0xFF, 6);
      break;
    }
    case FN_TYPE_PONG: {
      if (pkt.payloadLen >= 4) {
        uint32_t sendTime;
        memcpy(&sendTime, pkt.payload, 4);
        uint32_t rtt = millis() - sendTime;
        Serial.printf("[PONG from %s] RTT: %u ms\n",
                      macToStr(pkt.src).c_str(), rtt);
      }
      break;
    }
    default:
      Serial.printf("[%s] type=%s TTL:%d len:%d\n",
                    macToStr(pkt.src).c_str(),
                    pktTypeName(pkt.type), pkt.ttl, pkt.payloadLen);
  }
}

// ── Обработка команд из Serial ──────────────────────────────────
void handleCommand(String& cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "/ping") {
    mesh.sendPing();
    Serial.println(">> PING sent (via all transports)");
  }
  else if (cmd == "/stats") {
    mesh.printStats();
  }
  else if (cmd == "/info") {
    Serial.println("=== FreeNode v0.5 Node Info ===");
    Serial.printf("  MAC:    %s\n", macToStr(mesh.mac()).c_str());
    Serial.printf("  Heap:   %u bytes free\n", ESP.getFreeHeap());
    Serial.printf("  Uptime: %lu sec\n", millis() / 1000);
    mesh.printTransports();
    mesh.printStats();
  }
  else if (cmd == "/help") {
    Serial.println("=== FreeNode v0.5 Commands ===");
    Serial.println("  /ping   — send PING (all transports)");
    Serial.println("  /stats  — packet statistics");
    Serial.println("  /info   — node info + transports");
    Serial.println("  /help   — this help");
    Serial.println("  <text>  — send to mesh");
    Serial.printf("  NOTE: BLE max payload = %d bytes\n", BLE_MAX_PAYLOAD);
  }
  else if (cmd.startsWith("/")) {
    Serial.println("Unknown command. Type /help");
  }
  else {
    mesh.sendText(cmd.c_str());
    Serial.printf(">> %s\n", cmd.c_str());
    blinkLed(30);
  }
}

// ── Setup ───────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  delay(500);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.println();
  Serial.println("╔═══════════════════════════════════╗");
  Serial.println("║    F R E E N O D E  v0.5          ║");
  Serial.println("║    ESP32 Mesh Node                 ║");
  Serial.println("║    Transports: ESP-NOW + BLE       ║");
  Serial.println("║    FreeNod Co Unlimited            ║");
  Serial.println("╚═══════════════════════════════════╝");
  Serial.println();

  mesh.begin();
  mesh.onMessage(onMsg);
  Serial.printf("MAC: %s\n", macToStr(mesh.mac()).c_str());
  Serial.printf("[DEBUG] sizeof(FNPacket)=%d FN_HEADER_SIZE=%d\n",
    sizeof(FNPacket), FN_HEADER_SIZE);

  // ── Транспорт 1: ESP-NOW ──
  if (mesh.addTransport(&espnow)) {
    Serial.println("[OK] ESP-NOW transport active");
  } else {
    Serial.println("[FAIL] ESP-NOW transport failed!");
  }

  esp_wifi_set_max_tx_power(40);

  // ── Транспорт 2: BLE ──
  if (mesh.addTransport(&ble)) {
    Serial.println("[OK] BLE transport active");
  } else {
    Serial.println("[FAIL] BLE transport failed!");
  }

  Serial.println();
  mesh.printTransports();
  Serial.println();
  Serial.println("Ready. Type message or /help");
  Serial.println("────────────────────────────────────");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ── Main Loop ───────────────────────────────────────────────────
void loop() {
  mesh.loop();
  ble.update();  // BLE advertising lifecycle

  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    handleCommand(msg);
  }

  if (ledOffTime && millis() > ledOffTime) {
    digitalWrite(LED_PIN, LOW);
    ledOffTime = 0;
  }

  delay(10);
}
