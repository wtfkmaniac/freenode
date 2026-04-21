/*
 * freenode_mesh.ino — FreeNode Mesh Node
 * Platform: ESP32 (NodeMCU 32S)
 * Version: 0.5 — March 2026
 *
 * Dual transport: ESP-NOW + BLE (advertising-based)
 *
 * Зависимости (Library Manager):
 *   - NimBLE-Arduino by h2zero
 *
 * Settings:
 *   Board: "ESP32 Dev Module"
 *   Upload Speed: 921600
 *
 * Команды:
 *   /ping   — PING через все транспорты
 *   /stats  — статистика пакетов
 *   /info   — информация об узле
 *   /help   — справка
 *   <text>  — отправить в mesh
 */

#include "transport.h"
#include "espnow_transport.h"
#include "ble_transport.h"
#include "mesh_router.h"
#include <esp_wifi.h>

// ── Объекты ─────────────────────────────────────────────────────
EspNowTransport espnow;
BleTransport    ble;
MeshRouter      mesh;

// ── LED ─────────────────────────────────────────────────────────
#define LED_PIN 2
uint32_t ledOffTime = 0;
void blinkLed(uint16_t ms = 50) {
  digitalWrite(LED_PIN, HIGH);
  ledOffTime = millis() + ms;
}

// ── Утилиты ─────────────────────────────────────────────────────
String macToStr(const uint8_t* mac) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return String(buf);
}

const char* pktTypeName(uint8_t type) {
  switch (type) {
    case 0: return "TEXT";
    case 1: return "PING";
    case 2: return "PONG";
    case 3: return "ROUTE";
    case 4: return "ACK";
    default: return "???";
  }
}

// ── Callback: входящее сообщение ────────────────────────────────
void onMsg(FNPacket& pkt) {
  blinkLed(100);

  Serial.printf("[MSG] type=%d ttl=%d payloadLen=%d src=%s\n",
    pkt.type, pkt.ttl, pkt.payloadLen, macToStr(pkt.src).c_str());

  switch (pkt.type) {
    case FN_TYPE_TEXT: {
      char text[201];
      memcpy(text, pkt.payload, pkt.payloadLen);
      text[pkt.payloadLen] = 0;
      Serial.printf("\n\033[1;32m[%s]\033[0m TTL:%d %s\n",
        macToStr(pkt.src).c_str(), pkt.ttl, text);
      break;
    }
    case FN_TYPE_PING: { // PING → PONG
      FNPacket pong;
      fnPacketInit(pong, mesh.mac(), FN_TYPE_PONG, DEFAULT_TTL,
                   (uint16_t)esp_random());
      memcpy(pong.payload, pkt.payload, pkt.payloadLen);
      pong.payloadLen = pkt.payloadLen;
      mesh.sendPacket(pong);
      Serial.printf("[PING from %s] → PONG sent\n",
                    macToStr(pkt.src).c_str());
      break;
    }
    case FN_TYPE_PONG: {
      if (pkt.payloadLen >= 4) {
        uint32_t t0;
        memcpy(&t0, pkt.payload, 4);
        Serial.printf("[PONG from %s] RTT: %u ms\n",
          macToStr(pkt.src).c_str(), millis() - t0);
      }
      break;
    }
    default:
      Serial.printf("[%s] type=%s TTL:%d\n",
        macToStr(pkt.src).c_str(), pktTypeName(pkt.type), pkt.ttl);
  }
}

// ── Команды ─────────────────────────────────────────────────────
void handleCommand(String& cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  if (cmd == "/ping") {
    mesh.sendPing();
    Serial.println(">> PING sent (ESP-NOW + BLE)");
  }
  else if (cmd == "/stats") {
    mesh.printStats();
  }
  else if (cmd == "/info") {
    Serial.println("=== FreeNode v0.5 Info ===");
    Serial.printf("  MAC:    %s\n", macToStr(mesh.mac()).c_str());
    Serial.printf("  Heap:   %u bytes\n", ESP.getFreeHeap());
    Serial.printf("  Uptime: %lu sec\n", millis() / 1000);
    mesh.printTransports();
    mesh.printStats();
  }
  else if (cmd == "/help") {
    Serial.println("=== FreeNode v0.5 ===");
    Serial.println("  /ping  /stats  /info  /help");
    Serial.println("  <text> — send to mesh");
    Serial.println("  Transports: ESP-NOW + BLE");
    Serial.println("  BLE payload: ~9 bytes max (legacy ADV)");
  }
  else if (cmd.startsWith("/")) {
    Serial.println("Unknown cmd. /help");
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
  Serial.println("╔══════════════════════════════════════╗");
  Serial.println("║   F R E E N O D E  v0.5              ║");
  Serial.println("║   ESP32 Mesh — ESP-NOW + BLE          ║");
  Serial.println("║   FreeNod Co Unlimited                ║");
  Serial.println("╚══════════════════════════════════════╝");
  Serial.println();

  mesh.begin();
  mesh.onMessage(onMsg);
  Serial.printf("MAC: %s\n", macToStr(mesh.mac()).c_str());

  // Transport 1: ESP-NOW
  if (mesh.addTransport(&espnow)) {
    Serial.println("[OK] ESP-NOW");
  } else {
    Serial.println("[FAIL] ESP-NOW");
  }

  // Transport 2: BLE (NimBLE ADV-based)
  if (mesh.addTransport(&ble)) {
    Serial.println("[OK] BLE (NimBLE ADV)");
  } else {
    Serial.println("[FAIL] BLE");
  }

  esp_wifi_set_max_tx_power(40);

  Serial.println();
  mesh.printTransports();
  Serial.println("Ready. /help for commands");
  Serial.println("────────────────────────────────────");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ── Loop ────────────────────────────────────────────────────────
void loop() {
  mesh.loop();
  ble.update();  // BLE ADV↔SCAN state machine

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
