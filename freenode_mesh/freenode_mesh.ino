/*
 * freenode_mesh.ino — FreeNode Mesh Node
 * Platform: ESP32 (NodeMCU 32S)
 * Version: 0.2 — March 2026
 *
 * Этот скетч превращает ESP32 в узел FreeNode.
 * - Принимает текст из Serial Monitor и отправляет в mesh
 * - Принимает сообщения от других узлов и отображает
 * - Ретранслирует пакеты дальше (flooding)
 *
 * Подготовка:
 * 1. Arduino IDE → File → Preferences → Board Manager URLs:
 *    https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
 * 2. Tools → Board Manager → ищи "esp32" → установить
 * 3. Tools → Board → "ESP32 Dev Module" (или "NodeMCU-32S")
 * 4. Tools → Upload Speed → 921600
 * 5. Tools → Port → выбрать COM-порт
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
#include "mesh_router.h"
#include <esp_wifi.h>

// ── Объекты ─────────────────────────────────────────────────────
EspNowTransport espnow;
MeshRouter mesh;

// ── Светодиод для индикации ─────────────────────────────────────
// NodeMCU 32S: встроенный LED обычно на GPIO2
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
  blinkLed(100); // мигнуть при получении

  // Диагностика
  Serial.printf("[MSG] type=%d ttl=%d payloadLen=%d src=%s\n",
    pkt.type, pkt.ttl, pkt.payloadLen, macToStr(pkt.src).c_str());

  switch (pkt.type) {
    case 0: { // TEXT
      char text[201];
      memcpy(text, pkt.payload, pkt.payloadLen);
      text[pkt.payloadLen] = 0;

      Serial.printf("\n\033[1;32m[%s]\033[0m ", macToStr(pkt.src).c_str());
      Serial.printf("TTL:%d ", pkt.ttl);
      Serial.println(text);
      break;
    }
    case 1: { // PING — отвечаем PONG
      FNPacket pong;
      memset(&pong, 0, sizeof(pong));
      memcpy(pong.src, mesh.mac(), 6);
      memcpy(pong.dst, pkt.src, 6); // адресуем отправителю
      pong.ttl = DEFAULT_TTL;
      pong.id  = (uint16_t)esp_random();
      pong.type = 2; // PONG
      // Копируем timestamp из PING в PONG
      memcpy(pong.payload, pkt.payload, pkt.payloadLen);
      pong.payloadLen = pkt.payloadLen;

      Serial.printf("[PING from %s] → sending PONG\n",
                    macToStr(pkt.src).c_str());
      // Отправляем PONG broadcast (т.к. у нас пока flooding)
      memset(pong.dst, 0xFF, 6);
      // TODO: unicast когда будет таблица маршрутов
      break;
    }
    case 2: { // PONG — замеряем RTT
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
    Serial.println(">> PING sent");
  }
  else if (cmd == "/stats") {
    mesh.printStats();
  }
  else if (cmd == "/info") {
    Serial.println("=== FreeNode Node Info ===");
    Serial.printf("  MAC:    %s\n", macToStr(mesh.mac()).c_str());
    Serial.printf("  Heap:   %u bytes free\n", ESP.getFreeHeap());
    Serial.printf("  Uptime: %lu sec\n", millis() / 1000);
    mesh.printTransports();
    mesh.printStats();
  }
  else if (cmd == "/help") {
    Serial.println("=== FreeNode Commands ===");
    Serial.println("  /ping   — send PING, measure RTT");
    Serial.println("  /stats  — show packet statistics");
    Serial.println("  /info   — show node info");
    Serial.println("  /help   — this help");
    Serial.println("  <text>  — send text to mesh");
  }
  else if (cmd.startsWith("/")) {
    Serial.println("Unknown command. Type /help");
  }
  else {
    // Обычный текст — отправляем в mesh
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
  Serial.println("╔═══════════════════════════════╗");
  Serial.println("║    F R E E N O D E  v0.2      ║");
  Serial.println("║    ESP32 Mesh Node             ║");
  Serial.println("║    FreeNod Co Unlimited        ║");
  Serial.println("╚═══════════════════════════════╝");
  Serial.println();

  // Инициализация mesh-роутера
  mesh.begin();
  mesh.onMessage(onMsg);  // <-- РЕГИСТРИРУЕМ CALLBACK!
  Serial.printf("MAC: %s\n", macToStr(mesh.mac()).c_str());
  Serial.printf("[DEBUG] sizeof(FNPacket)=%d offsetof(payload)=%d\n",
    sizeof(FNPacket), offsetof(FNPacket, payload));

  // Добавляем транспорт: ESP-NOW
  if (mesh.addTransport(&espnow)) {
    Serial.println("[OK] ESP-NOW transport active");
  } else {
    Serial.println("[FAIL] ESP-NOW transport failed!");
  }

  // Снижаем мощность Wi-Fi — меньше нагрев, для дома хватит
  esp_wifi_set_max_tx_power(40); // 10 dBm (по умолчанию 80 = 20 dBm)

  // TODO: Phase 2 — добавить BLE transport
  // BleTransport ble;
  // mesh.addTransport(&ble);

  // TODO: Phase 2 — добавить LoRa transport
  // LoraTransport lora;
  // mesh.addTransport(&lora);

  Serial.println();
  mesh.printTransports();
  Serial.println();
  Serial.println("Ready. Type message or /help");
  Serial.println("────────────────────────────────");

  // Мигнуть 3 раза — готов
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ── Main Loop ───────────────────────────────────────────────────
void loop() {
  // Обработка mesh (приём, дедупликация, ретрансляция)
  mesh.loop();

  // Обработка ввода из Serial Monitor
  if (Serial.available()) {
    String msg = Serial.readStringUntil('\n');
    handleCommand(msg);
  }

  // Управление LED
  if (ledOffTime && millis() > ledOffTime) {
    digitalWrite(LED_PIN, LOW);
    ledOffTime = 0;
  }

  // Пауза — даём процессору и радио отдохнуть, снижаем нагрев
  delay(10);
}
