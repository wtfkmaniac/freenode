/*
 * freenode_lora.ino — FreeNode Dual-Transport Mesh Node
 * Platform: LILYGO T3S3 V1.2 (ESP32-S3 + SX1262 + SSD1306 OLED)
 * Version: 0.3.0 — March 2026
 *
 * ИЗМЕНЕНИЯ v0.3:
 *   - Unified protocol v0.3: magic (0xFE) + version в каждом пакете
 *   - Seen IDs дедупликация (кольцевой буфер 128 ID)
 *   - Cross-transport relay: пришло по LoRa → пересылаем по ESP-NOW и наоборот
 *   - FN_FLAG_RELAYED: пакет не ретранслируется дважды через один транспорт
 *   - Совместимость с freenode_mesh: одинаковый FNPacket, одинаковые типы
 *
 * Библиотеки:
 *   1. RadioLib by Jan Gromes
 *   2. Adafruit SSD1306
 *   3. Adafruit GFX Library
 *
 * Настройки Arduino IDE:
 *   Board: "ESP32S3 Dev Module"
 *   USB CDC On Boot: "Enabled"
 *   Upload Speed: 115200
 *   Flash Size: 4MB
 */

#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>

// ═══════════════════════════════════════════════════════════════
// Подключаем унифицированный протокол
// ═══════════════════════════════════════════════════════════════
// FNPacket, FN_MAGIC, FN_TYPE_*, FN_FLAG_RELAYED, FN_HEADER_SIZE
// определены в transport.h

// Копируем нужные определения (без .h инклюда в .ino — дублируем для Arduino)
#define FN_MAGIC          0xFE
#define FN_PROTO_VER      0x03
#define FN_TYPE_TEXT      0x00
#define FN_TYPE_PING      0x01
#define FN_TYPE_PONG      0x02
#define FN_TYPE_ROUTE     0x03
#define FN_TYPE_ACK       0x04
#define FN_TYPE_HEARTBEAT 0x05
#define FN_FLAG_RELAYED   0x01

#pragma pack(push, 1)
struct FNPacket {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  src[6];
  uint8_t  dst[6];
  uint8_t  ttl;
  uint16_t id;
  uint8_t  type;
  uint8_t  flags;
  uint8_t  payloadLen;
  uint8_t  payload[200];
};
#pragma pack(pop)

#define FN_HEADER_SIZE  (1+1+6+6+1+2+1+1+1)  // 20 байт

// ═══════════════════════════════════════════════════════════════
// ПИНЫ T3S3 V1.2
// ═══════════════════════════════════════════════════════════════
#define LORA_SCK    5
#define LORA_MISO   3
#define LORA_MOSI   6
#define LORA_CS     7
#define LORA_RST    8
#define LORA_DIO1   33
#define LORA_BUSY   34

#define OLED_SDA    18
#define OLED_SCL    17
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C

#define LED_PIN     37
#define BUTTON_PIN  0

// ═══════════════════════════════════════════════════════════════
// LoRa параметры
// ═══════════════════════════════════════════════════════════════
#define LORA_FREQ     869.0
#define LORA_BW       125.0
#define LORA_SF       9
#define LORA_CR       7
#define LORA_SW       0x12
#define LORA_POWER    22
#define LORA_PREAMBLE 8

// ═══════════════════════════════════════════════════════════════
// Конфигурация транспортов
// ═══════════════════════════════════════════════════════════════
#define ESPNOW_PEER_TIMEOUT_MS  10000
#define ESPNOW_CHANNEL          1
#define DEFAULT_TTL             5

// ═══════════════════════════════════════════════════════════════
// Seen IDs дедупликация
// ═══════════════════════════════════════════════════════════════
#define MAX_SEEN  128

uint16_t seenIds[MAX_SEEN];
uint8_t  seenIdx = 0;

bool alreadySeen(uint16_t id) {
  for (int i = 0; i < MAX_SEEN; i++) {
    if (seenIds[i] == id) return true;
  }
  return false;
}

void markSeen(uint16_t id) {
  seenIds[seenIdx] = id;
  seenIdx = (seenIdx + 1) % MAX_SEEN;
}

// ═══════════════════════════════════════════════════════════════
// Транспорт enum
// ═══════════════════════════════════════════════════════════════
enum TransportID { TRANSPORT_ESPNOW = 0, TRANSPORT_LORA = 1 };

// ═══════════════════════════════════════════════════════════════
// Объекты
// ═══════════════════════════════════════════════════════════════
SPIClass loraSpi(HSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

uint8_t myMac[6];
uint8_t myMacShort[2];
char    myMacStr[6];

uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint32_t lastEspNowRx = 0;

// Статистика
uint32_t txCountEspNow = 0, txCountLora = 0;
uint32_t rxCountEspNow = 0, rxCountLora = 0;
uint32_t relayCount = 0, dropCount = 0;

// Флаги приёма
volatile bool loraRxFlag = false;

volatile bool espNowRxReady = false;
uint8_t espNowRxBuf[256];
int espNowRxLen = 0;

// Принудительный режим
bool forceLoRa = false;

// OLED
#define MAX_LINES 4
String screenLines[MAX_LINES];
int lineCount = 0;

// ═══════════════════════════════════════════════════════════════
// Утилиты
// ═══════════════════════════════════════════════════════════════

void macShortToStr(uint8_t b1, uint8_t b2, char* out) {
  sprintf(out, "%02X:%02X", b1, b2);
}

bool isMyMac(const uint8_t* mac) {
  return memcmp(mac, myMac, 6) == 0;
}

// Выбор лучшего транспорта
TransportID chooseBestTransport() {
  if (!forceLoRa && (millis() - lastEspNowRx < ESPNOW_PEER_TIMEOUT_MS)) {
    return TRANSPORT_ESPNOW;
  }
  return TRANSPORT_LORA;
}

const char* transportName(TransportID t) {
  return t == TRANSPORT_ESPNOW ? "ESP-NOW" : "LoRa";
}

const char* transportIcon(TransportID t) {
  return t == TRANSPORT_ESPNOW ? "W" : "L";
}

// ═══════════════════════════════════════════════════════════════
// Сборка пакета FNPacket v0.3
// ═══════════════════════════════════════════════════════════════

void buildFNPacket(FNPacket& pkt, uint8_t type, uint8_t ttl,
                   const uint8_t* payload, uint8_t payloadLen) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic      = FN_MAGIC;
  pkt.version    = FN_PROTO_VER;
  memcpy(pkt.src, myMac, 6);
  memset(pkt.dst, 0xFF, 6);
  pkt.ttl        = ttl;
  pkt.id         = (uint16_t)esp_random();
  pkt.type       = type;
  pkt.flags      = 0;
  pkt.payloadLen = payloadLen;
  if (payload && payloadLen > 0) {
    memcpy(pkt.payload, payload, payloadLen);
  }
}

// ═══════════════════════════════════════════════════════════════
// Отправка через конкретный транспорт
// ═══════════════════════════════════════════════════════════════

bool sendViaEspNow(FNPacket& pkt) {
  size_t sz = FN_HEADER_SIZE + pkt.payloadLen;
  if (sz > 250) sz = 250;
  esp_err_t r = esp_now_send(broadcastAddr, (uint8_t*)&pkt, sz);
  if (r == ESP_OK) { txCountEspNow++; return true; }
  return false;
}

bool sendViaLora(FNPacket& pkt) {
  size_t sz = FN_HEADER_SIZE + pkt.payloadLen;
  int state = radio.transmit((uint8_t*)&pkt, sz);
  radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) { txCountLora++; return true; }
  return false;
}

bool sendViaBest(FNPacket& pkt) {
  if (chooseBestTransport() == TRANSPORT_ESPNOW) {
    return sendViaEspNow(pkt);
  }
  return sendViaLora(pkt);
}

// ═══════════════════════════════════════════════════════════════
// Cross-transport relay
// Пришло с транспорта A → отправляем через транспорт B
// ═══════════════════════════════════════════════════════════════

void relayPacket(FNPacket& pkt, TransportID sourceTransport) {
  if (pkt.ttl <= 1) return;

  FNPacket relay = pkt;
  relay.ttl--;
  relay.flags |= FN_FLAG_RELAYED;

  // Отправляем через ВСЕ транспорты кроме источника
  bool relayed = false;

  if (sourceTransport != TRANSPORT_ESPNOW) {
    if (sendViaEspNow(relay)) {
      relayed = true;
      Serial.printf("[RELAY] id=%u LoRa→ESP-NOW\n", pkt.id);
    }
  }

  if (sourceTransport != TRANSPORT_LORA) {
    if (sendViaLora(relay)) {
      relayed = true;
      Serial.printf("[RELAY] id=%u ESP-NOW→LoRa\n", pkt.id);
    }
  }

  if (relayed) relayCount++;
}

// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════

void oledRefresh() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("FreeNode v0.3 [%s]", myMacStr);

  display.setCursor(0, 10);
  display.printf("TX:%s R:%lu D:%lu",
    transportName(chooseBestTransport()), relayCount, dropCount);

  display.drawLine(0, 19, OLED_WIDTH, 19, SSD1306_WHITE);

  for (int i = 0; i < lineCount && i < MAX_LINES; i++) {
    display.setCursor(0, 22 + i * 10);
    String line = screenLines[i];
    if (line.length() > 21) line = line.substring(0, 21);
    display.print(line);
  }
  display.display();
}

void oledAddLine(String line) {
  if (lineCount >= MAX_LINES) {
    for (int i = 0; i < MAX_LINES - 1; i++) screenLines[i] = screenLines[i+1];
    screenLines[MAX_LINES - 1] = line;
  } else {
    screenLines[lineCount++] = line;
  }
  oledRefresh();
}

void oledShowStatus(String l1, String l2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);  display.println("FreeNode v0.3");
  display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(0, 16); display.println(l1);
  if (l2.length() > 0) { display.setCursor(0, 28); display.println(l2); }
  display.display();
}

// ═══════════════════════════════════════════════════════════════
// Callbacks
// ═══════════════════════════════════════════════════════════════

void loraRxCallback(void) {
  loraRxFlag = true;
}

void espNowRxCallback(const esp_now_recv_info_t *info,
                      const uint8_t *data, int len) {
  if (len > 0 && len <= 250 && !espNowRxReady) {
    memcpy((void*)espNowRxBuf, data, len);
    espNowRxLen = len;
    espNowRxReady = true;
  }
}

// ═══════════════════════════════════════════════════════════════
// Обработка входящего пакета (общая логика)
// ═══════════════════════════════════════════════════════════════

void handleRxPacket(FNPacket& pkt, TransportID via,
                    float rssi = 0, float snr = 0) {

  // Проверяем magic — принимаем v0.2 и v0.3
  if (pkt.magic != FN_MAGIC) {
    // Попробуем обратную совместимость: у v0.2 нет magic в структуре,
    // пакет мог быть скопирован как есть — не дропаем, логируем
    Serial.printf("[RX] Bad magic 0x%02X, dropping\n", pkt.magic);
    dropCount++;
    return;
  }

  // Игнорируем свои пакеты
  if (isMyMac(pkt.src)) return;

  // Дедупликация
  if (alreadySeen(pkt.id)) {
    dropCount++;
    return;
  }
  markSeen(pkt.id);

  // Обновляем таймер ESP-NOW
  if (via == TRANSPORT_ESPNOW) {
    lastEspNowRx = millis();
    rxCountEspNow++;
  } else {
    rxCountLora++;
  }

  // Строка для отображения отправителя
  char senderStr[6];
  macShortToStr(pkt.src[4], pkt.src[5], senderStr);

  // ── Обработка по типу ──────────────────────────────────────
  switch (pkt.type) {

    case FN_TYPE_TEXT: {
      char text[201];
      memcpy(text, pkt.payload, pkt.payloadLen);
      text[pkt.payloadLen] = '\0';

      Serial.printf("[RX %s] <%s> \"%s\"", transportName(via), senderStr, text);
      if (via == TRANSPORT_LORA)
        Serial.printf(" RSSI:%.0f SNR:%.1f", rssi, snr);
      Serial.println();

      String oledLine = String("[") + transportIcon(via) + "]"
                      + senderStr + "> " + String(text);
      oledAddLine(oledLine);

      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
      break;
    }

    case FN_TYPE_PING: {
      Serial.printf("[RX %s] PING from %s\n", transportName(via), senderStr);
      oledAddLine(String("[") + transportIcon(via) + "]" + senderStr + "> PING");

      // Отвечаем PONG через лучший транспорт
      FNPacket pong;
      buildFNPacket(pong, FN_TYPE_PONG, DEFAULT_TTL, NULL, 0);
      markSeen(pong.id);
      sendViaBest(pong);

      oledAddLine(String(">PONG via ") + transportName(chooseBestTransport()));
      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
      break;
    }

    case FN_TYPE_PONG: {
      Serial.printf("[RX %s] PONG from %s\n", transportName(via), senderStr);
      oledAddLine(String("[") + transportIcon(via) + "]" + senderStr + "> PONG");
      break;
    }

    case FN_TYPE_HEARTBEAT: {
      // Тихий — только обновляем таймер (уже сделано выше)
      break;
    }

    default:
      Serial.printf("[RX %s] Unknown type 0x%02X from %s\n",
        transportName(via), pkt.type, senderStr);
  }

  // ── Cross-transport relay ──────────────────────────────────
  // Ретранслируем если TTL позволяет
  // HEARTBEAT не ретранслируем — он локальный
  if (pkt.ttl > 1 && pkt.type != FN_TYPE_HEARTBEAT) {
    relayPacket(pkt, via);
  }
}

// ═══════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  memset(seenIds, 0, sizeof(seenIds));

  Serial.println();
  Serial.println("=== FreeNode v0.3 — Cross-Transport Mesh ===");
  Serial.println("Platform: LILYGO T3S3 V1.2");
  Serial.println("Transports: ESP-NOW + LoRa SX1262");
  Serial.println("Features: Dedup + Cross-Transport Relay");

  // OLED
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] FAILED");
  }
  oledShowStatus("Initializing...");

  // WiFi + MAC
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_efuse_mac_get_default(myMac);
  myMacShort[0] = myMac[4];
  myMacShort[1] = myMac[5];
  macShortToStr(myMacShort[0], myMacShort[1], myMacStr);
  Serial.printf("[MAC] %02X:%02X:%02X:%02X:%02X:%02X → ID: %s\n",
    myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5], myMacStr);
  oledShowStatus("Initializing...", String("ID: ") + myMacStr);

  // ESP-NOW
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED");
  } else {
    Serial.println("[ESP-NOW] Init OK");
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddr, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    esp_now_register_recv_cb(espNowRxCallback);
  }

  // LoRa SX1262
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  Serial.print("[LoRa] Init SX1262... ");
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SW, LORA_POWER, LORA_PREAMBLE);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("OK!");
  } else {
    Serial.printf("FAILED, code %d\n", state);
    oledShowStatus("LoRa INIT FAIL!", String("Error: ") + String(state));
    while (true) delay(1000);
  }

  radio.setCurrentLimit(60.0);
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);
  radio.setPacketReceivedAction(loraRxCallback);
  radio.startReceive();

  Serial.println();
  Serial.printf("[LoRa] %.1fMHz SF%d BW%.0fkHz +%ddBm\n",
    LORA_FREQ, LORA_SF, LORA_BW, LORA_POWER);
  Serial.printf("[Relay] Cross-transport relay: ENABLED\n");
  Serial.printf("[Dedup] Seen IDs table: %d slots\n", MAX_SEEN);
  Serial.println();
  Serial.println("Commands: <text> /ping /status /lora /auto /relay");

  oledShowStatus("Ready! v0.3", String("ID:") + myMacStr);
  delay(1000);

  // Startup heartbeat
  FNPacket hb;
  buildFNPacket(hb, FN_TYPE_HEARTBEAT, 1, NULL, 0);
  markSeen(hb.id);
  sendViaEspNow(hb);

  oledAddLine("Listening...");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ═══════════════════════════════════════════════════════════════
// Main Loop
// ═══════════════════════════════════════════════════════════════

void loop() {

  // ── LoRa RX ──
  if (loraRxFlag) {
    loraRxFlag = false;

    uint8_t buf[256];
    int state = radio.readData(buf, 0);

    if (state == RADIOLIB_ERR_NONE) {
      size_t len = radio.getPacketLength();
      float rssi = radio.getRSSI();
      float snr  = radio.getSNR();

      FNPacket pkt;
      memset(&pkt, 0, sizeof(pkt));
      memcpy(&pkt, buf, min(len, sizeof(pkt)));
      handleRxPacket(pkt, TRANSPORT_LORA, rssi, snr);
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[LoRa RX] CRC error");
    }

    radio.startReceive();
  }

  // ── ESP-NOW RX ──
  if (espNowRxReady) {
    espNowRxReady = false;

    FNPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    memcpy(&pkt, espNowRxBuf, min((size_t)espNowRxLen, sizeof(pkt)));
    handleRxPacket(pkt, TRANSPORT_ESPNOW);
  }

  // ── Serial команды ──
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) {}

    else if (input == "/ping") {
      FNPacket pkt;
      buildFNPacket(pkt, FN_TYPE_PING, DEFAULT_TTL, NULL, 0);
      markSeen(pkt.id);
      TransportID best = chooseBestTransport();
      sendViaBest(pkt);
      Serial.printf("[CMD] PING via %s\n", transportName(best));
      oledAddLine(String(">PING via ") + transportName(best));
    }

    else if (input == "/status") {
      Serial.println("─── FreeNode v0.3 Status ───");
      Serial.printf("  ID:          %s\n", myMacStr);
      Serial.printf("  Transport:   %s %s\n",
        transportName(chooseBestTransport()), forceLoRa ? "(forced)" : "(auto)");
      Serial.printf("  ESP-NOW TX:  %lu  RX: %lu\n", txCountEspNow, rxCountEspNow);
      Serial.printf("  LoRa    TX:  %lu  RX: %lu\n", txCountLora, rxCountLora);
      Serial.printf("  Relayed:     %lu\n", relayCount);
      Serial.printf("  Dropped:     %lu\n", dropCount);
      uint32_t age = lastEspNowRx ? millis() - lastEspNowRx : 0;
      Serial.printf("  ESP-NOW peer: %s\n",
        lastEspNowRx ? String(String(age) + "ms ago").c_str() : "never");
      Serial.println("────────────────────────────");
    }

    else if (input == "/lora") {
      forceLoRa = true;
      Serial.println("[CMD] Forced LoRa");
      oledAddLine("! Forced LoRa");
    }
    else if (input == "/auto") {
      forceLoRa = false;
      Serial.println("[CMD] Auto transport");
      oledAddLine("! Auto transport");
    }
    else if (input == "/relay") {
      Serial.printf("[RELAY] Packets relayed so far: %lu\n", relayCount);
    }

    else {
      // Текстовое сообщение
      FNPacket pkt;
      uint8_t pl[200];
      uint8_t plen = min((int)input.length(), 200);
      memcpy(pl, input.c_str(), plen);
      buildFNPacket(pkt, FN_TYPE_TEXT, DEFAULT_TTL, pl, plen);
      markSeen(pkt.id);

      TransportID best = chooseBestTransport();
      if (forceLoRa || best == TRANSPORT_LORA) {
        sendViaLora(pkt);
        oledAddLine(String("[L]>") + input.substring(0, 17));
      } else {
        // При ESP-NOW в ближней зоне — дублируем и по LoRa для дальних узлов
        sendViaEspNow(pkt);
        sendViaLora(pkt);
        oledAddLine(String("[W+L]>") + input.substring(0, 15));
      }
      Serial.printf("[TX] \"%s\" via %s\n", input.c_str(),
        forceLoRa ? "LoRa" : "ESP-NOW+LoRa");
    }
  }

  // ── Кнопка BOOT = PING ──
  static uint32_t lastButton = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButton > 500) {
    lastButton = millis();
    FNPacket pkt;
    buildFNPacket(pkt, FN_TYPE_PING, DEFAULT_TTL, NULL, 0);
    markSeen(pkt.id);
    sendViaBest(pkt);
    oledAddLine(String(">PING via ") + transportName(chooseBestTransport()));
  }

  // ── Периодическое обновление OLED ──
  static uint32_t lastOled = 0;
  if (millis() - lastOled > 2000) {
    lastOled = millis();
    oledRefresh();
  }

  // ── Heartbeat каждые 5 сек ──
  static uint32_t lastHB = 0;
  if (!forceLoRa && millis() - lastHB > 5000) {
    lastHB = millis();
    FNPacket hb;
    buildFNPacket(hb, FN_TYPE_HEARTBEAT, 1, NULL, 0);
    markSeen(hb.id);
    sendViaEspNow(hb);
  }

  delay(10);
}
