/*
 * freenode_lora.ino — FreeNode Dual-Transport Mesh Node
 * Platform: LILYGO T3S3 V1.2 (ESP32-S3 + SX1262 + SSD1306 OLED)
 * Version: 0.4.0 — March 2026
 *
 * ИЗМЕНЕНИЯ v0.4:
 *   - Peer Table: таблица соседей с RSSI, transport, last_seen
 *   - Unicast PING: /ping [id] — пинг конкретного узла
 *   - /ping без аргумента — broadcast ping, но PONG отвечает
 *     только ближайший узел (первый кто получил, остальные
 *     видят PONG в эфире и молчат — FN_FLAG_PONG_SENT)
 *   - /peers — список известных соседей
 *   - ANNOUNCE пакет: узел анонсирует себя при старте и периодически
 *   - Peer timeout: узел удаляется из таблицы если молчит >60 сек
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
// Протокол FreeNode v0.4
// ═══════════════════════════════════════════════════════════════

#define FN_MAGIC          0xFE
#define FN_PROTO_VER      0x04

#define FN_TYPE_TEXT      0x00
#define FN_TYPE_PING      0x01
#define FN_TYPE_PONG      0x02
#define FN_TYPE_ROUTE     0x03
#define FN_TYPE_ACK       0x04
#define FN_TYPE_HEARTBEAT 0x05
#define FN_TYPE_ANNOUNCE  0x06  // NEW: анонс узла в сеть

#define FN_FLAG_RELAYED    0x01  // пакет уже ретранслировался
#define FN_FLAG_UNICAST    0x02  // NEW: пакет адресован конкретному узлу
#define FN_FLAG_PONG_SENT  0x04  // NEW: кто-то уже ответил на этот PING

#pragma pack(push, 1)
typedef struct {
  uint8_t  magic;
  uint8_t  version;
  uint8_t  src[6];       // MAC отправителя
  uint8_t  dst[6];       // FF:FF:FF:FF:FF:FF = broadcast, иначе unicast
  uint8_t  ttl;
  uint16_t id;
  uint8_t  type;
  uint8_t  flags;
  uint8_t  payloadLen;
  uint8_t  payload[200];
} FNPacket;
#pragma pack(pop)

#define FN_HEADER_SIZE (1+1+6+6+1+2+1+1+1)  // 20 байт

typedef enum {
  TRANSPORT_ESPNOW = 0,
  TRANSPORT_LORA   = 1
} TransportID;

// ═══════════════════════════════════════════════════════════════
// Peer Table
// ═══════════════════════════════════════════════════════════════

#define MAX_PEERS       16
#define PEER_TIMEOUT_MS 60000  // 60 сек молчания — считаем ушедшим

typedef struct {
  uint8_t     mac[6];
  char        id[6];        // последние 2 октета "AA:BB"
  int16_t     rssi;         // последний RSSI (LoRa) или 0 (ESP-NOW)
  TransportID transport;    // по какому транспорту последний раз слышали
  uint32_t    lastSeen;     // millis()
  uint32_t    rxCount;      // сколько пакетов от него получили
  bool        active;
} PeerEntry;

PeerEntry peers[MAX_PEERS];
uint8_t   peerCount = 0;

// Найти или создать запись пира
PeerEntry* peerGet(const uint8_t* mac) {
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active && memcmp(peers[i].mac, mac, 6) == 0)
      return &peers[i];
  }
  return nullptr;
}

PeerEntry* peerGetOrCreate(const uint8_t* mac) {
  PeerEntry* p = peerGet(mac);
  if (p) return p;
  // Найти свободный слот
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].active) {
      memset(&peers[i], 0, sizeof(PeerEntry));
      memcpy(peers[i].mac, mac, 6);
      sprintf(peers[i].id, "%02X:%02X", mac[4], mac[5]);
      peers[i].active = true;
      peerCount++;
      return &peers[i];
    }
  }
  return nullptr;  // таблица полная
}

void peerUpdate(const uint8_t* mac, TransportID via, int16_t rssi) {
  PeerEntry* p = peerGetOrCreate(mac);
  if (!p) return;
  p->lastSeen  = millis();
  p->transport = via;
  p->rssi      = rssi;
  p->rxCount++;
}

void peerCleanup() {
  uint32_t now = millis();
  for (int i = 0; i < MAX_PEERS; i++) {
    if (peers[i].active && (now - peers[i].lastSeen > PEER_TIMEOUT_MS)) {
      Serial.printf("[PEERS] %s timed out\n", peers[i].id);
      peers[i].active = false;
      peerCount--;
    }
  }
}

void peerPrint() {
  Serial.println("─── Peer Table ───");
  Serial.printf("  %-6s  %-8s  %-5s  %-8s  %s\n",
    "ID", "Transport", "RSSI", "LastSeen", "RX");
  bool any = false;
  for (int i = 0; i < MAX_PEERS; i++) {
    if (!peers[i].active) continue;
    any = true;
    uint32_t age = (millis() - peers[i].lastSeen) / 1000;
    Serial.printf("  %-6s  %-8s  %-5d  %4lu sec  %lu\n",
      peers[i].id,
      peers[i].transport == TRANSPORT_ESPNOW ? "ESP-NOW" : "LoRa",
      peers[i].rssi,
      age,
      peers[i].rxCount);
  }
  if (!any) Serial.println("  (empty)");
  Serial.println("──────────────────");
}

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

#define ESPNOW_PEER_TIMEOUT_MS  10000
#define ESPNOW_CHANNEL          1
#define DEFAULT_TTL             5

// ═══════════════════════════════════════════════════════════════
// Seen IDs дедупликация
// ═══════════════════════════════════════════════════════════════
#define MAX_SEEN 128
uint16_t seenIds[MAX_SEEN];
uint8_t  seenIdx = 0;

bool alreadySeen(uint16_t id) {
  for (int i = 0; i < MAX_SEEN; i++) if (seenIds[i] == id) return true;
  return false;
}
void markSeen(uint16_t id) {
  seenIds[seenIdx] = id;
  seenIdx = (seenIdx + 1) % MAX_SEEN;
}

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
uint32_t txCountEspNow = 0, txCountLora = 0;
uint32_t rxCountEspNow = 0, rxCountLora = 0;
uint32_t relayCount = 0, dropCount = 0;

volatile bool loraRxFlag    = false;
volatile bool espNowRxReady = false;
uint8_t espNowRxBuf[256];
int     espNowRxLen = 0;
bool    forceLoRa = false;

#define MAX_LINES 4
String screenLines[MAX_LINES];
int    lineCount = 0;

// ═══════════════════════════════════════════════════════════════
// Утилиты
// ═══════════════════════════════════════════════════════════════

void macShortToStr(uint8_t b1, uint8_t b2, char* out) {
  sprintf(out, "%02X:%02X", b1, b2);
}
bool isMyMac(const uint8_t* mac) { return memcmp(mac, myMac, 6) == 0; }
bool isBroadcast(const uint8_t* mac) {
  for (int i=0; i<6; i++) if (mac[i] != 0xFF) return false;
  return true;
}

TransportID chooseBestTransport() {
  if (!forceLoRa && (millis() - lastEspNowRx < ESPNOW_PEER_TIMEOUT_MS))
    return TRANSPORT_ESPNOW;
  return TRANSPORT_LORA;
}
const char* tName(TransportID t) {
  return (t == TRANSPORT_ESPNOW) ? "ESP-NOW" : "LoRa";
}
const char* tIcon(TransportID t) {
  return (t == TRANSPORT_ESPNOW) ? "W" : "L";
}

// Лучший транспорт к конкретному пиру (или общий если не знаем)
TransportID bestTransportToPeer(const uint8_t* mac) {
  PeerEntry* p = peerGet(mac);
  if (p) return p->transport;
  return chooseBestTransport();
}

void buildFNPacket(FNPacket& pkt, uint8_t type, uint8_t ttl,
                   const uint8_t* payload, uint8_t payloadLen) {
  memset(&pkt, 0, sizeof(pkt));
  pkt.magic = FN_MAGIC;  pkt.version = FN_PROTO_VER;
  memcpy(pkt.src, myMac, 6);  memset(pkt.dst, 0xFF, 6);
  pkt.ttl = ttl;  pkt.id = (uint16_t)esp_random();
  pkt.type = type;  pkt.flags = 0;  pkt.payloadLen = payloadLen;
  if (payload && payloadLen > 0) memcpy(pkt.payload, payload, payloadLen);
}

// ═══════════════════════════════════════════════════════════════
// Отправка
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
  return (chooseBestTransport() == TRANSPORT_ESPNOW)
    ? sendViaEspNow(pkt) : sendViaLora(pkt);
}
bool sendViaBestToPeer(FNPacket& pkt, const uint8_t* mac) {
  return (bestTransportToPeer(mac) == TRANSPORT_ESPNOW)
    ? sendViaEspNow(pkt) : sendViaLora(pkt);
}

// ═══════════════════════════════════════════════════════════════
// Cross-transport relay
// ═══════════════════════════════════════════════════════════════

void relayPacket(FNPacket& pkt, TransportID srcTransport) {
  if (pkt.ttl <= 1) return;
  FNPacket relay = pkt;
  relay.ttl--;
  relay.flags |= FN_FLAG_RELAYED;
  bool relayed = false;
  if (srcTransport != TRANSPORT_ESPNOW)
    if (sendViaEspNow(relay)) { relayed = true; }
  if (srcTransport != TRANSPORT_LORA)
    if (sendViaLora(relay))   { relayed = true; }
  if (relayed) {
    relayCount++;
    Serial.printf("[RELAY] id=%u ttl=%d\n", pkt.id, relay.ttl);
  }
}

// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════

void oledRefresh() {
  display.clearDisplay();
  display.setTextSize(1);  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("FN v0.4 [%s] P:%d", myMacStr, peerCount);
  display.setCursor(0, 10);
  display.printf("TX:%s R:%lu D:%lu",
    tName(chooseBestTransport()), relayCount, dropCount);
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
    for (int i = 0; i < MAX_LINES-1; i++) screenLines[i] = screenLines[i+1];
    screenLines[MAX_LINES-1] = line;
  } else { screenLines[lineCount++] = line; }
  oledRefresh();
}
void oledShowStatus(String l1, String l2 = "") {
  display.clearDisplay();  display.setTextSize(1);  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0,0);  display.println("FreeNode v0.4");
  display.drawLine(0,10,OLED_WIDTH,10,SSD1306_WHITE);
  display.setCursor(0,16); display.println(l1);
  if (l2.length()>0) { display.setCursor(0,28); display.println(l2); }
  display.display();
}

// ═══════════════════════════════════════════════════════════════
// Callbacks
// ═══════════════════════════════════════════════════════════════

void loraRxCallback(void) { loraRxFlag = true; }
void espNowRxCallback(const esp_now_recv_info_t *info,
                      const uint8_t *data, int len) {
  if (len > 0 && len <= 250 && !espNowRxReady) {
    memcpy((void*)espNowRxBuf, data, len);
    espNowRxLen = len;
    espNowRxReady = true;
  }
}

// ═══════════════════════════════════════════════════════════════
// Отправить ANNOUNCE (анонс себя в сеть)
// ═══════════════════════════════════════════════════════════════

void sendAnnounce() {
  // payload = myMacStr (6 байт "AA:BB")
  FNPacket pkt;
  buildFNPacket(pkt, FN_TYPE_ANNOUNCE, DEFAULT_TTL,
                (uint8_t*)myMacStr, strlen(myMacStr));
  markSeen(pkt.id);
  sendViaEspNow(pkt);
  sendViaLora(pkt);
  Serial.printf("[ANNOUNCE] Sent ID:%s\n", myMacStr);
}

// ═══════════════════════════════════════════════════════════════
// Обработка входящего пакета
// ═══════════════════════════════════════════════════════════════

void handleRxPacket(FNPacket& pkt, TransportID via,
                    float rssi = 0, float snr = 0) {
  if (pkt.magic != FN_MAGIC) { dropCount++; return; }
  if (isMyMac(pkt.src)) return;

  // Unicast: если пакет адресован не нам — только relay, не обрабатываем
  bool isForMe = isBroadcast(pkt.dst) || isMyMac(pkt.dst);

  if (!isForMe) {
    // Relay чужого unicast (если TTL позволяет)
    if (!alreadySeen(pkt.id) && pkt.ttl > 1) {
      markSeen(pkt.id);
      relayPacket(pkt, via);
    }
    return;
  }

  if (alreadySeen(pkt.id)) { dropCount++; return; }
  markSeen(pkt.id);

  // Обновляем peer table
  peerUpdate(pkt.src, via, (int16_t)rssi);

  if (via == TRANSPORT_ESPNOW) { lastEspNowRx = millis(); rxCountEspNow++; }
  else rxCountLora++;

  char senderStr[6];
  macShortToStr(pkt.src[4], pkt.src[5], senderStr);

  switch (pkt.type) {

    case FN_TYPE_TEXT: {
      char text[201];
      memcpy(text, pkt.payload, pkt.payloadLen);
      text[pkt.payloadLen] = '\0';
      Serial.printf("[RX %s] <%s> \"%s\"", tName(via), senderStr, text);
      if (via == TRANSPORT_LORA)
        Serial.printf(" RSSI:%.0f SNR:%.1f", rssi, snr);
      Serial.println();
      oledAddLine(String("[")+tIcon(via)+"]"+senderStr+"> "+String(text));
      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
      break;
    }

    case FN_TYPE_PING: {
      // Проверяем: если это broadcast PING и флаг PONG_SENT уже стоит —
      // кто-то уже ответил, молчим
      if (isBroadcast(pkt.dst) && (pkt.flags & FN_FLAG_PONG_SENT)) {
        Serial.printf("[PING] id=%u already answered, skip\n", pkt.id);
        break;
      }

      Serial.printf("[RX %s] PING from %s (id=%u %s)\n",
        tName(via), senderStr, pkt.id,
        (pkt.flags & FN_FLAG_UNICAST) ? "unicast" : "broadcast");
      oledAddLine(String("[")+tIcon(via)+"]"+senderStr+"> PING");

      // Отправляем PONG unicast обратно отправителю
      FNPacket pong;
      buildFNPacket(pong, FN_TYPE_PONG, DEFAULT_TTL, NULL, 0);
      memcpy(pong.dst, pkt.src, 6);  // unicast к отправителю PING
      pong.flags = FN_FLAG_UNICAST;
      // В payload кладём ID исходного PING для корреляции
      memcpy(pong.payload, &pkt.id, 2);
      pong.payloadLen = 2;
      markSeen(pong.id);
      sendViaBestToPeer(pong, pkt.src);

      // Анонсируем всем что мы ответили на этот PING
      // (broadcast флаг PONG_SENT чтобы другие не отвечали)
      if (isBroadcast(pkt.dst)) {
        FNPacket flag = pkt;
        flag.flags |= FN_FLAG_PONG_SENT;
        flag.ttl = 2;  // short TTL — только ближние соседи
        sendViaEspNow(flag);
        sendViaLora(flag);
      }

      oledAddLine(String(">PONG→")+senderStr);
      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
      break;
    }

    case FN_TYPE_PONG: {
      uint32_t rtt = 0;
      uint16_t pingId = 0;
      if (pkt.payloadLen >= 2) memcpy(&pingId, pkt.payload, 2);

      Serial.printf("[RX %s] PONG from %s (ping_id=%u)\n",
        tName(via), senderStr, pingId);
      oledAddLine(String("[")+tIcon(via)+"]"+senderStr+"> PONG");
      break;
    }

    case FN_TYPE_ANNOUNCE: {
      // Просто обновили peer table (уже сделано выше)
      Serial.printf("[ANNOUNCE] %s via %s RSSI:%.0f\n",
        senderStr, tName(via), rssi);
      break;
    }

    case FN_TYPE_HEARTBEAT:
      break;

    default:
      Serial.printf("[RX] Unknown type 0x%02X from %s\n", pkt.type, senderStr);
  }

  // Cross-transport relay (только broadcast, не unicast-чужой)
  if (pkt.ttl > 1 && pkt.type != FN_TYPE_HEARTBEAT
      && isBroadcast(pkt.dst)) {
    relayPacket(pkt, via);
  }
}

// ═══════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);  delay(1000);
  pinMode(LED_PIN, OUTPUT);  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  memset(seenIds, 0, sizeof(seenIds));
  memset(peers,   0, sizeof(peers));

  Serial.println("=== FreeNode v0.4 — Peer Table + Unicast Ping ===");

  Wire.begin(OLED_SDA, OLED_SCL);
  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR);
  oledShowStatus("Initializing...");

  WiFi.mode(WIFI_STA);  WiFi.disconnect();  delay(100);
  esp_efuse_mac_get_default(myMac);
  myMacShort[0] = myMac[4];  myMacShort[1] = myMac[5];
  macShortToStr(myMacShort[0], myMacShort[1], myMacStr);
  Serial.printf("[MAC] ID: %s\n", myMacStr);
  oledShowStatus("FreeNode v0.4", String("ID: ") + myMacStr);

  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);  delay(50);
  if (esp_now_init() == ESP_OK) {
    esp_now_peer_info_t p = {};
    memcpy(p.peer_addr, broadcastAddr, 6);
    p.channel = ESPNOW_CHANNEL;  p.encrypt = false;
    esp_now_add_peer(&p);
    esp_now_register_recv_cb(espNowRxCallback);
    Serial.println("[ESP-NOW] OK");
  } else Serial.println("[ESP-NOW] FAIL");

  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SW, LORA_POWER, LORA_PREAMBLE);
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] FAIL %d\n", state);
    while (true) delay(1000);
  }
  radio.setCurrentLimit(60.0);
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);
  radio.setPacketReceivedAction(loraRxCallback);
  radio.startReceive();
  Serial.println("[LoRa] OK");

  Serial.println();
  Serial.println("Commands:");
  Serial.println("  /ping          — broadcast ping (1 PONG ответ)");
  Serial.println("  /ping AA:BB    — unicast ping к узлу AA:BB");
  Serial.println("  /peers         — список соседей");
  Serial.println("  /status        — статистика");
  Serial.println("  /lora /auto    — выбор транспорта");
  Serial.println("  <text>         — отправить сообщение");

  delay(500);
  sendAnnounce();
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
      FNPacket pkt;  memset(&pkt, 0, sizeof(pkt));
      memcpy(&pkt, buf, min(len, sizeof(pkt)));
      handleRxPacket(pkt, TRANSPORT_LORA, radio.getRSSI(), radio.getSNR());
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[LoRa] CRC error");
    }
    radio.startReceive();
  }

  // ── ESP-NOW RX ──
  if (espNowRxReady) {
    espNowRxReady = false;
    FNPacket pkt;  memset(&pkt, 0, sizeof(pkt));
    memcpy(&pkt, espNowRxBuf, min((size_t)espNowRxLen, sizeof(pkt)));
    handleRxPacket(pkt, TRANSPORT_ESPNOW);
  }

  // ── Serial команды ──
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) {

    } else if (input == "/ping") {
      // Broadcast ping — ответит только ближайший
      FNPacket pkt;
      buildFNPacket(pkt, FN_TYPE_PING, DEFAULT_TTL, NULL, 0);
      markSeen(pkt.id);
      sendViaEspNow(pkt);
      sendViaLora(pkt);
      Serial.printf("[CMD] Broadcast PING id=%u\n", pkt.id);
      oledAddLine(">PING broadcast");

    } else if (input.startsWith("/ping ")) {
      // Unicast ping: /ping AA:BB
      String target = input.substring(6);
      target.trim();
      // Ищем пира по ID
      PeerEntry* peer = nullptr;
      for (int i = 0; i < MAX_PEERS; i++) {
        if (peers[i].active && String(peers[i].id) == target) {
          peer = &peers[i];
          break;
        }
      }
      if (!peer) {
        Serial.printf("[PING] Peer %s not found. Use /peers to list\n",
          target.c_str());
      } else {
        FNPacket pkt;
        buildFNPacket(pkt, FN_TYPE_PING, DEFAULT_TTL, NULL, 0);
        memcpy(pkt.dst, peer->mac, 6);  // unicast
        pkt.flags = FN_FLAG_UNICAST;
        markSeen(pkt.id);
        sendViaBestToPeer(pkt, peer->mac);
        Serial.printf("[CMD] Unicast PING → %s via %s\n",
          target.c_str(), tName(peer->transport));
        oledAddLine(String(">PING→") + target);
      }

    } else if (input == "/peers") {
      peerPrint();

    } else if (input == "/status") {
      Serial.println("─── FreeNode v0.4 ───");
      Serial.printf("  ID:        %s\n", myMacStr);
      Serial.printf("  Peers:     %d\n", peerCount);
      Serial.printf("  Transport: %s %s\n",
        tName(chooseBestTransport()), forceLoRa ? "(forced)" : "(auto)");
      Serial.printf("  ESP-NOW TX:%lu RX:%lu\n", txCountEspNow, rxCountEspNow);
      Serial.printf("  LoRa    TX:%lu RX:%lu\n", txCountLora, rxCountLora);
      Serial.printf("  Relayed:   %lu  Dropped: %lu\n", relayCount, dropCount);
      peerPrint();

    } else if (input == "/lora") {
      forceLoRa = true;  oledAddLine("! Forced LoRa");
    } else if (input == "/auto") {
      forceLoRa = false; oledAddLine("! Auto transport");

    } else {
      FNPacket pkt;
      uint8_t pl[200];
      uint8_t plen = (uint8_t)min((int)input.length(), 200);
      memcpy(pl, input.c_str(), plen);
      buildFNPacket(pkt, FN_TYPE_TEXT, DEFAULT_TTL, pl, plen);
      markSeen(pkt.id);
      sendViaEspNow(pkt);
      sendViaLora(pkt);
      oledAddLine(String("[W+L]>") + input.substring(0, 15));
      Serial.printf("[TX] \"%s\"\n", input.c_str());
    }
  }

  // ── Кнопка BOOT = broadcast PING ──
  static uint32_t lastBtn = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 500) {
    lastBtn = millis();
    FNPacket pkt;
    buildFNPacket(pkt, FN_TYPE_PING, DEFAULT_TTL, NULL, 0);
    markSeen(pkt.id);
    sendViaEspNow(pkt);
    sendViaLora(pkt);
    oledAddLine(">PING broadcast");
  }

  // ── OLED обновление ──
  static uint32_t lastOled = 0;
  if (millis() - lastOled > 2000) {
    lastOled = millis();
    oledRefresh();
  }

  // ── Peer cleanup (каждые 10 сек) ──
  static uint32_t lastCleanup = 0;
  if (millis() - lastCleanup > 10000) {
    lastCleanup = millis();
    peerCleanup();
  }

  // ── ANNOUNCE каждые 30 сек ──
  static uint32_t lastAnnounce = 0;
  if (millis() - lastAnnounce > 30000) {
    lastAnnounce = millis();
    sendAnnounce();
  }

  // ── ESP-NOW heartbeat каждые 5 сек ──
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
