/*
 * freenode_lora.ino — FreeNode Triple-Transport Mesh Node
 * Platform: LILYGO T3S3 V1.2 (ESP32-S3 + SX1262 + SSD1306 OLED)
 * Version: 0.5 — March 2026
 *
 * TRIPLE TRANSPORT:
 *   ESP-NOW (быстрый, ~100м) + LoRa (дальний, ~1км+) + BLE (10-100м)
 *
 * BLE: advertising-based через NimBLE. Данные в manufacturer data.
 * Legacy ADV лимит: ~9 байт текста. Для PING/PONG — хватает.
 *
 * Логика выбора транспорта:
 *   1. Если peer по ESP-NOW в последние 10 сек → ESP-NOW
 *   2. Иначе → LoRa
 *   3. BLE: всегда дублирует отправку (параллельный broadcast)
 *
 * Библиотеки (Library Manager):
 *   1. RadioLib by Jan Gromes
 *   2. Adafruit SSD1306 + Adafruit GFX Library
 *   3. NimBLE-Arduino by h2zero
 *
 * Arduino IDE:
 *   Board: "ESP32S3 Dev Module"
 *   USB CDC On Boot: "Enabled"
 *   Upload Speed: 115200
 *   Flash Size: 4MB
 *
 * ВАЖНО: Подключи антенну LoRa перед включением!
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
#include <NimBLEDevice.h>

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
// LoRa
// ═══════════════════════════════════════════════════════════════
#define LORA_FREQ     869.0
#define LORA_BW       125.0
#define LORA_SF       9
#define LORA_CR       7
#define LORA_SW       0x12
#define LORA_POWER    22
#define LORA_PREAMBLE 8

// ═══════════════════════════════════════════════════════════════
// Транспорт
// ═══════════════════════════════════════════════════════════════
#define ESPNOW_PEER_TIMEOUT_MS  10000
#define ESPNOW_CHANNEL  1

// BLE
#define BLE_FN_COMPANY_ID    0xFFFF
#define BLE_FN_MAGIC_BYTE    0xFE
#define BLE_ADV_INTERVAL_MS  250
#define BLE_SCAN_INTERVAL    80
#define BLE_SCAN_WINDOW      40
#define BLE_ADV_DURATION_SEC 1
#define BLE_MFG_OVERHEAD     3
#define BLE_MAX_PKT_DATA     26

// ═══════════════════════════════════════════════════════════════
// Протокол FreeNode v0.2
// ═══════════════════════════════════════════════════════════════
#define FN_MAGIC    0xFE
#define FN_VERSION  0x02
#define FN_TYPE_TEXT      0x01
#define FN_TYPE_PING      0x02
#define FN_TYPE_PONG      0x03
#define FN_TYPE_HEARTBEAT 0x04

#define FN_HEADER_SIZE  6
#define FN_MAX_PAYLOAD  200

// ═══════════════════════════════════════════════════════════════
// Объекты
// ═══════════════════════════════════════════════════════════════
SPIClass loraSpi(HSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

uint8_t myMac[6];
uint8_t myMacShort[2];
char myMacStr[6];

uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ═══════════════════════════════════════════════════════════════
// Транспорт — состояние
// ═══════════════════════════════════════════════════════════════
typedef enum { TRANSPORT_ESPNOW, TRANSPORT_LORA, TRANSPORT_BLE } TransportType;
TransportType activeTransport = TRANSPORT_LORA;

uint32_t lastEspNowRx = 0;
uint32_t lastBleRx    = 0;

uint32_t txCountEspNow = 0;
uint32_t txCountLora   = 0;
uint32_t txCountBle    = 0;
uint32_t rxCountEspNow = 0;
uint32_t rxCountLora   = 0;
uint32_t rxCountBle    = 0;

volatile bool loraRxFlag = false;

volatile bool espNowRxReady = false;
uint8_t espNowRxBuf[256];
int espNowRxLen = 0;
uint8_t espNowRxMac[6];

// BLE RX буфер
#define BLE_RX_BUF_SIZE 8
struct BleRxItem {
  uint8_t data[64];
  int     len;
  bool    ready;
};
static BleRxItem bleRxBuf[BLE_RX_BUF_SIZE];
static volatile uint8_t bleRxHead = 0;
static volatile uint8_t bleRxTail = 0;

// BLE ADV state
static volatile bool     bleIsAdv    = false;
static volatile uint32_t bleAdvStart = 0;
NimBLEAdvertising* bleAdv  = nullptr;
NimBLEScan*        bleScan = nullptr;

#define MAX_LINES 4
String screenLines[MAX_LINES];
int lineCount = 0;

// ═══════════════════════════════════════════════════════════════
// Утилиты
// ═══════════════════════════════════════════════════════════════
void macShortToStr(uint8_t b1, uint8_t b2, char* out) {
  sprintf(out, "%02X:%02X", b1, b2);
}

int buildPacket(uint8_t* buf, uint8_t type, const char* payload, int payloadLen) {
  buf[0] = FN_MAGIC;
  buf[1] = FN_VERSION;
  buf[2] = type;
  buf[3] = myMacShort[0];
  buf[4] = myMacShort[1];
  buf[5] = (uint8_t)payloadLen;
  if (payloadLen > 0 && payload != NULL) {
    memcpy(buf + FN_HEADER_SIZE, payload, payloadLen);
  }
  return FN_HEADER_SIZE + payloadLen;
}

bool parsePacket(const uint8_t* buf, int len, uint8_t* type, uint8_t* senderMac,
                 char* payload, int* payloadLen) {
  if (len < FN_HEADER_SIZE) return false;
  if (buf[0] != FN_MAGIC) return false;
  if (buf[1] != FN_VERSION) return false;
  *type = buf[2];
  senderMac[0] = buf[3];
  senderMac[1] = buf[4];
  *payloadLen = buf[5];
  if (*payloadLen > FN_MAX_PAYLOAD) return false;
  if (len < FN_HEADER_SIZE + *payloadLen) return false;
  if (*payloadLen > 0) {
    memcpy(payload, buf + FN_HEADER_SIZE, *payloadLen);
  }
  payload[*payloadLen] = '\0';
  return true;
}

TransportType chooseBestTransport() {
  if (millis() - lastEspNowRx < ESPNOW_PEER_TIMEOUT_MS) {
    return TRANSPORT_ESPNOW;
  }
  return TRANSPORT_LORA;
}

const char* tName(TransportType t) {
  switch (t) {
    case TRANSPORT_ESPNOW: return "ESP-NOW";
    case TRANSPORT_LORA:   return "LoRa";
    case TRANSPORT_BLE:    return "BLE";
    default:               return "?";
  }
}

const char* tIcon(TransportType t) {
  switch (t) {
    case TRANSPORT_ESPNOW: return "W";
    case TRANSPORT_LORA:   return "L";
    case TRANSPORT_BLE:    return "B";
    default:               return "?";
  }
}

// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════
void oledAddLine(String line) {
  if (lineCount >= MAX_LINES) {
    for (int i = 0; i < MAX_LINES - 1; i++)
      screenLines[i] = screenLines[i + 1];
    screenLines[MAX_LINES - 1] = line;
  } else {
    screenLines[lineCount++] = line;
  }
  oledRefresh();
}

void oledRefresh() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.printf("FreeNode v0.5 [%s]", myMacStr);

  display.setCursor(0, 10);
  TransportType best = chooseBestTransport();
  display.printf("TX:%s E:%lu L:%lu B:%lu",
    tName(best), txCountEspNow, txCountLora, txCountBle);

  display.drawLine(0, 19, OLED_WIDTH, 19, SSD1306_WHITE);

  for (int i = 0; i < lineCount && i < MAX_LINES; i++) {
    display.setCursor(0, 22 + i * 10);
    display.print(screenLines[i].substring(0, 21));
  }
  display.display();
}

void oledShowStatus(String line1, String line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("FreeNode v0.5");
  display.drawLine(0, 10, OLED_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(0, 16);
  display.println(line1);
  if (line2.length() > 0) {
    display.setCursor(0, 28);
    display.println(line2);
  }
  display.display();
}

// ═══════════════════════════════════════════════════════════════
// BLE Scan Callback
// ═══════════════════════════════════════════════════════════════
class FNBLEScanCB : public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* dev) override {
    if (!dev->haveManufacturerData()) return;

    std::string raw = dev->getManufacturerData().toString();
    size_t len = raw.length();
    if (len < BLE_MFG_OVERHEAD + FN_HEADER_SIZE) return;

    const uint8_t* d = (const uint8_t*)raw.data();
    uint16_t cid = d[0] | (d[1] << 8);
    if (cid != BLE_FN_COMPANY_ID) return;
    if (d[2] != BLE_FN_MAGIC_BYTE) return;

    size_t pktLen = len - BLE_MFG_OVERHEAD;
    uint8_t next = (bleRxHead + 1) % BLE_RX_BUF_SIZE;
    if (next == bleRxTail) return;

    memset(&bleRxBuf[bleRxHead], 0, sizeof(BleRxItem));
    memcpy(bleRxBuf[bleRxHead].data, d + BLE_MFG_OVERHEAD, min(pktLen, (size_t)64));
    bleRxBuf[bleRxHead].len = pktLen;
    bleRxBuf[bleRxHead].ready = true;
    bleRxHead = next;
  }

  void onDiscoveryComplete(const NimBLEScanResults& results) override {
    NimBLEDevice::getScan()->start(0, false);
  }
};

static FNBLEScanCB fnBleScanCb;

// ═══════════════════════════════════════════════════════════════
// Callbacks — LoRa, ESP-NOW
// ═══════════════════════════════════════════════════════════════
void loraRxCallback(void) { loraRxFlag = true; }

void espNowRxCallback(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len > 0 && len <= 250 && !espNowRxReady) {
    memcpy((void*)espNowRxBuf, data, len);
    espNowRxLen = len;
    memcpy((void*)espNowRxMac, info->src_addr, 6);
    espNowRxReady = true;
  }
}

// ═══════════════════════════════════════════════════════════════
// Отправка
// ═══════════════════════════════════════════════════════════════

bool sendBleRaw(uint8_t* pktData, int pktLen) {
  uint8_t mfg[31];
  mfg[0] = BLE_FN_COMPANY_ID & 0xFF;
  mfg[1] = (BLE_FN_COMPANY_ID >> 8) & 0xFF;
  mfg[2] = BLE_FN_MAGIC_BYTE;
  memcpy(mfg + BLE_MFG_OVERHEAD, pktData, pktLen);
  int total = BLE_MFG_OVERHEAD + pktLen;

  bleScan->stop();

  NimBLEAdvertisementData advData;
  advData.setManufacturerData(std::string((char*)mfg, total));
  bleAdv->setAdvertisementData(advData);
  bleAdv->start(BLE_ADV_DURATION_SEC);

  bleIsAdv = true;
  bleAdvStart = millis();
  txCountBle++;

  Serial.printf("[TX BLE] %d bytes\n", total);
  return true;
}

bool sendPrimary(uint8_t* packet, int len) {
  TransportType best = chooseBestTransport();
  bool ok = false;

  if (best == TRANSPORT_ESPNOW) {
    esp_err_t r = esp_now_send(broadcastAddr, packet, len);
    ok = (r == ESP_OK);
    if (ok) txCountEspNow++;
    Serial.printf("[TX ESP-NOW] %s len=%d\n", ok ? "OK" : "FAIL", len);
  } else {
    int state = radio.transmit(packet, len);
    ok = (state == RADIOLIB_ERR_NONE);
    if (ok) txCountLora++;
    Serial.printf("[TX LoRa] %s len=%d\n", ok ? "OK" : "FAIL", len);
    radio.startReceive();
  }
  return ok;
}

bool sendBle(uint8_t* packet, int len) {
  int bleLen = len;
  if (bleLen > BLE_MAX_PKT_DATA) {
    int maxPayload = BLE_MAX_PKT_DATA - FN_HEADER_SIZE;
    if (maxPayload < 0) return false;
    bleLen = FN_HEADER_SIZE + maxPayload;
    uint8_t blePkt[64];
    memcpy(blePkt, packet, FN_HEADER_SIZE);
    blePkt[5] = maxPayload;
    memcpy(blePkt + FN_HEADER_SIZE, packet + FN_HEADER_SIZE, maxPayload);
    return sendBleRaw(blePkt, bleLen);
  }
  return sendBleRaw(packet, bleLen);
}

void sendAll(uint8_t* packet, int len) {
  sendPrimary(packet, len);
  sendBle(packet, len);
}

// ═══════════════════════════════════════════════════════════════
// Обработка входящего пакета
// ═══════════════════════════════════════════════════════════════
void handleRxPacket(const uint8_t* raw, int rawLen, TransportType via,
                    float rssi = 0, float snr = 0) {
  uint8_t type;
  uint8_t senderMac[2];
  char payload[FN_MAX_PAYLOAD + 1];
  int payloadLen;

  if (!parsePacket(raw, rawLen, &type, senderMac, payload, &payloadLen)) {
    Serial.printf("[RX %s] Invalid pkt len=%d\n", tName(via), rawLen);
    return;
  }

  if (senderMac[0] == myMacShort[0] && senderMac[1] == myMacShort[1]) return;

  char senderStr[6];
  macShortToStr(senderMac[0], senderMac[1], senderStr);

  if (via == TRANSPORT_ESPNOW) { lastEspNowRx = millis(); rxCountEspNow++; }
  else if (via == TRANSPORT_LORA) { rxCountLora++; }
  else if (via == TRANSPORT_BLE) { lastBleRx = millis(); rxCountBle++; }

  switch (type) {
    case FN_TYPE_TEXT: {
      Serial.printf("[RX %s] <%s> \"%s\"", tName(via), senderStr, payload);
      if (via == TRANSPORT_LORA) Serial.printf(" RSSI:%.0f SNR:%.1f", rssi, snr);
      Serial.println();

      oledAddLine(String("[") + tIcon(via) + "]" + senderStr + "> " + String(payload));
      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
      break;
    }
    case FN_TYPE_PING: {
      Serial.printf("[RX %s] PING from %s\n", tName(via), senderStr);
      oledAddLine(String("[") + tIcon(via) + "]" + senderStr + "> PING");

      uint8_t pkt[FN_HEADER_SIZE];
      int pktLen = buildPacket(pkt, FN_TYPE_PONG, NULL, 0);
      sendAll(pkt, pktLen);
      oledAddLine(">PONG");

      digitalWrite(LED_PIN, HIGH); delay(50); digitalWrite(LED_PIN, LOW);
      break;
    }
    case FN_TYPE_PONG: {
      Serial.printf("[RX %s] PONG from %s\n", tName(via), senderStr);
      oledAddLine(String("[") + tIcon(via) + "]" + senderStr + "> PONG");
      break;
    }
    case FN_TYPE_HEARTBEAT: {
      break;
    }
    default:
      Serial.printf("[RX %s] Unknown 0x%02X from %s\n", tName(via), type, senderStr);
  }
}

// ═══════════════════════════════════════════════════════════════
// BLE update
// ═══════════════════════════════════════════════════════════════
void bleUpdate() {
  if (bleIsAdv && millis() - bleAdvStart > (BLE_ADV_DURATION_SEC * 1000 + 100)) {
    bleAdv->stop();
    bleIsAdv = false;
    bleScan->start(0, false);
  }
}

// ═══════════════════════════════════════════════════════════════
// Setup
// ═══════════════════════════════════════════════════════════════
bool forceLoRa = false;

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  Serial.println();
  Serial.println("=== FreeNode v0.5 — Triple Transport ===");
  Serial.println("Platform: LILYGO T3S3 V1.2");
  Serial.println("Transports: ESP-NOW + LoRa + BLE");

  // ── OLED ──
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] FAILED");
  }
  oledShowStatus("Initializing...");

  // ── WiFi + MAC ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  esp_efuse_mac_get_default(myMac);
  myMacShort[0] = myMac[4];
  myMacShort[1] = myMac[5];
  macShortToStr(myMacShort[0], myMacShort[1], myMacStr);
  Serial.printf("[MAC] %02X:%02X:%02X:%02X:%02X:%02X → ID: %s\n",
    myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5], myMacStr);

  // ── ESP-NOW ──
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] FAILED");
  } else {
    Serial.println("[ESP-NOW] OK");
    esp_now_peer_info_t pi = {};
    memcpy(pi.peer_addr, broadcastAddr, 6);
    pi.channel = ESPNOW_CHANNEL;
    pi.encrypt = false;
    esp_now_add_peer(&pi);
    esp_now_register_recv_cb(espNowRxCallback);
  }

  // ── LoRa SX1262 ──
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  int state = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
                          LORA_SW, LORA_POWER, LORA_PREAMBLE);
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("[LoRa] OK");
  } else {
    Serial.printf("[LoRa] FAILED code %d\n", state);
    oledShowStatus("LoRa FAIL!", String("Err:") + String(state));
    while (true) delay(1000);
  }
  radio.setCurrentLimit(60.0);
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);
  radio.setPacketReceivedAction(loraRxCallback);
  radio.startReceive();

  // ── BLE (NimBLE) ──
  Serial.print("[BLE] Init... ");
  NimBLEDevice::init("FN");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);

  bleScan = NimBLEDevice::getScan();
  bleScan->setScanCallbacks(&fnBleScanCb, false);
  bleScan->setActiveScan(false);
  bleScan->setInterval(BLE_SCAN_INTERVAL);
  bleScan->setWindow(BLE_SCAN_WINDOW);
  bleScan->setMaxResults(0);

  if (bleScan->start(0, false)) {
    Serial.println("OK (scan active)");
  } else {
    Serial.println("scan FAILED");
  }

  bleAdv = NimBLEDevice::getAdvertising();
  uint16_t advUnits = (BLE_ADV_INTERVAL_MS * 1000) / 625;
  bleAdv->setMinInterval(advUnits);
  bleAdv->setMaxInterval(advUnits);
  bleAdv->setConnectableMode(BLE_GAP_CONN_MODE_NON);
  bleAdv->setScannable(false);

  // ── Ready ──
  Serial.println();
  Serial.printf("[LoRa] %.1fMHz SF%d BW%.0fkHz %ddBm\n",
    LORA_FREQ, LORA_SF, LORA_BW, LORA_POWER);
  Serial.printf("[ESP-NOW] CH:%d\n", ESPNOW_CHANNEL);
  Serial.printf("[BLE] ADV interval:%dms\n", BLE_ADV_INTERVAL_MS);
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  <text>  /ping  /status  /lora  /auto");
  Serial.println();

  oledShowStatus("Ready!", String("ID:") + myMacStr);
  delay(1000);

  // Startup heartbeat
  uint8_t hb[FN_HEADER_SIZE];
  buildPacket(hb, FN_TYPE_HEARTBEAT, NULL, 0);
  esp_now_send(broadcastAddr, hb, FN_HEADER_SIZE);

  oledAddLine("Waiting...");

  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ═══════════════════════════════════════════════════════════════
// Loop
// ═══════════════════════════════════════════════════════════════
void loop() {

  // ── 1. LoRa RX ──
  if (loraRxFlag) {
    loraRxFlag = false;
    uint8_t buf[256];
    int state = radio.readData(buf, 0);
    if (state == RADIOLIB_ERR_NONE) {
      size_t len = radio.getPacketLength();
      handleRxPacket(buf, len, TRANSPORT_LORA, radio.getRSSI(), radio.getSNR());
    }
    radio.startReceive();
  }

  // ── 2. ESP-NOW RX ──
  if (espNowRxReady) {
    espNowRxReady = false;
    handleRxPacket(espNowRxBuf, espNowRxLen, TRANSPORT_ESPNOW);
  }

  // ── 3. BLE RX ──
  if (bleRxHead != bleRxTail) {
    BleRxItem& item = bleRxBuf[bleRxTail];
    if (item.ready) {
      handleRxPacket(item.data, item.len, TRANSPORT_BLE);
      item.ready = false;
    }
    bleRxTail = (bleRxTail + 1) % BLE_RX_BUF_SIZE;
  }

  // ── 4. BLE state machine ──
  bleUpdate();

  // ── 5. Serial input ──
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    if (input.length() == 0) { /* skip */ }
    else if (input == "/ping") {
      uint8_t pkt[FN_HEADER_SIZE];
      int pktLen = buildPacket(pkt, FN_TYPE_PING, NULL, 0);
      sendAll(pkt, pktLen);
      oledAddLine(">PING (all)");
    }
    else if (input == "/status") {
      Serial.println("─── FreeNode v0.5 Status ───");
      Serial.printf("  ID:         %s\n", myMacStr);
      Serial.printf("  Primary TX: %s %s\n",
        tName(forceLoRa ? TRANSPORT_LORA : chooseBestTransport()),
        forceLoRa ? "(forced)" : "(auto)");
      Serial.printf("  ESP-NOW:    TX:%lu RX:%lu\n", txCountEspNow, rxCountEspNow);
      Serial.printf("  LoRa:       TX:%lu RX:%lu\n", txCountLora, rxCountLora);
      Serial.printf("  BLE:        TX:%lu RX:%lu\n", txCountBle, rxCountBle);
      if (lastEspNowRx == 0)
        Serial.println("  ESP-NOW peer: never");
      else
        Serial.printf("  ESP-NOW peer: %lu ms ago\n", millis() - lastEspNowRx);
      if (lastBleRx == 0)
        Serial.println("  BLE peer: never");
      else
        Serial.printf("  BLE peer: %lu ms ago\n", millis() - lastBleRx);
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
      oledAddLine("! Auto");
    }
    else {
      // Текстовое сообщение
      uint8_t pkt[FN_HEADER_SIZE + FN_MAX_PAYLOAD];
      int pl = input.length();
      if (pl > FN_MAX_PAYLOAD) pl = FN_MAX_PAYLOAD;
      int pktLen = buildPacket(pkt, FN_TYPE_TEXT, input.c_str(), pl);

      if (forceLoRa) {
        int st = radio.transmit(pkt, pktLen);
        if (st == RADIOLIB_ERR_NONE) txCountLora++;
        oledAddLine(String("[L]>") + input.substring(0, 17));
        radio.startReceive();
      } else {
        TransportType best = chooseBestTransport();
        if (best == TRANSPORT_ESPNOW) {
          esp_err_t r = esp_now_send(broadcastAddr, pkt, pktLen);
          if (r == ESP_OK) txCountEspNow++;
          oledAddLine(String("[W]>") + input.substring(0, 17));
        } else {
          int st = radio.transmit(pkt, pktLen);
          if (st == RADIOLIB_ERR_NONE) txCountLora++;
          oledAddLine(String("[L]>") + input.substring(0, 17));
          radio.startReceive();
        }
      }
      // Дубль через BLE
      sendBle(pkt, pktLen);
    }
  }

  // ── 6. Button = PING ──
  static uint32_t lastBtn = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastBtn > 500) {
    lastBtn = millis();
    uint8_t pkt[FN_HEADER_SIZE];
    int pktLen = buildPacket(pkt, FN_TYPE_PING, NULL, 0);
    sendAll(pkt, pktLen);
    oledAddLine(">PING (all)");
  }

  // ── 7. OLED refresh ──
  static uint32_t lastOled = 0;
  if (millis() - lastOled > 2000) {
    lastOled = millis();
    oledRefresh();
  }

  // ── 8. ESP-NOW heartbeat (5 sec) ──
  static uint32_t lastHB = 0;
  if (!forceLoRa && millis() - lastHB > 5000) {
    lastHB = millis();
    uint8_t pkt[FN_HEADER_SIZE];
    buildPacket(pkt, FN_TYPE_HEARTBEAT, NULL, 0);
    esp_now_send(broadcastAddr, pkt, FN_HEADER_SIZE);
  }

  delay(10);
}
