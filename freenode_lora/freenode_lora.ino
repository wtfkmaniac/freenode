/*
 * freenode_lora.ino — FreeNode Dual-Transport Mesh Node
 * Platform: LILYGO T3S3 V1.2 (ESP32-S3 + SX1262 + SSD1306 OLED)
 * Version: 0.2 — March 2026
 *
 * DUAL TRANSPORT: ESP-NOW (быстрый, ~100м) + LoRa (дальний, ~1км+)
 *
 * Логика выбора транспорта:
 *   - Если peer отвечал по ESP-NOW в последние 10 сек → ESP-NOW
 *   - Иначе → LoRa (fallback)
 *   - Все сообщения broadcast (пока без адресации)
 *   - На OLED: транспорт + последние 2 октета MAC отправителя
 *
 * Библиотеки:
 *   1. RadioLib by Jan Gromes
 *   2. Adafruit SSD1306
 *   3. Adafruit GFX Library
 *   (ESP-NOW и WiFi — встроены в ESP32)
 *
 * Настройки Arduino IDE:
 *   Board: "ESP32S3 Dev Module"
 *   USB CDC On Boot: "Enabled"
 *   Upload Speed: 115200
 *   Flash Size: 4MB
 *
 * ВАЖНО: Подключи антенну перед включением!
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
// ПИНЫ T3S3 V1.2
// ═══════════════════════════════════════════════════════════════

// LoRa SX1262 — SPI
#define LORA_SCK    5
#define LORA_MISO   3
#define LORA_MOSI   6
#define LORA_CS     7
#define LORA_RST    8
#define LORA_DIO1   33
#define LORA_BUSY   34

// OLED SSD1306 — I2C
#define OLED_SDA    18
#define OLED_SCL    17
#define OLED_WIDTH  128
#define OLED_HEIGHT 64
#define OLED_ADDR   0x3C

// LED & Button
#define LED_PIN     37
#define BUTTON_PIN  0

// ═══════════════════════════════════════════════════════════════
// LoRa параметры
// ═══════════════════════════════════════════════════════════════
#define LORA_FREQ     869.0
#define LORA_BW       125.0
#define LORA_SF       9        // SF9 для баланса
#define LORA_CR       7
#define LORA_SW       0x12     // FreeNode sync word
#define LORA_POWER    22       // Максимум SX1262 (+22 dBm)
#define LORA_PREAMBLE 8

// ═══════════════════════════════════════════════════════════════
// Транспорт — конфигурация
// ═══════════════════════════════════════════════════════════════

// Таймаут: если ESP-NOW peer не слышен дольше — fallback на LoRa
#define ESPNOW_PEER_TIMEOUT_MS  10000

// Канал WiFi для ESP-NOW (должен совпадать на обеих платах)
#define ESPNOW_CHANNEL  1

// ═══════════════════════════════════════════════════════════════
// Протокол пакетов FreeNode v0.2
// ═══════════════════════════════════════════════════════════════
//
// Формат пакета (одинаковый для LoRa и ESP-NOW):
//   [0]    — magic byte 0xFN (0xFE)
//   [1]    — версия протокола (0x02)
//   [2]    — тип: 0x01=TEXT, 0x02=PING, 0x03=PONG, 0x04=HEARTBEAT
//   [3..4] — последние 2 октета MAC отправителя
//   [5]    — длина payload
//   [6..]  — payload (текст)
//

#define FN_MAGIC    0xFE
#define FN_VERSION  0x02
#define FN_TYPE_TEXT      0x01
#define FN_TYPE_PING      0x02
#define FN_TYPE_PONG      0x03
#define FN_TYPE_HEARTBEAT 0x04  // Тихий пинг — не вызывает PONG, не спамит OLED

#define FN_HEADER_SIZE  6
#define FN_MAX_PAYLOAD  200

// ═══════════════════════════════════════════════════════════════
// Объекты
// ═══════════════════════════════════════════════════════════════

SPIClass loraSpi(HSPI);
SX1262 radio = new Module(LORA_CS, LORA_DIO1, LORA_RST, LORA_BUSY, loraSpi);
Adafruit_SSD1306 display(OLED_WIDTH, OLED_HEIGHT, &Wire, -1);

// Наш MAC (последние 2 октета)
uint8_t myMac[6];
uint8_t myMacShort[2];  // Для отображения
char myMacStr[6];        // "A3:F1"

// ESP-NOW broadcast адрес
uint8_t broadcastAddr[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// Состояние транспортов
enum Transport { TRANSPORT_ESPNOW, TRANSPORT_LORA };
Transport activeTransport = TRANSPORT_LORA;  // По умолчанию LoRa

// Когда последний раз видели peer по ESP-NOW
uint32_t lastEspNowRx = 0;

// Статистика
uint32_t txCountEspNow = 0;
uint32_t txCountLora = 0;
uint32_t rxCountEspNow = 0;
uint32_t rxCountLora = 0;

// Флаг приёма LoRa
volatile bool loraRxFlag = false;

// Буфер входящего ESP-NOW пакета (обработка в loop)
volatile bool espNowRxReady = false;
uint8_t espNowRxBuf[256];
int espNowRxLen = 0;
uint8_t espNowRxMac[6];

// OLED строки
#define MAX_LINES 4
String screenLines[MAX_LINES];
int lineCount = 0;

// ═══════════════════════════════════════════════════════════════
// Утилиты
// ═══════════════════════════════════════════════════════════════

// MAC 2 октета → строка "A3:F1"
void macShortToStr(uint8_t b1, uint8_t b2, char* out) {
  sprintf(out, "%02X:%02X", b1, b2);
}

// Собрать пакет FreeNode
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

// Разобрать пакет, вернуть true если валидный
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

// Определить лучший транспорт
Transport chooseBestTransport() {
  if (millis() - lastEspNowRx < ESPNOW_PEER_TIMEOUT_MS) {
    return TRANSPORT_ESPNOW;
  }
  return TRANSPORT_LORA;
}

const char* transportName(Transport t) {
  return t == TRANSPORT_ESPNOW ? "ESP-NOW" : "LoRa";
}

// Иконка транспорта для OLED (компактная)
const char* transportIcon(Transport t) {
  return t == TRANSPORT_ESPNOW ? "W" : "L";  // W=WiFi/ESP-NOW, L=LoRa
}

// ═══════════════════════════════════════════════════════════════
// OLED
// ═══════════════════════════════════════════════════════════════

void oledAddLine(String line) {
  if (lineCount >= MAX_LINES) {
    for (int i = 0; i < MAX_LINES - 1; i++) {
      screenLines[i] = screenLines[i + 1];
    }
    screenLines[MAX_LINES - 1] = line;
  } else {
    screenLines[lineCount] = line;
    lineCount++;
  }
  oledRefresh();
}

void oledRefresh() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // ── Строка 1: заголовок + мой MAC ──
  display.setCursor(0, 0);
  display.printf("FreeNode v0.2 [%s]", myMacStr);

  // ── Строка 2: статус транспортов ──
  display.setCursor(0, 10);
  Transport best = chooseBestTransport();
  display.printf("TX:%s  E:%lu L:%lu",
    transportName(best), txCountEspNow, txCountLora);

  display.drawLine(0, 19, OLED_WIDTH, 19, SSD1306_WHITE);

  // ── Сообщения ──
  for (int i = 0; i < lineCount && i < MAX_LINES; i++) {
    display.setCursor(0, 22 + i * 10);
    if (screenLines[i].length() > 21) {
      display.print(screenLines[i].substring(0, 21));
    } else {
      display.print(screenLines[i]);
    }
  }

  display.display();
}

void oledShowStatus(String line1, String line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("FreeNode v0.2");
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
// Callbacks
// ═══════════════════════════════════════════════════════════════

// LoRa RX callback
void loraRxCallback(void) {
  loraRxFlag = true;
}

// ESP-NOW RX callback
void espNowRxCallback(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len > 0 && len <= 250 && !espNowRxReady) {
    memcpy((void*)espNowRxBuf, data, len);
    espNowRxLen = len;
    memcpy((void*)espNowRxMac, info->src_addr, 6);
    espNowRxReady = true;
  }
}

// ═══════════════════════════════════════════════════════════════
// Отправка через выбранный транспорт
// ═══════════════════════════════════════════════════════════════

bool sendPacket(uint8_t* packet, int len) {
  Transport best = chooseBestTransport();
  bool ok = false;

  if (best == TRANSPORT_ESPNOW) {
    // Отправляем по ESP-NOW (broadcast)
    esp_err_t result = esp_now_send(broadcastAddr, packet, len);
    ok = (result == ESP_OK);
    if (ok) txCountEspNow++;
    Serial.printf("[TX ESP-NOW] %s, len=%d\n", ok ? "OK" : "FAIL", len);
  } else {
    // Отправляем по LoRa
    int state = radio.transmit(packet, len);
    ok = (state == RADIOLIB_ERR_NONE);
    if (ok) txCountLora++;
    Serial.printf("[TX LoRa] %s, len=%d\n", ok ? "OK" : "FAIL", len);
    // Вернуться в RX
    radio.startReceive();
  }

  return ok;
}

// ═══════════════════════════════════════════════════════════════
// Обработка входящего пакета (общая для обоих транспортов)
// ═══════════════════════════════════════════════════════════════

void handleRxPacket(const uint8_t* raw, int rawLen, Transport via,
                    float rssi = 0, float snr = 0) {
  uint8_t type;
  uint8_t senderMac[2];
  char payload[FN_MAX_PAYLOAD + 1];
  int payloadLen;

  if (!parsePacket(raw, rawLen, &type, senderMac, payload, &payloadLen)) {
    Serial.printf("[RX %s] Invalid packet, len=%d\n", transportName(via), rawLen);
    return;
  }

  // ── Фильтр: игнорируем свои собственные пакеты ──
  if (senderMac[0] == myMacShort[0] && senderMac[1] == myMacShort[1]) {
    return;  // Это наш broadcast вернулся — игнорируем
  }

  char senderStr[6];
  macShortToStr(senderMac[0], senderMac[1], senderStr);

  // Обновляем метку времени ESP-NOW
  if (via == TRANSPORT_ESPNOW) {
    lastEspNowRx = millis();
    rxCountEspNow++;
  } else {
    rxCountLora++;
  }

  switch (type) {
    case FN_TYPE_TEXT: {
      Serial.printf("[RX %s] <%s> \"%s\"", transportName(via), senderStr, payload);
      if (via == TRANSPORT_LORA) {
        Serial.printf(" RSSI:%.0f SNR:%.1f", rssi, snr);
      }
      Serial.println();

      // На OLED: [W]A3:F1> Hello  или  [L]A3:F1> Hello
      String oledLine = String("[") + transportIcon(via) + "]"
                        + senderStr + "> " + String(payload);
      oledAddLine(oledLine);

      // LED
      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      break;
    }

    case FN_TYPE_PING: {
      Serial.printf("[RX %s] PING from %s\n", transportName(via), senderStr);
      oledAddLine(String("[") + transportIcon(via) + "]" + senderStr + "> PING");

      // Автоответ PONG
      uint8_t pkt[FN_HEADER_SIZE];
      int pktLen = buildPacket(pkt, FN_TYPE_PONG, NULL, 0);
      sendPacket(pkt, pktLen);
      oledAddLine(String(">PONG via ") + transportName(chooseBestTransport()));

      digitalWrite(LED_PIN, HIGH);
      delay(50);
      digitalWrite(LED_PIN, LOW);
      break;
    }

    case FN_TYPE_PONG: {
      Serial.printf("[RX %s] PONG from %s\n", transportName(via), senderStr);
      oledAddLine(String("[") + transportIcon(via) + "]" + senderStr + "> PONG");
      break;
    }

    case FN_TYPE_HEARTBEAT: {
      // Тихий — только обновляем таймер ESP-NOW (уже сделано выше)
      // Не отвечаем PONG, не спамим на OLED
      break;
    }

    default:
      Serial.printf("[RX %s] Unknown type 0x%02X from %s\n",
        transportName(via), type, senderStr);
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

  Serial.println();
  Serial.println("=== FreeNode v0.2 — Dual Transport ===");
  Serial.println("Platform: LILYGO T3S3 V1.2");
  Serial.println("Transports: ESP-NOW + LoRa SX1262");

  // ── OLED Init ──
  Wire.begin(OLED_SDA, OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("[OLED] FAILED");
  } else {
    Serial.println("[OLED] OK");
  }
  oledShowStatus("Initializing...");

  // ── WiFi Init (нужно ДО чтения MAC!) ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);

  // ── Получаем MAC (после WiFi.mode!) ──
  esp_efuse_mac_get_default(myMac);
  myMacShort[0] = myMac[4];
  myMacShort[1] = myMac[5];
  macShortToStr(myMacShort[0], myMacShort[1], myMacStr);
  Serial.printf("[MAC] %02X:%02X:%02X:%02X:%02X:%02X → ID: %s\n",
    myMac[0], myMac[1], myMac[2], myMac[3], myMac[4], myMac[5], myMacStr);
  oledShowStatus("Initializing...", String("ID: ") + myMacStr);

  // ── Фиксируем канал WiFi ──
  esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
  delay(50);

  if (esp_now_init() != ESP_OK) {
    Serial.println("[ESP-NOW] Init FAILED");
    oledShowStatus("ESP-NOW FAIL!");
  } else {
    Serial.println("[ESP-NOW] Init OK");

    // Регистрируем broadcast peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddr, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      Serial.println("[ESP-NOW] Add broadcast peer FAILED");
    }

    // Callback на приём
    esp_now_register_recv_cb(espNowRxCallback);
  }

  // ── LoRa SX1262 Init ──
  loraSpi.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);

  Serial.print("[LoRa] Init SX1262... ");
  int state = radio.begin(
    LORA_FREQ, LORA_BW, LORA_SF, LORA_CR,
    LORA_SW, LORA_POWER, LORA_PREAMBLE
  );

  if (state == RADIOLIB_ERR_NONE) {
    Serial.println("OK!");
  } else {
    Serial.printf("FAILED, code %d\n", state);
    oledShowStatus("LoRa INIT FAIL!", String("Error: ") + String(state));
    while (true) { delay(1000); }
  }

  radio.setCurrentLimit(60.0);
  radio.setDio2AsRfSwitch(true);
  radio.setCRC(true);

  // RX с прерыванием
  radio.setPacketReceivedAction(loraRxCallback);
  state = radio.startReceive();
  if (state != RADIOLIB_ERR_NONE) {
    Serial.printf("[LoRa] startReceive failed: %d\n", state);
  }

  // ── Ready ──
  Serial.println();
  Serial.printf("[LoRa] Freq:%.1fMHz SF%d BW%.0fkHz Power:%ddBm\n",
    LORA_FREQ, LORA_SF, LORA_BW, LORA_POWER);
  Serial.printf("[ESP-NOW] Channel:%d\n", ESPNOW_CHANNEL);
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  <text>    — send message");
  Serial.println("  /ping     — send PING");
  Serial.println("  /status   — show stats");
  Serial.println("  /lora     — force LoRa");
  Serial.println("  /auto     — auto transport");
  Serial.println();

  oledShowStatus("Ready!", String("ID:") + myMacStr + " CH:" + String(ESPNOW_CHANNEL));
  delay(1500);

  // ── Startup HEARTBEAT по ESP-NOW для обнаружения соседей ──
  {
    uint8_t pkt[FN_HEADER_SIZE];
    int pktLen = buildPacket(pkt, FN_TYPE_HEARTBEAT, NULL, 0);
    esp_err_t r = esp_now_send(broadcastAddr, pkt, pktLen);
    Serial.printf("[STARTUP] ESP-NOW heartbeat: %s\n", r == ESP_OK ? "OK" : "FAIL");
  }

  oledAddLine("Waiting...");

  // Мигнуть 3 раза
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(100);
    digitalWrite(LED_PIN, LOW);  delay(100);
  }
}

// ═══════════════════════════════════════════════════════════════
// Принудительный выбор транспорта (через /lora, /auto)
// ═══════════════════════════════════════════════════════════════
bool forceLoRa = false;

// ═══════════════════════════════════════════════════════════════
// Main Loop
// ═══════════════════════════════════════════════════════════════

void loop() {

  // ── 1. Приём LoRa ──
  if (loraRxFlag) {
    loraRxFlag = false;

    uint8_t buf[256];
    size_t len = 0;
    int state = radio.readData(buf, 0);

    if (state == RADIOLIB_ERR_NONE) {
      len = radio.getPacketLength();
      float rssi = radio.getRSSI();
      float snr = radio.getSNR();
      handleRxPacket(buf, len, TRANSPORT_LORA, rssi, snr);
    } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      Serial.println("[LoRa RX] CRC error");
    } else {
      Serial.printf("[LoRa RX] Error %d\n", state);
    }

    radio.startReceive();
  }

  // ── 2. Приём ESP-NOW ──
  if (espNowRxReady) {
    espNowRxReady = false;
    handleRxPacket(espNowRxBuf, espNowRxLen, TRANSPORT_ESPNOW);
  }

  // ── 3. Отправка из Serial ──
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() == 0) {
      // skip
    }
    else if (input == "/ping") {
      uint8_t pkt[FN_HEADER_SIZE];
      int pktLen = buildPacket(pkt, FN_TYPE_PING, NULL, 0);

      Transport best = forceLoRa ? TRANSPORT_LORA : chooseBestTransport();
      Serial.printf("[CMD] PING via %s\n", transportName(best));
      oledAddLine(String(">PING via ") + transportName(best));
      sendPacket(pkt, pktLen);
    }
    else if (input == "/status") {
      Serial.println("─── FreeNode Status ───");
      Serial.printf("  My ID:       %s\n", myMacStr);
      Serial.printf("  Transport:   %s %s\n",
        transportName(forceLoRa ? TRANSPORT_LORA : chooseBestTransport()),
        forceLoRa ? "(forced)" : "(auto)");
      Serial.printf("  ESP-NOW TX:  %lu\n", txCountEspNow);
      Serial.printf("  ESP-NOW RX:  %lu\n", rxCountEspNow);
      Serial.printf("  LoRa TX:     %lu\n", txCountLora);
      Serial.printf("  LoRa RX:     %lu\n", rxCountLora);
      uint32_t espAge = millis() - lastEspNowRx;
      if (lastEspNowRx == 0) {
        Serial.println("  ESP-NOW peer: never seen");
      } else {
        Serial.printf("  ESP-NOW peer: %lu ms ago\n", espAge);
      }
      Serial.println("───────────────────────");
    }
    else if (input == "/lora") {
      forceLoRa = true;
      Serial.println("[CMD] Forced LoRa mode");
      oledAddLine("! Forced LoRa");
    }
    else if (input == "/auto") {
      forceLoRa = false;
      Serial.println("[CMD] Auto transport mode");
      oledAddLine("! Auto transport");
    }
    else {
      // Текстовое сообщение
      uint8_t pkt[FN_HEADER_SIZE + FN_MAX_PAYLOAD];
      int payloadLen = input.length();
      if (payloadLen > FN_MAX_PAYLOAD) payloadLen = FN_MAX_PAYLOAD;
      int pktLen = buildPacket(pkt, FN_TYPE_TEXT, input.c_str(), payloadLen);

      // Выбираем транспорт
      if (forceLoRa) {
        // LoRa only
        int state = radio.transmit(pkt, pktLen);
        bool ok = (state == RADIOLIB_ERR_NONE);
        if (ok) txCountLora++;
        Serial.printf("[TX LoRa] \"%s\" %s\n", input.c_str(), ok ? "OK" : "FAIL");
        oledAddLine(String("[L]>") + input.substring(0, 17));
        radio.startReceive();
      } else {
        Transport best = chooseBestTransport();

        if (best == TRANSPORT_ESPNOW) {
          // Дублируем: ESP-NOW (основной) + LoRa (для дальних)
          esp_err_t r = esp_now_send(broadcastAddr, pkt, pktLen);
          if (r == ESP_OK) txCountEspNow++;
          Serial.printf("[TX ESP-NOW] \"%s\" %s\n", input.c_str(),
            r == ESP_OK ? "OK" : "FAIL");
          oledAddLine(String("[W]>") + input.substring(0, 17));
        } else {
          int state = radio.transmit(pkt, pktLen);
          bool ok = (state == RADIOLIB_ERR_NONE);
          if (ok) txCountLora++;
          Serial.printf("[TX LoRa] \"%s\" %s\n", input.c_str(), ok ? "OK" : "FAIL");
          oledAddLine(String("[L]>") + input.substring(0, 17));
          radio.startReceive();
        }
      }
    }
  }

  // ── 4. Кнопка BOOT = PING ──
  static uint32_t lastButton = 0;
  if (digitalRead(BUTTON_PIN) == LOW && millis() - lastButton > 500) {
    lastButton = millis();

    uint8_t pkt[FN_HEADER_SIZE];
    int pktLen = buildPacket(pkt, FN_TYPE_PING, NULL, 0);

    Transport best = forceLoRa ? TRANSPORT_LORA : chooseBestTransport();
    Serial.printf("[BUTTON] PING via %s\n", transportName(best));
    oledAddLine(String(">PING via ") + transportName(best));
    sendPacket(pkt, pktLen);
  }

  // ── 5. Периодическое обновление OLED (раз в 2 сек) ──
  static uint32_t lastOledUpdate = 0;
  if (millis() - lastOledUpdate > 2000) {
    lastOledUpdate = millis();
    oledRefresh();
  }

  // ── 6. ESP-NOW heartbeat (тихий, каждые 5 сек) ──
  // Отправляем HEARTBEAT по ESP-NOW чтобы соседи знали что мы рядом
  // HEARTBEAT не вызывает PONG и не спамит OLED
  static uint32_t lastHeartbeat = 0;
  if (!forceLoRa && millis() - lastHeartbeat > 5000) {
    lastHeartbeat = millis();
    uint8_t pkt[FN_HEADER_SIZE];
    int pktLen = buildPacket(pkt, FN_TYPE_HEARTBEAT, NULL, 0);
    esp_now_send(broadcastAddr, pkt, pktLen);
  }

  delay(10);
}
