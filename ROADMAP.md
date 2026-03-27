# FreeNode Roadmap

## Готово ✅

### v0.1 — ESP-NOW Mesh (freenode_mesh)
- ESP-NOW broadcast mesh между ESP32
- Flooding с TTL и дедупликацией
- Команды: /ping, /stats, /info

### v0.2 — Dual Transport (freenode_lora)
- LILYGO T3S3 V1.2 (ESP32-S3 + SX1262 + OLED)
- ESP-NOW + LoRa одновременно
- Автовыбор транспорта по качеству сигнала
- HEARTBEAT для обнаружения соседей

### v0.3 — Cross-Transport Relay + Unified Protocol
- Единый протокол FNPacket (magic + version + flags)
- Кросс-совместимость freenode_mesh ↔ freenode_lora
- Seen IDs дедупликация в lora-версии
- Cross-transport relay: LoRa↔ESP-NOW
- FN_FLAG_RELAYED против петель

### v0.4 — Peer Table + Unicast PING
- Таблица соседей (до 16 узлов): MAC, transport, RSSI, lastSeen
- ANNOUNCE пакет при старте и каждые 30 сек
- Unicast PING: /ping AA:BB — ровно 1 PONG
- Broadcast PING с FN_FLAG_PONG_SENT — тоже ровно 1 PONG
- Peer timeout 60 сек + автоочистка

---

## В работе 🔧

### v0.5 — BLE Keyboard Input
- BLE HID Host на NimBLE (без внешних зависимостей)
- Подключение любой стандартной BLE клавиатуры
- Буфер ввода с отображением на OLED
- Отправка в mesh по Enter
- Поддержка Backspace, спецсимволов
- **Раскладка EN** (базовая)
- Переключение EN/RU по Ctrl+Space или аналогу

---

## Следующие шаги 📋

### v0.6 — Русский язык (Cyrillic support)
- Программный маппинг HID keycodes → кириллица
- Флаг текущей раскладки EN/RU
- Переключение по Ctrl+Space (как в Linux) или Caps Lock
- OLED: отображение текущей раскладки [EN]/[RU]
- Корректная обработка Shift+кириллица (заглавные)
- Трансляция в UTF-8 для payload пакета

### v0.7 — Адресная книга / Никнеймы
- Назначить имя узлу: /name MyNode
- Имя передаётся в ANNOUNCE пакете
- /peers показывает имена вместо AA:BB
- /msg MyNode text — отправить сообщение по имени

### v0.8 — История сообщений
- Кольцевой буфер последних N сообщений в RAM
- Прокрутка на OLED (кнопки или клавиатура)
- /history — вывод в Serial

### v0.9 — AES-256 шифрование
- Шифрование payload на уровне L1
- Pre-shared key конфигурируется при прошивке
- Нешифрованные служебные пакеты (ANNOUNCE, HEARTBEAT)
- Индикатор на OLED: 🔒 если зашифровано

### v1.0 — Stable Release
- Полная документация протокола FNPacket
- BOM и схемы подключения
- Инструкция по прошивке
- Тест покрытия: минимум 3 узла A→B→C relay

---

## Будущее 🚀

### LoRa Transport Plugin для freenode_mesh
- Унификация: freenode_lora как плагин к MeshRouter
- Один скетч для всех типов устройств

### Bridge узел (мост в интернет)
- Стегано-туннели через HTTPS/WebRTC
- Fallback: mesh работает без интернета

### freenode_mesh Phase 2
- BLE transport plugin
- Автовыбор транспорта по metrics()

### OLED UI
- Полноценный TUI: список соседей, история, статус
- Навигация с клавиатуры

### FreeToken (Proof of Relay)
- Начисление токенов за ретрансляцию пакетов
- Мотивация держать узел онлайн

---

*FreeNode by FreeNod Co Unlimited*
*"Если физически можно передать бит — FreeNode сумеет это использовать."*
