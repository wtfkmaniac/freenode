# FreeNode ESP32 Mesh — Quick Start

## Платформа
- **Плата:** NodeMCU 32S (38 pin) или любая ESP32 DevKit
- **Среда:** Arduino IDE 2.x
- **Arduino Core:** ESP32 v2.x или v3.x

## Установка Arduino IDE

1. **Добавить ESP32 в Board Manager:**
   - File → Preferences → Additional Board Manager URLs:
   - `https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json`
   - Tools → Board Manager → ищи "esp32" → Install

2. **Настройки платы:**
   - Tools → Board → "ESP32 Dev Module"
   - Tools → Upload Speed → 921600
   - Tools → Flash Size → "4MB (32Mb)"
   - Tools → Port → твой COM-порт

## Файлы проекта

Все файлы должны лежать в одной папке `freenode_mesh/`:

```
freenode_mesh/
├── freenode_mesh.ino      # Главный скетч
├── transport.h            # Абстрактный интерфейс транспорта
├── espnow_transport.h     # ESP-NOW плагин
└── mesh_router.h          # Mesh-маршрутизатор (flooding)
```

**ВАЖНО:** папка должна называться `freenode_mesh` 
(по имени .ino файла).

## Прошивка

1. Подключить ESP32 по USB
2. Открыть `freenode_mesh.ino` в Arduino IDE
3. Выбрать плату и порт
4. Нажать Upload
5. Открыть Serial Monitor (115200 baud, "Both NL & CR")

Повторить для каждой платы (2-6 штук).

## Тестирование

### Тест 1: Два узла рядом
- Прошить 2 ESP32 одинаковым скетчем
- Открыть Serial Monitor на обоих
- Написать "hello" на узле A → появляется на узле B
- Ожидаемый результат: `[MAC-A] TTL:5 hello`

### Тест 2: Ретрансляция через промежуточный узел
- 3 ESP32: A --- B --- C, расположены в линию
- A не достаёт до C напрямую (разнести по комнатам)
- Отправить сообщение с A
- B получает и ретранслирует
- C получает от A через B, TTL уменьшен

### Тест 3: PING/RTT
- На любом узле набрать `/ping`
- Другие узлы автоматически ответят PONG
- Замерить RTT (round-trip time)

### Тест 4: Статистика
- `/stats` — показать счётчики пакетов
- `/info` — полная информация об узле

## Команды Serial Monitor
```
/ping     — отправить PING всем, замерить RTT
/stats    — статистика пакетов (отправлено/получено/переслано/дропнуто)
/info     — информация об узле (MAC, heap, uptime, транспорты)
/help     — список команд
<текст>   — отправить текстовое сообщение в mesh
```

## Отличия от ESP8266 версии (v0.1)

| Что изменилось | ESP8266 (v0.1) | ESP32 (v0.2) |
|---|---|---|
| esp_now API | ESP8266-specific (role-based) | ESP32 esp_now.h (peer-based) |
| MAC адрес | `WiFi.macAddress()` | `WiFi.macAddress()` (тот же) |
| Random | `random(0, 65535)` | `esp_random()` (аппаратный RNG) |
| Роутер | 1 транспорт | Мульти-транспорт (до 4 плагинов) |
| Статистика | Нет | Счётчики пакетов |
| Команды | Нет | /ping, /stats, /info, /help |
| PING/PONG | Нет | Есть, с измерением RTT |
| LED индикация | Нет | Мигание при приёме/отправке |

## Следующие шаги

- **Phase 2:** BLE transport plugin (ESP32 имеет BLE из коробки)
- **Phase 2:** LoRa transport plugin (нужен SX1276/SX1262 модуль)
- **Phase 3:** Bridge — мост в интернет через Wi-Fi
- **Phase 3:** AES-256 шифрование пакетов

## Troubleshooting

**"Failed to find peer"** — перезагрузи обе платы. ESP-NOW иногда 
теряет broadcast peer после переинициализации.

**Пакеты не доходят** — убедись что обе платы на одном Wi-Fi канале.
По умолчанию channel=0 (автовыбор), должно работать.

**Компиляция ругается на callback** — возможно у тебя Arduino Core v3.x
с новой сигнатурой. См. комментарий в `espnow_transport.h`.

---
FreeNode by FreeNod Co Unlimited — Март 2026
