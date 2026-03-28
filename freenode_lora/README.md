# FreeNode v0.5 — Triple Transport: ESP-NOW + LoRa + BLE

## Что нового в v0.5

**BLE транспорт** — третий канал через NimBLE advertising.
Теперь три одновременных транспорта:

| Транспорт | Скорость | Дальность (город) | Роль |
|-----------|----------|--------------------|------|
| ESP-NOW (Wi-Fi 2.4GHz) | ~1 Mbps | 50–150 м | Основной, быстрый |
| LoRa SX1262 (869MHz) | ~3.5 kbps | 250–1000+ м | Резервный, дальний |
| BLE (2.4GHz ADV) | ~2 kbps | 10–100 м | Дополнительный broadcast |

**Логика отправки:**
- Primary: ESP-NOW (если peer виден) или LoRa (fallback)
- BLE: **всегда дублирует** отправку параллельно (обрезая до ~9 байт текста)
- PING/PONG идут через все три транспорта одновременно

**На OLED:** `[W]`=ESP-NOW, `[L]`=LoRa, `[B]`=BLE

## Платформа
- **Плата:** LILYGO T3S3 V1.2 (ESP32-S3 + SX1262 + SSD1306 OLED)
- **LoRa:** SX1262, 869 MHz, SF9, +22 dBm
- **ESP-NOW:** 2.4 GHz, канал 1
- **BLE:** NimBLE, advertising-based, 250ms interval

## Библиотеки (Library Manager)

| Библиотека | Зачем |
|-----------|-------|
| RadioLib by Jan Gromes | LoRa SX1262 |
| Adafruit SSD1306 | OLED дисплей |
| Adafruit GFX Library | Графика для OLED |
| **NimBLE-Arduino** by h2zero | **BLE transport (НОВОЕ)** |

## Arduino IDE

- Board: **ESP32S3 Dev Module**
- USB CDC On Boot: **Enabled**
- Upload Speed: 115200
- Flash Size: 4MB

## Команды

| Команда | Действие |
|---------|----------|
| `текст` | Отправить (primary + BLE) |
| `/ping` | PING через все три транспорта |
| `/status` | Статистика (ESP-NOW/LoRa/BLE отдельно) |
| `/lora` | Принудительно LoRa |
| `/auto` | Автовыбор транспорта |

## OLED

```
FreeNode v0.5 [A3:F1]     ← мой ID
TX:ESP-NOW E:5 L:2 B:3    ← транспорт + счётчики
─────────────────────
[W]B7:22> Привет           ← ESP-NOW
[L]B7:22> Далеко           ← LoRa
[B]B7:22> PONG             ← BLE
>PING (all)                ← мой PING
```

## Тест тройного транспорта

### Этап 1: рядом (0–50 м)
- Прошей обе T3S3
- Отправь текст — придёт по ESP-NOW и по BLE
- В `/status` видно счётчики обоих

### Этап 2: средняя дальность (~100–200 м)
- ESP-NOW отваливается → auto fallback на LoRa
- BLE тоже пропадёт на этой дистанции
- На OLED: `[L]` вместо `[W]`/`[B]`

### Этап 3: BLE-only тест
- На близкой дистанции (<30м)
- `/lora` на обоих — отключить LoRa как primary
- Если ESP-NOW peer timeout истёк → LoRa primary, но пакеты по BLE всё равно идут
- В `/status` видно `BLE: TX:N RX:M`

## Параметры LoRa

| Параметр | Значение |
|----------|----------|
| Частота | 869 MHz (RU868) |
| SF | 9 |
| BW | 125 kHz |
| TX Power | 22 dBm |
| Sync Word | 0x12 (FreeNode) |

## Параметры BLE

| Параметр | Значение |
|----------|----------|
| Company ID | 0xFFFF (testing) |
| ADV Interval | 250ms |
| Scan Interval | 50ms |
| Scan Window | 25ms |
| TX Power | +9 dBm |
| Max payload | ~9 bytes (legacy ADV) |

---
FreeNode v0.5 by FreeNod Co Unlimited — Март 2026
