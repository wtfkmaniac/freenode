# FreeNode ESP32 Mesh v0.5 — ESP-NOW + BLE

## Что нового в v0.5

**BLE транспорт** — третий канал связи через NimBLE advertising.
Данные передаются в manufacturer-specific data рекламных пакетов (connectionless broadcast).

| Транспорт | Скорость | Дальность | Роль |
|-----------|----------|-----------|------|
| ESP-NOW (2.4GHz) | ~1 Mbps | 50–200 м | Основной, быстрый |
| BLE (2.4GHz ADV) | ~2 kbps | 10–100 м | Дополнительный, экономичный |

**Ограничение:** legacy BLE ADV = 31 байт max. После overhead остаётся ~9 байт текста.
PING/PONG проходят полностью, текстовые сообщения обрезаются.

## Платформа
- **Плата:** ESP32 DevKit (NodeMCU 32S или аналог)
- **Среда:** Arduino IDE 2.x
- **Arduino Core:** ESP32 v3.x

## Зависимости (Library Manager)

| Библиотека | Зачем |
|-----------|-------|
| **NimBLE-Arduino** by h2zero | BLE transport |

ESP-NOW и WiFi встроены в ESP32 Core.

## Установка

1. Board Manager → ESP32 → Install
2. Library Manager → "NimBLE-Arduino" → Install
3. Board: "ESP32 Dev Module"
4. Upload Speed: 921600

## Файлы проекта

```
freenode_mesh/
├── freenode_mesh.ino      # Главный скетч
├── transport.h            # Абстрактный интерфейс транспорта
├── espnow_transport.h     # ESP-NOW плагин
├── ble_transport.h        # BLE плагин (NimBLE ADV-based)
└── mesh_router.h          # Mesh-маршрутизатор (flooding)
```

## Команды Serial Monitor (115200 baud)
```
/ping     — PING через все транспорты (ESP-NOW + BLE)
/stats    — статистика пакетов
/info     — информация об узле
/help     — справка
<текст>   — отправить в mesh
```

## Тестирование BLE

### Тест: BLE-only связь
1. Прошить 2 ESP32
2. На обоих: Serial Monitor 115200
3. На одном отправить "hi" — должно прийти по ESP-NOW И по BLE
4. В логах: `[BLE RX]` и `[ESP-NOW RX]` — пакет пришёл по обоим каналам
5. Дедупликация в mesh_router отфильтрует дубль (одинаковый ID)

### Тест: BLE при отключённом ESP-NOW
Пока нет команды `/ble` для принудительного BLE-only.
Для теста: закомментировать `mesh.addTransport(&espnow)` в setup().

## Как работает BLE транспорт

1. **Приём (scan):** NimBLE сканирует BLE advertising пакеты. Фильтрует по Company ID (0xFFFF) и magic byte (0xFE). Данные кладутся в кольцевой буфер.

2. **Передача (advertise):** `send()` останавливает scan, устанавливает manufacturer data с FNPacket внутри, запускает advertising burst на 1 секунду (~4 ADV пакета при 250ms интервале).

3. **Чередование:** `update()` в loop() следит за таймером ADV и переключает обратно на scan.

4. **Дедупликация:** mesh_router фильтрует дубли по packet ID — если один и тот же пакет пришёл по ESP-NOW и по BLE, второй будет dropped.

## Следующие шаги

- [ ] Extended Advertising (BLE 5) для полного FNPacket на ESP32-S3
- [ ] BLE Mesh по спецификации (NimBLE Mesh) для production
- [ ] BLE HID keyboard input (v0.6)
- [ ] Команда `/ble` для принудительного BLE-only режима

---
FreeNode v0.5 by FreeNod Co Unlimited — Март 2026
