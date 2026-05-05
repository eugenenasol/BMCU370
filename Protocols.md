# Анализ архитектуры BMCU370

## 1. Общий поток данных

```
UART ──→ KlipperCLI::Run() ──→ ProcessPacket() ──→ Handler*()
                                                          │
                                                    MMU_Logic::*()
                                                          │
                                              ┌───────────┴────────────┐
                                         HAL (BMCU_Hardware)      Flash_saves
                                              │
                                    ┌─────────┼─────────┐
                                 Моторы    Сенсоры    LED
```

---

## 2. Логика управления филаментом (`MMU_Logic`)

### Состояния позиции филамента

```
filament_idle          →  покой (поддержание давления)
filament_loading       →  загрузка (подача вперёд до экструдера)
filament_sending_out   →  подача в активное состояние
filament_using         →  используется (в процессе печати)
filament_pulling_back  →  обратная тяга (retract)
filament_unloading     →  выгрузка (полный retract)
```

### Режимы движения мотора (`filament_motion_enum`)

| Режим | Что делает |
|-------|-----------|
| `pressure_ctrl_idle` | PID по давлению, слабая петля — поддерживает натяжение в покое |
| `send` | Постоянная скорость вперёд (полная) |
| `slow_send` | Постоянная скорость вперёд (замедленная) |
| `pull` | Постоянная скорость назад |
| `pressure_ctrl_in_use` | PID по давлению — активная подача во время печати |
| `velocity_control` | Точное перемещение на заданную дистанцию |
| `stop` | Стоп |

### Машина состояний (упрощённо)

```
         [idle]
            │ need_send_out
            ▼
      [sending_out]  ──→  LED зелёный
            │  filament reaches extruder
            ▼
        [using]  ──→  slow_send на 1.5с
            │
            ▼
   [pressure_ctrl_in_use]  ◄──── давление в диапазоне 1.65–1.70В
            │  need_pull_back
            ▼
      [pulling_back]  ──→  pull до 200мм
            │
            ▼
         [idle]
```

### Управление давлением

Давление измеряется через АЦП (~0–3.3В, центр 1.65В):

- `MC_PULL_stu_raw[i]` — сырое напряжение
- `> PULL_voltage_up` → красный LED, превышение давления
- `< PULL_voltage_down` → синий LED, недостаточное давление

**PID во время печати (`pressure_ctrl_in_use`):**
```
if pressure < 1.55V  →  сбрасываем hysteresis флаг
if pressure < 1.65V  →  PID подаёт (+)
if pressure > 1.70V  →  PID тянет назад (-)
```

### Smart Save (отложенное сохранение в Flash)

```cpp
// Ждём 500мс тишины на UART ИЛИ 5с абсолютный таймаут
bool serial_idle = KlipperCLI_IsSerialIdle(500);
bool timeout_hit = (now - save_timer > 5000);
if (serial_idle || timeout_hit) SaveSettings();
```

---

## 3. Klipper JSON протокол

### Формат сообщений

**Запрос (Klipper → BMCU):**
```json
{"id": 42, "cmd": "COMMAND_NAME", "args": { ... }}
```

**Ответ OK:**
```json
{"id": 42, "ok": true, "code": "...", "msg": "..."}
```

**Ответ Error:**
```json
{"id": 42, "ok": false, "code": "ERR_CODE", "msg": "description"}
```

**Event (без запроса):**
```json
{"event": "STARTUP", "msg": "KlipperCLI Ready"}
```

### Таблица команд

| Команда | Аргументы | Описание |
|---------|-----------|----------|
| `PING` | — | Heartbeat, возвращает версию и uptime |
| `STATUS` | — | Статус всех 4 каналов |
| `GET_SENSORS` | — | Битовая маска наличия филамента |
| `MOVE` | `axis`, `dist_mm`, `speed` | Перемещение оси |
| `STOP` | — | Стоп всех моторов |
| `SELECT_LANE` | `lane` (0-3) | Выбрать активный канал |
| `SET_AUTO_FEED` | `lane`, `enable` | Авто-подача по давлению |
| `GET_FILAMENT_INFO` | `lane` | Данные о филаменте канала |
| `SET_FILAMENT_INFO` | `lane`, `name`, `id_str`, `temp_min`, `temp_max`, `color`, `meters` | Записать параметры |

### Пример STATUS ответа

```json
{
  "id": 1, "cmd": "STATUS", "ok": true,
  "lanes": [
    {
      "id": 0, "present": true, "motion": "Idle",
      "meters": 245.50, "pressure": 1.650,
      "rfid": "ABC12345", "name": "PLA",
      "temp_min": 200, "temp_max": 220,
      "color": [255, 100, 50, 255]
    }
  ]
}
```

---

## 4. Известные проблемы в коде

> [!WARNING]
> **LED не обновляются**: `SetLED` пишет в буфер NeoPixel, но `pixels.show()` нигде не вызывается из логики. Нужно добавить `UpdateLEDs()` в `I_MMU_Hardware`.

> [!WARNING]
> **200мм retract hardcoded**: `Prepare_For_filament_Pull_Back(200.0f)` — дистанция обратной тяги не конфигурируема.

> [!NOTE]
> **`is_two` всегда false**: логика двойного датчика (AMS Lite buffer) не активирована.

> [!NOTE]
> **`MOVE` с `axis="SELECTOR"`**: заглушка, ничего не делает.

---

## 5. Поток Klipper ↔ BMCU

```
G-code: BMCU_CALL CMD=STATUS
        │
        ▼
   bmcu.py (Klipper extra)
   → отправляет: {"id":1,"cmd":"STATUS"}\r\n через Serial
        │
        ▼
   BMCU firmware (KlipperCLI::Run)
   ← отвечает: {"id":1,"ok":true,"lanes":[...]}\r\n
        │
        ▼
   bmcu.py парсит и обновляет объект [bmcu] в Klipper
        │
        ▼
   Moonraker: GET /printer/objects/query?bmcu
        │
        ▼
   bmcu_card.html обновляет UI
```

---

## 6. Приоритетные улучшения

| Приоритет | Улучшение |
|-----------|-----------|
| 🔴 Высокий | Добавить `UpdateLEDs()` в `I_MMU_Hardware`, вызывать из `MMU_Logic::Run()` |
| 🔴 Высокий | Вынести retract distance (200мм) в конфигурируемый параметр |
| 🟡 Средний | Команды `LOAD` / `UNLOAD` в Klipper протоколе |
| 🟡 Средний | Heartbeat timeout — стоп моторов при потере связи с Klipper |
| 🟢 Низкий | `MOTOR_INVERT_CHx` → Flash-сохраняемые параметры |
