# Анализ соответствия команд Klipper и прошивки BMCU

Был проведен полный перекрестный анализ исходного кода прошивки (`KlipperCLI.cpp`) и модуля интеграции Klipper (`bmcu.py`, `bmcu_macros.cfg`). 

## 1. Общий вывод
**Базовый синтаксис и логика передачи данных полностью совпадают.** Интеграция работает корректно: Klipper успешно формирует JSON-пакеты, а прошивка успешно их парсит. Ключевые ограничения (например, длина имени до 20 символов и RFID до 8 символов) синхронизированы на обеих сторонах.

Однако, **не все команды прошивки имеют удобные G-code обертки в Klipper**. Часть расширенных команд доступна только через универсальный вызов `BMCU_CALL`.

## 2. Идеальные совпадения (Полная синхронизация)

Следующие команды имеют 100% соответствие полей и логики:
* **`PING`**: `BMCU_PING` ➔ `{"cmd": "PING"}`
* **`STATUS`**: `BMCU_FETCH_STATUS` ➔ `BMCU_CALL CMD=STATUS` ➔ `{"cmd": "STATUS"}`
* **`GET_SENSORS`**: `BMCU_GET_SENSORS` ➔ `{"cmd": "GET_SENSORS"}`
* **`STOP`**: `BMCU_STOP` ➔ `{"cmd": "STOP"}`
* **`SELECT_LANE`**: `BMCU_SELECT_LANE LANE=X` ➔ `{"cmd": "SELECT_LANE", "args": {"lane": X}}`
* **`MOVE` (и его алиасы FEED, SELECTOR, SPOOL)**: `BMCU_MOVE AXIS=X DIST=Y SPEED=Z` ➔ `{"cmd": "MOVE", "args": {"axis": "X", "dist_mm": Y, "speed": Z}}`
* **`GET_FILAMENT_INFO`**: `BMCU_GET_FILAMENT_INFO LANE=X` ➔ `{"cmd": "GET_FILAMENT_INFO", "args": {"lane": X}}`
* **`SET_FILAMENT_INFO`**: В Python модуле Klipper очень аккуратно реализован сбор параметров (`NAME`, `TEMP_MIN`, `TEMP_MAX`, `METERS`, `COLOR`). Отдельно стоит отметить правильный маппинг аргумента `RFID` (или `ID`) в Klipper в поле `id_str`, которое ожидает прошивка.

## 3. Частичные несовпадения (Урезанный функционал в Klipper)

### `SET_AUTO_FEED`
* **Прошивка ожидает:** `lane`, `enable` (или `ENABLE`), `overflow` (или `OVERFLOW`).
* **Klipper отправляет:** Только `lane` и `enable`.
* **Анализ:** Klipper-команда `BMCU_SET_AUTO_FEED` не позволяет управлять флагом `overflow`. Прошивка в этом случае сохраняет предыдущее состояние флага (благодаря логике `args.containsKey("overflow")`), что безопасно, но ограничивает управление этим параметром из консоли Klipper (можно обойти через `BMCU_CALL`).

## 4. Отсутствующие G-code команды в Klipper

Следующие важные команды прошивки **не имеют явных G-code оберток** в модуле `bmcu.py`. Их можно вызвать только через `BMCU_CALL CMD=... ARGS="..."`, что неудобно для повседневного использования:

1. **`FEED_TO_EXTRUDER`**
   * В прошивке это мощная команда с аргументами `lane`, `speed`, `max_mm`, `pressure_thr`, `stall_ms`.
   * В Klipper **нет** команды `BMCU_FEED_TO_EXTRUDER`.
2. **`SET_PRESSURE_ADVANCED` (или `SET_PA`)**
   * В прошивке используется для тонкой настройки тензодатчиков (`gain`, `offset`, `boost_thr`, `deadzone` и т.д.).
   * В Klipper **нет** команды `BMCU_SET_PA`.
3. **`SET_MOVE_PID`**
   * В прошивке задает коэффициенты ПИД `P`, `I`, `D`, `ZERO`.
   * В Klipper **нет** команды `BMCU_SET_MOVE_PID`.
4. **`CALIBRATE`**
   * В прошивке калибрует нули тензодатчиков (с аргументом `lane`).
   * В Klipper **нет** команды `BMCU_CALIBRATE`.
5. **`TEST_MOTOR`**
   * В прошивке используется для отладки моторов (`lane`, `pwm`, `duration`).
   * В Klipper **нет** команды `BMCU_TEST_MOTOR`.

## 5. Итог и рекомендации

Связь по передаче команд настроена **корректно и безопасно**. Синтаксис существующих команд полностью совпадает. 

**Рекомендация для дальнейшего портирования (Happy Hare):**
Для полноценного управления всеми функциями BMCU из Klipper, рекомендуется дописать в `bmcu.py` недостающие функции:
* `cmd_BMCU_FEED_TO_EXTRUDER`
* `cmd_BMCU_CALIBRATE`
* `cmd_BMCU_SET_PA`
* `cmd_BMCU_SET_MOVE_PID`

Это избавит от необходимости писать сложные JSON-строки в параметре `ARGS` команды `BMCU_CALL`.
