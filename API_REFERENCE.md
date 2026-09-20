# API Reference — can_manager

При расхождениях с `can_manager.h` ориентироваться на `.h` - он первичен, этот файл его сжатый
пересказ.

## Оглавление

- [Типы](#типы)
- [Инициализация](#инициализация)
- [Регистрация фильтров](#регистрация-фильтров)
- [Отправка](#отправка)
- [Диагностика](#диагностика)
- [Обработчики прерываний](#обработчики-прерываний)
- [Интеграция с stm32_logger](#интеграция-с-stm32_logger)

## Типы

| Тип | Описание |
|---|---|
| `CANMGR_CAN_HandleTypeDef` | Универсальный тип хэндла периферии - `FDCAN_HandleTypeDef` или `CAN_HandleTypeDef` в зависимости от бэкенда, выбранного при компиляции. |
| `CANMGR_Config_t` | Конфигурация одной шины - см. таблицу ниже. |
| `CANMGR_Handle_t` | Хэндл шины, возвращаемый `CANMGR_Init()`. |
| `CANMGR_RxCallback_t` | Тип callback-а приёма - см. сигнатуру ниже. |
| `CANMGR_RegStatus_t` | Код результата регистрации фильтра - см. таблицу ниже. |

### `CANMGR_Config_t`

| Поле | Тип | Назначение |
|---|---|---|
| `hcan` | `CANMGR_CAN_HandleTypeDef *` | Хэндл периферии, уже проинициализированный CubeMX-кодом (`HAL_FDCAN_Init`/`HAL_CAN_Init`), но не запущенный. |

### `CANMGR_Handle_t` — публичные поля

| Поле | Тип | Назначение |
|---|---|---|
| `config` | `CANMGR_Config_t` | Копия конфигурации, переданной в `CANMGR_Init()`. |
| `bus_off_count` | `uint32_t` | См. `CANMGR_GetBusOffCount()`. |
| `rx_overflow_count` | `uint32_t` | См. `CANMGR_GetRxOverflowCount()`. |
| `tx_queue_depth` | `uint16_t` | Текущая длина очереди отправки. |
| `index` | `uint8_t` | Позиция в пуле шин (справочно). |

Остальные поля структуры - внутренние, использовать только через функции API ниже.

### `CANMGR_RxCallback_t`

```c
typedef void (*CANMGR_RxCallback_t)(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                     const uint8_t *data, uint8_t len, void *user_ctx);
```

| Параметр | Назначение |
|---|---|
| `bus` | Хэндл шины, на которой принят кадр. |
| `id` | Идентификатор принятого кадра (11 или 29 бит, см. `is_extended`). |
| `is_extended` | `0` - Standard, `1` - Extended. |
| `data` | Данные кадра (0..8 байт, см. `len`); указатель временный, действителен только на время вызова. |
| `len` | Реальная длина данных. |
| `user_ctx` | Контекст, переданный при регистрации фильтра. |

### `CANMGR_RegStatus_t`

| Значение | Значение |
|---|---|
| `CANMGR_REG_OK` | Фильтр зарегистрирован. |
| `CANMGR_REG_ERR_INVALID_ARG` | `bus == NULL` либо `callback == NULL`. |
| `CANMGR_REG_ERR_OVERLAP` | Пересекается с уже зарегистрированным фильтром на этой шине. |
| `CANMGR_REG_ERR_FILTERS_FULL` | Исчерпан `CANMGR_MAX_FILTERS_PER_BUS`. |
| `CANMGR_REG_ERR_TOO_MANY_MASKS` | Исчерпан `CANMGR_MAX_MASK_GROUPS` (см. `can_manager.h`). |
| `CANMGR_REG_ERR_GROUP_FULL` | Исчерпан `CANMGR_MAX_FILTERS_PER_GROUP` для этого значения маски. |

## Инициализация

### `CANMGR_Init`

```c
CANMGR_Handle_t *CANMGR_Init(const CANMGR_Config_t *config);
```

Настраивает приёмный фильтр, активирует нотификации и запускает периферию. Идемпотентна: повторный
вызов с тем же `config->hcan` возвращает указатель на тот же хэндл.

| Параметр | Назначение |
|---|---|
| `config` | Заполненная конфигурация. |

**Возврат:** указатель на хэндл шины, либо `NULL` при ошибке (`config == NULL`, `config->hcan ==
NULL`, исчерпан `CANMGR_MAX_BUSES`, либо ошибка HAL при настройке).

## Регистрация фильтров

### `CANMGR_RegisterFilter`

```c
CANMGR_RegStatus_t CANMGR_RegisterFilter(CANMGR_Handle_t *bus, uint32_t id, uint32_t mask,
                                          uint8_t is_extended, CANMGR_RxCallback_t callback,
                                          void *user_ctx);
```

Регистрирует пару (фильтр, callback). Отклоняет регистрацию при пересечении с любым уже
зарегистрированным на этой шине фильтром.

| Параметр | Назначение |
|---|---|
| `bus` | Хэндл шины. |
| `id` | Значение id фильтра. |
| `mask` | Маска фильтра (`0` - подходит любой id того же типа кадра). |
| `is_extended` | `0` - Standard, `1` - Extended. |
| `callback` | Обработчик принятых кадров. |
| `user_ctx` | Произвольный контекст, передаётся в `callback` как есть. |

**Возврат:** `CANMGR_REG_OK` при успехе, иначе код ошибки (см. таблицу `CANMGR_RegStatus_t` выше).

## Отправка

### `CANMGR_Send`

```c
HAL_StatusTypeDef CANMGR_Send(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                               const uint8_t *data, uint8_t len);
```

Кладёт кадр в программную очередь менеджера (единую на шину, строгий порядок - см. README.md).
Неблокирующая.

| Параметр | Назначение |
|---|---|
| `bus` | Хэндл шины. |
| `id` | Идентификатор кадра. |
| `is_extended` | `0` - Standard, `1` - Extended. |
| `data` | Данные кадра, 0..8 байт. |
| `len` | Длина данных (обрезается до 8, если больше). |

**Возврат:** `HAL_OK` - кадр принят на отправку; `HAL_ERROR` - `bus == NULL`, `data == NULL` при
`len > 0`, либо программная очередь переполнена.

### `CANMGR_SendLatest`

```c
HAL_StatusTypeDef CANMGR_SendLatest(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                     const uint8_t *data, uint8_t len);
```

Отправляет актуальное значение с указанным id: если в программной очереди уже стоит пакет с тем же
(`id`, `is_extended`), заменяет его данные на новые БЕЗ изменения позиции в очереди - устаревшее
значение просто не будет отправлено. Если такого пакета в очереди нет - работает как `CANMGR_Send()`.
См. обоснование выбора между этой функцией и `CANMGR_Send()` в `can_manager.h`.

| Параметр | Назначение |
|---|---|
| `bus` | Хэндл шины. |
| `id` | Идентификатор кадра. |
| `is_extended` | `0` - Standard, `1` - Extended. |
| `data` | Данные кадра, 0..8 байт. |
| `len` | Длина данных (обрезается до 8, если больше). |

**Возврат:** `HAL_OK` - значение принято (заменено в очереди на месте, ушло напрямую в аппаратный
буфер, либо встало в очередь новым пакетом); `HAL_ERROR` - `bus == NULL`, `data == NULL` при
`len > 0`, либо (для случая нового пакета) программная очередь переполнена.

## Диагностика

| Функция | Возврат |
|---|---|
| `uint32_t CANMGR_GetBusOffCount(const CANMGR_Handle_t *bus)` | Число событий Bus-Off с момента `CANMGR_Init()`. |
| `uint32_t CANMGR_GetRxOverflowCount(const CANMGR_Handle_t *bus)` | Число переполнений Rx FIFO0 с момента `CANMGR_Init()`. |
| `uint16_t CANMGR_GetTxQueueDepth(const CANMGR_Handle_t *bus)` | Текущая длина программной очереди отправки. |

Все три возвращают `0`, если `bus == NULL`.

## Обработчики прерываний

Вызываются ИЗ пользовательского HAL-callback-а - см. "Подключение обработчиков к HAL-callback-ам"
в [README.md](README.md) для полного примера подключения на обоих бэкендах.

| Функция | Вызывать из |
|---|---|
| `CANMGR_RxFifo_Handler(hcan, RxFifo0ITs)` *(только FDCAN)* / `CANMGR_RxFifo_Handler(hcan)` *(только bxCAN)* | `HAL_FDCAN_RxFifo0Callback` / `HAL_CAN_RxFifo0MsgPendingCallback` |
| `CANMGR_TxComplete_Handler(hcan)` | `HAL_FDCAN_TxFifoEmptyCallback` / все три `HAL_CAN_TxMailboxXCompleteCallback` |
| `CANMGR_ErrorStatus_Handler(hcan, ErrorStatusITs)` *(только FDCAN)* / `CANMGR_ErrorStatus_Handler(hcan)` *(только bxCAN)* | `HAL_FDCAN_ErrorStatusCallback` / `HAL_CAN_ErrorCallback` |

Каждый обработчик сам проверяет, что `hcan` относится к одной из инициализированных здесь шин, и
тихо выходит, если нет - несколько модулей могут безопасно делить один и тот же HAL-callback.

## Интеграция с stm32_logger

Опциональная, необязательная зависимость - см. "ОПЦИОНАЛЬНАЯ ИНТЕГРАЦИЯ С stm32_logger" в шапке
`can_manager.h`. Включается парой независимых define'ов:

| Define | Где | Назначение |
|---|---|---|
| `CANMGR_ENABLE_LOGGER` | до `#include "can_manager.h"` | Включает вызовы `LOGGER_Log()` внутри `can_manager.c`. Без него `logger.h` не подключается вовсе. |
| `LOGGER_ENABLE_CANMGR` | до `#include "logger_codes.h"` | Включает блок кодов `LOG_CODE_CANMGR_*` в таблице `LOGGER_LogTable` проекта. |

Коды, которые использует `can_manager.c` (адресное пространство и приоритеты закреплены в
`logger_codes.h` проекта, согласованы с автором stm32_logger):

| Код | Приоритет | Когда | `value` |
|---|---|---|---|
| `LOG_CODE_CANMGR_INIT_OK` | LOW | `CANMGR_Init()` успешно завершена | `0` |
| `LOG_CODE_CANMGR_INIT_FAIL` | HIGH | `CANMGR_Init()` - исчерпан `CANMGR_MAX_BUSES` либо ошибка HAL | `0` |
| `LOG_CODE_CANMGR_REG_REJECTED` | MEDIUM | `CANMGR_RegisterFilter()` вернула не `CANMGR_REG_OK` (кроме `CANMGR_REG_ERR_INVALID_ARG`) | код `CANMGR_RegStatus_t` |
| `LOG_CODE_CANMGR_TX_QUEUE_FULL` | HIGH | `CANMGR_Send()`/`CANMGR_SendLatest()` отклонили пакет - очередь переполнена | `id` кадра |
| `LOG_CODE_CANMGR_RX_OVERFLOW` | MEDIUM | Переполнение аппаратного Rx FIFO0 | `rx_overflow_count` после инкремента |
| `LOG_CODE_CANMGR_BUS_OFF` | HIGH | Обнаружен и автовосстановлен Bus-Off | `bus_off_count` после инкремента |

Во всех случаях `source_id` в `LOGGER_Log()` - это `bus->index` (при исчерпании `CANMGR_MAX_BUSES`,
когда конкретной шины ещё нет, - `0xFFFF`).
