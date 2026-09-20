# can_manager — единый менеджер шины CAN/FDCAN для STM32

Портируемая библиотека для STM32, которая единолично владеет периферией CAN/FDCAN: настраивает
приёмный фильтр, отправку и восстановление после Bus-Off, а другим библиотекам и пользовательскому
коду даёт единый механизм регистрации фильтров приёма и общую очередь отправки на шину.

## Возможности

- Один менеджер = одна физическая шина CAN/FDCAN; несколько шин - несколько независимых экземпляров.
- Произвольное число потребителей может зарегистрировать произвольное число фильтров приёма
  (id + маска, Standard и Extended ID) каждый со своим обработчиком - от одного до сотен фильтров на
  шину.
- Менеджер отклоняет регистрацию фильтра, если он пересекается с уже зарегистрированным (от любого
  потребителя на этой шине) - маршрутизация принятых кадров детерминирована по построению.
- Отправка - через единую программную очередь на шину со строгим порядком: новый пакет никогда не
  обгоняет уже стоящие в очереди пакеты. Два режима постановки в очередь: "отправить именно этот
  пакет" (`CANMGR_Send`) и "отправить актуальное значение" (`CANMGR_SendLatest` - заменяет уже
  стоящий в очереди пакет с тем же id на месте, без изменения его позиции, если такой есть).
- Автоматическое обнаружение и восстановление после Bus-Off для обоих аппаратных бэкендов
  (FDCAN и классический bxCAN), с диагностическими счётчиками.
- Автовыбор бэкенда (FDCAN/bxCAN) на этапе компиляции по тому, какой HAL-модуль включён в CubeMX -
  без правок кода при переносе между STM32F0/F4 (bxCAN) и STM32H7 (FDCAN) и другими чипами с той же
  периферией.
- Неблокирующий API: ни одна функция не содержит циклов ожидания.
- Опциональная интеграция с [stm32_logger](../../stm32_logger/git/README.md) - ключевые события
  (ошибки Init, отклонённая регистрация фильтра, переполнение очереди отправки, переполнение
  Rx FIFO0, Bus-Off) логируются через `LOGGER_Log()`, если включено
  `#define CANMGR_ENABLE_LOGGER` (и `#define LOGGER_ENABLE_CANMGR` в `logger_codes.h` проекта) -
  без этих define'ов `logger.h` не подключается вовсе, зависимость необязательна.

## Требования к настройке в CubeMX

| Периферия | Что настроить |
|---|---|
| CAN/FDCAN | Тактирование, битрейт (Bit Timing), включить приём в Rx FIFO0. Периферию **не запускать** - старт делает библиотека. |
| NVIC (FDCAN) | Прерывание приёма/ошибок/опустошения Tx (обычно `FDCANx_IT0`) - включить. |
| NVIC (bxCAN) | `CANx_RX0_IRQn`, `CANx_TX_IRQn` и **обязательно** `CANx_SCE_IRQn` (без него события Bus-Off/переполнения Rx FIFO0 не дойдут до обработчика ошибок). |
| Фильтры в CubeMX | Не настраивать - библиотека сама конфигурирует один широкий приёмный фильтр, вся содержательная фильтрация происходит программно. |

## Быстрый старт

```c
/* Инициализация одной шины */
CANMGR_Config_t cfg = { .hcan = &hfdcan1 };
CANMGR_Handle_t *bus = CANMGR_Init(&cfg);

/* Потребитель регистрирует свои фильтры (пример: конвенция ID этой базы
   знаний - (cmd_id << 8) | device_id, Extended) */
CANMGR_RegisterFilter(bus, (9U << 8), 0xFF00U, 1U, my_status_callback, my_context);
CANMGR_RegisterFilter(bus, (14U << 8), 0xFF00U, 1U, my_status_callback, my_context);

/* Обработчик приёма */
void my_status_callback(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                         const uint8_t *data, uint8_t len, void *user_ctx)
{
    /* ... */
}

/* Отправка - каждый вызов обязан уйти на шину отдельным пакетом */
uint8_t payload[4] = { 0x00, 0x00, 0x01, 0x2C };
CANMGR_Send(bus, (1U << 8) | 5U, 1U, payload, sizeof(payload));

/* Отправка актуального значения - если пакет с этим id ещё стоит в
   очереди (не успел уйти), его данные заменяются на месте; устаревшие
   промежуточные значения так не накапливаются в очереди */
CANMGR_SendLatest(bus, (2U << 8) | 5U, 1U, payload, sizeof(payload));
```

## Подключение обработчиков к HAL-callback-ам

FDCAN (STM32H7 и т.п., `HAL_FDCAN_MODULE_ENABLED`):

```c
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
    CANMGR_RxFifo_Handler(hfdcan, RxFifo0ITs);
}
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan) {
    CANMGR_TxComplete_Handler(hfdcan);
}
void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    CANMGR_ErrorStatus_Handler(hfdcan, ErrorStatusITs);
}
```

NVIC: включить прерывание `FDCANx_IT0` (приём, опустошение Tx FIFO и Bus-Off по умолчанию
роутятся на эту же линию у большинства чипов - сверьтесь с распределением строк вашего
конкретного МК).

bxCAN (STM32F0/F4 и т.п., `HAL_CAN_MODULE_ENABLED`):

```c
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CANMGR_RxFifo_Handler(hcan);
}
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef *hcan) { CANMGR_TxComplete_Handler(hcan); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef *hcan) { CANMGR_TxComplete_Handler(hcan); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef *hcan) { CANMGR_TxComplete_Handler(hcan); }
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
    CANMGR_ErrorStatus_Handler(hcan);
}
```

NVIC: обязательно включить `CANx_RX0_IRQn`, `CANx_TX_IRQn` и `CANx_SCE_IRQn` (Status Change
Error) - без последнего события Bus-Off/переполнения Rx FIFO0 физически не дойдут до
`HAL_CAN_ErrorCallback`, даже если нотификация активирована программно.

Таблицы обработчиков - см. [API_REFERENCE.md](API_REFERENCE.md).

## Модель фильтров

Фильтр = пара `(id, mask)`: кадр с идентификатором `X` подходит под фильтр, если
`(X & mask) == (id & mask)` - как в обычном CAN acceptance-фильтре. Standard (11 бит) и Extended
(29 бит) кадры - разные адресные пространства: одинаковый числовой `id` с разным типом кадра не
пересекается. При регистрации нового фильтра менеджер проверяет отсутствие пересечения со всеми уже
зарегистрированными на этой шине фильтрами (от любых потребителей) и отклоняет регистрацию при
пересечении - см. `CANMGR_RegStatus_t` в [API_REFERENCE.md](API_REFERENCE.md).

## Честные ограничения

- Поддерживаются только Classic CAN/FDCAN кадры (0..8 байт данных) - FDCAN FD-кадры с BRS (до 64
  байт) не поддерживаются.
- Используется только RxFIFO0 - RxFIFO1 не задействуется.
- Очередь отправки - простой FIFO без приоритетов между потребителями (см. `can_manager.h` за
  обоснованием этого выбора).
- Число различных значений маски, одновременно используемых на одной шине, ограничено
  (`CANMGR_MAX_MASK_GROUPS`, по умолчанию 8) - подавляющее большинство фильтров с РАЗНЫМ `id`, но
  ОДИНАКОВОЙ маской не расходует лимит.
- Библиотека протестирована функционально на хостовой заглушке HAL и собрана без предупреждений
  против настоящих заголовков HAL STM32H7 (FDCAN) и STM32F4 (bxCAN); реальное аппаратное тестирование
  на живой шине не проводилось.

## Лицензия

Библиотека распространяется на условиях **PolyForm Noncommercial License 1.0.0** — свободное
использование, копирование, изменение и распространение для любых НЕКОММЕРЧЕСКИХ целей, при
условии сохранения уведомления об авторских правах. Коммерческое использование требует отдельного
разрешения правообладателя. Полный текст — файл [LICENSE](LICENSE).
