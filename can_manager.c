/**
 ******************************************************************************
 * @file    can_manager.c
 * @brief   Реализация единого менеджера шины CAN/FDCAN для STM32 - см.
 *          архитектурные решения и обоснования в шапке can_manager.h.
 * @author  Mechanic
 * @date    17.09.2026
 * @version 0.1
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include <string.h>
#include "can_manager.h"

/* ========================================================================
 *  Статический пул шин (без malloc, как во всех библиотеках этой базы)
 * ======================================================================== */

static CANMGR_Handle_t s_bus_pool[CANMGR_MAX_BUSES];

/* ========================================================================
 *  Внутренние утилиты общего назначения
 * ======================================================================== */

static CANMGR_Handle_t *canmgr_find_bus(const CANMGR_CAN_HandleTypeDef *hcan)
{
    for (uint32_t i = 0U; i < CANMGR_MAX_BUSES; i++)
    {
        if (s_bus_pool[i].used && (s_bus_pool[i].config.hcan == hcan))
        {
            return &s_bus_pool[i];
        }
    }
    return NULL;
}

static CANMGR_Handle_t *canmgr_find_free_bus(void)
{
    for (uint32_t i = 0U; i < CANMGR_MAX_BUSES; i++)
    {
        if (!s_bus_pool[i].used)
        {
            return &s_bus_pool[i];
        }
    }
    return NULL;
}

/** Бинарный поиск ключа в отсортированном по возрастанию массиве.
 *  @retval индекс совпадения, либо -1, если ключа нет в массиве. */
static int32_t canmgr_bsearch(const uint32_t *sorted, uint16_t count, uint32_t key)
{
    int32_t lo = 0;
    int32_t hi = (int32_t)count - 1;

    while (lo <= hi)
    {
        int32_t mid = lo + ((hi - lo) / 2);
        if (sorted[mid] == key)
        {
            return mid;
        }
        if (sorted[mid] < key)
        {
            lo = mid + 1;
        }
        else
        {
            hi = mid - 1;
        }
    }
    return -1;
}

/** Позиция, куда нужно вставить key, чтобы массив остался отсортированным
 *  (аналог std::lower_bound) - используется при вставке нового фильтра в
 *  группу масок. */
static uint16_t canmgr_lower_bound(const uint32_t *sorted, uint16_t count, uint32_t key)
{
    uint16_t lo = 0U;
    uint16_t hi = count;

    while (lo < hi)
    {
        uint16_t mid = (uint16_t)(lo + ((hi - lo) / 2U));
        if (sorted[mid] < key)
        {
            lo = (uint16_t)(mid + 1U);
        }
        else
        {
            hi = mid;
        }
    }
    return lo;
}

/* ========================================================================
 *  Аппаратный уровень (port_*) - здесь и только здесь код различается
 *  между бэкендами FDCAN и bxCAN. Подход и восстановление после Bus-Off
 *  переиспользуют проверенную на практике механику can_vesc_stm32/
 *  motor_vesc.c (референс на чтение, см. AGENTS.md проекта) - см.
 *  обоснование в шапке can_manager.h.
 * ======================================================================== */

#if defined(CANMGR_BACKEND_FDCAN)

/** Индекс фильтра в списке FDCAN. У каждого инстанса FDCAN (FDCAN1, FDCAN2,
 *  ...) свои, независимые банки фильтров - конфликтов между разными шинами
 *  здесь не бывает, можно использовать одно и то же значение для всех. */
#define CANMGR_FDCAN_FILTER_INDEX   0U

static uint32_t canmgr_len_to_fdcan_dlc(uint8_t len)
{
    /* Classic CAN/FDCAN-кадр (без BRS/FD, см. "Чего в этой версии нет" в
     * can_manager.h): для длины 0..8 код DLC в поле HAL совпадает с самим
     * числом байт, сдвинутым в позицию поля (биты 19:16) - см. макросы
     * FDCAN_DLC_BYTES_0..FDCAN_DLC_BYTES_8 в HAL. */
    return ((uint32_t)len) << 16;
}

static uint8_t canmgr_fdcan_dlc_to_len(uint32_t dlc)
{
    uint32_t code = (dlc >> 16) & 0x0FU;
    /* Коды 9..15 - это FD-длины (12..64 байт), сюда попасть не должны, т.к.
     * мы принимаем только Classic-кадры - на всякий случай защищаемся от
     * выхода за границы буфера данных. */
    return (uint8_t)((code <= CANMGR_MAX_DATA_LEN) ? code : CANMGR_MAX_DATA_LEN);
}

static HAL_StatusTypeDef port_configure_filter(CANMGR_CAN_HandleTypeDef *hcan)
{
    /* Один широкий фильтр на каждый тип ID (Standard/Extended) - маска 0
     * пропускает все кадры в RxFIFO0. Реальная фильтрация по потребителям
     * происходит программно (см. быструю диспетчеризацию в can_manager.h)
     * - библиотека не тратит ограниченное число аппаратных банков на
     * фильтры отдельных потребителей. */
    FDCAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));
    filter.IdType       = FDCAN_STANDARD_ID;
    filter.FilterIndex  = CANMGR_FDCAN_FILTER_INDEX;
    filter.FilterType   = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1    = 0U;
    filter.FilterID2    = 0U; /* маска 0 - совпадает любой Standard ID */
    if (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    filter.IdType = FDCAN_EXTENDED_ID;
    if (HAL_FDCAN_ConfigFilter(hcan, &filter) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* Кадры, не подошедшие ни под один явный фильтр (в этой конфигурации -
     * не должно случаться, т.к. оба фильтра выше пропускают всё), на
     * всякий случай отбрасываем, а не роутим в FIFO0/1 бесконтрольно. */
    return HAL_FDCAN_ConfigGlobalFilter(hcan, FDCAN_REJECT, FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);
}

static HAL_StatusTypeDef port_activate_notifications(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_ActivateNotification(hcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
        FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF, 0U);
}

static HAL_StatusTypeDef port_start(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_Start(hcan);
}

static uint32_t port_get_tx_free_level(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(hcan);
}

static HAL_StatusTypeDef port_send(CANMGR_CAN_HandleTypeDef *hcan, uint32_t id,
                                    uint8_t is_extended, const uint8_t *data, uint8_t len)
{
    FDCAN_TxHeaderTypeDef header;

    memset(&header, 0, sizeof(header));
    header.Identifier          = id;
    header.IdType              = is_extended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    header.TxFrameType         = FDCAN_DATA_FRAME;
    header.DataLength          = canmgr_len_to_fdcan_dlc(len);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch       = FDCAN_BRS_OFF;
    header.FDFormat            = FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    header.MessageMarker       = 0U;

    return HAL_FDCAN_AddMessageToTxFifoQ(hcan, &header, data);
}

/** FDCAN аппаратно НИКОГДА не выходит из Bus-Off самостоятельно - это
 *  официально задокументированное поведение периферии, программный сброс
 *  бита CCCR.INIT обязателен всегда (см. can_vesc_stm32/motor_vesc.c,
 *  port_bus_off_recover - тот же приём). */
static void port_bus_off_recover(CANMGR_CAN_HandleTypeDef *hcan)
{
    hcan->Instance->CCCR &= ~FDCAN_CCCR_INIT;
}

static uint8_t port_receive(CANMGR_CAN_HandleTypeDef *hcan, uint32_t *id,
                             uint8_t *is_extended, uint8_t *data, uint8_t *len)
{
    FDCAN_RxHeaderTypeDef header;

    if (HAL_FDCAN_GetRxMessage(hcan, FDCAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return 0U;
    }

    *id          = header.Identifier;
    *is_extended = (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
    *len         = canmgr_fdcan_dlc_to_len(header.DataLength);
    return 1U;
}

#else /* CANMGR_BACKEND_BXCAN */

/** Банки фильтров 0..27 у чипов с CAN1+CAN2 физически общие на обе
 *  периферии (регистр общий на пару). Каждая инициализированная здесь шина
 *  получает свой банк по номеру индекса в пуле - этого достаточно, т.к. на
 *  шину нужен всего один широкий (accept-all) банк, реальная фильтрация -
 *  программная (см. can_manager.h). На чипах с одним CAN (STM32F0 и т.п.)
 *  SlaveStartFilterBank этим значением просто не используется HAL. */
#define CANMGR_BXCAN_SLAVE_START_BANK   14U

static HAL_StatusTypeDef port_configure_filter(CANMGR_CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter;
    CANMGR_Handle_t *bus = canmgr_find_bus(hcan);

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank           = (bus != NULL) ? bus->index : 0U;
    filter.FilterMode           = CAN_FILTERMODE_IDMASK;
    filter.FilterScale          = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh         = 0U;
    filter.FilterIdLow          = 0U;
    filter.FilterMaskIdHigh     = 0U;
    filter.FilterMaskIdLow      = 0U; /* маска 0 - совпадает любой ID (Standard и Extended) */
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation     = ENABLE;
    filter.SlaveStartFilterBank = CANMGR_BXCAN_SLAVE_START_BANK;

    return HAL_CAN_ConfigFilter(hcan, &filter);
}

static HAL_StatusTypeDef port_activate_notifications(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_ActivateNotification(hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY |
        CAN_IT_BUSOFF | CAN_IT_RX_FIFO0_OVERRUN);
}

static HAL_StatusTypeDef port_start(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_Start(hcan);
}

static uint32_t port_get_tx_free_level(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}

static HAL_StatusTypeDef port_send(CANMGR_CAN_HandleTypeDef *hcan, uint32_t id,
                                    uint8_t is_extended, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef header;
    uint32_t tx_mailbox;

    memset(&header, 0, sizeof(header));
    if (is_extended)
    {
        header.ExtId = id;
        header.IDE   = CAN_ID_EXT;
    }
    else
    {
        header.StdId = id;
        header.IDE   = CAN_ID_STD;
    }
    header.RTR                 = CAN_RTR_DATA;
    header.DLC                 = len;
    header.TransmitGlobalTime  = DISABLE;

    return HAL_CAN_AddTxMessage(hcan, &header, data, &tx_mailbox);
}

/** Классический bxCAN восстанавливается из Bus-Off аппаратно сам, ТОЛЬКО
 *  если в CubeMX включена опция ABOM (Automatic Bus-Off Management). Если
 *  она выключена (частый случай по умолчанию), нужна программная помощь -
 *  Stop+Start периферии (см. can_vesc_stm32/motor_vesc.c, тот же приём). */
static void port_bus_off_recover(CANMGR_CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Stop(hcan);
    HAL_CAN_Start(hcan);
}

static uint8_t port_receive(CANMGR_CAN_HandleTypeDef *hcan, uint32_t *id,
                             uint8_t *is_extended, uint8_t *data, uint8_t *len)
{
    CAN_RxHeaderTypeDef header;

    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK)
    {
        return 0U;
    }

    *is_extended = (header.IDE == CAN_ID_EXT) ? 1U : 0U;
    *id          = *is_extended ? header.ExtId : header.StdId;
    *len         = (uint8_t)header.DLC;
    return 1U;
}

#endif /* CANMGR_BACKEND_... */

/* ========================================================================
 *  Модель фильтров и быстрая диспетчеризация (общий код для обоих
 *  бэкендов - см. архитектурное обоснование в can_manager.h)
 * ======================================================================== */

static canmgr_mask_group_t *canmgr_find_mask_group(CANMGR_Handle_t *bus, uint32_t mask,
                                                     uint8_t is_extended)
{
    for (uint32_t i = 0U; i < bus->mask_group_count; i++)
    {
        canmgr_mask_group_t *g = &bus->mask_groups[i];
        if (g->used && (g->mask == mask) && (g->is_extended == is_extended))
        {
            return g;
        }
    }
    return NULL;
}

static void canmgr_dispatch_frame(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                                   const uint8_t *data, uint8_t len)
{
    for (uint32_t i = 0U; i < bus->mask_group_count; i++)
    {
        canmgr_mask_group_t *g = &bus->mask_groups[i];
        if (!g->used || (g->is_extended != is_extended))
        {
            continue;
        }

        uint32_t key = id & g->mask;
        int32_t pos  = canmgr_bsearch(g->sorted_key, g->count, key);
        if (pos >= 0)
        {
            /* Пересечения между ВСЕМИ зарегистрированными фильтрами (в т.ч.
             * из разных групп масок) запрещены при регистрации - значит на
             * конкретный id может найтись не более одного подходящего
             * фильтра суммарно по всем группам, дальше можно не искать. */
            canmgr_filter_t *f = &bus->filters[g->filter_index[pos]];
            if (f->callback != NULL)
            {
                f->callback(bus, id, is_extended, data, len, f->user_ctx);
            }
            return;
        }
    }
    /* Кадр не подошёл ни под один зарегистрированный фильтр - это НЕ
     * ошибка (см. Doxygen CANMGR_RxFifo_Handler в can_manager.h), молча
     * отбрасываем. */
}

/* ========================================================================
 *  Программная очередь отправки - строгий FIFO, единый на шину (см.
 *  обоснование выбора против round-robin в шапке can_manager.h)
 * ======================================================================== */

/** Продвигает очередь отправки: пока в очереди есть пакеты и в аппаратном
 *  буфере есть место - отправляет очередной пакет с головы очереди, строго
 *  по порядку постановки. Вызывается и из CANMGR_TxComplete_Handler (по
 *  прерыванию опустошения буфера), и из CANMGR_Send (см. ниже - почему). */
static void canmgr_service_queue(CANMGR_Handle_t *bus)
{
    while (bus->tx_queue_depth > 0U)
    {
        if (port_get_tx_free_level(bus->config.hcan) == 0U)
        {
            break;
        }

        __disable_irq();
        canmgr_tx_item_t item = bus->tx_queue[bus->tx_head];
        __enable_irq();

        if (port_send(bus->config.hcan, item.id, item.is_extended, item.data, item.len) != HAL_OK)
        {
            /* Редкая гонка (свободный слот успел занять кто-то ещё между
             * проверкой уровня и самой отправкой) - пакет остаётся в
             * голове очереди, попробуем снова на следующем событии. */
            break;
        }

        __disable_irq();
        bus->tx_head = (uint16_t)((bus->tx_head + 1U) % CANMGR_TX_QUEUE_SIZE);
        bus->tx_queue_depth--;
        __enable_irq();
    }
}

/* ========================================================================
 *  Публичный API
 * ======================================================================== */

CANMGR_Handle_t *CANMGR_Init(const CANMGR_Config_t *config)
{
    if ((config == NULL) || (config->hcan == NULL))
    {
        return NULL;
    }

    CANMGR_Handle_t *bus = canmgr_find_bus(config->hcan);
    if (bus != NULL)
    {
        return bus; /* идемпотентность - тот же физический hcan, тот же хэндл */
    }

    bus = canmgr_find_free_bus();
    if (bus == NULL)
    {
        return NULL; /* исчерпан CANMGR_MAX_BUSES */
    }

    memset(bus, 0, sizeof(*bus));
    bus->used         = 1U;
    bus->config       = *config;
    bus->index        = (uint8_t)(bus - s_bus_pool);

    if ((port_configure_filter(bus->config.hcan) != HAL_OK) ||
        (port_activate_notifications(bus->config.hcan) != HAL_OK) ||
        (port_start(bus->config.hcan) != HAL_OK))
    {
        bus->used = 0U; /* освобождаем слот - инициализация не удалась */
        return NULL;
    }

    return bus;
}

CANMGR_RegStatus_t CANMGR_RegisterFilter(CANMGR_Handle_t *bus, uint32_t id, uint32_t mask,
                                          uint8_t is_extended, CANMGR_RxCallback_t callback,
                                          void *user_ctx)
{
    if ((bus == NULL) || (callback == NULL))
    {
        return CANMGR_REG_ERR_INVALID_ARG;
    }

    /* Обрезаем id/mask до реального адресного пространства кадра - лишние
     * старшие биты не должны участвовать в проверке пересечений. */
    uint32_t space_mask = is_extended ? 0x1FFFFFFFU : 0x7FFU;
    id   &= space_mask;
    mask &= space_mask;

    /* Проверка пересечения со ВСЕМИ уже зарегистрированными фильтрами на
     * этой шине (см. формулу и обоснование в шапке can_manager.h) - редкая
     * операция (регистрация), может себе позволить честный O(N) перебор. */
    for (uint32_t i = 0U; i < bus->filter_count; i++)
    {
        canmgr_filter_t *f = &bus->filters[i];
        if (!f->used || (f->is_extended != is_extended))
        {
            continue;
        }
        if (((f->id ^ id) & f->mask & mask) == 0U)
        {
            return CANMGR_REG_ERR_OVERLAP;
        }
    }

    if (bus->filter_count >= CANMGR_MAX_FILTERS_PER_BUS)
    {
        return CANMGR_REG_ERR_FILTERS_FULL;
    }

    canmgr_mask_group_t *group = canmgr_find_mask_group(bus, mask, is_extended);
    if (group == NULL)
    {
        if (bus->mask_group_count >= CANMGR_MAX_MASK_GROUPS)
        {
            return CANMGR_REG_ERR_TOO_MANY_MASKS;
        }
        group = &bus->mask_groups[bus->mask_group_count++];
        group->mask        = mask;
        group->is_extended = is_extended;
        group->used        = 1U;
        group->count       = 0U;
    }

    if (group->count >= CANMGR_MAX_FILTERS_PER_GROUP)
    {
        return CANMGR_REG_ERR_GROUP_FULL;
    }

    uint16_t filter_idx = bus->filter_count;
    canmgr_filter_t *f  = &bus->filters[filter_idx];
    f->id          = id;
    f->mask        = mask;
    f->is_extended = is_extended;
    f->used        = 1U;
    f->callback    = callback;
    f->user_ctx    = user_ctx;
    bus->filter_count++;

    /* Вставка в отсортированный по (id & mask) массив группы - вставка
     * редкая (регистрация), сдвиг хвоста массива на один элемент допустим. */
    uint32_t key = id & mask;
    uint16_t pos = canmgr_lower_bound(group->sorted_key, group->count, key);
    for (uint16_t i = group->count; i > pos; i--)
    {
        group->sorted_key[i]   = group->sorted_key[i - 1U];
        group->filter_index[i] = group->filter_index[i - 1U];
    }
    group->sorted_key[pos]   = key;
    group->filter_index[pos] = filter_idx;
    group->count++;

    return CANMGR_REG_OK;
}

HAL_StatusTypeDef CANMGR_Send(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
                               const uint8_t *data, uint8_t len)
{
    if ((bus == NULL) || ((data == NULL) && (len > 0U)))
    {
        return HAL_ERROR;
    }
    if (len > CANMGR_MAX_DATA_LEN)
    {
        len = CANMGR_MAX_DATA_LEN;
    }

    __disable_irq();

    /* Явное требование пользователя: новый пакет НЕ должен обгонять уже
     * стоящие в очереди пакеты. Поэтому прямая отправка в аппаратный
     * буфер, минуя очередь, возможна ТОЛЬКО когда очередь в этот момент
     * пуста - даже если в буфере физически есть место, но очередь не
     * пуста, пакет обязан встать в её хвост (см. обоснование в
     * can_manager.h). */
    if ((bus->tx_queue_depth == 0U) && (port_get_tx_free_level(bus->config.hcan) > 0U))
    {
        if (port_send(bus->config.hcan, id, is_extended, data, len) == HAL_OK)
        {
            __enable_irq();
            return HAL_OK;
        }
        /* Редкая гонка (место в буфере успело исчезнуть) - падаем в
         * программный путь ниже, как обычно. */
    }

    if (bus->tx_queue_depth >= CANMGR_TX_QUEUE_SIZE)
    {
        __enable_irq();
        return HAL_ERROR; /* очередь переполнена */
    }

    canmgr_tx_item_t *item = &bus->tx_queue[bus->tx_tail];
    item->id          = id;
    item->is_extended = is_extended;
    item->len         = len;
    if (len > 0U)
    {
        memcpy(item->data, data, len);
    }
    bus->tx_tail = (uint16_t)((bus->tx_tail + 1U) % CANMGR_TX_QUEUE_SIZE);
    bus->tx_queue_depth++;

    __enable_irq();
    return HAL_OK;
}

uint32_t CANMGR_GetBusOffCount(const CANMGR_Handle_t *bus)
{
    return (bus != NULL) ? bus->bus_off_count : 0U;
}

uint32_t CANMGR_GetRxOverflowCount(const CANMGR_Handle_t *bus)
{
    return (bus != NULL) ? bus->rx_overflow_count : 0U;
}

uint16_t CANMGR_GetTxQueueDepth(const CANMGR_Handle_t *bus)
{
    return (bus != NULL) ? bus->tx_queue_depth : 0U;
}

#if defined(CANMGR_BACKEND_FDCAN)
void CANMGR_RxFifo_Handler(CANMGR_CAN_HandleTypeDef *hcan, uint32_t RxFifo0ITs)
{
    CANMGR_Handle_t *bus = (hcan != NULL) ? canmgr_find_bus(hcan) : NULL;
    if (bus == NULL)
    {
        return; /* не наша шина */
    }
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    uint32_t id;
    uint8_t  is_extended;
    uint8_t  data[CANMGR_MAX_DATA_LEN];
    uint8_t  len;

    /* Вычитываем ВСЕ кадры, накопившиеся в RxFIFO0 к моменту прерывания -
     * HAL_FDCAN_GetRxMessage возвращает не HAL_OK, когда FIFO опустело. */
    while (port_receive(hcan, &id, &is_extended, data, &len) != 0U)
    {
        canmgr_dispatch_frame(bus, id, is_extended, data, len);
    }
}
#else
void CANMGR_RxFifo_Handler(CANMGR_CAN_HandleTypeDef *hcan)
{
    CANMGR_Handle_t *bus = (hcan != NULL) ? canmgr_find_bus(hcan) : NULL;
    if (bus == NULL)
    {
        return; /* не наша шина */
    }

    uint32_t id;
    uint8_t  is_extended;
    uint8_t  data[CANMGR_MAX_DATA_LEN];
    uint8_t  len;

    while (port_receive(hcan, &id, &is_extended, data, &len) != 0U)
    {
        canmgr_dispatch_frame(bus, id, is_extended, data, len);
    }
}
#endif

void CANMGR_TxComplete_Handler(CANMGR_CAN_HandleTypeDef *hcan)
{
    CANMGR_Handle_t *bus = (hcan != NULL) ? canmgr_find_bus(hcan) : NULL;
    if (bus == NULL)
    {
        return; /* не наша шина */
    }

    canmgr_service_queue(bus);
}

#if defined(CANMGR_BACKEND_FDCAN)
void CANMGR_ErrorStatus_Handler(CANMGR_CAN_HandleTypeDef *hcan, uint32_t ErrorStatusITs)
{
    CANMGR_Handle_t *bus = (hcan != NULL) ? canmgr_find_bus(hcan) : NULL;
    if (bus == NULL)
    {
        return; /* не наша шина */
    }

    if ((ErrorStatusITs & FDCAN_IT_BUS_OFF) != 0U)
    {
        bus->bus_off_count++;
        port_bus_off_recover(hcan); /* см. can_manager.h - FDCAN сам из Bus-Off не выходит */
    }
    if ((ErrorStatusITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        bus->rx_overflow_count++;
    }
}
#else
void CANMGR_ErrorStatus_Handler(CANMGR_CAN_HandleTypeDef *hcan)
{
    CANMGR_Handle_t *bus = (hcan != NULL) ? canmgr_find_bus(hcan) : NULL;
    if (bus == NULL)
    {
        return; /* не наша шина */
    }

    uint32_t err = HAL_CAN_GetError(hcan);

    if ((err & HAL_CAN_ERROR_BOF) != 0U)
    {
        bus->bus_off_count++;
        port_bus_off_recover(hcan); /* см. can_manager.h - без ABOM bxCAN сам из Bus-Off не выходит */
    }
    if ((err & HAL_CAN_ERROR_RX_FOV0) != 0U)
    {
        bus->rx_overflow_count++;
    }
}
#endif
