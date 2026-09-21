/**
 ******************************************************************************
 * @file    can_manager.c
 * @brief   Реализация единого менеджера шины CAN/FDCAN для STM32 - см.
 *          README.md / API_REFERENCE.md за архитектурой и API.
 * @author  Mechanic
 * @date    19.09.2026
 * @version 0.4
 *
 * @copyright Copyright (c) 2026 Mechanic.
 *            Свободное некоммерческое использование и модификация. Условия
 *            распространения - см. LICENSE / README.md в составе проекта.
 ******************************************************************************
 */

#include <string.h>
#include "can_manager.h"

/* ========================================================================
 *  Опциональная зависимость от stm32_logger (см. README.md за подробностями).
 *  Включается пользователем библиотеки через "#define CANMGR_ENABLE_LOGGER"
 *  до включения can_manager.h/.c - без него библиотека собирается и работает
 *  ровно как раньше, logger.h вообще не подключается. Коды LOG_CODE_CANMGR_*
 *  и их приоритеты/описания живут не здесь, а в logger_codes.h конкретного
 *  проекта (единая точка учёта адресных пространств кодов, см. правила
 *  интеграции в logger_codes.h) - can_manager только вызывает LOGGER_Log()
 *  с уже готовыми кодами, никогда не хранит числовые коды сам.
 * ======================================================================== */
#ifdef CANMGR_ENABLE_LOGGER
#include "logger.h"
#include "logger_codes.h"
#define CANMGR_LOG(code, source_id, value) LOGGER_Log((code), (uint16_t)(source_id), (int32_t)(value))
#else
#define CANMGR_LOG(code, source_id, value) ((void)0)
#endif

/* ========================================================================
 *  Статический пул шин (без malloc, как во всех библиотеках этой базы)
 * ======================================================================== */

static CANMGR_Handle_t s_bus_pool[CANMGR_MAX_BUSES];

/* ========================================================================
 *  Внутренние утилиты общего назначения
 * ======================================================================== */

/** @brief  Находит шину по хэндлу периферии.
 *  @param  hcan Хэндл периферии.
 *  @return Указатель на шину, либо NULL, если такой шины нет. */
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

/** @brief  Находит свободный слот в пуле шин.
 *  @return Указатель на свободный слот, либо NULL, если пул исчерпан. */
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

/** @brief  Бинарный поиск ключа в отсортированном по возрастанию массиве.
 *  @param  sorted Отсортированный по возрастанию массив.
 *  @param  count  Число элементов в массиве.
 *  @param  key    Искомый ключ.
 *  @return Индекс совпадения, либо -1, если ключа нет в массиве. */
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

/** @brief  Позиция вставки key, сохраняющая сортировку массива (аналог
 *          std::lower_bound).
 *  @param  sorted Отсортированный по возрастанию массив.
 *  @param  count  Число элементов в массиве.
 *  @param  key    Значение для вставки.
 *  @return Индекс позиции вставки. */
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
 *  motor_vesc.c (референс на чтение, см. AGENTS.md проекта).
 * ======================================================================== */

#if defined(CANMGR_BACKEND_FDCAN)

/** Индекс фильтра в списке FDCAN. У каждого инстанса FDCAN (FDCAN1, FDCAN2,
 *  ...) свои, независимые банки фильтров - конфликтов между разными шинами
 *  здесь не бывает, можно использовать одно и то же значение для всех. */
#define CANMGR_FDCAN_FILTER_INDEX   0U

/** @brief  Переводит длину данных в код DLC поля HAL FDCAN.
 *  @param  len Длина данных, 0..8 байт.
 *  @return Код DLC, готовый для записи в поле заголовка HAL. */
static uint32_t canmgr_len_to_fdcan_dlc(uint8_t len)
{
    /* Classic CAN/FDCAN-кадр (без BRS/FD, см. "Чего в этой версии нет" в
     * can_manager.h): для длины 0..8 код DLC в поле HAL совпадает с самим
     * числом байт, сдвинутым в позицию поля (биты 19:16) - см. макросы
     * FDCAN_DLC_BYTES_0..FDCAN_DLC_BYTES_8 в HAL. */
    return ((uint32_t)len) << 16;
}

/** @brief  Переводит код DLC поля HAL FDCAN обратно в длину данных.
 *  @param  dlc Код DLC из заголовка HAL.
 *  @return Длина данных, отклампленная до CANMGR_MAX_DATA_LEN. */
static uint8_t canmgr_fdcan_dlc_to_len(uint32_t dlc)
{
    uint32_t code = (dlc >> 16) & 0x0FU;
    /* Коды 9..15 - это FD-длины (12..64 байт), сюда попасть не должны, т.к.
     * мы принимаем только Classic-кадры - на всякий случай защищаемся от
     * выхода за границы буфера данных. */
    return (uint8_t)((code <= CANMGR_MAX_DATA_LEN) ? code : CANMGR_MAX_DATA_LEN);
}

/** @brief  Настраивает широкий приёмный фильтр FDCAN (Standard и Extended).
 *  @param  hcan Хэндл периферии.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
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

/** @brief  Включает нотификации приёма/ошибок/опустошения Tx для FDCAN.
 *  @param  hcan Хэндл периферии.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
static HAL_StatusTypeDef port_activate_notifications(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_ActivateNotification(hcan,
        FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_RX_FIFO0_MESSAGE_LOST |
        FDCAN_IT_TX_FIFO_EMPTY | FDCAN_IT_BUS_OFF, 0U);
}

/** @brief  Запускает периферию FDCAN.
 *  @param  hcan Хэндл периферии.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
static HAL_StatusTypeDef port_start(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_Start(hcan);
}

/** @brief  Число свободных слотов в аппаратном Tx FIFO FDCAN.
 *  @param  hcan Хэндл периферии.
 *  @return Число свободных слотов. */
static uint32_t port_get_tx_free_level(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_FDCAN_GetTxFifoFreeLevel(hcan);
}

/** @brief  Кладёт кадр напрямую в аппаратный Tx FIFO FDCAN.
 *  @param  hcan        Хэндл периферии.
 *  @param  id          Идентификатор кадра.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @param  data        Данные кадра, 0..8 байт.
 *  @param  len         Длина данных.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
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
/** @param  hcan Хэндл периферии. */
static void port_bus_off_recover(CANMGR_CAN_HandleTypeDef *hcan)
{
    hcan->Instance->CCCR &= ~FDCAN_CCCR_INIT;
}

/** @brief  Читает один принятый кадр FDCAN из RxFIFO0.
 *  @param  hcan        Хэндл периферии.
 *  @param  id          [out] Идентификатор кадра.
 *  @param  is_extended [out] 0 - Standard, 1 - Extended.
 *  @param  data        [out] Буфер под данные, минимум CANMGR_MAX_DATA_LEN байт.
 *  @param  len         [out] Длина данных.
 *  @return 1, если кадр прочитан; 0, если FIFO пусто. */
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

/** Банки фильтров 0..27 у чипов с CAN1+CAN2 физически ОБЩИЙ регистровый
 *  ресурс на обе периферии, но не взаимозаменяемый: банки [0 ..
 *  CANMGR_BXCAN_SLAVE_START_BANK) аппаратно принадлежат master-у (CAN1),
 *  банки [CANMGR_BXCAN_SLAVE_START_BANK .. 28) - slave-у (CAN2). Эта
 *  граница задаётся полем SlaveStartFilterBank и одинакова для ВСЕХ
 *  банков на обеих периферих одновременно - если шине на CAN2 назначить
 *  номер банка из диапазона CAN1 (что и происходило раньше при наивном
 *  использовании общего индекса шины в пуле как номера банка), реально
 *  конфигурируется банк, принадлежащий CAN1, а не CAN2 - фильтрация
 *  CAN1 при этом молча ломается, а CAN2 остаётся без рабочего фильтра.
 *  Поэтому номер банка выделяется НИЖЕ отдельным счётчиком на каждый из
 *  двух физических инстансов, а не просто индексом шины в общем пуле. */
#define CANMGR_BXCAN_SLAVE_START_BANK   14U

/** Выделяет следующий свободный номер банка для конкретного физического
 *  инстанса (CAN1 либо CAN2 - см. обоснование выше). Раздельные счётчики
 *  на инстанс переживают отдельные вызовы CANMGR_Init() (static) - банк
 *  не переиспользуется, даже если конкретная инициализация впоследствии
 *  завершится неудачей (см. CANMGR_Init) - это безопасный, но не строго
 *  экономный выбор: 28 банков с большим запасом хватает на реалистичное
 *  число шин (CANMGR_MAX_BUSES по умолчанию - 4). */
/** @brief  Выделяет следующий свободный номер банка фильтра bxCAN.
 *  @param  hcan Хэндл периферии (CAN1 или CAN2).
 *  @return Номер банка фильтра. */
static uint32_t port_alloc_filter_bank(const CANMGR_CAN_HandleTypeDef *hcan)
{
#if defined(CAN2)
    static uint32_t s_can2_banks_used = 0U;
    if (hcan->Instance == CAN2)
    {
        return CANMGR_BXCAN_SLAVE_START_BANK + s_can2_banks_used++;
    }
#else
    (void)hcan; /* чипы без CAN2 (STM32F0 и т.п.) - все банки принадлежат единственному CAN */
#endif
    static uint32_t s_can1_banks_used = 0U;
    return s_can1_banks_used++;
}

/** @brief  Настраивает широкий приёмный фильтр bxCAN (Standard и Extended).
 *  @param  hcan Хэндл периферии.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
static HAL_StatusTypeDef port_configure_filter(CANMGR_CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));
    filter.FilterBank           = port_alloc_filter_bank(hcan);
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

/** @brief  Включает нотификации приёма/ошибок/опустошения Tx для bxCAN.
 *  @param  hcan Хэндл периферии.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
static HAL_StatusTypeDef port_activate_notifications(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_ActivateNotification(hcan,
        CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_TX_MAILBOX_EMPTY |
        CAN_IT_BUSOFF | CAN_IT_RX_FIFO0_OVERRUN);
}

/** @brief  Запускает периферию bxCAN.
 *  @param  hcan Хэндл периферии.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
static HAL_StatusTypeDef port_start(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_Start(hcan);
}

/** @brief  Число свободных почтовых ящиков Tx bxCAN.
 *  @param  hcan Хэндл периферии.
 *  @return Число свободных почтовых ящиков. */
static uint32_t port_get_tx_free_level(CANMGR_CAN_HandleTypeDef *hcan)
{
    return HAL_CAN_GetTxMailboxesFreeLevel(hcan);
}

/** @brief  Кладёт кадр напрямую в аппаратный почтовый ящик bxCAN.
 *  @param  hcan        Хэндл периферии.
 *  @param  id          Идентификатор кадра.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @param  data        Данные кадра, 0..8 байт.
 *  @param  len         Длина данных.
 *  @return HAL_OK при успехе, иначе код ошибки HAL. */
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
/** @param  hcan Хэндл периферии. */
static void port_bus_off_recover(CANMGR_CAN_HandleTypeDef *hcan)
{
    HAL_CAN_Stop(hcan);
    HAL_CAN_Start(hcan);
}

/** @brief  Читает один принятый кадр bxCAN из RxFIFO0.
 *  @param  hcan        Хэндл периферии.
 *  @param  id          [out] Идентификатор кадра.
 *  @param  is_extended [out] 0 - Standard, 1 - Extended.
 *  @param  data        [out] Буфер под данные, минимум CANMGR_MAX_DATA_LEN байт.
 *  @param  len         [out] Длина данных.
 *  @return 1, если кадр прочитан; 0, если FIFO пусто. */
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

    /* header.DLC - 4-битное аппаратное поле (0..15) как пришло с провода.
     * Classic CAN гарантирует полезную нагрузку не более 8 байт, но само
     * поле DLC способно физически нести 9..15 (неисправный или недобро-
     * совестный узел на шине может это выставить - HAL считывает регистр
     * как есть, без клампа). Буфер data[], в который HAL уже записал
     * данные несколькими строками выше, имеет размер ровно
     * CANMGR_MAX_DATA_LEN - без клампа здесь потребитель получил бы
     * len > CANMGR_MAX_DATA_LEN и мог прочитать data[] за границей буфера
     * (переполнение чтения стека), спровоцированное чужим кадром на шине. */
    *len = (uint8_t)header.DLC;
    if (*len > CANMGR_MAX_DATA_LEN)
    {
        *len = CANMGR_MAX_DATA_LEN;
    }
    return 1U;
}

#endif /* CANMGR_BACKEND_... */

/* ========================================================================
 *  Модель фильтров и быстрая диспетчеризация (общий код для обоих
 *  бэкендов - см. архитектурное обоснование в can_manager.h)
 * ======================================================================== */

/** @brief  Находит группу фильтров с данным значением маски.
 *  @param  bus         Хэндл шины.
 *  @param  mask        Значение маски.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @return Указатель на группу, либо NULL, если такой группы нет. */
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

/** @brief  Находит подходящий фильтр и вызывает его callback.
 *  @param  bus         Хэндл шины.
 *  @param  id          Идентификатор принятого кадра.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @param  data        Данные кадра.
 *  @param  len         Длина данных. */
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
 *  Программная очередь отправки - строгий FIFO, единый на шину
 * ======================================================================== */

/** Продвигает очередь отправки: пока в очереди есть пакеты и в аппаратном
 *  буфере есть место - отправляет очередной пакет с головы очереди, строго
 *  по порядку постановки. Вызывается и из CANMGR_TxComplete_Handler (по
 *  прерыванию опустошения буфера), и из CANMGR_Send (см. ниже - почему).
 *  @param  bus Хэндл шины. */
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

/** @brief  Инициализирует шину CAN/FDCAN (см. API_REFERENCE.md).
 *  @param  config Конфигурация шины.
 *  @return Хэндл шины, либо NULL при ошибке. */
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
        CANMGR_LOG(LOG_CODE_CANMGR_INIT_FAIL, 0xFFFFU, 0); /* пул шин исчерпан, конкретной шины ещё нет */
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
        CANMGR_LOG(LOG_CODE_CANMGR_INIT_FAIL, bus->index, 0);
        bus->used = 0U; /* освобождаем слот - инициализация не удалась */
        return NULL;
    }

    CANMGR_LOG(LOG_CODE_CANMGR_INIT_OK, bus->index, 0);
    return bus;
}

/** @brief  Регистрирует пару (фильтр, callback) на шине (см. API_REFERENCE.md).
 *  @param  bus         Хэндл шины.
 *  @param  id          Значение id фильтра.
 *  @param  mask        Маска фильтра.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @param  callback    Обработчик принятых кадров.
 *  @param  user_ctx    Контекст, передаваемый в callback.
 *  @return Код результата регистрации. */
CANMGR_RegStatus_t CANMGR_RegisterFilter(CANMGR_Handle_t *bus, uint32_t id, uint32_t mask,
                                          uint8_t is_extended, CANMGR_RxCallback_t callback,
                                          void *user_ctx)
{
    if ((bus == NULL) || (callback == NULL))
    {
        return CANMGR_REG_ERR_INVALID_ARG;
    }

    /* Нормализуем is_extended строго к 0/1 - CANMGR_RxFifo_Handler всегда
     * передаёт в canmgr_dispatch_frame ровно 0 либо 1 (см. port_receive
     * обоих бэкендов), а сравнение там - строгое (!=), не булево. Без этой
     * нормализации вызывающий код, передавший сюда любое другое ненулевое
     * значение (например бит из битовой маски флагов, а не буквальную
     * единицу), зарегистрировал бы фильтр, который НИКОГДА не совпадёт ни
     * с одним реально принятым кадром - тихий, трудно диагностируемый баг. */
    is_extended = (is_extended != 0U) ? 1U : 0U;

    /* Обрезаем id/mask до реального адресного пространства кадра - лишние
     * старшие биты не должны участвовать в проверке пересечений. */
    uint32_t space_mask = is_extended ? 0x1FFFFFFFU : 0x7FFU;
    id   &= space_mask;
    mask &= space_mask;

    /* Проверка пересечения со ВСЕМИ уже зарегистрированными фильтрами на
     * этой шине - редкая операция (регистрация), может себе позволить
     * честный O(N) перебор. */
    for (uint32_t i = 0U; i < bus->filter_count; i++)
    {
        canmgr_filter_t *f = &bus->filters[i];
        if (!f->used || (f->is_extended != is_extended))
        {
            continue;
        }
        if (((f->id ^ id) & f->mask & mask) == 0U)
        {
            CANMGR_LOG(LOG_CODE_CANMGR_REG_REJECTED, bus->index, CANMGR_REG_ERR_OVERLAP);
            return CANMGR_REG_ERR_OVERLAP;
        }
    }

    if (bus->filter_count >= CANMGR_MAX_FILTERS_PER_BUS)
    {
        CANMGR_LOG(LOG_CODE_CANMGR_REG_REJECTED, bus->index, CANMGR_REG_ERR_FILTERS_FULL);
        return CANMGR_REG_ERR_FILTERS_FULL;
    }

    canmgr_mask_group_t *group = canmgr_find_mask_group(bus, mask, is_extended);
    if (group == NULL)
    {
        if (bus->mask_group_count >= CANMGR_MAX_MASK_GROUPS)
        {
            CANMGR_LOG(LOG_CODE_CANMGR_REG_REJECTED, bus->index, CANMGR_REG_ERR_TOO_MANY_MASKS);
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
        CANMGR_LOG(LOG_CODE_CANMGR_REG_REJECTED, bus->index, CANMGR_REG_ERR_GROUP_FULL);
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
     * редкая (регистрация), сдвиг хвоста массива на один элемент допустим.
     *
     * КРИТИЧЕСКАЯ СЕКЦИЯ: пока элементы сдвигаются, массив group->sorted_key
     * временно находится в промежуточном, не полностью упорядоченном
     * состоянии (значение, которое "сдвигается" в соседний индекс, на один
     * шаг присутствует одновременно в двух позициях, а старое значение
     * ещё не записано на новое место). CANMGR_RxFifo_Handler может прийти
     * по прерыванию В ЛЮБОЙ момент - в т.ч. между инициализацией одного
     * потребителя и другого, когда шина уже реально принимает трафик для
     * уже зарегистрированных ранее фильтров (регистрация нарочно не
     * привязана к моменту "до первого кадра на шине").
     * Без запрета прерываний здесь canmgr_dispatch_frame(), выполняющий
     * бинарный поиск по этому же массиву ровно в этот момент, мог бы
     * временно не найти уже существующий, корректно зарегистрированный
     * фильтр - кадр был бы молча потерян не из-за отсутствия подписки, а
     * из-за гонки при регистрации СОВСЕМ ДРУГОГО фильтра. */
    __disable_irq();

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

    __enable_irq();

    return CANMGR_REG_OK;
}

/** @brief  Ставит кадр в очередь отправки шины (см. API_REFERENCE.md).
 *  @param  bus         Хэндл шины.
 *  @param  id          Идентификатор кадра.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @param  data        Данные кадра, 0..8 байт.
 *  @param  len         Длина данных.
 *  @return HAL_OK при успехе, иначе HAL_ERROR. */
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
        CANMGR_LOG(LOG_CODE_CANMGR_TX_QUEUE_FULL, bus->index, id);
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

/** @brief  Отправляет актуальное значение с заменой в очереди (см. API_REFERENCE.md).
 *  @param  bus         Хэндл шины.
 *  @param  id          Идентификатор кадра.
 *  @param  is_extended 0 - Standard, 1 - Extended.
 *  @param  data        Данные кадра, 0..8 байт.
 *  @param  len         Длина данных.
 *  @return HAL_OK при успехе, иначе HAL_ERROR. */
HAL_StatusTypeDef CANMGR_SendLatest(CANMGR_Handle_t *bus, uint32_t id, uint8_t is_extended,
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

    /* Ищем уже стоящий в ПРОГРАММНОЙ очереди пакет с тем же (id,
     * is_extended) - обходим ровно bus->tx_queue_depth слотов начиная с
     * головы, как они реально идут по кольцевому буферу. Если находим -
     * подменяем данные ПРЯМО В ЭТОМ СЛОТЕ, не трогая tx_head/tx_tail и,
     * тем самым, не меняя позицию пакета в очереди. Пакет, уже покинувший
     * программную очередь (ушедший в аппаратный буфер), этим поиском не
     * достаётся - и не должен: он уже был самым актуальным значением на
     * момент, когда до него дошла очередь. */
    __disable_irq();
    for (uint16_t n = 0U; n < bus->tx_queue_depth; n++)
    {
        uint16_t idx = (uint16_t)((bus->tx_head + n) % CANMGR_TX_QUEUE_SIZE);
        canmgr_tx_item_t *item = &bus->tx_queue[idx];
        if ((item->id == id) && (item->is_extended == is_extended))
        {
            item->len = len;
            if (len > 0U)
            {
                memcpy(item->data, data, len);
            }
            __enable_irq();
            return HAL_OK;
        }
    }
    __enable_irq();

    /* Пакета с таким id в очереди нет - обычная отправка: прямо в
     * аппаратный буфер (если очередь пуста и есть место) либо новым
     * пакетом в хвост очереди - см. CANMGR_Send(). */
    return CANMGR_Send(bus, id, is_extended, data, len);
}

/** @brief  Число событий Bus-Off с момента CANMGR_Init().
 *  @param  bus Хэндл шины.
 *  @return Счётчик событий, либо 0 при bus == NULL. */
uint32_t CANMGR_GetBusOffCount(const CANMGR_Handle_t *bus)
{
    return (bus != NULL) ? bus->bus_off_count : 0U;
}

/** @brief  Число переполнений Rx FIFO0 с момента CANMGR_Init().
 *  @param  bus Хэндл шины.
 *  @return Счётчик переполнений, либо 0 при bus == NULL. */
uint32_t CANMGR_GetRxOverflowCount(const CANMGR_Handle_t *bus)
{
    return (bus != NULL) ? bus->rx_overflow_count : 0U;
}

/** @brief  Текущая длина программной очереди отправки.
 *  @param  bus Хэндл шины.
 *  @return Длина очереди, либо 0 при bus == NULL. */
uint16_t CANMGR_GetTxQueueDepth(const CANMGR_Handle_t *bus)
{
    return (bus != NULL) ? bus->tx_queue_depth : 0U;
}

#if defined(CANMGR_BACKEND_FDCAN)
/** @brief  Обработчик приёма FDCAN - вызывать из HAL_FDCAN_RxFifo0Callback.
 *  @param  hcan        Хэндл периферии.
 *  @param  RxFifo0ITs  Флаги причины прерывания из HAL. */
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
/** @brief  Обработчик приёма bxCAN - вызывать из HAL_CAN_RxFifo0MsgPendingCallback.
 *  @param  hcan Хэндл периферии. */
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

/** @brief  Обработчик опустошения Tx - продвигает программную очередь отправки.
 *  @param  hcan Хэндл периферии. */
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
/** @brief  Обработчик ошибок FDCAN - вызывать из HAL_FDCAN_ErrorStatusCallback.
 *  @param  hcan           Хэндл периферии.
 *  @param  ErrorStatusITs Флаги причины прерывания из HAL. */
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
        port_bus_off_recover(hcan); /* FDCAN сам из Bus-Off не выходит */
        CANMGR_LOG(LOG_CODE_CANMGR_BUS_OFF, bus->index, bus->bus_off_count);
    }
    if ((ErrorStatusITs & FDCAN_IT_RX_FIFO0_MESSAGE_LOST) != 0U)
    {
        bus->rx_overflow_count++;
        CANMGR_LOG(LOG_CODE_CANMGR_RX_OVERFLOW, bus->index, bus->rx_overflow_count);
    }
}
#else
/** @brief  Обработчик ошибок bxCAN - вызывать из HAL_CAN_ErrorCallback.
 *  @param  hcan Хэндл периферии. */
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
        port_bus_off_recover(hcan); /* без ABOM bxCAN сам из Bus-Off не выходит */
        CANMGR_LOG(LOG_CODE_CANMGR_BUS_OFF, bus->index, bus->bus_off_count);
    }
    if ((err & HAL_CAN_ERROR_RX_FOV0) != 0U)
    {
        bus->rx_overflow_count++;
        CANMGR_LOG(LOG_CODE_CANMGR_RX_OVERFLOW, bus->index, bus->rx_overflow_count);
    }
}
#endif
