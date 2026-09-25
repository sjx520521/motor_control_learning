#include "canfd_app.h"

#include <string.h>

static FDCAN_HandleTypeDef *g_canfd_handle;
static CanFdProtocol g_canfd_protocol;

static uint32_t canfd_length_to_dlc(uint8_t length)
{
    if (length <= 8U)
    {
        static const uint32_t dlc[9] =
        {
            FDCAN_DLC_BYTES_0, FDCAN_DLC_BYTES_1,
            FDCAN_DLC_BYTES_2, FDCAN_DLC_BYTES_3,
            FDCAN_DLC_BYTES_4, FDCAN_DLC_BYTES_5,
            FDCAN_DLC_BYTES_6, FDCAN_DLC_BYTES_7,
            FDCAN_DLC_BYTES_8
        };
        return dlc[length];
    }

    if (length <= 12U) return FDCAN_DLC_BYTES_12;
    if (length <= 16U) return FDCAN_DLC_BYTES_16;
    if (length <= 20U) return FDCAN_DLC_BYTES_20;
    if (length <= 24U) return FDCAN_DLC_BYTES_24;
    if (length <= 32U) return FDCAN_DLC_BYTES_32;
    if (length <= 48U) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

static uint8_t canfd_dlc_to_length(uint32_t dlc)
{
    switch (dlc)
    {
    case FDCAN_DLC_BYTES_0: return 0U;
    case FDCAN_DLC_BYTES_1: return 1U;
    case FDCAN_DLC_BYTES_2: return 2U;
    case FDCAN_DLC_BYTES_3: return 3U;
    case FDCAN_DLC_BYTES_4: return 4U;
    case FDCAN_DLC_BYTES_5: return 5U;
    case FDCAN_DLC_BYTES_6: return 6U;
    case FDCAN_DLC_BYTES_7: return 7U;
    case FDCAN_DLC_BYTES_8: return 8U;
    case FDCAN_DLC_BYTES_12: return 12U;
    case FDCAN_DLC_BYTES_16: return 16U;
    case FDCAN_DLC_BYTES_20: return 20U;
    case FDCAN_DLC_BYTES_24: return 24U;
    case FDCAN_DLC_BYTES_32: return 32U;
    case FDCAN_DLC_BYTES_48: return 48U;
    case FDCAN_DLC_BYTES_64: return 64U;
    default: return 0U;
    }
}

static uint8_t canfd_app_send_frame(const CanFdFrame *frame)
{
    FDCAN_TxHeaderTypeDef header = {0};

    if (g_canfd_handle == 0 || frame == 0 ||
        frame->length > CANFD_MAX_DATA_BYTES)
    {
        return 0U;
    }

    header.Identifier = frame->identifier;
    header.IdType = (frame->is_extended != 0U)
                        ? FDCAN_EXTENDED_ID
                        : FDCAN_STANDARD_ID;
    header.TxFrameType = FDCAN_DATA_FRAME;
    header.DataLength = canfd_length_to_dlc(frame->length);
    header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    header.BitRateSwitch = (frame->bitrate_switch != 0U)
                               ? FDCAN_BRS_ON
                               : FDCAN_BRS_OFF;
    header.FDFormat = (frame->is_fd != 0U)
                          ? FDCAN_FD_CAN
                          : FDCAN_CLASSIC_CAN;
    header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return (HAL_FDCAN_AddMessageToTxFifoQ(g_canfd_handle,
                                          &header,
                                          frame->data) == HAL_OK)
               ? 1U
               : 0U;
}

void canfd_app_init(FDCAN_HandleTypeDef *hfdcan,
                    uint8_t node_id,
                    uint32_t control_timeout_ms)
{
    FDCAN_FilterTypeDef filter = {0};

    g_canfd_handle = hfdcan;
    canfd_protocol_init(&g_canfd_protocol,
                        node_id,
                        control_timeout_ms);

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0U;
    filter.FilterType = FDCAN_FILTER_DUAL;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = g_canfd_protocol.control_id;
    filter.FilterID2 = g_canfd_protocol.command_id;

    if (HAL_FDCAN_ConfigFilter(g_canfd_handle, &filter) != HAL_OK ||
        HAL_FDCAN_ConfigGlobalFilter(g_canfd_handle,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT,
                                     FDCAN_REJECT_REMOTE,
                                     FDCAN_REJECT_REMOTE) != HAL_OK ||
        HAL_FDCAN_Start(g_canfd_handle) != HAL_OK ||
        HAL_FDCAN_ActivateNotification(
            g_canfd_handle,
            FDCAN_IT_RX_FIFO0_NEW_MESSAGE | FDCAN_IT_BUS_OFF,
            0U) != HAL_OK)
    {
        Error_Handler();
    }
}

void canfd_app_update(uint32_t now_ms)
{
    canfd_protocol_update(&g_canfd_protocol, now_ms);
}

uint8_t canfd_app_send_status(const CanFdStatus *status)
{
    CanFdFrame frame;

    if (canfd_protocol_pack_status(&g_canfd_protocol,
                                   status,
                                   &frame) == 0U)
    {
        return 0U;
    }

    return canfd_app_send_frame(&frame);
}

const CanFdControlTarget *canfd_app_get_target(void)
{
    return canfd_protocol_get_target(&g_canfd_protocol);
}

uint8_t canfd_app_control_alive(void)
{
    return canfd_protocol_is_control_alive(&g_canfd_protocol);
}

uint32_t canfd_app_fault_code(void)
{
    return g_canfd_protocol.fault_code;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef header;
    CanFdFrame frame;

    if (hfdcan != g_canfd_handle ||
        (RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)
    {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(
               hfdcan,
               FDCAN_RX_FIFO0) > 0U)
    {
        memset(&frame, 0, sizeof(frame));
        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                   FDCAN_RX_FIFO0,
                                   &header,
                                   frame.data) != HAL_OK)
        {
            return;
        }

        frame.identifier = header.Identifier;
        frame.is_extended =
            (header.IdType == FDCAN_EXTENDED_ID) ? 1U : 0U;
        frame.is_fd =
            (header.FDFormat == FDCAN_FD_CAN) ? 1U : 0U;
        frame.bitrate_switch =
            (header.BitRateSwitch == FDCAN_BRS_ON) ? 1U : 0U;
        frame.length = canfd_dlc_to_length(header.DataLength);

        (void)canfd_protocol_handle_frame(&g_canfd_protocol,
                                          &frame,
                                          HAL_GetTick());
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    if (hfdcan == g_canfd_handle)
    {
        g_canfd_protocol.fault_code |= CANFD_FAULT_BAD_FRAME;
    }
}
