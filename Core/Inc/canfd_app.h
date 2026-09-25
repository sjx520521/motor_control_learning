#ifndef CANFD_APP_H
#define CANFD_APP_H

#include "canfd_protocol.h"
#include "stm32g4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

void Error_Handler(void);

void canfd_app_init(FDCAN_HandleTypeDef *hfdcan,
                    uint8_t node_id,
                    uint32_t control_timeout_ms);

void canfd_app_update(uint32_t now_ms);

uint8_t canfd_app_send_status(const CanFdStatus *status);

const CanFdControlTarget *canfd_app_get_target(void);

uint8_t canfd_app_control_alive(void);

uint32_t canfd_app_fault_code(void);

#ifdef __cplusplus
}
#endif

#endif /* CANFD_APP_H */
