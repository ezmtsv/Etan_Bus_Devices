/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */
#ifndef _EBUS_STD_H_
#define _EBUS_STD_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include <stdint.h>

#define EBUS_REQ_KEEPALIVE  0x0BU
#define EBUS_REQ_AUTH       0x0CU
#define EBUS_REQ_SETHBTIME  0x0EU
#define EBUS_REQ_EVENT      0x11U
#define EBUS_REQ_SETADDR    0x13U
#define EBUS_REQ_TEST       0x14U


#define EBUS_DEV_BUTTON     0x16U
#define EBUS_DEV_RELAY      0x17U
#define EBUS_DEV_ALARM      0x18U

#define EBUS_RESET_CMD      0x00U
#define EBUS_RESET_LEN      0x02U

#define EBUS_DEV_MOVE       0x19U
#define EBUS_DEV_FIRE       0x1AU
#define EBUS_DEV_LIGHT      0x1BU
#define EBUS_DEV_RFID       0x1CU
#define EBUS_DEV_OW         0x1DU
#define EBUS_DEV_RFKEY      0x1EU
#define EBUS_DEV_OWKEY      0x1FU
#define EBUS_DEV_COMBO      0x20U
#define EBUS_DEV_BROKE      0x21U

#ifdef __cplusplus
}
#endif
#endif //_EBUS_STD_H_
