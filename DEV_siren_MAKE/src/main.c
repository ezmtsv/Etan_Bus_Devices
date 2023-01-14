/* This is a part of the ETAN BUS project
 * Copyright (c) 2018-2021 ETAN Research & Development Center.
 */
#include "compiler.h"
#include "system.h"
#include "ebus_errors.h"
#include "ebus.h"
#include "ebus_utils.h"
#include "ebus_std.h"
#include <string.h>

void bus_pkt_rcvd(void* data, size_t bitcount, int priority) {
    ebus_rxfifo_push(data, bitcount, priority);
}

typedef enum {
    power_on,
    configured,
    sleeped,
} states_t;

static int try_transmit(const void* data, size_t bits, int priority) {
    int res;
    for(int counter = 10; counter > 0; counter--) {
        led_g_on();
        res = ebus_transmit(data, bits, priority);
        led_g_off();
        if (res == ERROR_OK) {
            break;
        }
        led_g_on();
    }
    led_g_off();
    return res;
}

static void test_dev(void) {
	for (uint16_t i = 0; i<4; ++i) {
		led_g_on(); delay_us(100000);
		led_g_off(); delay_us(100000);
	}
	alarm_on();
	delay_ms(2000);
	alarm_off();
}


uint8_t device_address[] = {0xFFU, 0xFFU};
states_t state = power_on;
uint32_t req_life = 1000;

struct {
    uint32_t uid;
    uint32_t cid;
} device_serial;

static int process_req(const uint8_t* data, size_t bits) {
		if (data[0] == EBUS_REQ_SETADDR) {
        memcpy(device_address, &data[1], 2);
//        req_life = (uint16_t)data[3] << 8 | data[4];
				req_life = ((uint16_t)data[4] << 8 | data[3])*1000;		/// время получаем в секундах, 3 байт младший. Переводим в миллисекунды.
        state = configured;
        return ERROR_OK;
		}
		if (data[0] == EBUS_REQ_SETHBTIME) {
//        req_life = (uint16_t)data[1] << 8 | data[2];
				req_life = ((uint16_t)data[2] << 8 | data[1])*1000;		/// время получаем в секундах, 3 байт младший. Переводим в миллисекунды.
        return ERROR_OK;
		}
		if (data[0] == EBUS_REQ_TEST) {
        test_dev();
        return ERROR_OK;
		}	else{
				return ERROR_BAD_COMMAND;
		}
}

int main(void) {
	  uint8_t buf[LINE_BUFSIZE];
    ebus_pkt_t* pkt;

    device_serial.cid = 0x15151515UL;
    device_serial.uid = ebus_uid();

    ebus_rxfifo_init();
    ebus_txfifo_init();
    ebus_init(bus_pkt_rcvd, NULL);
    ebus_up();

	while (1) {
//		led_g_on(); delay_us(100000); led_g_off(); delay_us(100000);

        if (line_down == ebus_get_line_status()) {
            led_g_on();
            continue;
        }
        led_g_off();
        pkt = ebus_rxfifo_packet();
        if (pkt) {
            if (pkt->bitscount > 72 && 0 == memcmp(pkt->data, &device_serial, 8)) {
                process_req(&pkt->data[8], pkt->bitscount - 64);
            } else if (pkt->bitscount > 16 && 0 == memcmp(pkt->data, device_address, 2)) {
                process_req(&pkt->data[2], pkt->bitscount - 16);
            } else if (pkt->bitscount < 8) {
                process_req(&pkt->data[0], pkt->bitscount);
            }
            ebus_rxfifo_next();
        }

        if (state == power_on && heartbeat_count == 0) {
            memcpy(&buf[0], &device_serial, 8);
            buf[8] = EBUS_REQ_AUTH;
            buf[9] = EBUS_DEV_ALARM;
						buf[10] = 16; // кол-во бит данных в пакете (команда и код девайса)
            ebus_txfifo_push(buf, 88, 1);
            heartbeat_count = req_life;
        }
        if (state == configured && heartbeat_count == 0) {
            memcpy(&buf[0], &device_address, 2);
            buf[2] = EBUS_REQ_KEEPALIVE;
            buf[3] = EBUS_DEV_ALARM;
            buf[4] = device_state();
            buf[5] = 24;	// кол-во бит данных в пакете (команда, код девайса, статус)
            ebus_txfifo_push(buf, 48, 1);
            heartbeat_count = req_life;
        }
        pkt = ebus_txfifo_packet();
        if (pkt && ERROR_OK == try_transmit(pkt->data, pkt->bitscount, pkt->priority)) {
            ebus_txfifo_next();
        }
	}
}
