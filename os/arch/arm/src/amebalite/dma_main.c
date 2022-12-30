/*
 *  Routines to access hardware
 *
 *  Copyright (c) 2013 Realtek Semiconductor Corp.
 *
 *  This module is a confidential and proprietary property of RealTek and
 *  possession or use of this module requires written permission of RealTek.
 */

#include "device.h"
#include "serial_api.h"
#include "serial_ex_api.h"
//#include "main.h"

#define UART_TX    PA_20
#define UART_RX    PA_19

/*note that dma mode buffer length should be integral multiple of 32 bytes*/
#define SRX_BUF_SZ    (32*5)
/*note that dma mode buffer address should be 32 bytes aligned*/
char rx_buf[SRX_BUF_SZ]__attribute__((aligned(32))) = {0};

volatile uint32_t tx_busy = 0;
volatile uint32_t rx_done = 0;

void uart_send_string_done(uint32_t id)
{
	lldbg("transmitted string\n");
	(void)id;
	tx_busy = 0;
}

void uart_recv_string_done(uint32_t id)
{
	lldbg("recieved string\n");
	(void)id;
	rx_done = 1;
}

void uart_send_string(serial_t *sobj, char *pstr)
{
	int32_t ret = 0;
	if (tx_busy) {
		lldbg("tx is busy\n");
		return;
	}

	tx_busy = 1;
	ret = serial_send_stream_dma(sobj, pstr, _strlen(pstr));
	if (ret != 0) {
		lldbg("%s Error(%d)\n", __FUNCTION__, ret);
		tx_busy = 0;
	} else {
		lldbg("dma transfer done\n");
	}
}

void uart_stream_dma(void)
{
	serial_t sobj;
	int ret;
	int i = 0;
	int len;

	sobj.uart_idx = 3;

	serial_init(&sobj, UART_TX, UART_RX);
	serial_baud(&sobj, 1500000);
	serial_format(&sobj, 8, ParityNone, 1);

	serial_send_comp_handler(&sobj, (void *)uart_send_string_done, (uint32_t) &sobj);
	serial_recv_comp_handler(&sobj, (void *)uart_recv_string_done, (uint32_t) &sobj);

	for (int j = 0; j < SRX_BUF_SZ; j++) {
		rx_buf[j] = 0;
	}

	lldbg("Receive 13-byte-data %d\n", sobj.uart_idx);

	ret = serial_recv_stream_dma(&sobj, rx_buf, 13);

	if (ret) {
		lldbg(" %s: Recv Error(%d)\n", __FUNCTION__, ret);
		rx_done = 1;
	}

	while (1) {
		if (rx_done) {
			uart_send_string(&sobj, rx_buf);
			rx_done = 0;

			len = (i + 3) % 15 + 1;
			i++;

			/* Wait for inputing x character to initiate DMA. */
			lldbg("Receive %d-byte-data\n", len);

			ret = serial_recv_stream_dma(&sobj, rx_buf, len);
			rx_buf[len] = 0;    // end of string

			if (ret) {
				lldbg(" %s: Recv Error(%d)\n", __FUNCTION__, ret);
				rx_done = 1;
			}

		}
	}

}

void main(void)
{
	lldbg("This is dma_main task\n");
	uart_stream_dma();
}

