/****************************************************************************
 *
 * Copyright 2016 Samsung Electronics All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing,
 * software distributed under the License is distributed on an
 * "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND,
 * either express or implied. See the License for the specific
 * language governing permissions and limitations under the License.
 *
 ****************************************************************************/
/****************************************************************************
 * examples/hello/hello_main.c
 *
 *   Copyright (C) 2008, 2011-2012 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdio.h>

#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h>

#include <tinyara/sched.h>
#include <tinyara/spi/spi.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/

#define CPU_ZERO(s) do { *(s) = 0; } while (0)
#define CPU_SET(c,s) do { *(s) |= (1 << (c)); } while (0)

char buffer[1024];

void test_thread(void);

void spi_read() {
	FAR struct spi_dev_s *spi = up_spiinitialize(0);

                SPI_SETMODE(spi, SPIDEV_MODE0);
                SPI_SETFREQUENCY(spi, 12000000);
                SPI_SETBITS(spi, 8);

		SPI_LOCK(spi, 1);

                while (true) {
                                       uint8_t read_flag = 1 << 7; /* msb for read is 1 and write is 0 */

                       uint8_t data[2];
                       data[0] = 0x00 | read_flag;
                       data[1] = 0xFF;

                       uint8_t recv[2];
                       recv[0] = 0x00;
                       recv[1] = 0x00;

		       for (int i = 0; i < 24000; i++) {
			SPI_SELECT(spi, 0, true);
                       	SPI_EXCHANGE(spi, data, recv, 2);
                       	SPI_SELECT(spi, 0, false);
		       }

                       //for (int i = 0; i < 2; i++)
                       //lldbg("SPI0 read test result sent : %x rec : %x\n", data[1], recv[1]);

		       sleep(1);

                }

		SPI_LOCK(spi, 0);

                return;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	if (argc > 1) {
#if 0
		FAR struct spi_dev_s *spi = up_spiinitialize(0);

                SPI_SETMODE(spi, SPIDEV_MODE0);
                SPI_SETFREQUENCY(spi, 12000000);
                SPI_SETBITS(spi, 8);
		while (true) {
		                       uint8_t read_flag = 1 << 7; /* msb for read is 1 and write is 0 */

                       uint8_t data[2];
                       data[0] = 0x00 | read_flag;
                       data[1] = 0xFF;

                       uint8_t recv[2];
                       recv[0] = 0x00;
                       recv[1] = 0x00;

                       SPI_SELECT(spi, 0, true);
                       SPI_EXCHANGE(spi, data, recv, 2);
                       SPI_SELECT(spi, 0, false);

                       for (int i = 0; i < 2; i++)
                               printf("SPI0 read test result sent : %x rec : %x\n", data[i], recv[i]);

		}
#endif
		task_create("spi test", 100, 4096, spi_read, NULL);
		return 0;
	}

	pthread_attr_t attr;

	int ret = pthread_attr_init(&attr);

	CPU_ZERO(&attr.affinity);
        CPU_SET(1, &attr.affinity);

	pid_t thread_id;

	ret = pthread_create(&thread_id, &attr, test_thread, NULL);

	while (true);

}

void test_thread() {

	printf("Hello, World!!, cpu : %d\n", sched_getcpu());

	//sleep(5);

	int try = 0;

	FAR struct spi_dev_s *spi = up_spiinitialize(1);

        SPI_SETMODE(spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(spi, 12000000);
        SPI_SETBITS(spi, 8);
	
	SPI_LOCK(spi, 1);

	while (true) {
		for (int i = 0; i < 20000; i++) {		
		SPI_SELECT(spi, 0, true);
		(void)SPI_SEND(spi, 0x9f);
        	uint32_t manufacturer = SPI_SEND(spi, 0xa5);
        	uint32_t memory = SPI_SEND(spi, 0xa5);
		uint32_t capacity = SPI_SEND(spi, 0xa5);
		//lldbg("manufacturer: %02x memory: %02x capacity: %02x\n", manufacturer, memory, capacity);
		if (manufacturer != 0xef || memory != 0x40 || capacity != 0x20) {
			lldbg("manufacturer: %02x memory: %02x capacity: %02x\n", manufacturer, memory, capacity);
		}
		SPI_SELECT(spi, 0, false);
		}


#if 0
		int fd = open("/res/product/GUI/main/800x480/image/3241b578f1ec35ec6d90ec.png", O_RDONLY);
		if (fd < 0) {
			printf("Error!! open failed\n");
		}
		int ret;
		ret = read(fd, buffer, 1024);
		if (ret > 0) {
		//	printf("%d : %s", try, buffer);
		} else {
			printf("Error!! read failed\n");
		}
		close(fd);
		try++;
#endif
	}

	SPI_LOCK(spi, 0);

	return 0;
}
