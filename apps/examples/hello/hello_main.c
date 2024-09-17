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
#include <tinyara/spi/spi.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
uint8_t page_data[2176];

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");
	
	struct spi_dev_s *spi = up_spiinitialize(1);

	SPI_SETMODE(spi, SPIDEV_MODE0);
	SPI_SETFREQUENCY(spi, 1000000);
	SPI_SETBITS(spi, 8);

	// get device ID to verify the flash
	uint8_t cmd_data[3];
	uint8_t dev_id[2];

	cmd_data[0] = 0x9F;
	cmd_data[1] = 0x00;

	SPI_SELECT(spi, 0, true);
	SPI_SNDBLOCK(spi, cmd_data, 2);
	SPI_RECVBLOCK(spi, dev_id, 2);
	SPI_SELECT(spi, 0, false);
	
	printf("MfID : %02x, DID : %02x\n", dev_id[0], dev_id[1]);

	// get default value of features
	
	cmd_data[0] = 0x0F;
	cmd_data[1] = 0x90;

	for (int i = 0; i < 4; i++) {
		cmd_data[1] += 1 << 4;
		SPI_SELECT(spi, 0, true);
		SPI_SNDBLOCK(spi, cmd_data, 2);
		SPI_RECVBLOCK(spi, dev_id, 1);
		SPI_SELECT(spi, 0, false);
		printf("Feature : %02x, value : %02x\n", cmd_data[1], dev_id[0]);
	}

	// read contents of block 0 page 0
	uint8_t address24[3];
	
	for (int i = 0; i < 3; i++) address24[i] = 0;
	
	// read page to cache
	cmd_data[0] = 0x13;
	SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 1);
        SPI_SNDBLOCK(spi, address24, 3);
	SPI_SELECT(spi, 0, false);

	// wait for op to be done
	while (true) {
		cmd_data[0] = 0x0F;
		cmd_data[1] = 0xC0;

		SPI_SELECT(spi, 0, true);
                SPI_SNDBLOCK(spi, cmd_data, 2);
                SPI_RECVBLOCK(spi, dev_id, 1);
                SPI_SELECT(spi, 0, false);
                printf("Feature : %02x, value : %02x\n", cmd_data[1], dev_id[0]);
		
		if (dev_id[0] & 1) {
			usleep(1000);
			continue;
		} else {
			break;
		}
	}

	// read page from cache
        cmd_data[0] = 0x03;
        SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 1);
        SPI_SNDBLOCK(spi, address24, 3);
	// 1 dummy byte exchange
	SPI_RECVBLOCK(spi, dev_id, 1);
	// start reading actual data
	SPI_RECVBLOCK(spi, page_data, 2176);
	SPI_SELECT(spi, 0, false);
	
	printf("First Byte in block 0 : %02x\n", page_data[0]);
	printf("Bad block mark for block 0 : %x\n", page_data[2048]);

	for (int i = 0; i < 2176; i++) {
  		if (i % 32 == 0) printf("\n");
 		printf("%02X", page_data[i]);
  	}

	if (page_data[0] != 0xFF) {
		for (int i = 0; i < 2176; i++) {
			if (i % 32 == 0) printf("\n");
			printf("%02X", page_data[i]);
		}
		printf("page already has data, return from here\n");
		return 0;
	}

	if (page_data[2048] == 0) {
		// found bad block
		printf("Bad block found, return from here\n");
		return 0;
	}

	for (int i = 0; i < 2048; i++) page_data[i] = 0xAA;

	// Now, we try to write to the first page
	// first set BP0, BP1, BP2 to zeros
	
	cmd_data[0] = 0x1F;
	cmd_data[1] = 0xA0;
	cmd_data[2] = 0x00;

	SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 3);
        SPI_SELECT(spi, 0, false);

	cmd_data[0] = 0x0F;
        cmd_data[1] = 0xA0;

	SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 2);
        SPI_RECVBLOCK(spi, dev_id, 1);
	SPI_SELECT(spi, 0, false);

	// updated value for write protection
	printf("Value of write protection register (%02x) : %02x\n", cmd_data[1], dev_id[0]);


	// Now try to write to the page
	// program load
	cmd_data[0] = 0x02;
	// we are writing from the start
	cmd_data[1] = 0x00;

	SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 2);
        SPI_SNDBLOCK(spi, page_data, 2048);
	SPI_SELECT(spi, 0, false);

	// write enable
	cmd_data[0] = 0x06;
	SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 1);
        SPI_SELECT(spi, 0, false);
	
	// start programming
        cmd_data[0] = 0x10;
        SPI_SELECT(spi, 0, true);
        SPI_SNDBLOCK(spi, cmd_data, 1);
        SPI_SNDBLOCK(spi, address24, 3);
        SPI_SELECT(spi, 0, false);

	// wait till done
	// wait for op to be done
        while (true) {
                cmd_data[0] = 0x0F;
                cmd_data[1] = 0xC0;

                SPI_SELECT(spi, 0, true);
                SPI_SNDBLOCK(spi, cmd_data, 2);
                SPI_RECVBLOCK(spi, dev_id, 1);
                SPI_SELECT(spi, 0, false);
                printf("Feature : %02x, value : %02x\n", cmd_data[1], dev_id[0]);

                if (dev_id[0] & 1) {
                        usleep(1000);
                        continue;
                } else {
                        break;
                }
        }

	return 0;
}
