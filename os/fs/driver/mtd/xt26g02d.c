/****************************************************************************
 *
 * Copyright 2024 Samsung Electronics All Rights Reserved.
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

/************************************************************************************
 * Included Files
 ************************************************************************************/

#include <tinyara/mtd/nand_raw.h>
#include <tinyara/mtd/nand_scheme.h>
#include <debug.h>

// Lets disable HWECC also for now, we only need to implement three apis
// Note that we need SPI to be provided to nand_raw to talk to the flash

int xt26g02d_eraseblock(FAR struct nand_raw_s *raw, off_t block) {
	uint32_t address = block << 6; // to get block address, now we populate it
        uint8_t * addr = &address;
	// erase block before writing
        // write enable
        
	SPI_SETMODE(raw->spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(raw->spi, 40000000);
        SPI_SETBITS(raw->spi, 8);

        // get device ID to verify the flash
        uint8_t cmd_data[6];
        uint8_t dev_id[2];

	cmd_data[0] = 0x06;
        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 1);
        SPI_SELECT(raw->spi, 0, false);

        // erase block 0
        cmd_data[0] = 0xD8;
        cmd_data[1] = addr[2];
        cmd_data[2] = addr[1];
        cmd_data[3] = addr[0];

        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 4);
        SPI_SELECT(raw->spi, 0, false);

        // wait till done
        // wait for op to be done
        while (true) {
                cmd_data[0] = 0x0F;
                cmd_data[1] = 0xC0;

                SPI_SELECT(raw->spi, 0, true);
                SPI_SNDBLOCK(raw->spi, cmd_data, 2);
                SPI_RECVBLOCK(raw->spi, dev_id, 1);
                SPI_SELECT(raw->spi, 0, false);

                if (dev_id[0] & 1) {
                        usleep(1000);
                        continue;
                } else {
                        break;
                }
        }
	return 1;
}
  
int xt26g02d_rawread(FAR struct nand_raw_s *raw, off_t block,
                      unsigned int page, FAR void *data, FAR void *spare) {
	uint32_t address = (block << 6) + page; // to get block address, now we populate it
        uint8_t * addr = &address;
	uint8_t address24[3];
	for (int i = 0; i < 3; i++) address24[i] = 0;
	SPI_SETMODE(raw->spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(raw->spi, 40000000);
        SPI_SETBITS(raw->spi, 8);

        // get device ID to verify the flash
        uint8_t cmd_data[6];
        uint8_t dev_id[2];

	// read page to cache
        cmd_data[0] = 0x13;
	// complete page address
	address24[0] = addr[2];
        address24[1] = addr[1];
        address24[2] = addr[0];
        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 1);
        SPI_SNDBLOCK(raw->spi, address24, 3);
        SPI_SELECT(raw->spi, 0, false);

        // wait for op to be done
        while (true) {
                cmd_data[0] = 0x0F;
                cmd_data[1] = 0xC0;

                SPI_SELECT(raw->spi, 0, true);
                SPI_SNDBLOCK(raw->spi, cmd_data, 2);
                SPI_RECVBLOCK(raw->spi, dev_id, 1);
                SPI_SELECT(raw->spi, 0, false);

                if (dev_id[0] & 1) {
                        continue;
                } else {
                        break;
                }
        }

        // read page from cache
        cmd_data[0] = 0x03;
	SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 1);
        if (data != NULL) {
		// read everything from start
		for (int i = 0; i < 3; i++) address24[i] = 0;
	} else {
		// only read spare, spare is from 2048 onwards for 64 (128 bytes) bytes.
		for (int i = 0; i < 3; i++) address24[i] = 0;
		uint16_t addr = 2048;
		uint8_t * p = &addr;
		address24[0] = p[1];
		address24[1] = p[0];
	}
	SPI_SNDBLOCK(raw->spi, address24, 3);
	if (data)
	SPI_RECVBLOCK(raw->spi, data, 2048);
	if (spare)
	SPI_RECVBLOCK(raw->spi, spare, 64);
        SPI_SELECT(raw->spi, 0, false);
	return 1;
}

int xt26g02d_rawwrite(FAR struct nand_raw_s *raw, off_t block,
                       unsigned int page, FAR const void *data,
                       FAR const void *spare) {
	uint32_t address = (block << 6) + page; // to get block address, now we populate it
        uint8_t * addr = &address;

	SPI_SETMODE(raw->spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(raw->spi, 40000000);
        SPI_SETBITS(raw->spi, 8);
	        uint8_t address24[3];
		for (int i = 0; i < 3; i++) address24[i] = 0;
        // get device ID to verify the flash
        uint8_t cmd_data[6];
        uint8_t dev_id[2];
	
	cmd_data[0] = 0x1F;
        cmd_data[1] = 0xA0;
        cmd_data[2] = 0x00;

        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 3);
        SPI_SELECT(raw->spi, 0, false);	

	// Now try to write to the page
        // program load
        cmd_data[0] = 0x02;
        // we are writing from the start
//        cmd_data[1] = 0x00;
//        cmd_data[2] = 0x00;

	if (data != NULL) {
		cmd_data[1] = 0x00;
		cmd_data[2] = 0x00;
	} else {
		uint16_t addr = 2048;
                uint8_t * p = &addr;
                cmd_data[1] = p[1];
                cmd_data[2] = p[0];
	}

        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 3);
        if (data != NULL)
	SPI_SNDBLOCK(raw->spi, data, 2048);
	if (spare != NULL)
	SPI_SNDBLOCK(raw->spi, spare, 64);
        SPI_SELECT(raw->spi, 0, false);

        // write enable
        cmd_data[0] = 0x06;
        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 1);
        SPI_SELECT(raw->spi, 0, false);

        // start programming
        cmd_data[0] = 0x10;
        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 1);
        SPI_SNDBLOCK(raw->spi, address24, 3);
        SPI_SELECT(raw->spi, 0, false);
	
	// wait till done
        // wait for op to be done
        while (true) {
                cmd_data[0] = 0x0F;
                cmd_data[1] = 0xC0;

                SPI_SELECT(raw->spi, 0, true);
                SPI_SNDBLOCK(raw->spi, cmd_data, 2);
                SPI_RECVBLOCK(raw->spi, dev_id, 1);
                SPI_SELECT(raw->spi, 0, false);

                if (dev_id[0] & 1) {
                        continue;
                } else {
                        break;
                }
        }
	return 1;
}

FAR struct mtd_dev_s *xt26g02d_initialize(FAR struct spi_dev_s *spi)
{
	struct nand_raw_s * raw = (struct nand_raw_s *)malloc(sizeof(struct nand_raw_s));
	raw->eraseblock = xt26g02d_eraseblock;
	raw->rawread = xt26g02d_rawread;
	raw->rawwrite = xt26g02d_rawwrite;
	raw->spi = spi;
	raw->model.devid = 0x32;
	raw->model.pagesize = 2048;
	raw->model.sparesize = 128;
	raw->model.devsize = 256;
	raw->model.blocksize = 128;
	raw->model.scheme = &g_nand_sparescheme2048;
	return nand_initialize(raw);
}
