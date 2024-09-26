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
#include <tinyara/fs/mtd.h>
#include <debug.h>

#include <time.h>
#include <tinyara/fs/ioctl.h>
#include <fcntl.h>

#include <sys/time.h>
#include <errno.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/

#define BUF_SIZE   65536
#define SIZE_1MB   1048576

#define printf lldbg

static long sys_start_time = 0;
static long sys_end_time = 0;
static long systick_time_ms = 0;
static char blockname[32];

static void fill_buffer(FAR uint32_t *buff)
{
	off_t offset = 0;
	for (int i = 0; i < BUF_SIZE / sizeof(uint32_t); i++) {
		buff[i] = offset;
		offset += 4;
	}
}

static inline long get_time(void)
{
	struct timeval tv;

	gettimeofday(&tv, NULL);

	return (tv.tv_sec * 1000 + tv.tv_usec / 1000);
}

static int bch_write(void)
{
	FAR uint32_t *buffer;
	int fd;
	ssize_t nbytes;
	off_t offset;
	int ret = ERROR;
	int g_partno = 0;

	/* Allocate a buffer */
	buffer = (FAR uint32_t *)malloc(BUF_SIZE);

	if (!buffer) {
		printf("ERROR: failed to allocate a sector buffer\n");
		return ret;
	}

	fd = open("/dev/mtdblock99", O_WRONLY);
	if (fd < 0) {
		printf("ERROR: open /dev/mtd%d failed: %d\n", g_partno, errno);
		goto out_free_buffer;
	}
	/* Now write the offset into every block */
	printf("Writing to media: buff Size %d, times %d\n", BUF_SIZE, (SIZE_1MB/BUF_SIZE));

	offset = 0;
	/* Fill the block with the offset */
	fill_buffer(buffer);

	sys_start_time = get_time();
	printf("BCH Write start\n");

	/* And write it using the character driver */
	for (int i = 0; i < (SIZE_1MB/BUF_SIZE); i++) {
		nbytes = write(fd, buffer, (size_t)(BUF_SIZE));
		if (nbytes < 0) {
			printf("ERROR: write to /dev/mtd%d failed: %d\n", g_partno, errno);
			goto out_close_fd;
		}
	}
	printf("BCH Write End\n");
	sys_end_time = get_time();
	printf("\nSystick time taken for Flash bch Write %ld(ms)\n", (sys_end_time - sys_start_time));
	ret = OK;

out_close_fd:
	close(fd);
out_free_buffer:
	free(buffer);
	return ret;
}

static int bch_read(void)
{
	FAR uint32_t *buffer;
	FAR uint32_t *rd_buffer;
	int fd;
	ssize_t nbytes;
	off_t offset;
	int g_partno = 0;
	int ret = ERROR;

	/* Allocate a buffer */
	buffer = (FAR uint32_t *)malloc(BUF_SIZE);

	if (!buffer) {
		printf("ERROR: failed to allocate a sector buffer\n");
		return ret;
	}
	rd_buffer = (FAR uint32_t *)malloc(BUF_SIZE);
	if (!buffer) {
		printf("ERROR: failed to allocate a sector buffer\n");
		free(buffer);
		return ret;
	}

	/* Open the master MTD FLASH character driver for writing */

	fd = open("/dev/mtdblock99", O_RDONLY);
	if (fd < 0) {
		printf("ERROR: open /dev/mtd%d failed: %d\n", g_partno, errno);
		goto out_free_buffer;
	}

	/* Now Read the offset into every block */
	printf("Reading from media: buff Size %d, times %d\n", BUF_SIZE, (SIZE_1MB/BUF_SIZE));

	offset = 0;
	systick_time_ms = 0;
	/* Fill the block with the offset */

	fill_buffer(buffer);

	/* And write it using the character driver */
	printf("BCH Read start\n");
	for (int j = 0; j < (SIZE_1MB/BUF_SIZE); j++) {
		sys_start_time = get_time();
		nbytes = read(fd, rd_buffer, (size_t)(BUF_SIZE));
		if (nbytes < 0) {
			printf("ERROR: Read to /dev/mtd%d failed: %d\n", g_partno, errno);
			close(fd);
			goto out_close_fd;
		}
		printf("%02x (read), %02x (orig)\n", rd_buffer[0], buffer[0]);
		printf("%02x (read), %02x (orig)\n", rd_buffer[1], buffer[1]);
		sys_end_time = get_time();
		systick_time_ms += sys_end_time - sys_start_time;
		if (memcmp(rd_buffer, buffer, nbytes)) {
			printf("ERROR: Read value comparison failed to /dev/mtd%d failed: %d\n", g_partno, errno);
			//close(fd);
			//goto out_close_fd;
		}
	}
	printf("BCH Read End\n");
	printf("\nSystick time taken for Flash bch Read %ld(ms)\n", systick_time_ms);
	ret = OK;

out_close_fd:
	close(fd);
out_free_buffer:
	free(buffer);
	free(rd_buffer);
	return ret;
}

#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, World!!\n");
	
	struct spi_dev_s *spi = up_spiinitialize(1);

	FAR struct mtd_dev_s *dev_mtd = NULL;

	dev_mtd = xt26g02d_initialize(spi);

	dhara_initialize(99, dev_mtd);
	
	int ret = bch_write();

	if (ret != OK) {
		printf("bch write failed\n");
		return 0;
	}
	
	ret = bch_read();

	if (ret != OK) {
		printf("bch read failed\n");
		return 0;
	}
	
	return 0;
}
