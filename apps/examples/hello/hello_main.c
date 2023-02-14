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
#include <tinyara/binary_manager.h>
#include <binary_manager/binary_manager.h>

/****************************************************************************
 * hello_main
 ****************************************************************************/
static int fail_cnt = 0;

static void binary_update_cb(void)
{
	printf(" ==========================================================================\n");
	printf("   The state changed callback is executed in WIFI. %s state is changed. \n", "kernel");
	printf(" ========================================================================= \n");
}

static void print_binary_info(binary_update_info_t *binary_info)
{
	printf(" =========== binary [%s] info ============ \n", binary_info->name);
	printf(" %10s | %8s\n", "Version", "Available size");
	printf(" -------------------------------------------- \n");
	printf(" %8.1u | %8d\n", binary_info->version, binary_info->available_size);
	printf(" ============================================ \n");
}

static void print_binary_info_list(binary_update_info_list_t *binary_info_list)
{
	int bin_idx;

	printf(" ============== ALL binary info : %d count ================ \n", binary_info_list->bin_count);
	printf(" %4s | %6s | %10s | %8s\n", "Idx", "Name", "Version", "Available size");
	printf(" -------------------------------------------------------- \n");
	for (bin_idx = 0; bin_idx < binary_info_list->bin_count; bin_idx++) {
		printf(" %4d | %6s | %8.1u | %8d\n", bin_idx, \
		binary_info_list->bin_info[bin_idx].name, binary_info_list->bin_info[bin_idx].version, \
		binary_info_list->bin_info[bin_idx].available_size);
	}
	printf(" ======================================================== \n");
}

static int binary_update_check_test_result(binary_update_info_t *pre_bin_info, binary_update_info_t *cur_bin_info, int condition)
{
	int ret = ERROR;

	printf(" ========== [%5s] Update info =========== \n", cur_bin_info->name);
	printf(" %4s | %10s \n", "Con", "Version");
	printf(" ----------------------------------------- \n");
	printf(" %4s | %8.1u \n", "Pre", pre_bin_info->version);
	printf(" %4s | %8.1u \n", "Cur", cur_bin_info->version);
	printf(" ========================================== \n");

	if (true) {
		if (pre_bin_info->version == cur_bin_info->version) {
			fail_cnt++;
			printf("Fail to load valid higher version binary.\n");
		} else {
			ret = OK;
			printf("Success to load valid higher version binary.\n");
		}
	} else { //DOWNLOAD_INVALID_BIN
		if (pre_bin_info->version != cur_bin_info->version) {
			fail_cnt++;
			printf("Warning! Load invalid binary.\n");
		} else {
			ret = OK;
			printf("No update with invalid binary.\n");
		}
	}

	return ret;
}

static void binary_update_getinfo_all(void)
{
	int ret;
	binary_update_info_list_t bin_info_list;

	printf("\n** Binary Update GETINFO_ALL test.\n");
	ret = binary_manager_get_update_info_all(&bin_info_list);
	if (ret == OK) {
		print_binary_info_list(&bin_info_list);
	} else {
		fail_cnt++;
		printf("Get binary info all FAIL %d\n", ret);
	}
}

static int binary_update_getinfo(char *name, binary_update_info_t *bin_info)
{
	int ret;

	printf("\n** Binary Update GETINFO [%s] test.\n", name);
	ret = binary_manager_get_update_info(name, bin_info);
	if (ret == OK) {
		print_binary_info(bin_info);
	} else {
		fail_cnt++;
		printf("Get binary info FAIL, ret %d\n", ret);
	}

	return ret;
}

static int binary_update_reload(void)
{
	int ret;

	printf("\n** Binary Update RELOAD test.\n");
	ret = binary_manager_update_binary();
	if (ret == OK) {
		printf("RELOAD SUCCESS\n");
	} else {
		fail_cnt++;
		printf("Reload binary FAIL, ret %d\n", ret);
	}

	return ret;
}

static int binary_update_register_state_changed_callback(void)
{
	int ret;

	printf("\n** Binary Update Register state changed callback test.\n");
	ret = binary_manager_register_state_changed_callback((binmgr_statecb_t)binary_update_cb, NULL);
	if (ret == OK) {
		printf("Register state changed callback SUCCESS\n");
	} else {
		fail_cnt++;
		printf("Register state changed callback FAIL, ret %d\n", ret);
	}

	return ret;
}

static int binary_update_unregister_state_changed_callback(void)
{
	int ret;

	printf("\n** Binary Update Unregister state changed callback test.\n");
	ret = binary_manager_unregister_state_changed_callback();
	if (ret == OK) {
		printf("Unregister state changed callback SUCCESS\n");
	} else {
		fail_cnt++;
		printf("Unregister state changed callback FAIL, ret %d\n", ret);
	}

	return ret;
}


#ifdef CONFIG_BUILD_KERNEL
int main(int argc, FAR char *argv[])
#else
int hello_main(int argc, char *argv[])
#endif
{
	printf("Hello, Abhishek!!\n");
	int ret;
	uint8_t type = 0;
	binary_setbp_result_t result;
	binary_update_info_t pre_bin_info;
	binary_update_info_t cur_bin_info;

	printf("\n** Binary Update New Version Test. **\n");

	ret = binary_update_getinfo("kernel", &pre_bin_info);
	if (ret != OK) {
		return ret;
	}

	BM_SET_GROUP(type, BINARY_KERNEL);
	ret = binary_manager_set_bootparam(type, &result);
	printf("binary_manager_set_bootparam %d\n", ret);
	if (ret != OK) {
		if (ret == BINMGR_ALREADY_UPDATED) {
			int idx;
			for (idx = 0; idx < BINARY_TYPE_MAX; idx++) {
				printf("[%d] result %d\n", idx, result.result[idx]);
			}
		}
		return ret;
	}

	ret = binary_update_reload();
	return 0;
}
