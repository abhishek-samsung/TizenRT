/****************************************************************************
 *
 * Copyright 2020 Samsung Electronics All Rights Reserved.
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
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>
#include <stdlib.h>
#include <debug.h>
#include <stdio.h>
#include <string.h>
#include <queue.h>
#include <sys/types.h>
#include <tinyara/mm/mm.h>
#include <tinyara/mm/heap_regioninfo.h>
#include <binary_manager/binary_manager.h>

#include "../binfmt/libelf/libelf.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define MEM_USED   0
#define MEM_LEAK   1
#define MEM_BROKEN 2

#define MEM_ACCESS_UNIT    0x04
#define MAX_ALLOC_COUNT    CONFIG_MEM_LEAK_CHECKER_MAX_ALLOC_COUNT
#define HASH_SIZE          CONFIG_MEM_LEAK_CHECKER_HASH_TABLE_SIZE

//#define RAM_START_ADDR  kregionx_start[0]
#define RAM_START_ADDR  kregionx_start[0]
#define RAM_END_ADDR	(kregionx_start[CONFIG_KMM_REGIONS - 1] + kregionx_size[CONFIG_KMM_REGIONS - 1])
#define HEAP_START_ADDR (void *)leak_checker.heap_start
#define HEAP_END_ADDR   (void *)leak_checker.heap_end

struct alloc_node_info_s {
	volatile struct mm_allocnode_s *node;
	struct alloc_node_info_s *next;
};

static struct alloc_node_info_s *hash_table[HASH_SIZE];
static struct alloc_node_info_s node_info[MAX_ALLOC_COUNT];
static struct mem_leak_checker_info_s leak_checker;

#ifdef CONFIG_MM_KERNEL_HEAP
extern struct mm_heap_s g_kmmheap[CONFIG_KMM_NHEAPS];
#endif
extern sq_queue_t g_bin_addr_list;
static void hash_init(void)
{
	int index;
	for (index = 0; index < HASH_SIZE; ++index) {
		hash_table[index] = NULL;
	}
}

static void add_hash(int index)
{
	long key;
	struct alloc_node_info_s *cur;
	key = (long)node_info[index].node % HASH_SIZE;
	if (hash_table[key] == NULL) {
		hash_table[key] = &node_info[index];
		return;
	}

	cur = hash_table[key];
	while (cur->next) {
		cur = cur->next;
	}
	cur->next = &node_info[index];
}

static bool search_hash(unsigned long value)
{
	long key = value % HASH_SIZE;
	struct alloc_node_info_s *cur = hash_table[key];
	while (cur != NULL) {
		if ((unsigned long)cur->node == value && (unsigned long)cur->node != (value + (unsigned long)SIZEOF_MM_ALLOCNODE)) {
			if (cur->node->reserved == MEM_USED) {
				return false;
			}
			cur->node->reserved = MEM_USED;
			return true;
		}
		if (cur->next == NULL) {
			return false;
		}
		cur = cur->next;
	}
	return false;
}

static int get_node_cnt(void)
{
	volatile struct mm_allocnode_s *node;
	mmsize_t node_size;
	node_size = SIZEOF_MM_ALLOCNODE;

	int ret = 0;
	for (node = (struct mm_allocnode_s *)HEAP_START_ADDR; node < (struct mm_allocnode_s *)HEAP_END_ADDR; node = (struct mm_allocnode_s *)((char *)node + node->size)) {
		/* Ignore the heap start checking, because there is a guard node in heap start */
		if (node == HEAP_START_ADDR) {
			continue;
		}
		/* Check broken link */
		if (node_size != (node->preceding & 0x7FFFFFFF)) {
			continue;
		}
		node_size = node->size;
		/* Check if the node corresponds to an allocated memory chunk */
		if ((node->preceding & MM_ALLOC_BIT) != 0) {
			ret++;
		}
	}
	return ret;
}

static void fill_hash_table(int *leak_cnt, int *broken_cnt)
{
	volatile struct mm_allocnode_s *node;
	mmsize_t node_size;
	node_size = SIZEOF_MM_ALLOCNODE;

	mm_takesemaphore((struct mm_heap_s *)leak_checker.heap);

	for (node = (struct mm_allocnode_s *)HEAP_START_ADDR; node < (struct mm_allocnode_s *)HEAP_END_ADDR; node = (struct mm_allocnode_s *)((char *)node + node->size)) {
		/* Ignore the heap start checking, because there is a guard node in heap start */
		if (node == HEAP_START_ADDR) {
			continue;
		}

		/* Check broken link */
		if (node_size != (node->preceding & 0x7FFFFFFF)) {
			node->reserved = MEM_BROKEN;
			(*broken_cnt)++;
			continue;
		}
		node_size = node->size;
		if ((unsigned long)node + (unsigned long)SIZEOF_MM_ALLOCNODE == (unsigned long)node_info || (unsigned long)node + (unsigned long)SIZEOF_MM_ALLOCNODE == (unsigned long)hash_table) {
			continue;
		}
		/* Check if the node corresponds to an allocated memory chunk */
		if ((node->preceding & MM_ALLOC_BIT) != 0) {
			node_info[*leak_cnt].node = node;
			node_info[*leak_cnt].next = NULL;
			node->reserved = MEM_LEAK;
			add_hash(*leak_cnt);
			(*leak_cnt)++;
		}
	}
	mm_givesemaphore((struct mm_heap_s *)leak_checker.heap);
}

static void search_addr(void *start_addr, void *end_addr, int *leak_cnt)
{
	   lldbg("SEARCHING FROM : %p\n", start_addr);
	   lldbg("SEARCHIGN TO : %p\n", end_addr);
	   lldbg("LEAK COUNT : %d\n", *leak_cnt);
	/* This function traverse the memory from start_addr to end_addr for comparing the address based on hash table. */
	void *leak_chk;
	/* Not to access over its region, subtract 0x04 from the end of the address. */
	for (leak_chk = start_addr; leak_chk < end_addr - MEM_ACCESS_UNIT; leak_chk++) {
		if (leak_chk == (void *)&node_info) {
			/* Skip to check hash table */
			leak_chk += (sizeof(struct alloc_node_info_s) * MAX_ALLOC_COUNT);
			leak_chk--;
			continue;
		}
		if (search_hash(*(unsigned long volatile *)leak_chk - (unsigned long)SIZEOF_MM_ALLOCNODE)) {
			(*leak_cnt)--;
		}
	}
}

static void heap_check(int *leak_cnt)
{
	void *leak_chk;
	struct mm_allocnode_s *visit_node;
	void *exclude_top;
	void *exclude_bottom;

	exclude_top = leak_checker.stack_top;
	exclude_bottom = leak_checker.stack_bottom;

	for (visit_node = (struct mm_allocnode_s *)HEAP_START_ADDR; visit_node < (struct mm_allocnode_s *)HEAP_END_ADDR; visit_node = (struct mm_allocnode_s *)((char *)visit_node + visit_node->size)) {
		if ((visit_node->preceding & MM_ALLOC_BIT) != 0) {
			for (leak_chk = (void *)visit_node; leak_chk < (void *)(((char *)visit_node) + visit_node->size); leak_chk++) {
				if ((leak_chk >= exclude_bottom && leak_chk <= exclude_top)) {
					continue;
				}
				if (search_hash(*(unsigned long volatile *)leak_chk - (unsigned long)SIZEOF_MM_ALLOCNODE)) {
					(*leak_cnt)--;
				}
			}
		}
	}
}

extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t __bss_start__;
extern uint32_t __bss_end__;
extern uint32_t __psram_bss_start__;
extern uint32_t __psram_bss_end__; 
extern uint32_t __psram_data_start__;
extern uint32_t __psram_data_end__;

static void ram_check(char *bin_name, int *leak_cnt, uint32_t *bin_text_addr)
{
#ifdef CONFIG_APP_BINARY_SEPARATION
	bin_addr_info_t *info;
	int bin_idx;

	info = (bin_addr_info_t *)get_bin_addr_list();
	for (bin_idx = 0; bin_idx <= CONFIG_NUM_APPS; bin_idx++) {
		if (strncmp(BIN_NAME(bin_idx), bin_name, strlen(bin_name)) == 0) {
			/* For calculating the owner from binary, save the text section address. */
			*bin_text_addr = info[bin_idx].text_addr;
		}
	}
#endif
	/* Visit the RAM region, except heap region.
	 * So check from RAM start to heap start, and from heap end to RAM end.
	 */
//	lldbg("RAM START ADDR : %d\n",RAM_START_ADDR);
//	lldbg("RAM END ADDR : %d\n",RAM_END_ADDR);
//	lldbg("HEAP_START_ADDR : %d\n", HEAP_START_ADDR);
//	lldbg("HEAP_END_ADDR : %d\n", HEAP_END_ADDR);

	//search_addr(RAM_START_ADDR, HEAP_START_ADDR, leak_cnt);
	search_addr(HEAP_END_ADDR, RAM_END_ADDR, leak_cnt);

	search_addr(&_sdata, &_edata, leak_cnt);
	search_addr(&__bss_start__, &__bss_end__, leak_cnt);
	search_addr(&__psram_bss_start__, &__psram_bss_end__, leak_cnt);
	search_addr(&__psram_data_start__, &__psram_data_end__, leak_cnt);

	/* Visit heap region */
	heap_check(leak_cnt);
}

static void print_info(char *bin_name, int leak_cnt, int broken_cnt, uint32_t bin_text_addr)
{
	volatile struct mm_allocnode_s *node;
	uint32_t owner_addr;	

#ifdef CONFIG_APP_BINARY_SEPARATION
	elf_show_all_bin_section_addr();
#endif


	if (leak_cnt > 0 || broken_cnt > 0) {
		lldbg("Type   |    Addr   |   Size   |  Owner  | PID \n");
		lldbg("---------------------------------------------------\n");

		mm_takesemaphore((struct mm_heap_s *)leak_checker.heap);

		for (node = (struct mm_allocnode_s *)HEAP_START_ADDR; node < (struct mm_allocnode_s *)HEAP_END_ADDR; node = (struct mm_allocnode_s *)((char *)node + node->size)) {
			if (node->reserved == MEM_LEAK) {
				/* alloc_call_addr can be from kernel, app or common binary.
				 * based on the text addresses printed, user needs to check the
				 * corresponding binaries accordingly 
				 */
				owner_addr = node->alloc_call_addr;
				pid_t pid = node->pid;
				if (pid < 0) {
					/* For stack allocated node, pid is negative value.
					 * To use the pid, change it to original positive value.
					 */
					pid = (-1) * pid;
				}
				lldbg("LEAK   |  %p  | %8d | %p | %d\n", (void *)((char *)node + SIZEOF_MM_ALLOCNODE), node->size - SIZEOF_MM_ALLOCNODE, owner_addr, pid);
			} else if (node->reserved == MEM_BROKEN) {
				lldbg("BROKEN | %p\n", node);
			}
		}

		mm_givesemaphore((struct mm_heap_s *)leak_checker.heap);

		lldbg("*** %d LEAKS, %d BROKENS.\n", leak_cnt, broken_cnt);
	} else {
		lldbg("*** NO MEMORY LEAK.\n");
	}
}

static void init_mem_leak_checker(int checker_pid, char *bin_name)
{
	struct tcb_s *checker_tcb = sched_gettcb(checker_pid);
	struct mm_heap_s *heap;

	/* stack information is for leak checker task. */
	leak_checker.stack_top = checker_tcb->adj_stack_ptr;
	leak_checker.stack_bottom = leak_checker.stack_top - checker_tcb->adj_stack_size;
#ifdef CONFIG_APP_BINARY_SEPARATION
	if (strncmp(bin_name, "kernel", strlen("kernel")) == 0) {
		heap = g_kmmheap;
	} else {
		heap = mm_get_app_heap_with_name(bin_name);
	}
#else
	heap = umm_get_heap(leak_checker.stack_top);
#endif
	/* Set the target heap information */
	leak_checker.heap = heap;
	leak_checker.heap_start = heap->mm_heapstart[0];
	leak_checker.heap_end = heap->mm_heapend[0];
}

int run_mem_leak_checker(int checker_pid, char *bin_name)
{
	int leak_cnt = 0;
	int node_cnt = 0;
	int broken_cnt = 0;
	uint32_t bin_text_addr = (uint32_t)NULL;

	init_mem_leak_checker(checker_pid, bin_name);

	node_cnt = get_node_cnt();
	if (MAX_ALLOC_COUNT < node_cnt) {
		lldbg("Available buffer size (%d) is small.\nPlease increase CONFIG_MEM_LEAK_CHECKER_MAX_ALLOC_COUNT value more than %d.\n", MAX_ALLOC_COUNT, node_cnt);
		return ERROR;
	}
	hash_init();

	fill_hash_table(&leak_cnt, &broken_cnt);

	/* Visit RAM region */
	ram_check(bin_name, &leak_cnt, &bin_text_addr);

	print_info(bin_name, leak_cnt, broken_cnt, bin_text_addr);

	return OK;
}
