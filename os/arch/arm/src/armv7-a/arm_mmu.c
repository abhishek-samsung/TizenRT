/****************************************************************************
 *
 * Copyright 2023 Samsung Electronics All Rights Reserved.
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
 * arch/arm/src/armv7-a/arm_mmu.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <stdint.h>

#include "cp15_cacheops.h"
#include "mmu.h"

#ifdef CONFIG_APP_BINARY_SEPARATION
#include <tinyara/mm/mm.h>
#endif
/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void mmu_set_flags(uint32_t *val, bool ro, bool exec, uint8_t isL1, uint8_t isGlobal)
{
	if (isL1) {
		if (ro && exec) {
			*val |= MMU_APP_L1_ROX;
		} else if (ro) {
			*val |= MMU_APP_L1_RO;
		} else {
			*val |= MMU_APP_L1_RW;
		}

		if (!isGlobal) {
			*val |= PMD_SECT_NG;
		}
	} else {
		if (ro && exec) {
			*val |= MMU_APP_L2_ROX;
		} else if (ro) {
			*val |= MMU_APP_L2_RO;
		} else {
			*val |= MMU_APP_L2_RW;
		}

		if (!isGlobal) {
			*val |= PTE_NG;
		}
	}
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mmu_l1_setentry
 *
 * Description:
 *   Set a one level 1 translation table entry.  Only a single L1 page table
 *   is supported.
 *
 * Input Parameters:
 *   paddr - The physical address to be mapped.  Must be aligned to a 1MB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 1MB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_setentry(uint32_t paddr, uint32_t vaddr, uint32_t mmuflags)
{
  uint32_t *l1table = mmu_l1_pgtable();
  uint32_t  index   = vaddr >> 20;

  /* Save the page table entry */

  l1table[index] = (paddr | mmuflags);

  /* Flush the data cache entry.  Make sure that the modified contents
   * of the page table are flushed into physical memory.
   */

  cp15_clean_dcache_bymva((uint32_t)&l1table[index]);

  /* Invalidate the TLB cache associated with virtual address range */

  mmu_invalidate_region(vaddr, SECTION_SIZE);
}
#endif

/****************************************************************************
 * Name: mmu_l1_restore
 *
 * Description:
 *   Restore one L1 table entry previously returned by mmu_l1_getentry() (or
 *   any other encoded L1 page table value).
 *
 * Input Parameters:
 *   vaddr - A virtual address to be mapped
 *   l1entry - The value to write into the page table entry
 *
 ****************************************************************************/

#if !defined(CONFIG_ARCH_ROMPGTABLE) && defined(CONFIG_ARCH_ADDRENV)
void mmu_l1_restore(uintptr_t vaddr, uint32_t l1entry)
{
  uint32_t *l1table = mmu_l1_pgtable();
  uint32_t  index   = vaddr >> 20;

  /* Set the encoded page table entry */

  l1table[index] = l1entry;

  /* Flush the data cache entry.  Make sure that the modified contents
   * of the page table are flushed into physical memory.
   */

  cp15_clean_dcache_bymva((uint32_t)&l1table[index]);

  /* Invalidate the TLB cache associated with virtual address range */

  mmu_invalidate_region(vaddr & PMD_PTE_PADDR_MASK, SECTION_SIZE);
}
#endif

/****************************************************************************
 * Name: mmu_l2_setentry
 *
 * Description:
 *   Set one small (4096B) entry in a level2 translation table.
 *
 * Input Parameters:
 *   l2vaddr - the virtual address of the beginning of the L2 translation
 *     table.
 *   paddr - The physical address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   vaddr - The virtual address to be mapped.  Must be aligned to a 4KB
 *     address boundary
 *   mmuflags - The MMU flags to use in the mapping.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l2_setentry(uint32_t l2vaddr, uint32_t paddr, uint32_t vaddr,
                     uint32_t mmuflags)
{
  uint32_t *l2table  = (uint32_t *)l2vaddr;
  uint32_t  index;

  /* The table divides a 1Mb address space up into 256 entries, each
   * corresponding to 4Kb of address space.  The page table index is
   * related to the offset from the beginning of 1Mb region.
   */

  index = (vaddr & 0x000ff000) >> 12;

  /* Save the table entry */

  l2table[index] = (paddr | mmuflags);

  /* Flush the data cache entry.  Make sure that the modified contents
   * of the page table are flushed into physical memory.
   */

  cp15_clean_dcache_bymva((uint32_t)&l2table[index]);

  /* Invalidate the TLB cache associated with virtual address range */

  cp15_invalidate_tlb_bymva(vaddr);
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_region
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a
 *   region of memory.
 *
 * Input Parameters:
 *   mapping - Describes the mapping to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_region(const struct section_mapping_s *mapping)
{
  uint32_t paddr    = mapping->physbase;
  uint32_t vaddr    = mapping->virtbase;
  uint32_t mmuflags = mapping->mmuflags;
  int i;

  /* Loop, writing each mapping into the L1 page table */

  for (i = 0; i < mapping->nsections; i++)
    {
      mmu_l1_setentry(paddr, vaddr, mmuflags);
      paddr += SECTION_SIZE;
      vaddr += SECTION_SIZE;
    }
}
#endif

/****************************************************************************
 * Name: mmu_l1_map_regions
 *
 * Description:
 *   Set multiple level 1 translation table entries in order to map a region
 *   array of memory.
 *
 * Input Parameters:
 *   mappings - Describes the array of mappings to be performed.
 *   count    - The number of mappings to be performed.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_l1_map_regions(const struct section_mapping_s *mappings,
                        size_t count)
{
  size_t i;

  for (i = 0; i < count; i++)
    {
      mmu_l1_map_region(&mappings[i]);
    }
}
#endif

/****************************************************************************
 * Name: mmu_invalidate_region
 *
 * Description:
 *   Invalidate TLBs for a range of addresses (all 4KB aligned).
 *
 * Input Parameters:
 *   vaddr - The beginning of the region to invalidate.
 *   size  - The size of the region in bytes to be invalidated.
 *
 ****************************************************************************/

#ifndef CONFIG_ARCH_ROMPGTABLE
void mmu_invalidate_region(uint32_t vstart, size_t size)
{
  uint32_t vaddr = vstart & 0xfffff000;
  uint32_t vend  = vstart + size;

  /* Loop, invalidating regions */

  while (vaddr < vend)
    {
      cp15_invalidate_tlb_bymva(vaddr);
      vaddr += 4096;
    }
}
#endif

#ifdef CONFIG_APP_BINARY_SEPARATION
/****************************************************************************
 * Name: mmu_get_os_l1_pgtbl
 *
 * Description:
 *   Returns the virtual address of the kernel L1 page table.
 *
 * Input Parameters:
 *
 * Returned Value:
 * Page table address
 ****************************************************************************/
uint32_t *mmu_get_os_l1_pgtbl(void)
{
	return (uint32_t *)PGTABLE_BASE_VADDR;
}

/****************************************************************************
 * Name: mmu_allocate_app_l1_pgtbl
 *
 * Description:
 *   Allocate space for L1 page table of application, in accordance with
 *   the requirements of the arch specific mmu.
 *
 * Input Parameters:
 *
 * Returned Value:
 * L1 Page table address
 ****************************************************************************/
uint32_t *mmu_allocate_app_l1_pgtbl(int app_id)
{
	uint32_t *addr = (uint32_t *)(PGTABLE_BASE_VADDR + (app_id * 16384));
	// uint32_t *addr = (uint32_t *)kmm_memalign(L1_PGTBL_ALIGNMENT, L1_PGTBL_SIZE);
	// ASSERT(addr);
	// memset(addr, 0, L1_PGTBL_SIZE);
	return addr;
}

/****************************************************************************
 * Name: mmu_allocate_app_l2_pgtbl
 *
 * Description:
 *   Allocate space for L2 page table of application, in accordance with
 *   the requirements of the arch specific mmu.
 *
 * Input Parameters:
 *
 * Returned Value:
 * L2 Page table address
 ****************************************************************************/
uint32_t *mmu_allocate_app_l2_pgtbl(int app_id, int l2_idx)
{
	app_id--;
	uint32_t *addr = (uint32_t *)(PGTABLE_BASE_VADDR + (2 * 16384) + (app_id * l2_idx * 1024));
	// uint32_t *addr = (uint32_t *)kmm_memalign(L2_PGTBL_ALIGNMENT, L2_PGTBL_SIZE);
	// ASSERT(addr);
	// memset(addr, 0, L2_PGTBL_SIZE);
	return addr;
}

/****************************************************************************
 * Name: mmu_update_app_l1_pgtbl_ospgtbl
 *
 * Description:
 * Loop through the L1 page table.
 * Copy kernel L1 page table to app page table.
 * If the entry is pointing to a L2 page table
 * Allocate L2 page table for app.
 * Copy entries from kernel to app L2 table.
 * Update the L2 page table address in L1 table.
 * 
 * Input Parameters:
 * app_pgtbl: Pointer to L1 page table of app
 *
 ****************************************************************************/
void mmu_update_app_l1_pgtbl_ospgtbl(uint32_t *app_l1_pgtbl)
{
	uint32_t *os_l1_pgtbl = (uint32_t *)PGTABLE_BASE_VADDR;

	memcpy((void *)app_l1_pgtbl, (void *)os_l1_pgtbl, L1_PGTBL_SIZE);
	cp15_flush_dcache((uintptr_t)app_l1_pgtbl, (uintptr_t)app_l1_pgtbl + L1_PGTBL_SIZE);

#ifdef CONFIG_SUPPORT_COMMON_BINARY
	for (int i = 0; i < L1_PGTBL_NENTRIES; i++) {
		if ((os_l1_pgtbl[i] & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
			//Found a L2 page table.
			uint32_t *os_l2_pgtbl = (uint32_t *)(os_l1_pgtbl[i] & PMD_PTE_PADDR_MASK);
			uint32_t *app_l2_pgtbl = mmu_allocate_app_l2_pgtbl();
			memcpy(app_l2_pgtbl, os_l2_pgtbl, L2_PGTBL_SIZE);
			app_l1_pgtbl[i] &= ~PMD_PTE_PADDR_MASK;
			app_l1_pgtbl[i] |= (uint32_t)app_l2_pgtbl & PMD_PTE_PADDR_MASK;
		}	
	}
#endif
}

/****************************************************************************
 * Name: mmu_map_app_region
 *
 * Description
 *
 * Input Parameters:
 *
 * Returned Value:
 ****************************************************************************/
void mmu_map_app_region(int app_id, uint32_t *l1_pgtbl, uint32_t start, uint32_t size, bool ro, bool exec, bool global)
{
	uint32_t idx;
	uint32_t val;
	uint32_t end = start + size;
	irqstate_t flags;

	lldbg("start = 0x%08x end = 0x%08x size = %x\n", start, end, size);

	// Run a loop until the entire region is mapped.
	while (start < end) {
		// Check if this address can be mapped to a section.
		if (!(start & SECTION_MASK) && !(size & SECTION_MASK)) {
			// Yes. Update the section entry in the the L1 page table.
			idx = start >> 20;
			val = start & PMD_PTE_PADDR_MASK;
			mmu_set_flags(&val, ro, exec, true, global);

			lldbg("Add section for addr 0x%08x idx = %d\n", start, idx);

			if (global) {
  				// If this update is for the common binary, then it is done
				// in the kernel page tables and so the cache and tlbs need
				// to be flushed and invalidated.
				flags = enter_critical_section();
				l1_pgtbl[idx] = val;
				cp15_clean_dcache_bymva((uint32_t)&l1_pgtbl[idx]);
  				mmu_invalidate_region(start, SECTION_SIZE);
				leave_critical_section(flags);
			} else {
				l1_pgtbl[idx] = val;
				cp15_clean_dcache_bymva((uint32_t)&l1_pgtbl[idx]);
			}

			// Advance the memory region address.
			start += SECTION_SIZE;
		} else {	// Check if this address can be mapped to a small page.

			// Check if L2 page table is not created.
			idx = (start & 0xfff00000) >> 20;
			int l2_idx = 0;
			uint32_t *l2_pgtbl = (uint32_t *)(l1_pgtbl[idx] & PMD_PTE_PADDR_MASK);
			if ((l1_pgtbl[idx] & PMD_TYPE_MASK) != PMD_TYPE_PTE) {
				// Yes. Allocate L2 page table for app.
				l2_pgtbl = mmu_allocate_app_l2_pgtbl(app_id, l2_idx++);

				lldbg("Allocated L2 pgtbl at 0x%08x\n", l2_pgtbl);
				if (global) {
  					// If this update is for the common binary, then it is done
					// in the kernel page tables and so the cache and tlbs need
					// to be flushed and invalidated.

					flags = enter_critical_section();

					// Fill default entries into L2 page table.
					uint32_t tmp = start;
					for (idx = 0; idx < L2_PGTBL_NENTRIES; idx++) {
						val = tmp & PTE_SMALL_PADDR_MASK;
						val |= MMU_MEMFLAGS;
						l2_pgtbl[idx] = val;
						cp15_clean_dcache_bymva((uint32_t)&l2_pgtbl[idx]);
						tmp += 4096;
					}

					// Update L2 page table address in L1 page table. 
					val = (uint32_t)l2_pgtbl & PMD_PTE_PADDR_MASK;
					val |= MMU_L1_DATAFLAGS;
					l1_pgtbl[idx] = val;

					cp15_clean_dcache_bymva((uint32_t)&l1_pgtbl[idx]);
					cp15_invalidate_tlb_bymva(start);
					leave_critical_section(flags);
				} else {
					// Update L2 page table address in L1 page table.
					val = (uint32_t)l2_pgtbl & PMD_PTE_PADDR_MASK;
					val |= MMU_L1_PGTABFLAGS;
					l1_pgtbl[idx] = val;
					// dbg("Set l1 pte at 0x%08x = 0x%08x\n", &l1_pgtbl[idx], val);
					cp15_clean_dcache_bymva((uint32_t)&l1_pgtbl[idx]);
				}
			}

			// Update the L2 page table entry.
  			idx = (start & 0x000ff000) >> 12;
			val = start & PTE_SMALL_PADDR_MASK;
			mmu_set_flags(&val, ro, exec, false, global);

			if (global) {
				flags = enter_critical_section();
				l2_pgtbl[idx] = val;
  				// If this update is for the common binary, then it is done
				// in the kernel page tables and so the cache and tlbs need
				// to be flushed and invalidated.
				cp15_clean_dcache_bymva((uint32_t)&l2_pgtbl[idx]);
				cp15_invalidate_tlb_bymva(start);
				leave_critical_section(flags);
			} else {
				l2_pgtbl[idx] = val;
				// dbg("Set l2 pte at 0x%08x = 0x%08x\n", &l2_pgtbl[idx], val);
				cp15_clean_dcache_bymva((uint32_t)&l2_pgtbl[idx]);
			}

			// Advance the memory region address.
			start += SMALL_PAGE_SIZE;
		}
	}
}

#endif // CONFIG_APP_BINARY_SEPARATION


void mmu_dump_pgtbl(void)
{
	struct tcb_s *rtcb = sched_self();
	if (rtcb->app_id < 1) {
		return;
	}

	uint32_t *l1tbl = mmu_l1_pgtable();

	lldbg("L1 page table base addr = 0x%08x\n", l1tbl);

	lldbg("================================================\n");
	lldbg("ENTRY      TYPE    OUT           NG    AP     XN\n");
	lldbg("ADDR               ADDR                         \n");
	lldbg("================================================\n");
	for (int i = 0; i < L1_PGTBL_NENTRIES; i++) {
		bool ng = (l1tbl[i] & PMD_SECT_NG) ? 1 : 0;
		if (ng && (l1tbl[i] & PMD_TYPE_MASK) == PMD_TYPE_SECT) {
			lldbg("0x%08x SECT    0x%08x    %d    %1x%1x    %d\n", 
					&l1tbl[i], 
					l1tbl[i] & PMD_SECT_PADDR_MASK, 
					(l1tbl[i] & PMD_SECT_NG) ? 1 : 0,
					(l1tbl[i] & PMD_SECT_AP2) ? 1 : 0, 
					(l1tbl[i] & PMD_SECT_AP_MASK) >> PMD_SECT_AP_SHIFT,
					(l1tbl[i] & PMD_SECT_XN) ? 1 : 0);
		} else if((l1tbl[i] & PMD_TYPE_MASK) == PMD_TYPE_PTE) {
			lldbg("0x%08x L1PTE   0x%08x\n", &l1tbl[i], l1tbl[i] & PMD_PTE_PADDR_MASK); 
			uint32_t *l2tbl = (uint32_t)l1tbl[i] & PMD_PTE_PADDR_MASK;
			for (int j = 0; j < L2_PGTBL_NENTRIES; j++) {
				bool ng = (l2tbl[j] & PTE_NG) ? 1 : 0;
				if (ng && ((l2tbl[j] & PTE_TYPE_MASK) != PTE_TYPE_FAULT)) {
					lldbg("0x%08x PAGE    0x%08x    %d    %1x%1x    %d\n", 
						&l2tbl[j], 
						l2tbl[j] & PTE_SMALL_PADDR_MASK, 
						(l2tbl[j] & PTE_NG) ? 1 : 0,
						(l2tbl[j] & PTE_AP2) ? 1 : 0, 
						(l2tbl[j] & PTE_AP_MASK) >> PTE_AP_SHIFT,
						(l2tbl[j] & PTE_SMALL_XN) ? 1 : 0);
				}
			}
		}
	}
	lldbg("=============================================\n");
}
