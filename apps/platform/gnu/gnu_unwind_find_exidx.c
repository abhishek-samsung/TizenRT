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
 * Included Files
 ****************************************************************************/

#include <stdbool.h>
#include <elf32.h>
#include <tinyara/elf.h>

#include <unwind.h>
#include <tinyara/userspace.h>

/****************************************************************************
 * Private Data
 ****************************************************************************/

void *__exidx_start_ptr;
void *__exidx_end_ptr;

extern void * __exidx_start;
extern void * __exidx_end;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  __gnu_Unwind_Find_exidx
 *
 * Description:
 *    This function is called (if exists) by the gcc generated unwind
 *    run-time in order to retrieve an alternative .ARM.exidx Exception
 *    index section.
 *    This is the case for an ELF module loaded by the elf binary loader.
 *    It is needed to support exception handling for loadable ELF modules.
 *
 ****************************************************************************/

void up_init_exidx(void * exidx_start, void * exidx_end) {
	__exidx_start_ptr = exidx_start;
	__exidx_end_ptr = exidx_end;
}

_Unwind_Ptr __gnu_Unwind_Find_exidx(_Unwind_Ptr return_address, int *nrecp)
{
      *nrecp = &__exidx_end - &__exidx_start;
      return (_Unwind_Ptr)&__exidx_start;
}
