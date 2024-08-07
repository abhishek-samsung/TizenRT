/****************************************************************************
 *
 * Copyright 2018 Samsung Electronics All Rights Reserved.
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
 * libc/stdio/lib_vasprintf.c
 *
 *   Copyright (C) 2011-2012 Gregory Nutt. All rights reserved.
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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <assert.h>
#include <tinyara/mm/mm.h>

#include "lib_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* On some architectures, va_list is really a pointer to a structure on the
 * stack.  And the va_arg builtin will modify that instance of va_list.  Since
 * vasprintf traverse the parameters in the va_list twice, the va_list will
 * be altered in this first cases and the second usage will fail.  So far, I
 * have seen this only on the X86 family with GCC.
 */

#undef CLONE_APLIST
#define ap1 ap
#define ap2 ap

#if defined(CONFIG_ARCH_X86)
#define CLONE_APLIST 1
#undef ap2
#elif defined(CONFIG_ARCH_SIM) && (defined(CONFIG_HOST_X86) || defined(CONFIG_HOST_X86_64))
#define CLONE_APLIST 1
#undef ap2
#endif

/****************************************************************************
 * Private Type Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Global Constant Data
 ****************************************************************************/

/****************************************************************************
 * Global Variables
 ****************************************************************************/

/****************************************************************************
 * Private Constant Data
 ****************************************************************************/

/****************************************************************************
 * Private Variables
 ****************************************************************************/

/****************************************************************************
 * Global Functions
 ****************************************************************************/

/****************************************************************************
 * Name: vasprintf
 *
 * Description:
 *   This function is similar to vsprintf, except that it dynamically
 *   allocates a string (as with malloc) to hold the output, instead of
 *   putting the output in a buffer you allocate in advance.  The ptr
 *   argument should be the address of a char * object, and a successful
 *   call to vasprintf stores a pointer to the newly allocated string at that
 *   location.
 *
 * Returned Value:
 *   The returned value is the number of characters allocated for the buffer,
 *   or less than zero if an error occurred. Usually this means that the buffer
 *   could not be allocated.
 *
 ****************************************************************************/

int vasprintf(FAR char **ptr, const char *fmt, va_list ap)
{
	struct lib_outstream_s nulloutstream;
	struct lib_memoutstream_s memoutstream;
#ifdef CLONE_APLIST
	va_list ap2;
#endif
	FAR char *buf;
	int nbytes;

	DEBUGASSERT(ptr && fmt);

#ifdef CLONE_APLIST
	/* Clone the va_list so that the contents of the input values are not altered */

	va_copy(ap2, ap);
#endif

	/* First, use a nullstream to get the size of the buffer.  The number
	 * of bytes returned may or may not include the null terminator.
	 */

	lib_nulloutstream(&nulloutstream);
	lib_vsprintf((FAR struct lib_outstream_s *)&nulloutstream, fmt, ap1);

	/* Then allocate a buffer to hold that number of characters, adding one
	 * for the null terminator.
	 */

	buf = (FAR char *)lib_malloc(nulloutstream.nput + 1);
	if (!buf) {
		return ERROR;
	}
	DEBUG_SET_CALLER_ADDR(buf);

	/* Initialize a memory stream to write into the allocated buffer.  The
	 * memory stream will reserve one byte at the end of the buffer for the
	 * null terminator and will not report this in the number of output bytes.
	 */

	lib_memoutstream((FAR struct lib_memoutstream_s *)&memoutstream, buf, nulloutstream.nput + 1);

	/* Then let lib_vsprintf do it's real thing */

	nbytes = lib_vsprintf((FAR struct lib_outstream_s *)&memoutstream.public, fmt, ap2);

	/* Return a pointer to the string to the caller.  NOTE: the memstream put()
	 * method has already added the NUL terminator to the end of the string (not
	 * included in the nput count).
	 *
	 * Hmmm.. looks like the memory would be stranded if lib_vsprintf() returned
	 * an error.  Does that ever happen?
	 */

	DEBUGASSERT(nbytes < 0 || nbytes == nulloutstream.nput);
	*ptr = buf;
	return nbytes;
}
