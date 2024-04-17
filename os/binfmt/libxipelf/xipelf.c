/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <tinyara/config.h>

#include <sys/types.h>
#include <stdint.h>
#include <string.h>
#include <debug.h>
#include <errno.h>

#include <tinyara/arch.h>
#include <tinyara/binfmt/binfmt.h>
#include <tinyara/binary_manager.h>

#include <tinyara/userspace.h>

static int xipelf_loadbinary(FAR struct binary_s *binp);

static struct binfmt_s g_xipelfbinfmt = {
	NULL,				/* next */
	xipelf_loadbinary,		/* load */
	NULL,				/* unload */
};

static int xip_loadbinary(FAR struct binary_s *binp)
{
	/* simply setup the data, bss and heap */
	struct userspace_s uspace;

	if (binp->binary_idx == 0) {
		memcpy(&uspace, CONFIG_COMMON_BINARY_START_ADDR + sizeof(common_binary_header_t) + 4, sizeof(struct userspace_s));
		binp->sections[BIN_TEXT] = CONFIG_COMMON_BINARY_START_ADDR + sizeof(common_binary_header_t);
	} else if (binp->binary_idx == 1) {
		memcpy(&uspace, CONFIG_APP1_START_ADDR + sizeof(user_binary_header_t) + 4, sizeof(struct userspace_s));
		binp->sections[BIN_TEXT] = CONFIG_APP1_START_ADDR + sizeof(user_binary_header_t);
	} else if (binp->binary_idx == 2) {
		memcpy(&uspace, CONFIG_APP2_START_ADDR + sizeof(user_binary_header_t) + 4, sizeof(struct userspace_s));
		binp->sections[BIN_TEXT] = CONFIG_APP2_START_ADDR + sizeof(user_binary_header_t);
	} else {
		return ERROR;
	}

	/* Temporarily allocate the data and bss sections in RAM, will change the impl later to allocate and map them using MMU */

	/* zero out the bss... */
	for (uint8_t * bss = uspace.bss_start; bss < uspace.bss_end; bss++) *bss = 0x00;

	/* copy the data... */
	uint8_t * orig_data = uspace.data_start_in_flash;
	for (uint8_t * data = uspace.data_start_in_ram; data < uspace.data_end_in_ram; data++) {
		*data = *orig_data;
		orig_data++;
	}
	/* all the required setup is done, lets just populate them in binp structure */
	
	/* Allocate Heap... */
	binp->sections[BIN_HEAP] = uspace.heap_start;
	binp->sizes[BIN_HEAP] = 96*1024;

	binp->sections[BIN_DATA] = uspace.data_start_in_ram;

	binp->entrypt = uspace.entry;
	
	return OK;
}

int xipelf_initialize(void)
{
        int ret;

        /* Register ourselves as a binfmt loader */

        binfo("Registering ELF\n");

        ret = register_binfmt(&g_xipelfbinfmt);
        if (ret != 0) {
                berr("Failed to register binfmt: %d\n", ret);
        }

        return ret;
}

void xipelf_uninitialize(void)
{
        (void)unregister_binfmt(&g_xipbinfmt);
}
