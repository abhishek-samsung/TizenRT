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

#include <tinyara/userspace.h>

static int xip_loadbinary(FAR struct binary_s *binp);

static struct binfmt_s g_xipbinfmt = {
	NULL,						/* next */
	xip_loadbinary,				/* load */
	NULL,						/* unload */
};

static int xip_loadbinary(FAR struct binary_s *binp)
{
	/* simply setup the data, bss and heap */
	struct userspace_s uspace;
	memcpy(&uspace, 0x000000000824d030, sizeof(struct userspace_s));
	printf("bss start address : %x\n", uspace.bss_start);
	
	/* zero out the bss */
	for (uint8_t * bss = uspace.bss_start; bss < uspace.bss_end; bss++) *bss = 0x00;

	/* copy the data */
	uint8_t * orig_data = uspace.data_start_in_flash;
	for (uint8_t * data = uspace.data_start_in_ram; data < uspace.data_end_in_ram; data++) {
		*data = *orig_data;
		orig_data++;
	}
	/* all the required setup is done, lets just populate them in binp structure */
	/* we atlest need heap details...... */

	binp->sections[BIN_HEAP] = uspace.heap_start;
	binp->sizes[BIN_HEAP] = 512*1024;

	binp->sections[BIN_DATA] = uspace.data_start_in_ram;

	binp->entrypt = uspace.entry;

	binp->sections[BIN_TEXT] = 0x000000000824d030;

	return OK;

}

int xip_initialize(void)
{
        int ret;

        /* Register ourselves as a binfmt loader */

        binfo("Registering ELF\n");

        ret = register_binfmt(&g_xipbinfmt);
        if (ret != 0) {
                berr("Failed to register binfmt: %d\n", ret);
        }

        return ret;
}

void xip_uninitialize(void)
{
        (void)unregister_binfmt(&g_xipbinfmt);
}

