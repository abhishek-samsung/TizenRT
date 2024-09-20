#include <tinyara/mtd/nand_raw.h>
#include <tinyara/mtd/nand_scheme.h>
#include <debug.h>
#if 0

struct nand_raw_s
{
  /* NAND data description */

  struct nand_model_s model; /* The NAND model storage */
  uintptr_t cmdaddr;         /* NAND command address base */
  uintptr_t addraddr;        /* NAND address address base */
  uintptr_t dataaddr;        /* NAND data address */
  uint8_t ecctype;           /* See NANDECC_* definitions */

  /* NAND operations */

  CODE int (*eraseblock)(FAR struct nand_raw_s *raw, off_t block);
  CODE int (*rawread)(FAR struct nand_raw_s *raw, off_t block,
                      unsigned int page, FAR void *data, FAR void *spare);
  CODE int (*rawwrite)(FAR struct nand_raw_s *raw, off_t block,
                       unsigned int page, FAR const void *data,
                       FAR const void *spare);

#ifdef CONFIG_MTD_NAND_HWECC
  CODE int (*readpage)(FAR struct nand_raw_s *raw, off_t block,
                       unsigned int page, FAR void *data, FAR void *spare);
  CODE int (*writepage)(FAR struct nand_raw_s *raw, off_t block,
                        unsigned int page, FAR const void *data,
                        FAR const void *spare);
#endif

#if defined(CONFIG_MTD_NAND_SWECC) || defined(CONFIG_MTD_NAND_HWECC)
  /* ECC working buffers */

  uint8_t spare[CONFIG_MTD_NAND_MAXPAGESPARESIZE];
  uint8_t ecc[CONFIG_MTD_NAND_MAXSPAREECCBYTES];
#endif
};

struct nand_model_s
{
  uint8_t  devid;         /* Identifier for the device */
  uint8_t  options;       /* Special options for the NandFlash */
  uint16_t pagesize;      /* Size of the data area of a page in bytes */
  uint16_t sparesize;     /* Size of the spare area of a page in bytes */
  uint16_t devsize;       /* Size of the device in MB */
  uint16_t blocksize;     /* Size of one block in kilobytes */

  /* Spare area placement scheme */

  FAR const struct nand_scheme_s *scheme;
};

#endif

// Lets disable HWECC also for now, we only need to implement three apis
// Note that we need SPI to be provided to nand_raw to talk to the flash


int xt26g02d_eraseblock(FAR struct nand_raw_s *raw, off_t block) {
	lldbg("erase block called %u\n", block);
	return OK;
	// erase block before writing
        // write enable
        
	SPI_SETMODE(raw->spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(raw->spi, 1000000);
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
        cmd_data[1] = 0x00;
        cmd_data[2] = 0x00;
        cmd_data[3] = 0x00;

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
                lldbg("Feature : %02x, value : %02x\n", cmd_data[1], dev_id[0]);

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
	lldbg("page read called %u block, %u page\n", block, page);
	if (spare) *(uint8_t *)spare = 0xff;
	return 0;
	uint8_t address24[3];
	for (int i = 0; i < 3; i++) address24[i] = 0;
	SPI_SETMODE(raw->spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(raw->spi, 1000000);
        SPI_SETBITS(raw->spi, 8);

        // get device ID to verify the flash
        uint8_t cmd_data[6];
        uint8_t dev_id[2];

	// read page to cache
        cmd_data[0] = 0x13;
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
                lldbg("Feature : %02x, value : %02x\n", cmd_data[1], dev_id[0]);

                if (dev_id[0] & 1) {
                        usleep(1000);
                        continue;
                } else {
                        break;
                }
        }

        // read page from cache
        cmd_data[0] = 0x03;
        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 1);
        SPI_SNDBLOCK(raw->spi, address24, 3);
	SPI_RECVBLOCK(raw->spi, data, 2048);
	SPI_RECVBLOCK(raw->spi, spare, 128);
        SPI_SELECT(raw->spi, 0, false);
	return 1;
}

int xt26g02d_rawwrite(FAR struct nand_raw_s *raw, off_t block,
                       unsigned int page, FAR const void *data,
                       FAR const void *spare) {
	lldbg("page write called %u block, %u page\n", block, page);
	return 0;

	SPI_SETMODE(raw->spi, SPIDEV_MODE0);
        SPI_SETFREQUENCY(raw->spi, 1000000);
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
        cmd_data[1] = 0x00;
        cmd_data[2] = 0x00;

        SPI_SELECT(raw->spi, 0, true);
        SPI_SNDBLOCK(raw->spi, cmd_data, 3);
        SPI_SNDBLOCK(raw->spi, data, 2048);
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
                lldbg("Feature : %02x, value : %02x\n", cmd_data[1], dev_id[0]);

                if (dev_id[0] & 1) {
                        usleep(1000);
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
