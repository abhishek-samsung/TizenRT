/****************************************************************************
 *
 * Copyright 2022 Samsung Electronics All Rights Reserved.
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
#include <sys/types.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <unistd.h>
#include <semaphore.h>
#include <queue.h>
#include <errno.h>
#include <fixedmath.h>
#include <debug.h>
#include <tinyara/kmalloc.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/audio/i2s.h>
#include <tinyara/audio/audio.h>
#include <tinyara/audio/ub6470.h>
#include <tinyara/math.h>
#include <math.h>
#include <tinyara/i2c.h>

#include "ub6470.h"
#include "ub6470scripts.h"

#define UB6470_I2S_TIMEOUT_MS 200

/* Default configuration values */
#ifndef CONFIG_UB6470_BUFFER_SIZE
#define CONFIG_UB6470_BUFFER_SIZE       4096
#endif

#ifndef CONFIG_UB6470_NUM_BUFFERS
#define CONFIG_UB6470_NUM_BUFFERS       4
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/
static UB6470_REG_DATA_TYPE g_ub6470_i2c_mode = UB6470_REG_D_1BYTE;

static const struct audio_ops_s g_audioops = {
        .getcaps = ub6470_getcaps,             /* getcaps        */
        .configure = ub6470_configure,         /* configure      */
        .shutdown = ub6470_shutdown,           /* shutdown       */
        .start = ub6470_start,                 /* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
        .stop = ub6470_stop,                   /* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
        .pause = ub6470_pause,                 /* pause          */
        .resume = ub6470_resume,               /* resume         */
#endif
        .allocbuffer = NULL,                    /* allocbuffer    */
        .freebuffer = NULL,                     /* freebuffer     */
        .enqueuebuffer = ub6470_enqueuebuffer, /* enqueue_buffer */
        .cancelbuffer = ub6470_cancelbuffer,   /* cancel_buffer  */
        .ioctl = ub6470_ioctl,                 /* ioctl          */
        .read = NULL,                           /* read           */
        .read = NULL,                           /* write          */
        .reserve = ub6470_reserve,             /* reserve        */
        .release = ub6470_release,             /* release        */
};

/****************************************************************************
 * Name: delay
 *
 * Description
 *    Delay in ms.
 ****************************************************************************/
static void delay(unsigned int mS)
{
        //usleep(mS * 1000);
	int ans;
	for (int i = 0; i < 100000000; i++) {
                        ans = (ans + i)%5;
                        if (i %50000000 == 0) printf("waiting === delay\n", ans);
        }
	ans += 1;
}

/****************************************************************************
 * Name: ub6470_readreg
 *
 * Description
 *    Read the specified 8-bit register from the UB6470 device.
 *
 *	enable i2c readwrite in config for this.
 *
 ****************************************************************************/
uint8_t ub6470_readreg_1byte(FAR struct ub6470_dev_s *priv, uint8_t regaddr)
{
	return 0;
        uint8_t reg[4];
        FAR struct i2c_dev_s *dev = priv->i2c;
        FAR struct i2c_config_s *ub6470_i2c_config = &(priv->lower->i2c_config);
        uint8_t reg_w[4];
        reg_w[0] = regaddr;
	//int ret = i2c_writeread(dev, ub6470_i2c_config, reg_w, 1, reg, 1);
        int ret = i2c_write(dev, ub6470_i2c_config, reg_w, 1);
	ret =  i2c_read(dev, ub6470_i2c_config, reg, 1);
	if (ret < 0) {
                printf("Error, cannot read reg %x\n", regaddr);
		PANIC();
                return FAIL_8;
        }
	//else printf("read to 0x%x ret = %d\n", regaddr, ret);
        return reg[0];
}

/****************************************************************************
 * Name: ub6470_readreg
 *
 * Description
 *    Read the specified 32-bit register from the UB6470 device. (big endian)
 *
 ****************************************************************************/
uint32_t ub6470_readreg_4byte(FAR struct ub6470_dev_s *priv, uint8_t regaddr)
{
	return 0;
        int32_t ret;
        uint8_t reg[4];
        uint32_t regval;
        FAR struct i2c_dev_s *dev = priv->i2c;
        FAR struct i2c_config_s *ub6470_i2c_config = &(priv->lower->i2c_config);

        reg[0] = regaddr;

        ret = i2c_write(dev, ub6470_i2c_config, reg, 1);
        if (ret < 0) {
                auddbg("Error, cannot read reg %x\n", regaddr);
                return FAIL_32;
        }
        ret = i2c_read(dev, ub6470_i2c_config, reg, 3);
        if (ret < 0) {
                auddbg("Error, cannot read reg %x\n", regaddr);
                return FAIL_32;
        }
        regval = ((reg[0] << 24) | (reg[1] << 16) | (reg[2] << 8) | (reg[3]));
	printf("%02x\t%02x\t%02x\t%02x\n", regaddr, reg[0], reg[1], reg[2]);
        return ret;
}

/************************************************************************************
 * Name: ub6470_writereg_1byte
 *
 * Description:
 *   Write the specified 8-bit register to the UB6470 device.
 *
 ************************************************************************************/
static int ub6470_writereg_1byte(FAR struct ub6470_dev_s *priv, uint8_t regaddr, uint8_t regval)
{
	return 0;
        int ret;
        uint8_t reg[2];
        FAR struct i2c_dev_s *dev = priv->i2c;
        FAR struct i2c_config_s *ub6470_i2c_config = &(priv->lower->i2c_config);

        reg[0] = regaddr;
        reg[1] = regval;

        ret = i2c_write(dev, ub6470_i2c_config, (uint8_t *)reg, 2);
        if (ret != 2) {
                auddbg("Error, cannot write reg %x\n", regaddr);
        }
	//else printf("written to 0x%x val 0x%x ret = %d\n", regaddr, regval, ret);
	uint8_t regval2 = ub6470_readreg_1byte(priv, regaddr);       
	printf("0x%02x\t0x%02x\n", regaddr, regval2);
	return ret;
}

/************************************************************************************
 * Name: ub6470_writereg_4byte
 *
 * Description:
 *   Write the specified 32-bit register to the UB6470 device.
 *
 ************************************************************************************/
static int ub6470_writereg_4byte(FAR struct ub6470_dev_s *priv, uint8_t regaddr, uint8_t regval[4])
{
	return 0;
        int ret;
        uint8_t reg[5];
        FAR struct i2c_dev_s *dev = priv->i2c;
        FAR struct i2c_config_s *ub6470_i2c_config = &(priv->lower->i2c_config);

        reg[0] = regaddr;
        reg[1] = regval[1];
        reg[2] = regval[2];
        reg[3] = regval[3];
        //reg[4] = regval[3];

        ret = i2c_write(dev, ub6470_i2c_config, (uint8_t *)reg, 4);
        if (ret != 4) {
                auddbg("Error, cannot write reg %x\n", regaddr);
        }
	ret = ub6470_readreg_4byte(priv, regaddr);
        return ret;
}

/************************************************************************************
 * Name: ub6470_exec_i2c_script
 *
 * Description:
 *   Executes given script through i2c to configuure UB6470 device.
 *
 ************************************************************************************/
static int ub6470_exec_i2c_script(FAR struct ub6470_dev_s *priv, t_codec_init_script_entry *script, uint32_t size)
{
	return 0;
        uint32_t i;
        uint16_t ret = 0;
        UB6470_REG_DATA_TYPE reg_acc;

        for (i = 0; i < size; i++) {
                reg_acc = (UB6470_REG_DATA_TYPE)script[i].type;
                if (reg_acc == UB6470_REG_D_1BYTE) {
                        ret = ub6470_writereg_1byte(priv, (uint8_t)script[i].addr, script[i].val[0]);
                } else if (reg_acc == UB6470_REG_D_4BYTE) {
                        ret = ub6470_writereg_4byte(priv, (uint8_t)script[i].addr, script[i].val);
                } 
		
		/*
		if ((uint8_t)script[i].addr == 0x01) {
                        delay(script[i].delay); // wait as we are setting pll registers
			uint8_t regval2 = ub6470_readreg_1byte(priv, 0x01);
			printf("val of 0x01 : 0x%02x\n", regval2);
                }
		*/
		
        }
        return ret;
}

/************************************************************************************
 * Name: ub6470_takesem
 *
 * Description:
 *  Take a semaphore count, handling the nasty EINTR return if we are interrupted
 *  by a signal.
 *
 ************************************************************************************/
static void ub6470_takesem(sem_t *sem)
{
        int ret;

        do {
                ret = sem_wait(sem);
                DEBUGASSERT(ret == 0 || errno == EINTR);
        } while (ret < 0);
}

/************************************************************************************
 * Name: ub6470_setvolume
 *
 * Description:
 *   Set the right and left volume values in the UB6470 device based on the current
 *   volume and balance settings.
 *
 ************************************************************************************/
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
static void ub6470_setvolume(FAR struct ub6470_dev_s *priv)
{
	printf("set volume is called\n");
        /* if no audio device object return */
        if (!priv) {
                auddbg("Error, Device's private data Not available\n");
                return;
        }

        if (priv->running) {
                audvdbg("volume=%u mute=%u\n", priv->volume, priv->mute);
                if (priv->mute) {
                        ub6470_exec_i2c_script(priv, codec_init_mute_on_script, sizeof(codec_init_mute_on_script) / sizeof(t_codec_init_script_entry));
                } else {
                        codec_set_master_volume_script[1].val[0] = (uint8_t)priv->volume;
                        int ret = ub6470_exec_i2c_script(priv, codec_set_master_volume_script, sizeof(codec_set_master_volume_script) / sizeof(t_codec_init_script_entry));
			printf("set volume output : %d\n", ret);
                }
        } else {
                audvdbg("not running[volume=%u mute=%u]\n", priv->volume, priv->mute);
        }
}
#endif                                                  /* CONFIG_AUDIO_EXCLUDE_VOLUME */

/************************************************************************************
 * Name: ub6470_setbass
 *
 * Description:
 *   Set the bass level.
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ************************************************************************************/
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void ub6470_setbass(FAR struct ub6470_dev_s *priv, uint8_t bass)
{
        audvdbg("bass=%u\n", bass);
}
#endif                                                  /* CONFIG_AUDIO_EXCLUDE_TONE */

/************************************************************************************
 * Name: ub6470_settreble
 *
 * Description:
 *   Set the treble level .
 *
 *   The level and range are in whole percentage levels (0-100).
 *
 ************************************************************************************/
#ifndef CONFIG_AUDIO_EXCLUDE_TONE
static void ub6470_settreble(FAR struct ub6470_dev_s *priv, uint8_t treble)
{
        audvdbg("treble=%u\n", treble);
}
#endif

/****************************************************************************
 * Name: ub6470_set_i2s_datawidth
 *
 * Description:
 *   Set the 8- 16- 24- bit data modes
 *
 ****************************************************************************/
static void ub6470_set_i2s_datawidth(FAR struct ub6470_dev_s *priv)
{
        /* if no audio device object return */
        if (!priv) {
                auddbg("Error, Device's private data Not available\n");
                return;
        }

        if (priv->inout) {
                I2S_RXDATAWIDTH(priv->i2s, priv->bpsamp);
        } else {
                I2S_TXDATAWIDTH(priv->i2s, priv->bpsamp);
        }
}

/****************************************************************************
 * Name: ub6470_set_i2s_samplerate
 *
 * Description:
 *
 ****************************************************************************/
static void ub6470_set_i2s_samplerate(FAR struct ub6470_dev_s *priv)
{
        /* if no audio device object return */
        if (!priv) {
                auddbg("Error, Device's private data Not available\n");
                return;
        }

        if (priv->inout) {
                I2S_RXSAMPLERATE(priv->i2s, priv->samprate);
        } else {
                I2S_TXSAMPLERATE(priv->i2s, priv->samprate);
        }
}

/****************************************************************************
 * Name: ub6470_getcaps
 *
 * Description:
 *   Get the audio device capabilities
 *
 ****************************************************************************/
static int ub6470_getcaps(FAR struct audio_lowerhalf_s *dev, int type, FAR struct audio_caps_s *caps)
{
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) || !defined(CONFIG_AUDIO_EXCLUDE_TONE)
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;
#endif
	/* Validate the structure */

	DEBUGASSERT(caps && caps->ac_len >= sizeof(struct audio_caps_s));
	audvdbg("type=%d ac_type=%d\n", type, caps->ac_type);

	/* Fill in the caller's structure based on requested info */

	caps->ac_controls.w = 0;

	switch (caps->ac_type) {
	/* Caller is querying for the types of units we support */

	case AUDIO_TYPE_QUERY:

		/* Provide our overall capabilities.  The interfacing software
		 * must then call us back for specific info for each capability.
		 */

		caps->ac_channels = UB6470_CHANNELS;	/* Stereo output */

		switch (caps->ac_subtype) {
		case AUDIO_TYPE_QUERY:
			/* We don't decode any formats!  Only something above us in
			 * the audio stream can perform decoding on our behalf.
			 */

			/* The types of audio units we implement */
			caps->ac_controls.b[0] = AUDIO_TYPE_OUTPUT | AUDIO_TYPE_FEATURE;

			break;

		default:
			caps->ac_controls.b[0] = AUDIO_SUBFMT_END;
			break;
		}

		break;

	/* Provide capabilities of our INPUT & OUTPUT unit */
	case AUDIO_TYPE_OUTPUT:
		caps->ac_channels = UB6470_CHANNELS;
		switch (caps->ac_subtype) {
		case AUDIO_TYPE_QUERY:
			/* Report the Sample rates we support */
			caps->ac_controls.b[0] = AUDIO_SAMP_RATE_TYPE_48K;
			break;

		case AUDIO_FMT_MP3:
		case AUDIO_FMT_WMA:
		case AUDIO_FMT_PCM:
			break;

		default:
			break;
		}

		break;

	/* Provide capabilities of our FEATURE units */

	case AUDIO_TYPE_FEATURE:

		/* If the sub-type is UNDEF, then report the Feature Units we support */

		if (caps->ac_subtype == AUDIO_FU_UNDEF) {
			/* Fill in the ac_controls section with the Feature Units we have */

			caps->ac_controls.b[0] = AUDIO_FU_VOLUME | AUDIO_FU_BASS | AUDIO_FU_TREBLE;
			caps->ac_controls.b[1] = AUDIO_FU_BALANCE >> 8;
		} else {
			/* TODO:  Do we need to provide specific info for the Feature Units,
			 * such as volume setting ranges, etc.?
			 */
		}

		switch (caps->ac_format.hw) {
		case AUDIO_FU_VOLUME:
			caps->ac_controls.hw[0] = UB6470_SPK_VOL_MAX;
			caps->ac_controls.hw[1] = priv->volume;
			break;
		default:
			break;
		}

		break;

	/* All others we don't support */

	default:

		/* Zero out the fields to indicate no support */

		caps->ac_subtype = 0;
		caps->ac_channels = 0;

		break;
	}

	/* Return the length of the audio_caps_s struct for validation of
	 * proper Audio device type.
	 */

	return caps->ac_len;
}

/****************************************************************************
 * Name: ub6470_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_configure(FAR struct audio_lowerhalf_s *dev, FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int ub6470_configure(FAR struct audio_lowerhalf_s *dev, FAR const struct audio_caps_s *caps)
#endif
{
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) || !defined(CONFIG_AUDIO_EXCLUDE_TONE)
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;
#endif
	int ret = OK;

	DEBUGASSERT(priv && caps);
	audvdbg("ac_type: %d\n", caps->ac_type);


	/* UB6470 supports on the fly changes for almost all changes
	   so no need to do anything. But if any issue, worth looking here */

	switch (caps->ac_type) {
	case AUDIO_TYPE_FEATURE:
		audvdbg("  AUDIO_TYPE_FEATURE\n");

		/* Inner swich case: Process based on Feature Unit */
		switch (caps->ac_format.hw) {
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
		case AUDIO_FU_VOLUME: {
			/* Set the volume */
			uint16_t volume = caps->ac_controls.hw[0];
			audvdbg("    Volume: %d\n", volume);
			if (volume < UB6470_SPK_VOL_MIN) {
				priv->volume = UB6470_SPK_VOL_MIN;
			} else if (volume >= UB6470_SPK_VOL_MAX) {
				priv->volume = UB6470_SPK_VOL_MAX;
			} else {
				priv->volume = volume;
			}
			printf("set volume is called\n");
			ub6470_setvolume(priv);
		}
		break;
		case AUDIO_FU_MUTE: {
			/* Mute or unmute:	true(1) or false(0) */
			bool mute = caps->ac_controls.b[0];
			audvdbg("mute: 0x%x\n", mute);
			priv->mute = mute;
			ub6470_setvolume(priv);
		}
		break;
#endif							/* CONFIG_AUDIO_EXCLUDE_VOLUME */

#ifndef CONFIG_AUDIO_EXCLUDE_TONE
		case AUDIO_FU_BASS: {
			/* Set the bass.  The percentage level (0-100) is in the
			 * ac_controls.b[0] parameter. */

			uint8_t bass = caps->ac_controls.b[0];
			audvdbg("    Bass: %d\n", bass);
			if (bass <= UB6470_BASS_MAX) {
				ub6470_setbass(priv, bass);
			}
		}
		break;
		case AUDIO_FU_TREBLE: {
			/* Set the treble.  The percentage level (0-100) is in the
			 * ac_controls.b[0] parameter. */

			uint8_t treble = caps->ac_controls.b[0];
			audvdbg("    Treble: %d\n", treble);
			if (treble <= UB6470_TREBLE_MAX) {
				ub6470_settreble(priv, treble);
			}
		}
		break;
#endif							/* CONFIG_AUDIO_EXCLUDE_TONE */
		default:
			auddbg("    ERROR: Unrecognized feature unit\n");
			break;
		}
		break;					/* Break for inner switch case */
	case AUDIO_TYPE_OUTPUT:
		audvdbg("  AUDIO_TYPE :%s\n", caps->ac_type == AUDIO_TYPE_INPUT ? "INPUT" : "OUTPUT");
		/* Verify that all of the requested values are supported */

		ret = -EDOM;
		if (caps->ac_channels != UB6470_CHANNELS) {
			auddbg("ERROR: Unsupported number of channels: %d\n", caps->ac_channels);
			break;
		}

		if (caps->ac_controls.b[2] != UB6470_BPSAMP) {
			auddbg("ERROR: Unsupported bits per sample: %d\n", caps->ac_controls.b[2]);
			break;
		}

		/* Save the current stream configuration */

		priv->samprate = caps->ac_controls.hw[0];
		priv->nchannels = caps->ac_channels;
		priv->bpsamp = caps->ac_controls.b[2];

		audvdbg("    Number of channels: 0x%x\n", priv->nchannels);
		audvdbg("    Sample rate:        0x%x\n", priv->samprate);
		audvdbg("    Sample width:       0x%x\n", priv->bpsamp);

		/* Reconfigure the FLL to support the resulting number or channels,
		 * bits per sample, and bitrate.
		 */
		ret = OK;
		priv->inout = false;
		break;

	case AUDIO_TYPE_PROCESSING:
		break;

	default:
		break;
	}
	return ret;
}

/****************************************************************************
 * Name: ub6470_shutdown
 *
 * Description:
 *   Shutdown the UB6470 chip and put it in the lowest power state possible.
 *
 ****************************************************************************/
static int ub6470_shutdown(FAR struct audio_lowerhalf_s *dev)
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;
	struct ub6470_lower_s *lower = priv->lower;

	DEBUGASSERT(priv);

	if (!priv) {
		return -EINVAL;
	}

	/* First disable interrupts */
	ub6470_takesem(&priv->devsem);
	sq_entry_t *tmp = NULL;
	for (tmp = (sq_entry_t *)sq_peek(&priv->pendq); tmp; tmp = sq_next(tmp)) {
		sq_rem(tmp, &priv->pendq);
		audvdbg("(tasshutdown)removing tmp with addr 0x%x\n", tmp);
	}
	sq_init(&priv->pendq);

	if (priv->running) {
		I2S_STOP(priv->i2s, I2S_TX);
		ub6470_exec_i2c_script(priv, codec_stop_script, sizeof(codec_stop_script) / sizeof(t_codec_init_script_entry));
		priv->running = false;
	}
	priv->paused = false;

	ub6470_givesem(&priv->devsem);

	/* Now issue a software reset.  This puts all UB6470 registers back in
	 * their default state.
	 */

	if (lower->control_powerdown) {
		lower->control_powerdown(1); //need hardware support here, func in board dir
	}
	return OK;
}

/****************************************************************************
 * Name: ub6470_io_err_cb
 *
 * Description:
 *   Callback function for io error
 *
 ****************************************************************************/
static void ub6470_io_err_cb(FAR struct i2s_dev_s *dev, FAR void *arg, int flags)
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)arg;

	/* Call upper callback, let it post msg to user q
	 * apb is set NULL, okay? Rethink
	 */
	if (priv && priv->dev.upper) {
		priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_IOERR, NULL, flags);
	}

}

/****************************************************************************
 * Name: ub6470_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ub6470_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
	lldbg("start called\n");
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;

	if (!priv) {
		return -EINVAL;
	}

	audvdbg(" ub6470_start Entry\n");
	ub6470_takesem(&priv->devsem);
	if (priv->running) {
		goto ub_start_withsem;
	}

	/* Register cb for io error */
	I2S_ERR_CB_REG(priv->i2s, ub6470_io_err_cb, priv);

	/* Finally set ub6470 to be running */
	priv->running = true;
	priv->mute = false;
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
	ub6470_setvolume(priv);
#endif

	sq_entry_t *tmp = NULL;
	sq_queue_t *q = &priv->pendq;
	for (tmp = sq_peek(q); tmp; tmp = sq_next(tmp)) {
		ub6470_enqueuebuffer(dev, (struct ap_buffer_s *)tmp);
	}

	/* Exit reduced power modes of operation */
	/* REVISIT */

ub_start_withsem:

	ub6470_givesem(&priv->devsem);
	return OK;
}

/****************************************************************************
 * Name: ub6470_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ub6470_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;

	if (!priv) {
		return -EINVAL;
	}
	audvdbg(" ub6470_stop Entry\n");
	ub6470_takesem(&priv->devsem);
	I2S_STOP(priv->i2s, I2S_TX);
	/* Need to run the stop script here */
	ub6470_exec_i2c_script(priv, codec_stop_script, sizeof(codec_stop_script) / sizeof(t_codec_init_script_entry));

	priv->running = false;
	priv->mute = true;
	ub6470_givesem(&priv->devsem);

	/* Enter into a reduced power usage mode */
	/* REVISIT: */

	return OK;
}
#endif

/****************************************************************************
 * Name: ub6470_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ub6470_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;

	if (!priv) {
		return -EINVAL;
	}
	audvdbg(" ub6470_pause Entry\n");
	ub6470_takesem(&priv->devsem);

	if (priv->running && !priv->paused) {
		/* Disable interrupts to prevent us from suppling any more data */

		priv->paused = true;
		priv->mute = true;
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
		ub6470_setvolume(priv);
#endif
		I2S_PAUSE(priv->i2s, I2S_TX);
	}

	ub6470_givesem(&priv->devsem);

	return OK;
}
#endif							/* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: ub6470_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ub6470_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;

	if (!priv) {
		return -EINVAL;
	}

	audvdbg(" ub6470_resume Entry\n");
	ub6470_takesem(&priv->devsem);

	if (priv->running && priv->paused) {
		priv->paused = false;
		priv->mute = false;
#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
		ub6470_setvolume(priv);
#endif

		I2S_RESUME(priv->i2s, I2S_TX);

	}

	ub6470_givesem(&priv->devsem);
	return OK;
}
#endif							/* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: ub6470_rxcallback
 *
 * Description: Called when I2S filled a buffer. No recycling mechanism now.
 *
 ****************************************************************************/

static void ub6470_txcallback(FAR struct i2s_dev_s *dev, FAR struct ap_buffer_s *apb, FAR void *arg, int result)
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)arg;

	DEBUGASSERT(priv && apb);

	ub6470_takesem(&priv->devsem);
	sq_entry_t *tmp;
	for (tmp = (sq_entry_t *)sq_peek(&priv->pendq); tmp; tmp = sq_next(tmp)) {
		if (tmp == (sq_entry_t *)apb) {
			sq_rem(tmp, &priv->pendq);
			audvdbg("found the apb to remove 0x%x\n", tmp);
			break;
		}
	}

	/* Call upper callback, let it post msg to user q */
	priv->dev.upper(priv->dev.priv, AUDIO_CALLBACK_DEQUEUE, apb, OK);

	ub6470_givesem(&priv->devsem);
}

/****************************************************************************
 * Name: ub6470_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/
static int ub6470_enqueuebuffer(FAR struct audio_lowerhalf_s *dev, FAR struct ap_buffer_s *apb)
{
	//printf("In enqueue buffer\n");
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;
	int ret;

	if (!priv || !apb) {
		return -EINVAL;
	}

	if (!priv->running) {
		printf("device not runnning\n");
		/* Add the new buffer to the tail of pending audio buffers */
		ub6470_takesem(&priv->devsem);
		sq_addlast((sq_entry_t *)&apb->dq_entry, &priv->pendq);
		printf("enqueue added buf 0x%x\n", apb);
		ub6470_givesem(&priv->devsem);
		return OK;
	}
	//printf("I2S send is called\n");
	priv->volume = UB6470_SPK_VOL_MAX; 
	/* debugging prupose to check if volume is not on mute*/
        //printf("set volume to max\n");
        //ub6470_setvolume(priv);
	/*
	uint8_t regval2 = ub6470_readreg_1byte(priv, 0x3C);
        printf("val of 0x3C : 0x%02x\n", regval2);
	regval2 = ub6470_readreg_1byte(priv, 0x3D);
        printf("val of 0x3D : 0x%02x\n", regval2);
	regval2 = ub6470_readreg_1byte(priv, 0x0F);
        printf("val of 0x0F : 0x%02x\n", regval2);
	regval2 = ub6470_readreg_1byte(priv, 0x2A);
        printf("val of 0x2A : 0x%02x\n", regval2);
	*/
	ret = I2S_SEND(priv->i2s, apb, ub6470_txcallback, priv, UB6470_I2S_TIMEOUT_MS);

	return ret;
}

/****************************************************************************
 * Name: ub6470_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/
static int ub6470_cancelbuffer(FAR struct audio_lowerhalf_s *dev, FAR struct ap_buffer_s *apb)
{
	audvdbg("apb=%p\n", apb);
	/* Need to add logic here */
	return OK;
}

/****************************************************************************
 * Name: ub6470_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/
static int ub6470_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd, unsigned long arg)
{
	FAR struct ap_buffer_info_s *bufinfo;
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;

	if (!priv) {
		return -EINVAL;
	}

	/* Deal with ioctls passed from the upper-half driver */

	switch (cmd) {

	case AUDIOIOC_PREPARE: {
		audvdbg("AUDIOIOC_PREPARE: ub6470 prepare\n");
		uint8_t regval;
		struct ub6470_lower_s *lower = priv->lower;

		/* Take semaphore */
		ub6470_takesem(&priv->devsem);

		/* Pause i2s channel */
		I2S_PAUSE(priv->i2s, I2S_TX);

		if (lower->control_powerdown) {
			lower->control_powerdown(0);
			//delay(10);  //spec is 100us
		}
		//ub6470_exec_i2c_script(priv, codec_initial_script, sizeof(codec_initial_script) / sizeof(t_codec_init_script_entry));
		
		/* Amp Error Check */ // add error reg here
/*		ub6470_writereg_1byte(priv, (uint8_t)0x0E, 0x00);
		regval = ub6470_readreg_1byte(priv, 0x0E);
		if (regval != 0x00) {
		 	auddbg("ERROR: Amp has some error! register : 0x%02x value : 0x%02x \n", 0x0E, regval);
		}
*/
		/* Resume I2S */
		I2S_RESUME(priv->i2s, I2S_TX);

		/* Give semaphore */
		ub6470_givesem(&priv->devsem);
	}
	break;
	case AUDIOIOC_HWRESET: {
		/* REVISIT:  Should we completely re-initialize the chip?   We
		 * can't just issue a software reset; that would puts all UB6470
		 * registers back in their default state.
		 */

		audvdbg("AUDIOIOC_HWRESET:\n");
		ub6470_hw_reset(priv);
	}
	break;

		/* Report our preferred buffer size and quantity */

	case AUDIOIOC_GETBUFFERINFO: {
		/* Report our preferred buffer size and quantity */
		audvdbg("AUDIOIOC_GETBUFFERINFO:\n");
		/* Take semaphore */
		ub6470_takesem(&priv->devsem);
		
		bufinfo = (FAR struct ap_buffer_info_s *)arg;
#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
		bufinfo->buffer_size = CONFIG_UB6470_BUFFER_SIZE;
		bufinfo->nbuffers = CONFIG_UB6470_NUM_BUFFERS;
#else
		bufinfo->buffer_size = CONFIG_UB6470_BUFFER_SIZE;
		bufinfo->nbuffers = CONFIG_UB6470_NUM_BUFFERS;
#endif
		audvdbg("buffer_size : %d nbuffers : %d\n", bufinfo->buffer_size, bufinfo->nbuffers);

		/* Give semaphore */
		ub6470_givesem(&priv->devsem);
	}
	break;


	default:
		audvdbg("ub6470_ioctl received unkown cmd 0x%x\n", cmd);
		break;
	}

	return OK;
}

/****************************************************************************
 * Name: ub6470_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int ub6470_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;
	int ret = OK;

	if (!priv) {
		return -EINVAL;
	}

	/* Borrow the APBQ semaphore for thread sync */

	ub6470_takesem(&priv->devsem);
	if (priv->reserved) {
		ret = -EBUSY;
	} else {
		/* Initialize the session context */

#ifdef CONFIG_AUDIO_MULTI_SESSION
		*session = NULL;
#endif
		priv->inflight = 0;
		priv->running = false;
		priv->paused = false;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
		priv->terminating = false;
#endif
		priv->reserved = true;
	}

	ub6470_givesem(&priv->devsem);

	return ret;
}

/****************************************************************************
 * Name: ub6470_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ub6470_release(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ub6470_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
	FAR struct ub6470_dev_s *priv = (FAR struct ub6470_dev_s *)dev;

	if (!priv) {
		return -EINVAL;
	}
	ub6470_takesem(&priv->devsem);
	if (priv->running) {
		I2S_STOP(priv->i2s, I2S_TX);
		ub6470_exec_i2c_script(priv, codec_stop_script, sizeof(codec_stop_script) / sizeof(t_codec_init_script_entry));
		priv->running = false;
	}
	priv->reserved = false;
	ub6470_givesem(&priv->devsem);

	return OK;
}

/* Reset and reconfigure the UB6470 hardwaqre */
/****************************************************************************
 * Name: ub6470_reconfigure
 *
 * Description:
 *   re-initialize the UB6470
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void ub6470_reconfigure(FAR struct ub6470_dev_s *priv)
{
	priv->inout = false;
	/* Put audio output back to its initial configuration */
	priv->samprate = UB6470_DEFAULT_SAMPRATE;
	priv->nchannels = UB6470_DEFAULT_NCHANNELS;
	priv->bpsamp = UB6470_DEFAULT_BPSAMP;
#if !defined(CONFIG_AUDIO_EXCLUDE_VOLUME) && !defined(CONFIG_AUDIO_EXCLUDE_BALANCE)
	priv->balance = b16HALF;	/* Center balance */
#endif

	/* Software reset.	This puts all UB6470 registers back in their
	 * default state.
	 */
	return;
	int res = ub6470_exec_i2c_script(priv, codec_initial_script, sizeof(codec_initial_script) / sizeof(t_codec_init_script_entry));
	uint8_t regval2;
       	while (true) {
		regval2 = ub6470_readreg_1byte(priv, 0x01);
		if (regval2 == 0x02) {
			printf("value of 0x01 is set to 0x02\n");
			for (int i = 0; i < 1000000; i++) {
				regval2++;
			}
			break;
		}
		printf("value of 0x01 is set to :%d\n", regval2);
		for (int i = 0; i < 1000000; i++) {
			regval2++;
		}
	}
	res = ub6470_exec_i2c_script(priv, codec_initial_script2, sizeof(codec_initial_script2) / sizeof(t_codec_init_script_entry));
	printf("reconfigure output : %d\n", res);
//	ub6470_set_i2s_samplerate(priv);
//	ub6470_set_i2s_datawidth(priv);
}

/****************************************************************************
 * Name: ub6470_hw_reset
 *
 * Description:
 *   Reset and re-initialize the UB6470
 *
 * Input Parameters:
 *   priv - A reference to the driver state structure
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/
static void ub6470_hw_reset(FAR struct ub6470_dev_s *priv)
{
	if (!priv) {
		return;
	}

	struct ub6470_lower_s *lower = priv->lower;

	if (lower->control_hw_reset) {
		lower->control_hw_reset(true);
		delay(100);
		lower->control_hw_reset(false);
		delay(100);
	}
	ub6470_reconfigure(priv);
	//ub6470_exec_i2c_script(priv, codec_init_mute_on_script, sizeof(codec_init_mute_on_script) / sizeof(t_codec_init_script_entry));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ub6470_initialize
 *
 * Description:
 *   Initialize the UB6470 device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the UB6470 device is returned on
 *   success; NULL is returned on failure.
 *
 ****************************************************************************/
FAR struct audio_lowerhalf_s *ub6470_initialize(FAR struct i2c_dev_s *i2c, FAR struct i2s_dev_s *i2s, FAR struct ub6470_lower_s *lower)
{

	FAR struct ub6470_dev_s *priv;
	uint8_t regval;
	/* Sanity check */
	DEBUGASSERT(i2c && lower && i2s);

	auddbg("I2s dev addr is 0x%x\n", i2s);

	/* Allocate a UB6470 device structure */
	priv = (FAR struct ub6470_dev_s *)kmm_zalloc(sizeof(struct ub6470_dev_s));

	if (!priv) {
		return NULL;
	}

	/* Initialize the UB6470 device structure.  Since we used kmm_zalloc,
	 * only the non-zero elements of the structure need to be initialized.
	 */

	priv->dev.ops = &g_audioops;
	priv->lower = lower;
	priv->i2c = i2c;
	priv->i2s = i2s;

	sem_init(&priv->devsem, 0, 1);
	sq_init(&priv->pendq);

	/* Software reset.  This puts all UB6470 registers back in their
	 * default state.
	 */

	/*reconfigure the UB6470 hardwaqre */
	ub6470_reconfigure(priv);
	//ub6470_exec_i2c_script(priv, codec_init_mute_on_script, sizeof(codec_init_mute_on_script) / sizeof(t_codec_init_script_entry));

#ifndef CONFIG_AUDIO_EXCLUDE_VOLUME
	priv->volume = UB6470_SPK_VOL_DEF;
	ub6470_setvolume(priv);
#endif

	return &priv->dev;

errout_with_dev:
	sem_destroy(&priv->devsem);
	kmm_free(priv);
	return NULL;
}

