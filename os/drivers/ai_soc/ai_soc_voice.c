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

#include <tinyara/config.h>

#include <sys/types.h>
#include <sys/ioctl.h>

#include <stdint.h>
#include <stdio.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <errno.h>
#include <mqueue.h>
#include <debug.h>

#include <tinyara/kmalloc.h>
#include <tinyara/fs/fs.h>
#include <tinyara/fs/ioctl.h>
#include <tinyara/audio/audio.h>
#include <tinyara/ai_soc/ai_soc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_AUDIO_SPEECH_DETECT_FEATURES
#define AUDIO_NULL_KEYWORD_DETECT_THRESHOLD 10

enum speech_state_e {
	AUDIO_NULL_SPEECH_STATE_NONE = 0,
	AUDIO_NULL_SPEECH_STATE_IDLE = 1,
#ifdef CONFIG_AUDIO_KEYWORD_DETECT
	AUDIO_NULL_SPEECH_STATE_KD = 2,
#endif
};

typedef enum speech_state_e speech_state_t;
#endif

struct ai_soc_voice_dev_s {
	struct audio_lowerhalf_s dev;	/* Audio lower half (this device) */
	struct ai_soc_dev_s *aisoc;
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
	volatile bool terminate;	/* True: request to terminate audio operation */
#endif
#ifdef CONFIG_AUDIO_SPEECH_DETECT_FEATURES
	volatile speech_state_t speech_state;
#ifdef CONFIG_AUDIO_KEYWORD_DETECT
	volatile bool keyword_detect;
#endif
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int ai_soc_voice_getcaps(FAR struct audio_lowerhalf_s *dev, int type, FAR struct audio_caps_s *caps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_configure(FAR struct audio_lowerhalf_s *dev, FAR void *session, FAR const struct audio_caps_s *caps);
#else
static int ai_soc_voice_configure(FAR struct audio_lowerhalf_s *dev, FAR const struct audio_caps_s *caps);
#endif
static int ai_soc_voice_shutdown(FAR struct audio_lowerhalf_s *dev);
static void *ai_soc_voice_workerthread(pthread_addr_t pvarg);
static void *ai_soc_voice_processthread(pthread_addr_t pvarg);

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_start(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int ai_soc_voice_start(FAR struct audio_lowerhalf_s *dev);
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int ai_soc_voice_stop(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session);
static int ai_soc_voice_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int ai_soc_voice_pause(FAR struct audio_lowerhalf_s *dev);
static int ai_soc_voice_resume(FAR struct audio_lowerhalf_s *dev);
#endif
#endif
static int ai_soc_voice_enqueuebuffer(FAR struct audio_lowerhalf_s *dev, FAR struct ap_buffer_s *apb);
static int ai_soc_voice_cancelbuffer(FAR struct audio_lowerhalf_s *dev, FAR struct ap_buffer_s *apb);
static int ai_soc_voice_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd, unsigned long arg);
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session);
#else
static int ai_soc_voice_reserve(FAR struct audio_lowerhalf_s *dev);
#endif
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_release(FAR struct audio_lowerhalf_s *dev, FAR void *session);
#else
static int ai_soc_voice_release(FAR struct audio_lowerhalf_s *dev);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct audio_ops_s g_audioops = {
	ai_soc_voice_getcaps,				/* getcaps        */
	ai_soc_voice_configure,				/* configure      */
	ai_soc_voice_shutdown,				/* shutdown       */
	ai_soc_voice_start,					/* start          */
#ifndef CONFIG_AUDIO_EXCLUDE_STOP
	ai_soc_voice_stop,					/* stop           */
#endif
#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
	ai_soc_voice_pause,					/* pause          */
	ai_soc_voice_resume,				/* resume         */
#endif
	NULL,						/* allocbuffer    */
	NULL,						/* freebuffer     */
	ai_soc_voice_enqueuebuffer,			/* enqueue_buffer */
	ai_soc_voice_cancelbuffer,			/* cancel_buffer  */
	ai_soc_voice_ioctl,					/* ioctl          */
	NULL,						/* read           */
	NULL,						/* write          */
	ai_soc_voice_reserve,				/* reserve        */
	ai_soc_voice_release				/* release        */
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: ai_soc_voice_getcaps
 *
 * Description: Get the audio device capabilities
 *
 ****************************************************************************/

static int ai_soc_voice_getcaps(FAR struct audio_lowerhalf_s *dev, int type, FAR struct audio_caps_s *caps)
{
	FAR struct ai_soc_voice_dev_s *priv = (FAR struct ai_soc_voice_dev_s *)dev;

	/* get caps from the actual aisoc device */
	return priv->aisoc->ops->getcaps(priv->aisoc, type, caps);
}

/****************************************************************************
 * Name: ai_soc_voice_configure
 *
 * Description:
 *   Configure the audio device for the specified  mode of operation.
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_configure(FAR struct audio_lowerhalf_s *dev, FAR void *session, FAR const struct audio_caps_s *caps)
#else
static int ai_soc_voice_configure(FAR struct audio_lowerhalf_s *dev, FAR const struct audio_caps_s *caps)
#endif
{
	FAR struct ai_soc_voice_dev_s *priv = (FAR struct ai_soc_voice_dev_s *)dev;

        /* pass on the function to configure, need to see if session changes are requiered here */
	return priv->aisoc->ops->configure(priv->aisoc, caps);
}

/****************************************************************************
 * Name: ai_soc_voice_shutdown
 *
 * Description:
 *   Shutdown the driver and put it in the lowest power state possible.
 *
 ****************************************************************************/

static int ai_soc_voice_shutdown(FAR struct audio_lowerhalf_s *dev)
{
	audvdbg("Return OK\n");
	return OK;
}

/****************************************************************************
 * Name: ai_soc_voice_processthread
 *
 *  This is the thread that feeds data to the chip and keeps the audio
 *  stream going.
 *
 ****************************************************************************/
#ifdef CONFIG_AUDIO_ENDPOINT_DETECT
#define AUDIO_MSG_EPD              10
#endif

#ifdef CONFIG_AUDIO_KEYWORD_DETECT
#define AUDIO_MSG_KD               11
#endif

/* Note that this function should be called when ever we get a match for keyword detect
 * either by interrupt or by polling (incase of polling, aisoc file should start a process) 
 */
static void ai_soc_voice_kd_callback()
{
#ifdef CONFIG_AUDIO_KEYWORD_DETECT
		struct audio_msg_s msg;
		/* keyword detect will be provided by interrupt, this callback needs to be sent by  */
		if ((priv->keyword_detect) && (priv->speech_state == AUDIO_NULL_SPEECH_STATE_IDLE)) {
			audvdbg("Keyword Detected!!\n");
			priv->speech_state = AUDIO_NULL_SPEECH_STATE_KD;
			msg.msgId = AUDIO_MSG_KD;
			msg.u.pPtr = NULL;
			mq_send(priv->dev.process_mq, (FAR const char *)&msg, sizeof(msg), CONFIG_AUDIO_NULL_MSG_PRIO);
		}
#endif
}

/****************************************************************************
 * Name: ai_soc_voice_start
 *
 * Description:
 *   Start the configured operation (audio streaming, volume enabled, etc.).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_start(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ai_soc_voice_start(FAR struct audio_lowerhalf_s *dev)
#endif
{
	FAR struct ai_soc_voice_dev_s *priv = (FAR struct ai_soc_voice_dev_s *)dev;

	audvdbg("Entry\n");
	audvdbg("Return %d\n", ret);

	return OK;
}

/****************************************************************************
 * Name: ai_soc_voice_stop
 *
 * Description: Stop the configured operation (audio streaming, volume
 *              disabled, etc.).
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_STOP
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_stop(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ai_soc_voice_stop(FAR struct audio_lowerhalf_s *dev)
#endif
{
	audvdbg("Return OK\n");
	return OK;
}
#endif

/****************************************************************************
 * Name: ai_soc_voice_pause
 *
 * Description: Pauses the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_pause(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ai_soc_voice_pause(FAR struct audio_lowerhalf_s *dev)
#endif
{
	audvdbg("Return OK\n");
	return OK;
}
#endif							/* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: ai_soc_voice_resume
 *
 * Description: Resumes the playback.
 *
 ****************************************************************************/

#ifndef CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME
#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_resume(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ai_soc_voice_resume(FAR struct audio_lowerhalf_s *dev)
#endif
{
	audvdbg("Return OK\n");
	return OK;
}
#endif							/* CONFIG_AUDIO_EXCLUDE_PAUSE_RESUME */

/****************************************************************************
 * Name: ai_soc_voice_enqueuebuffer
 *
 * Description: Enqueue an Audio Pipeline Buffer for playback/ processing.
 *
 ****************************************************************************/

static int ai_soc_voice_enqueuebuffer(FAR struct audio_lowerhalf_s *dev, FAR struct ap_buffer_s *apb)
{
	FAR struct ai_soc_voice_dev_s *priv = (FAR struct ai_soc_voice_dev_s *)dev;
	bool done;

	DEBUGASSERT(priv && apb && priv->dev.upper);

	audvdbg("apb=%p curbyte=%d nbytes=%d nmaxbytes %d\n", apb, apb->curbyte, apb->nbytes, apb->nmaxbytes);

	/* callback needs to be called by the lower layer, like dev->upper().... whcih inturn calls this layers upper callback */
	priv->aisoc->ops->enqueuebuffer(priv->aisoc, apb);
	
	return OK;
}

/****************************************************************************
 * Name: ai_soc_voice_cancelbuffer
 *
 * Description: Called when an enqueued buffer is being cancelled.
 *
 ****************************************************************************/

static int ai_soc_voice_cancelbuffer(FAR struct audio_lowerhalf_s *dev, FAR struct ap_buffer_s *apb)
{
	audvdbg("apb=%p curbyte=%d nbytes=%d, return OK\n", apb, apb->curbyte, apb->nbytes);

	return OK;
}

/****************************************************************************
 * Name: ai_soc_voice_ioctl
 *
 * Description: Perform a device ioctl
 *
 ****************************************************************************/

static int ai_soc_voice_ioctl(FAR struct audio_lowerhalf_s *dev, int cmd, unsigned long arg)
{
	FAR struct ai_soc_voice_dev_s *priv = (FAR struct ai_soc_voice_dev_s *)dev;
	int ret = OK;

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
	FAR struct ap_buffer_info_s *bufinfo;
#endif

	auddbg("cmd=%d arg=%ld\n");

	/* Deal with ioctls passed from the upper-half driver */

	switch (cmd) {
		/* Check for AUDIOIOC_HWRESET ioctl.  This ioctl is passed straight
		 * through from the upper-half audio driver.
		 */
	case AUDIOIOC_HWRESET: {
		audvdbg("AUDIOIOC_HWRESET:\n");
	}
	break;
	/* Report our preferred buffer size and quantity */

#ifdef CONFIG_AUDIO_DRIVER_SPECIFIC_BUFFERS
	case AUDIOIOC_GETBUFFERINFO: {
		audvdbg("AUDIOIOC_GETBUFFERINFO:\n");
		bufinfo = (FAR struct ap_buffer_info_s *)arg;
		bufinfo->buffer_size = CONFIG_AUDIO_NULL_BUFFER_SIZE;
		bufinfo->nbuffers = CONFIG_AUDIO_NULL_NUM_BUFFERS;
	}
	break;
#endif
	case AUDIOIOC_REGISTERPROCESS: {
#ifdef CONFIG_AUDIO_PROCESSING_FEATURES
#ifdef CONFIG_AUDIO_SPEECH_DETECT_FEATURES
		priv->speech_state = AUDIO_NULL_SPEECH_STATE_NONE;
#endif
		ret = ai_soc_voice_registerprocess(dev, (mqd_t) arg);
		if (ret != OK) {
			auddbg("Process Start Failed ret : %d\n", ret);
			return ret;
		}
		ret = OK;
#else
		auddbg("Register Process Failed - Device Doesn't support\n");
		ret = -EINVAL;
#endif
	}
	break;
	case AUDIOIOC_UNREGISTERPROCESS: {
#ifdef CONFIG_AUDIO_PROCESSING_FEATURES
		ret = ai_soc_voice_unregisterprocess(dev);
		if (ret != OK) {
			auddbg("Process Start Failed ret : %d\n", ret);
			return ret;
		}
		ret = OK;

#else
		auddbg("UnRegister Process Failed - Device Doesn't support\n");
		ret = -EINVAL;
#endif
	}
	break;

	case AUDIOIOC_STARTPROCESS: {
		auddbg("set start process!!\n");
#ifdef CONFIG_AUDIO_PROCESSING_FEATURES
		priv->process_terminate = false;
		priv->speech_state = AUDIO_NULL_SPEECH_STATE_IDLE;
		sem_post(&priv->processing_sem);
		ret = OK;
#else
		auddbg("start Process Failed - Device Doesn't support\n");
		ret = -EINVAL;
#endif
	}
	break;

	case AUDIOIOC_STOPPROCESS: {
		auddbg("set stop process!!\n");
#ifdef CONFIG_AUDIO_PROCESSING_FEATURES
		priv->speech_state = AUDIO_NULL_SPEECH_STATE_NONE;
		priv->process_terminate = true;
		ret = OK;
#else
		auddbg("start Process Failed - Device Doesn't support\n");
		ret = -EINVAL;
#endif
	}
	break;

	default:
		break;
	}

	audvdbg("Return OK\n");
	return ret;
}

/****************************************************************************
 * Name: ai_soc_voice_reserve
 *
 * Description: Reserves a session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_reserve(FAR struct audio_lowerhalf_s *dev, FAR void **session)
#else
static int ai_soc_voice_reserve(FAR struct audio_lowerhalf_s *dev)
#endif
{
	audvdbg("Return OK\n");
	return OK;
}

/****************************************************************************
 * Name: ai_soc_voice_release
 *
 * Description: Releases the session (the only one we have).
 *
 ****************************************************************************/

#ifdef CONFIG_AUDIO_MULTI_SESSION
static int ai_soc_voice_release(FAR struct audio_lowerhalf_s *dev, FAR void *session)
#else
static int ai_soc_voice_release(FAR struct audio_lowerhalf_s *dev)
#endif
{
	return OK;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: audio_ai_soc_voice_initialize
 *
 * Description:
 *   Initialize the null audio device.
 *
 * Input Parameters:
 *   i2c     - An I2C driver instance
 *   i2s     - An I2S driver instance
 *   lower   - Persistent board configuration data
 *
 * Returned Value:
 *   A new lower half audio interface for the NULL audio device is returned
 *   on success; NULL is returned on failure.
 *
 ****************************************************************************/

FAR struct audio_lowerhalf_s *audio_ai_soc_voice_initialize(void)
{
	FAR struct ai_soc_voice_dev_s *priv;

	/* Allocate the null audio device structure */

	priv = (FAR struct ai_soc_voice_dev_s *)kmm_zalloc(sizeof(struct ai_soc_voice_dev_s));
	if (priv) {
		/* Initialize the null audio device structure.  Since we used kmm_zalloc,
		 * only the non-zero elements of the structure need to be initialized.
		 */

		priv->dev.ops = &g_audioops;
#ifdef CONFIG_AUDIO_PROCESSING_FEATURES
		priv->process_terminate = false;
#ifdef CONFIG_AUDIO_KEYWORD_DETECT
		priv->keyword_detect = false;
#endif
#endif
		return &priv->dev;
	}

	auddbg("ERROR: Failed to allocate null audio device\n");
	return NULL;
}
