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

struct ai_soc_ops_s {
	/* basic required ops */
	CODE int (*getcaps)(FAR struct ai_soc_dev_s *dev, int type, FAR struct audio_caps_s *pCaps);
#ifdef CONFIG_AUDIO_MULTI_SESSION
        CODE int (*configure)(FAR struct ai_soc_dev_s *dev, FAR void *session, FAR const struct audio_caps_s *pCaps);
#else
        CODE int (*configure)(FAR struct ai_soc_dev_s *dev, FAR const struct audio_caps_s *pCaps);
#endif
        CODE int (*enqueuebuffer)(FAR struct ai_soc_dev_s *dev, FAR struct ap_buffer_s *apb);
        CODE int (*ioctl)(FAR struct ai_soc_dev_s *dev, int cmd, unsigned long arg);
};

struct ai_soc_dev_s {
        struct ai_soc_ops_s *ops;
	audio_callback_t upper; /* upper layer callback, same structure can be used but differnt callback func */
	FAR void * priv; /* pointer to provide for upper layer callback */
	/* any device specific data */
};
