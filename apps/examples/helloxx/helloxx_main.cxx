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

//***************************************************************************
// Included Files
//***************************************************************************

#include <tinyara/config.h>

#include <cstdio>
#include <debug.h>

#include <tinyara/init.h>

#include <media/MediaPlayer.h>
#include <media/MediaPlayerObserverInterface.h>
#include <media/FileInputDataSource.h>
#include <media/FileOutputDataSource.h>
#include <media/MediaRecorder.h>
#include <media/MediaRecorderObserverInterface.h>
#include <media/BufferOutputDataSource.h>
#include <media/voice/SpeechDetector.h>

#include <iostream>
#include <memory>

using namespace std;
using namespace media;
using namespace media::stream;

sem_t tts_sem;
sem_t finished_sem;

media::MediaPlayer mp;
//media::MediaRecorder mr;

static const char *filePath = "";

class _Observer : public media::MediaPlayerObserverInterface, public std::enable_shared_from_this<_Observer> {
	void onPlaybackStarted(media::MediaPlayer &mediaPlayer) override {
		printf("##################################\n");
		printf("####    onPlaybackStarted     ####\n");
		printf("##################################\n");
	}
	void onPlaybackFinished(media::MediaPlayer &mediaPlayer) override {
		printf("##################################\n");
		printf("####    onPlaybackFinished    ####\n");
		printf("##################################\n");
		sem_post(&tts_sem);
	}
	void onPlaybackError(media::MediaPlayer &mediaPlayer, media::player_error_t error) override {
		printf("##################################\n");
		printf("####      onPlaybackError     ####\n");
		printf("##################################\n");
	}
	void onStartError(media::MediaPlayer &mediaPlayer, media::player_error_t error) override {

	}
	void onStopError(media::MediaPlayer &mediaPlayer, media::player_error_t error) override {

	}
	void onPauseError(media::MediaPlayer &mediaPlayer, media::player_error_t error) override {

	}
	void onPlaybackPaused(media::MediaPlayer &mediaPlayer) override {

	}
};
#if 0
class BufferReceiver : public media::MediaRecorderObserverInterface, public std::enable_shared_from_this<BufferReceiver>
{
	void onRecordStarted(media::MediaRecorder& mediaRecorder) override
	{
		printf("##################################\n");
		printf("####     onRecordStarted      ####\n");
		printf("##################################\n");
	}
	void onRecordPaused(media::MediaRecorder& mediaRecorder) override
	{
		printf("##################################\n");
		printf("####      onRecordPaused      ####\n");
		printf("##################################\n");
	}
	void onRecordFinished(media::MediaRecorder& mediaRecorder) override
	{
		printf("##################################\n");
                printf("####      onRecordFinished    ####\n");
                printf("##################################\n");
		sem_post(&finished_sem);
	}
	void onRecordStartError(media::MediaRecorder& mediaRecorder, media::recorder_error_t errCode) override
	{
		printf("#### onRecordStartError!! errCode : %d\n", errCode);
	}
	void onRecordPauseError(media::MediaRecorder& mediaRecorder, media::recorder_error_t errCode) override
	{
		printf("#### onRecordPauseError!! errCode : %d\n", errCode);
	}
	void onRecordStopError(media::MediaRecorder& mediaRecorder, media::recorder_error_t errCode) override
	{
		printf("#### onRecordStopError!! errCode : %d\n", errCode);
	}

	void onRecordBufferDataReached(media::MediaRecorder& mediaRecorder, std::shared_ptr<unsigned char> data, size_t size) override {
	
	}
};
#endif
static void take_sem(sem_t *sem)
{
	int ret;

	do {
		ret = sem_wait(sem);
		DEBUGASSERT(ret == 0 || errno == EINTR);
	} while (ret < 0);
}

extern "C"
{
	int helloxx_main(int argc, char *argv[])
	{
		int count;
		if (argc == 1) {
			printf("missing arg, usage : helloxx count");
			return 0;	
		} else count = atoi(argv[1]);

		while (count--) {
		sem_init(&tts_sem, 0, 0);
		sem_init(&finished_sem, 0, 0);
#if 0
		media::recorder_result_t mret = mr.create();
		if (mret == media::RECORDER_OK) {
			printf("#### [MR] create succeeded.\n");
		} else {
			printf("#### [MR] create failed.\n");
			return 0;
		}

		filePath = "/tmp/record.pcm";
                mret = mr.setDataSource(unique_ptr<FileOutputDataSource>(
                                                        new FileOutputDataSource(2, 16000, AUDIO_FORMAT_TYPE_S16_LE, filePath)));

		if (mret == media::RECORDER_OK) {
			printf("#### [MR] setDataSource succeeded.\n");
		} else {
			printf("#### [MR] setDataSource failed.\n");
			return 0;
		}

		mret = mr.setObserver(std::make_shared<BufferReceiver>());
		if (mret == media::RECORDER_OK) {
			printf("#### [MR] setObserver succeeded.\n");
		} else {
			printf("#### [MR] setObserver failed.\n");
			return 0;
		}

		if (mr.setDuration(7) == RECORDER_ERROR_NONE && mr.prepare() == RECORDER_ERROR_NONE) {
                	printf("#### [MR] prepare succeeded.\n");
		} else {
			printf("#### [MR] prepare failed.\n");
			return 0;
		}

		printf("###################################\n");
		printf("#### Wait for wakeup triggered ####\n");
		printf("###################################\n");

		auto sd = media::voice::SpeechDetector::instance();

		if (!sd->initKeywordDetect(16000, 1)) {
			printf("#### [SD] init failed.\n");
			return 0;
		}

		sd->startKeywordDetect(-1);
		printf("#### [SD] keyword detected.\n");

		mr.start();

		take_sem(&finished_sem);
		
		mr.unprepare();
		mr.destroy();
#endif
		mp.create();
		filePath = "/mnt/file.raw";
		auto source = std::move(unique_ptr<media::stream::FileInputDataSource>(new media::stream::FileInputDataSource(filePath)));
		source->setSampleRate(24000);
		source->setChannels(1);
		source->setPcmFormat(media::AUDIO_FORMAT_TYPE_S16_LE);
		mp.setObserver(std::make_shared<_Observer>());
		mp.setDataSource(std::move(source));
		mp.prepare();
		mp.setVolume(5);
		mp.start();

		take_sem(&tts_sem);

		printf("##################################\n");
		printf("####   Playback done!!        ####\n");
		printf("##################################\n");

		mp.unprepare();
		mp.destroy();

		//sd->deinitKeywordDetect();
		}
		return 0;
	}
}
