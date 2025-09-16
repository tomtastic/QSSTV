/**************************************************************************
*   Copyright (C) 2023 by Mario Klebsch                                   *
*   mario@klebsch.de                                                      *
*   http://klebsch.de                                                     *
*                                                                         *
*   This program is free software; you can redistribute it and/or modify  *
*   it under the terms of the GNU General Public License as published by  *
*   the Free Software Foundation; either version 2 of the License, or     *
*   (at your option) any later version.                                   *
*                                                                         *
*   This program is distributed in the hope that it will be useful,       *
*   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
*   GNU General Public License for more details.                          *
*                                                                         *
*   You should have received a copy of the GNU General Public License     *
*   along with this program; if not, write to the                         *
*   Free Software Foundation, Inc.,                                       *
*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
***************************************************************************/

#include "soundmacos.h"
#include "soundbase.h"
#include "soundconfig.h"
#include "CaptureDeviceAuthorization.h"

#include <memory>
#include <deque>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <future>

#include <stdio.h>
#include <stdarg.h>

#include <AudioToolbox/AudioToolbox.h>
#include <CoreFoundation/CFString.h>



/*
static std::string vsPrintf(const char *format, va_list ap)
{
	char *s;
	vasprintf(&s, format, ap);
	return std::unique_ptr<char, void(*)(void*)>(s, free).get();
}

static std::string sPrintf(const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	auto ret = vsPrintf(format, ap);
	va_end(ap);
	return ret;
}
*/

namespace soundMacos
{
static void EnableAudioInputCB(int allowed, void *context)
{
	auto result = static_cast<std::promise<bool>*>(context);
	result->set_value(allowed);
	
}

bool EnableAudioInput()
{
	std::promise<bool> result;
	CaptureDeviceAuthorizationStatus(EnableAudioInputCB, &result);
	return result.get_future().get();
}


class exception:public std::runtime_error
{
public:
	using runtime_error::runtime_error;
	
	exception(const char *format, ...): runtime_error(format)
	{
		va_list ap;
		va_start(ap, format);
		char *s;
		vasprintf(&s, format, ap);
		va_end(ap);

		auto ptr = std::unique_ptr<char, void(*)(void*)>(s, free);
		auto error = std::runtime_error(ptr.get());
		static_cast<std::runtime_error&>(*this)=error;
	}
};

static bool DeviceHasBuffersInScope(AudioObjectID deviceID, AudioObjectPropertyScope scope)
{
	assert(deviceID != kAudioObjectUnknown);

	AudioObjectPropertyAddress propertyAddress = {
		.mSelector  = kAudioDevicePropertyStreamConfiguration,
		.mScope     = scope,
		.mElement   = kAudioObjectPropertyElementWildcard
	};

	UInt32 dataSize = 0;
	OSStatus result = AudioObjectGetPropertyDataSize(deviceID, &propertyAddress, 0, NULL, &dataSize);
	if(result != kAudioHardwareNoError)
		return false;

	std::vector<char> buffer(dataSize);
	AudioBufferList *bufferList = (AudioBufferList *)buffer.data();
	if(!bufferList)
		return false;

	result = AudioObjectGetPropertyData(deviceID, &propertyAddress, 0, NULL, &dataSize, bufferList);
	if(result != kAudioHardwareNoError)
		return false;

	return bufferList->mNumberBuffers > 0;
}

bool DeviceSupportsInput(AudioObjectID deviceID)
{
	return DeviceHasBuffersInScope(deviceID, kAudioObjectPropertyScopeInput);
}

bool DeviceSupportsOutput(AudioObjectID deviceID)
{
	return DeviceHasBuffersInScope(deviceID, kAudioObjectPropertyScopeOutput);
}

static AudioObjectPropertyAddress DevicesPropertyAddress = {
	.mSelector = kAudioHardwarePropertyDevices,
	   .mScope    = kAudioObjectPropertyScopeGlobal,
	   .mElement  = kAudioObjectPropertyElementMaster
   };

static std::vector<AudioObjectID> getAudioDevices()
{
	OSStatus ret;
	UInt32 Size = 0;
	ret = AudioObjectGetPropertyDataSize(kAudioObjectSystemObject, &DevicesPropertyAddress, 0, NULL, &Size);
	if (ret)
	{
		fprintf(stderr, "AudioObjectGetPropertyDataSize(DevicesPropertyAddress) returned %d\n", ret);
		std::vector<AudioObjectID>();
	}
	std::vector<AudioObjectID> Devices(Size/sizeof(AudioObjectID));
	ret = AudioObjectGetPropertyData(kAudioObjectSystemObject, &DevicesPropertyAddress, 0, NULL,
									 &Size, Devices.data());
	if (ret)
	{
		fprintf(stderr, "AudioObjectGetPropertyData(DevicesPropertyAddress) returned %d\n", ret);
		std::vector<AudioObjectID>();
	}
	return Devices;
}


std::string to_string(CFStringRef s)
{
	auto ptr = CFStringGetCStringPtr(s, kCFStringEncodingUTF8);
	if (ptr)
		return ptr;
	std::vector<char> buffer(CFStringGetMaximumSizeForEncoding(CFStringGetLength(s), kCFStringEncodingUTF8) + 1);
	if (!CFStringGetCString(s, buffer.data(), buffer.size(), kCFStringEncodingUTF8))
		return "";
	return buffer.data();
}

class CFString
{
	CFStringRef s = nullptr;
public:
	CFString() {}
	explicit CFString(CFStringRef s):s(s){};
	explicit CFString(const char *s):
	s(CFStringCreateWithCString(kCFAllocatorDefault, s, kCFStringEncodingUTF8)){}

	CFString(const CFString&other):s((CFStringRef)CFRetain(other.s)) {}
	CFString(CFString&&other):s(other.s) { other.s=nullptr; }

	~CFString()
	{
		reset();
	}
	
	CFString&operator=(const CFString &other)
	{
		reset();
		s = (CFStringRef)CFRetain(other.s);
		return *this;
	}

	CFString&operator=(CFString &&other)
	{
		reset();
		s = other.s;
		other.s = nullptr;
		return *this;
	}

	void reset()
	{
		if (s)
			CFRelease(s);
	}
	
	void reset(CFStringRef other)
	{
		reset();
		if (other)
			s = other;
	}
	
	const CFStringRef * operator&() const { return &s; }
	CFStringRef * operator&() { return &s; }
	
	operator CFStringRef() { return s; }
	operator CFStringRef() const { return s; }
	
	explicit operator bool() const { return s; }

};

std::string getDeviceName(AudioObjectID Id)
{
	static AudioObjectPropertyAddress DeviceNamePropertyAddress = {
		.mSelector = kAudioDevicePropertyDeviceNameCFString,
		.mScope    = kAudioObjectPropertyScopeGlobal,
		.mElement  = kAudioObjectPropertyElementMaster
	};

	CFString DeviceName;
	UInt32 Size = sizeof(DeviceName);
	OSStatus ret = AudioObjectGetPropertyData(Id, &DeviceNamePropertyAddress,
											  0, NULL, &Size, &DeviceName);
	if (ret)
	{
		fprintf(stderr, "AudioObjectGetPropertyDataSize(DeviceNamePropertyAddress) returned %d\n", ret);
		return "";
	}
	return to_string(DeviceName);
}

std::string getDeviceUID(AudioObjectID Id)
{
	static AudioObjectPropertyAddress DeviceUIDPropertyAddress = {
		.mSelector = kAudioDevicePropertyDeviceUID,
		.mScope    = kAudioObjectPropertyScopeGlobal,
		.mElement  = kAudioObjectPropertyElementMaster
	};

	CFString DeviceName;
	UInt32 Size = sizeof(DeviceName);
	OSStatus ret = AudioObjectGetPropertyData(Id, &DeviceUIDPropertyAddress,
											  0, NULL, &Size, &DeviceName);
	if (ret)
	{
		fprintf(stderr, "AudioObjectGetPropertyDataSize(DeviceUIDPropertyAddress) returned %d\n", ret);
		return "";
	}
	return to_string(DeviceName);
}

std::list<soundCard> getCardList()
{
	std::list<soundCard> ret;

	struct makeSoundCard:public soundCard
	{
		makeSoundCard(AudioObjectID device)
		{
			Label = getDeviceName(device);
			UID   = getDeviceUID(device);
			SupportsInput  = DeviceSupportsInput(device);
			SupportsOutput = DeviceSupportsOutput(device);
		}
	};

	for (auto device:getAudioDevices())
		ret.push_back(makeSoundCard(device));
	return ret;
}

class audio_channel
{
protected:
	static const int sample_size_in_bytes = sizeof(int16_t);
	static const int buffer_size_in_samples = 1024;
	const int buffers_for_one_second;
	const int buffer_size_in_bytes;
	const long usecs_per_buffer;

	AudioStreamBasicDescription format;
	AudioQueueRef queue = nullptr;

	std::deque<AudioQueueBufferRef> buffers;
	mutable std::mutex              buffers_mutex;
	std::condition_variable         buffers_condition;

public:
	audio_channel(const audio_channel&) = delete;
	audio_channel& operator=(const audio_channel&)=delete;
	
	audio_channel(int rate, int channels):
	buffers_for_one_second{(rate+buffer_size_in_samples-1)/buffer_size_in_samples},
	buffer_size_in_bytes{buffer_size_in_samples * sample_size_in_bytes * channels},
	usecs_per_buffer{1000000 * buffer_size_in_samples / rate}
	{
		memset(&format, 0 , sizeof(format));
		format.mSampleRate       = rate;
		format.mFormatID         = kAudioFormatLinearPCM;
		format.mFormatFlags      = kLinearPCMFormatFlagIsSignedInteger | kAudioFormatFlagIsPacked;
		format.mBitsPerChannel   = 8 * sample_size_in_bytes;
		format.mChannelsPerFrame = channels;
		format.mBytesPerFrame    = sample_size_in_bytes * format.mChannelsPerFrame;
		format.mFramesPerPacket  = 1;
		format.mBytesPerPacket   = format.mBytesPerFrame * format.mFramesPerPacket;
		format.mReserved         = 0;
	}
	
	virtual ~audio_channel()
	{
		if (queue)
		{
			auto ret = AudioQueueDispose(queue, 1);
			if (ret)
				fprintf(stderr, "%s: AudioQueueDispose() failed: %d", __FUNCTION__, ret);
		}
	}

protected:
	std::deque<AudioQueueBufferRef> make_buffers(size_t n)
	{
		std::deque<AudioQueueBufferRef> buffers;
		for (unsigned i = 0; i < n; i++)
		{
			AudioQueueBufferRef buffer;
			auto ret = AudioQueueAllocateBuffer(queue, buffer_size_in_bytes, &buffer);
			if (ret) throw exception("AudioQueueAllocateBuffer", ret);

			buffers.push_front(buffer);
		}
		return buffers;
	}
};

class input_channel: public audio_channel
{
	size_t bytes_consumed = 0;
	mutable std::atomic<unsigned> overruns{0};
	size_t queued_size = 0;

public:
	input_channel(int rate, int channels, const char *uid):audio_channel(rate, channels)
	{
		printf("XXX: input_channel(%s)\n", uid);
		EnableAudioInput();
		auto ret = AudioQueueNewInput(&format, staticCallback, this, CFRunLoopGetCurrent(), kCFRunLoopCommonModes, 0, &queue);
		if (ret) throw exception("AudioQueueNewInput", ret);

		for (auto buffer: make_buffers(buffers_for_one_second))
		{
			auto ret = AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
			if (ret) exception("AudioQueueEnqueueBuffer", ret);
		}

		const auto UID = CFString(uid);
		AudioQueueSetProperty(queue, kAudioQueueProperty_CurrentDevice, &UID, sizeof(UID));
		
		ret = AudioQueueStart(queue, NULL);
		if (ret) throw exception("AudioQueueStart", ret);
	}

private:
	static void staticCallback(void *ctx, AudioQueueRef queue, AudioQueueBufferRef buffer, const AudioTimeStamp *inStartTime,
					UInt32 inNumberPacketDescriptions, const AudioStreamPacketDescription *inPacketDescs)
	{
		auto ch = (input_channel*)ctx;
		assert(ch);
		assert(queue == ch->queue);
		ch->Callback(buffer, inStartTime, inNumberPacketDescriptions, inPacketDescs);
	}

	void Callback(AudioQueueBufferRef buffer, const AudioTimeStamp *inStartTime,
				  UInt32 inNumberPacketDescriptions, const AudioStreamPacketDescription *inPacketDescs)
	{
		(void)inStartTime;
		(void)inNumberPacketDescriptions;
		(void)inPacketDescs;
//		printf("XXX: input_channel::Callback()\n");
		std::unique_lock<std::mutex> Lock(buffers_mutex);
		buffers.push_front(buffer);
		queued_size += buffer->mAudioDataByteSize;
		if (buffers.size()+2 >= buffers_for_one_second)
		{
			recyle_buffer();
			overruns++;
		}

#ifdef XXX
		auto casted_buffer = (const int16_t *)buffer->mAudioData;

		int16_t min_sample=0;
		int16_t max_sample=0;
		unsigned samples = buffer->mAudioDataByteSize / sizeof(int16_t);
		for (unsigned i = 0; i < samples; i++)
		{
			auto int_sample = *casted_buffer++;
			if (int_sample < min_sample)
				min_sample = int_sample;
			else if (int_sample > max_sample)
				max_sample = int_sample;
		}
		fprintf(stderr, "%s: samples=%6u, min=%6d, max=%6d\n", __FUNCTION__, samples, min_sample, max_sample);
#endif
	}

public:
	size_t bytes_available() const
	{
		std::unique_lock<std::mutex> Lock(buffers_mutex);
		auto bytes_avail = queued_size - bytes_consumed;
		Lock.unlock();
		auto lost = overruns.exchange(0);
		if (lost)
			printf("%s: %zu bytes available, lost %u\n", __FUNCTION__, bytes_avail, lost);
//			else
// 				printf("%s: %u usecs\n", __FUNCTION__, ret);
		return bytes_avail;
	}

private:
	void recyle_buffer()
	{
		auto buffer = buffers.back();
		buffers.pop_back();

		queued_size -= buffer->mAudioDataByteSize;
		AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);
		bytes_consumed = 0;
	}

public:
	void read(void *data, size_t bytes)
	{
//			printf("read(%zu bytes)\n", bytes);
		std::unique_lock<std::mutex> Lock(buffers_mutex);
		auto bytes_avail = queued_size  - bytes_consumed;
		assert(bytes_avail >= bytes);
		while (bytes)
		{
			const auto buffer = buffers.back();
			const auto n = std::min(bytes, buffer->mAudioDataByteSize - bytes_consumed);
			memcpy(data, static_cast<const char*>(buffer->mAudioData) + bytes_consumed, n);
			bytes_consumed += n;
			bytes -= n;
			data = static_cast<char*>(data) + n;
			if (bytes_consumed >= buffer->mAudioDataByteSize)
				recyle_buffer();
		}
	}
	
	void drain()
	{
		std::unique_lock<std::mutex> Lock(buffers_mutex);
		while (!buffers.empty())
			recyle_buffer();
		overruns.exchange(0);
	}


};

class output_channel: public audio_channel
{
	std::atomic<unsigned> buffers_in_queue{0};
	bool started = false;

public:
	output_channel(int rate, int channels, const char *uid):audio_channel(rate, channels)
	{
		printf("XXX: output_channel(%s)\n", uid);

		auto ret = AudioQueueNewOutput(&format, staticCallback, this, CFRunLoopGetCurrent(), kCFRunLoopCommonModes, 0, &queue);
		if (ret) throw exception("AudioQueueNewOutput", ret );
		
		for (auto buffer: make_buffers(10))
			buffers.push_front(buffer);

		const auto UID = CFString(uid);
		AudioQueueSetProperty(queue, kAudioQueueProperty_CurrentDevice, &UID, sizeof(UID));
	}

private:
	static void staticCallback(void *ctx, AudioQueueRef queue, AudioQueueBufferRef buffer)
	{
		auto ch = (output_channel*)ctx;
		assert(ch);
		assert(queue == ch->queue);
		ch->Callback(buffer);
	}

	void Callback(AudioQueueBufferRef buffer)
	{
//			printf("XXX: mac_output_channel::Callback()\n");
		std::unique_lock<std::mutex> Lock(buffers_mutex);
		assert(buffers_in_queue);
		buffers_in_queue--;
		buffers.push_front(buffer);
		buffers_condition.notify_all();
	}
	
	AudioQueueBufferRef get_buffer()
	{
		std::unique_lock<std::mutex> Lock(buffers_mutex);
		while (buffers.empty())
			buffers_condition.wait(Lock);
		auto buffer = buffers.back();
		buffers.pop_back();
		return buffer;
	}
	
public:
	void write(const void *data, size_t bytes)
	{
//		printf("XXX: mac_output_channel::write(%zu bytes)\n", bytes);
#ifdef XXX
		auto casted_buffer = (const int16_t *)data;

		int16_t min_sample=0;
		int16_t max_sample=0;
		unsigned samples = bytes / sizeof(*casted_buffer);
		for (unsigned i = 0; i < samples; i++)
		{
			auto int_sample = *casted_buffer++;
			if (int_sample < min_sample)
				min_sample = int_sample;
			else if (int_sample > max_sample)
				max_sample = int_sample;
		}
		fprintf(stderr, "%s: samples=%6u, min=%6d, max=%6d\n", __FUNCTION__, samples, min_sample, max_sample);
#endif
		while (bytes)
		{
			auto buffer = get_buffer();
			const auto n = std::min(bytes, size_t(buffer->mAudioDataBytesCapacity));
			assert(n);
			memcpy(buffer->mAudioData, data, n);
			buffer->mAudioDataByteSize = n;
			buffers_in_queue++;
			AudioQueueEnqueueBuffer(queue, buffer, 0, NULL);

			bytes -= n;
			data = static_cast<const char*>(data) + n;
			if (!started)
			{
				auto ret = AudioQueueStart(queue, NULL);
				if (ret) throw exception("AudioQueueStart", ret );
				printf("XXX: mac_output_channelwrite(): started\n");
				started = true;
			}
		}
	}

	void stop(bool wait_queue_empty)
	{
		if (!started)
			return;

		if (wait_queue_empty)
		{
			std::unique_lock<std::mutex> Lock(buffers_mutex);
			while (buffers_in_queue)
				buffers_condition.wait(Lock);
		}

		auto ret = AudioQueueStop(queue,true);
		if (ret) throw exception("AudioQueueStop", ret );
		started = false;
	}
};


struct impl: public soundBase
{
	std::unique_ptr<input_channel> input;
	std::unique_ptr<output_channel> output;

	
	bool init(int samplerate) override
	{
		printf("XXX: %s(samplerate=%d)\n", __FUNCTION__, samplerate);
		try
		{
			input.reset(new  input_channel (samplerate, 1, inputAudioDevice.toLatin1().data()));
			output.reset(new output_channel(samplerate, 2, outputAudioDevice.toLatin1().data()));
			isStereo=false;
			soundDriverOK=true;
			return true;
		}
		catch (const std::runtime_error &err)
		{
			errorHandler("macos audio init error",QString(err.what()));
		}
		input.reset();
		output.reset();
		soundDriverOK=false;
		return false;
	}

	void closeDevices() override
	{
		printf("XXX: %s()\n", __FUNCTION__);
		input.reset();
		output.reset();
	}


	int read(int &countAvailable) override
	{
		if (!input) return 0;
		try {
			countAvailable = int(input->bytes_available());
			if (size_t(countAvailable) < sizeof(qint16)*DOWNSAMPLESIZE)
				return 0;

//			printf("XXX: %s(%d bytes available)\n", __FUNCTION__, countAvailable);

			input->read(tempRXBuffer,sizeof(qint16)*DOWNSAMPLESIZE);
			return DOWNSAMPLESIZE;
		}
		catch (const std::runtime_error &err)
		{
			errorHandler("macos audio read error",QString(err.what()));
		}
		return -1;
	}
	
	int write(uint numFrames) override
	{
//		printf("XXX: %s(numFrames=%u)\n", __FUNCTION__, numFrames);
		if (!output) return 0;
		if (!numFrames) return 0;
		try {
			output->write(tempTXBuffer, sizeof(qint16)*2*numFrames);
			return numFrames;
		}
		catch (const std::runtime_error &err)
		{
			errorHandler("macos audio write error",QString(err.what()));
		}
		return -1;
	}
	void flushCapture() override
	{
		printf("XXX: %s()\n", __FUNCTION__);
		input->drain();
		output->stop(false);
	}

	void flushPlayback() override
	{
		printf("XXX: %s()\n", __FUNCTION__);
		output->stop(false);
	}
	
	void waitPlaybackEnd()override
	{
		printf("XXX: %s()\n", __FUNCTION__);
		output->stop(true);
	}
};


soundBase *Create()
{
	return new impl();
}

} // namespace soundMacos
