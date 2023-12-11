/*
Copyright 2018 Google Inc. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS-IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include "platforms/unity/unity.h"

#include <algorithm>
#include <memory>

#include "base/audio_buffer.h"
#include "base/constants_and_types.h"
#include "base/logging.h"
#include "base/misc_math.h"
#include "graph/resonance_audio_api_impl.h"
#include "platforms/common/room_effects_utils.h"

#if !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
#include "utils/ogg_vorbis_recorder.h"
#endif  // !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))

namespace vraudio {
namespace unity {

namespace {

// Output channels must be stereo for the ResonanceAudio system to run properly.
constexpr size_t kNumOutputChannels = 2;

#if !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
// Ambisonics order (index) to recording channels.
constexpr std::array<size_t, 4> kNumRecordingChannelsByAmbisonicsOrder {
  kNumMonoChannels,
  kNumFirstOrderAmbisonicChannels,
  kNumSecondOrderAmbisonicChannels,
  kNumThirdOrderAmbisonicChannels
};

// Recording time hard limit.
constexpr float kMaxRecordingTime = 600.0f;

// Record compression quality.
constexpr float kRecordQuality = 1.0f;
#endif  // !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))

// Stores the necessary components for the ResonanceAudio system. Methods called
// from the native implementation below must check the validity of this
// instance.
struct ResonanceAudioSystem {
  ResonanceAudioSystem(int sample_rate, size_t num_channels,
                       size_t frames_per_buffer)
      : api(CreateResonanceAudioApi(num_channels, frames_per_buffer,
                                    sample_rate))
#if !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
      ,
        sample_rate(sample_rate),
	frames_per_buffer(frames_per_buffer),
        is_recording_soundfield(false),
        num_record_channels(0)
#endif  // !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
  {}

  // ResonanceAudio API instance to communicate with the internal system.
  std::unique_ptr<ResonanceAudioApi> api;

  // Default room properties, which effectively disable the room effects.
  ReflectionProperties null_reflection_properties;
  ReverbProperties null_reverb_properties;

#if !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
  // Constants we need later but cannot access from ResonanceAudioApi.
  const int sample_rate;
  const size_t frames_per_buffer;

  // Denotes whether the soundfield recording is currently in progress.
  std::atomic<bool> is_recording_soundfield;

  // Last initialized number of recording channels.
  size_t num_record_channels;

  // Ambisonic soundfield recorder.
  std::unique_ptr<OggVorbisRecorder> soundfield_recorder;

  // Pre-allocated recording buffers.
  std::vector<std::unique_ptr<AudioBuffer>> record_buffers;
#endif  // !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
};

// Singleton |ResonanceAudioSystem| instance to communicate with the internal
// API.
static std::shared_ptr<ResonanceAudioSystem> resonance_audio = nullptr;

}  // namespace

void Initialize(int sample_rate, size_t num_channels,
                size_t frames_per_buffer) {
  CHECK_GE(sample_rate, 0);
  CHECK_EQ(num_channels, kNumOutputChannels);
  CHECK_GE(frames_per_buffer, 0);
  resonance_audio = std::make_shared<ResonanceAudioSystem>(
      sample_rate, num_channels, frames_per_buffer);
}

void Shutdown() { resonance_audio.reset(); }

void ProcessListener(size_t num_frames, float* output) {
  CHECK(output != nullptr);

  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy == nullptr) {
    return;
  }

  if (!resonance_audio_copy->api->FillInterleavedOutputBuffer(
          kNumOutputChannels, num_frames, output)) {
    // No valid output was rendered, fill the output buffer with zeros.
    const size_t buffer_size_samples = kNumOutputChannels * num_frames;
    CHECK(!vraudio::DoesIntegerMultiplicationOverflow<size_t>(
        kNumOutputChannels, num_frames, buffer_size_samples));

    std::fill(output, output + buffer_size_samples, 0.0f);
  }

#if !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
  if (resonance_audio_copy->is_recording_soundfield.load()) {

    if (resonance_audio_copy->record_buffers.empty()) {
       LOG(WARNING) << "No recording buffers left, skipping frame";
       return;
    }

    // Get the next record buffer.
    auto record_buffer = std::move (resonance_audio_copy->record_buffers.back());
    resonance_audio_copy->record_buffers.pop_back();

    // Record output into soundfield.
    auto* const resonance_audio_api_impl =
        static_cast<ResonanceAudioApiImpl*>(resonance_audio_copy->api.get());
    const auto* soundfield_buffer =
        resonance_audio_api_impl->GetAmbisonicOutputBuffer();
    if (soundfield_buffer != nullptr) {
      CHECK_EQ(num_frames, record_buffer->num_frames());
      const auto channels = std::min (resonance_audio_copy->num_record_channels,
                                      soundfield_buffer->num_channels());
      for (size_t ch = 0; ch < channels; ++ch) {
        (*record_buffer)[ch] = (*soundfield_buffer)[ch];
      }
    }
    resonance_audio_copy->soundfield_recorder->AddInput(
        std::move(record_buffer));
  }
#endif  // !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
}

void SetListenerGain(float gain) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetMasterVolume(gain);
  }
}

void SetListenerStereoSpeakerMode(bool enable_stereo_speaker_mode) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetStereoSpeakerMode(enable_stereo_speaker_mode);
  }
}

void SetListenerTransform(float px, float py, float pz, float qx, float qy,
                          float qz, float qw) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetHeadPosition(px, py, pz);
    resonance_audio_copy->api->SetHeadRotation(qx, qy, qz, qw);
  }
}

ResonanceAudioApi::SourceId CreateSoundfield(int num_channels) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    return resonance_audio_copy->api->CreateAmbisonicSource(num_channels);
  }
  return ResonanceAudioApi::kInvalidSourceId;
}

ResonanceAudioApi::SourceId CreateSoundObject(RenderingMode rendering_mode) {
  SourceId id = ResonanceAudioApi::kInvalidSourceId;
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    id = resonance_audio_copy->api->CreateSoundObjectSource(rendering_mode);
    resonance_audio_copy->api->SetSourceDistanceModel(
        id, DistanceRolloffModel::kNone, 0.0f, 0.0f);
  }
  return id;
}

void DestroySource(ResonanceAudioApi::SourceId id) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->DestroySource(id);
  }
}

void ProcessSource(ResonanceAudioApi::SourceId id, size_t num_channels,
                   size_t num_frames, float* input) {
  CHECK(input != nullptr);

  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetInterleavedBuffer(id, input, num_channels,
                                                    num_frames);
  }
}

void SetSourceDirectivity(ResonanceAudioApi::SourceId id, float alpha,
                          float order) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSoundObjectDirectivity(id, alpha, order);
  }
}

void SetSourceDistanceAttenuation(ResonanceAudioApi::SourceId id,
                                  float distance_attenuation) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSourceDistanceAttenuation(
        id, distance_attenuation);
  }
}

void SetSourceGain(ResonanceAudioApi::SourceId id, float gain) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSourceVolume(id, gain);
  }
}

void SetSourceListenerDirectivity(ResonanceAudioApi::SourceId id, float alpha,
                                  float order) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSoundObjectListenerDirectivity(id, alpha,
                                                                 order);
  }
}

void SetSourceNearFieldEffectGain(ResonanceAudioApi::SourceId id,
                                  float near_field_effect_gain) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSoundObjectNearFieldEffectGain(
        id, near_field_effect_gain);
  }
}

void SetSourceOcclusionIntensity(ResonanceAudioApi::SourceId id,
                                 float intensity) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSoundObjectOcclusionIntensity(id, intensity);
  }
}

void SetSourceRoomEffectsGain(ResonanceAudioApi::SourceId id,
                              float room_effects_gain) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSourceRoomEffectsGain(id, room_effects_gain);
  }
}

void SetSourceSpread(int id, float spread_deg) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSoundObjectSpread(id, spread_deg);
  }
}

void SetSourceTransform(int id, float px, float py, float pz, float qx,
                        float qy, float qz, float qw) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy != nullptr) {
    resonance_audio_copy->api->SetSourcePosition(id, px, py, pz);
    resonance_audio_copy->api->SetSourceRotation(id, qx, qy, qz, qw);
  }
}

void SetRoomProperties(RoomProperties* room_properties, float* rt60s) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy == nullptr) {
    return;
  }
  if (room_properties == nullptr) {
    resonance_audio_copy->api->SetReflectionProperties(
        resonance_audio_copy->null_reflection_properties);
    resonance_audio_copy->api->SetReverbProperties(
        resonance_audio_copy->null_reverb_properties);
    return;
  }

  const auto reflection_properties =
      ComputeReflectionProperties(*room_properties);
  resonance_audio_copy->api->SetReflectionProperties(reflection_properties);
  const auto reverb_properties =
      (rt60s == nullptr)
          ? ComputeReverbProperties(*room_properties)
          : ComputeReverbPropertiesFromRT60s(
                rt60s, room_properties->reverb_brightness,
                room_properties->reverb_time, room_properties->reverb_gain);
  resonance_audio_copy->api->SetReverbProperties(reverb_properties);
}

#if !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))
bool InitSoundfieldRecorder(size_t ambisonics_order,
                            float max_recording_time) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy == nullptr) {
    return false;
  }
  if (resonance_audio_copy->is_recording_soundfield.load()) {
    LOG(ERROR) << "Cannot initialize soundfield recorder while recording is in progress";
    return false;
  }

  // Determine number of recording channels.
  ambisonics_order = std::min(ambisonics_order,
                              kNumRecordingChannelsByAmbisonicsOrder.size() - 1);
  const auto num_record_channels = kNumRecordingChannelsByAmbisonicsOrder.at(ambisonics_order);

  resonance_audio_copy->num_record_channels = num_record_channels;
  resonance_audio_copy->record_buffers.clear();

  // Determine number of record buffers and allocate.
  const auto frames_per_buffer = resonance_audio_copy->frames_per_buffer;
  const auto sample_rate = resonance_audio_copy->sample_rate;
  const auto num_record_buffers = size_t(std::min(max_recording_time, kMaxRecordingTime) /
                                         (frames_per_buffer / double(sample_rate)));

  if (num_record_buffers == 0) {
    LOG(ERROR) << "Cannot initialize soundfield recorder with zero recording time";
    return false;
  }

  for (size_t i = 0; i < num_record_buffers; ++i)
    resonance_audio_copy->record_buffers.emplace_back(std::make_unique<AudioBuffer>(num_record_channels, frames_per_buffer));

  // Create soundfield recorder object.
  resonance_audio_copy->soundfield_recorder.reset(
      new OggVorbisRecorder(sample_rate, num_record_channels,
                            frames_per_buffer, num_record_buffers));

  return true;
}

bool StartSoundfieldRecorder() {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy == nullptr) {
    return false;
  }

  if (resonance_audio_copy->soundfield_recorder == nullptr) {
    LOG(ERROR) << "Soundfield recorder object not initialized, call InitSoundfieldRecorder() first";
    return false;
  }

  if (resonance_audio_copy->record_buffers.empty()) {
    LOG(ERROR) << "Recording buffers not initialized or none left, call InitSoundfieldRecorder() first";
    return false;
  }

  if (resonance_audio_copy->is_recording_soundfield.exchange(true)) {
    LOG(ERROR) << "Another soundfield recording already in progress";
    return false;
  }

  return true;
}

bool StopSoundfieldRecorder() {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy == nullptr) {
    return false;
  }

  if (!resonance_audio_copy->is_recording_soundfield.exchange(false)) {
    LOG(ERROR) << "No recorded soundfield found";
    return false;
  }

  return true;
}

bool WriteSoundfieldRecordingToFile(const char* file_path,
                                    bool seamless) {
  auto resonance_audio_copy = resonance_audio;
  if (resonance_audio_copy == nullptr) {
    return false;
  }

  if (resonance_audio_copy->soundfield_recorder == nullptr) {
    LOG(ERROR) << "Soundfield recorder object not initialized, call InitSoundfieldRecorder() first";
    return false;
  }

  // Stop recording just in case.
  resonance_audio_copy->is_recording_soundfield.store(false);

  if (file_path == nullptr) {
    resonance_audio_copy->soundfield_recorder->Reset();
    return false;
  }

  resonance_audio_copy->soundfield_recorder->WriteToFile(
      file_path, kRecordQuality, seamless);
  return true;
}
#endif  // !(defined(PLATFORM_ANDROID) || defined(PLATFORM_IOS))

}  // namespace unity
}  // namespace vraudio
