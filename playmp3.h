#include <iostream>
#include <alsa/asoundlib.h>

#define BUFFER_SIZE 4096  // Adjust buffer size as needed

void playAudio(const char* audioFilePath) {
    int err;
    unsigned char buffer[BUFFER_SIZE];
    snd_pcm_t *pcm_handle;
    snd_pcm_hw_params_t *params;

    // Open PCM playback device
    err = snd_pcm_open(&pcm_handle, "plughw:1,0", SND_PCM_STREAM_PLAYBACK, 0);
    if (err < 0) {
        std::cerr << "Error opening PCM device: " << snd_strerror(err) << std::endl;
        return;
    }

    // Initialize PCM parameters
    snd_pcm_hw_params_alloca(&params);
    snd_pcm_hw_params_any(pcm_handle, params);
    snd_pcm_hw_params_set_access(pcm_handle, params, SND_PCM_ACCESS_RW_INTERLEAVED);
    snd_pcm_hw_params_set_format(pcm_handle, params, SND_PCM_FORMAT_S16_LE);
    snd_pcm_hw_params_set_channels(pcm_handle, params, 2);  // Stereo
    unsigned int sample_rate = 16000;
    snd_pcm_hw_params_set_rate_near(pcm_handle, params, &sample_rate, 0);
    snd_pcm_hw_params(pcm_handle, params);

    // Open audio file for reading
    FILE *audioFile = fopen(audioFilePath, "rb");
    if (!audioFile) {
        std::cerr << "Error opening audio file for reading." << std::endl;
        snd_pcm_close(pcm_handle);
        return;
    }

    // Read and play audio data
    while (!feof(audioFile)) {
        size_t bytesRead = fread(buffer, 1, BUFFER_SIZE, audioFile);
        if (bytesRead > 0) {
            snd_pcm_writei(pcm_handle, buffer, bytesRead / 4);  // Assuming 16-bit stereo audio
        }
    }

    // Clean up and close PCM handle and audio file
    fclose(audioFile);
    snd_pcm_drain(pcm_handle);
    snd_pcm_close(pcm_handle);
    //std::cout << "check\n";
}