#ifndef _WAVPLAYER_H
#define _WAVPLAYER_H

#include<Arduino.h>
#include "mySdFat.h"
#include <stdint.h>
#include <freertos/task.h>

typedef enum{
    AUDIO_OUT_BT, AUDIO_OUT_DAC
}audio_out_t;

bool mp3Player_init(const char* songsFolder);

void mp3PlayerProcess();

void mp3ReadIR();

bool mp3IsPlaying();

void mp3PlayerStop();

#endif  //_WAVPLAYER_H