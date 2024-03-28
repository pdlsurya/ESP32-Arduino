#ifndef _WAVPLAYER_H
#define _WAVPLAYER_H

#include<Arduino.h>
#include "mySdFat.h"

bool mp3Player_init(const char* songsFolder);

void mp3PlayerProcess();

void mp3ReadIR();

extern bool play_in_progress;
extern myFile songFile;
extern myFile songFolder;
extern uint8_t songIndex;
extern bool is_output_started;
extern bool paused;



#endif  //_WAVPLAYER_H