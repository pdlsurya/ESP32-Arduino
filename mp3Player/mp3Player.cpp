#include <Arduino.h>
#include <string.h>
#include "freertos/queue.h"
#include "mySdFat.h"
#include "i2sDac.h"
#include "myOled.h"
#include "IR_receiver.h"

#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_STDIO
#include "minimp3.h"

#define SPEAKER_PIN 25
#define LED 2

#define NEXT_TRACK_CMD 31
#define PREV_TRACK_CMD 30
#define PLAY_PAUSE_CMD 64
#define VOLUME_UP_CMD 24
#define VOLUME_DOWN_CMD 23
#define SHUFFLE_CMD 37
#define FF_CMD 65

#define BUTTON_DEBOUNCE_DELAY 200000
#define I2S_LOAD_BUFF_INTERVAL 23200

#define BUFFER_SIZE 1024

// hw_timer_t *btnDebTimer = NULL;
// hw_timer_t *i2sLoadTimer = NULL;

uint8_t volume_level = 5;
volatile bool paused = false;
volatile bool next = false;
volatile bool prev = false;
volatile bool shuffle = false;
volatile bool fast_forward = false;
bool play_in_progress = false;
uint16_t ff_loop_cnt;
static bool led_state = false;

uint16_t timeCnt = 0;
uint16_t total_duration;
uint8_t minutes, hours;
uint16_t seconds;
char current_time[8] = "0:0:0 \0";
char total_time[8] = "0:0:0 \0";
char songName[22] = {0};

myFile songFolder;
myFile songFile;
myFile prevFile;
uint32_t songEntIndex[255];
uint8_t songIndex = 0;
uint8_t songsCount;

uint32_t fileTotalBytesRead;
uint32_t to_read = BUFFER_SIZE;
uint32_t buffered;
uint32_t decoded;
volatile uint16_t samples = 0;
bool is_output_started = false;
int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
uint8_t input_buf[BUFFER_SIZE];

mp3dec_t mp3d = {};
mp3dec_frame_info_t info = {};

volatile bool button_debounce_timer_started = false;
volatile uint8_t interruptCnt = 0;
volatile bool i2sLoadBuf = false;

uint8_t scroll_cnt = 0;

bool isMp3File(const char *filename)
{ //.mp3
    uint8_t len = strlen(filename);
    if (strcmp(filename + len - 3, "mp3") == 0)
        return true;
    return false;
}

/**
void IRAM_ATTR next_sw_isr()
{
    if (!button_debounce_timer_started)
    {
        timerAlarmEnable(btnDebTimer);
        button_debounce_timer_started = true;
    }
}

void IRAM_ATTR btnDebTimer_handler()
{
    interruptCnt++;
    if (interruptCnt == 2)
    {
        if (!next)
            next = true;

        button_debounce_timer_started = false;
        interruptCnt = 0;
        timerAlarmDisable(btnDebTimer);
    }
}
*/

bool mp3IsPlaying()
{
    return play_in_progress;
}

void mp3PlayerStop()
{
    fileClose(&songFile);
    play_in_progress = false;
    paused = false;
    songFolder.entryIndex = songEntIndex[songIndex];
    is_output_started = false;
    i2sDacStop();
}

void IRAM_ATTR i2sLoadTimer_handler()
{
    if (!i2sLoadBuf)
    {
        i2sLoadBuf = true;
    }
}

bool mp3Player_init(const char *songsPath)
{

    /*
         attachInterrupt(digitalPinToInterrupt(NEXT_SW_PIN), next_sw_isr, FALLING);

         btnDebTimer = timerBegin(0, 80, true);
         timerAttachInterrupt(btnDebTimer, btnDebTimer_handler, true);
         timerAlarmWrite(btnDebTimer, BUTTON_DEBOUNCE_DELAY, true);

        i2sLoadTimer = timerBegin(1, 80, true);
        timerAttachInterrupt(i2sLoadTimer, i2sLoadTimer_handler, true);
        timerAlarmWrite(i2sLoadTimer, I2S_LOAD_BUFF_INTERVAL, true);
    */
    mp3dec_init(&mp3d);

    if (!mySdFat_init())
        return false;

    songFolder = fileOpen(songsPath, NULL);
    if (!isValidFile(&songFolder))
        return false;

    while (!isEndOfDir(&(songFile = nextFile(&songFolder))))
    {
        if (isValidFile(&songFile) && !isDirectory(&songFile) && isMp3File(fileName))
            songEntIndex[songIndex++] = songFolder.entryIndex - (fileLfnEntCnt(&songFile) + 1);
    }

    songFolder.entryIndex = songEntIndex[0];
    songsCount = songIndex;
    Serial.printf("Total mp3 files:%d", songsCount);
    songIndex = 0;
    setVolume((float)volume_level * 0.1);

    return true;
}

void mp3ReadIR()
{
    if (ir_data.updated && !ir_data.repeat)
    {
        if (ir_data.command == SHUFFLE_CMD)
        {
            if (!shuffle)
                shuffle = true;
            else
                shuffle = false;
        }
        if (ir_data.command == FF_CMD)
        {
            if (!fast_forward)
                fast_forward = true;
            else
                fast_forward = false;
        }

        else if (ir_data.command == NEXT_TRACK_CMD)
        {
            if (!next)
                next = true;
        }

        else if (ir_data.command == PREV_TRACK_CMD)
        {

            if (!prev)
                prev = true;
        }
        else if (ir_data.command == VOLUME_DOWN_CMD)
        {
            volume_level--;
            if (volume_level < 1)
                volume_level = 1;
            setVolume((float)volume_level * 0.1);
        }

        else if (ir_data.command == VOLUME_UP_CMD)
        {
            volume_level++;
            if (volume_level > 10)
                volume_level = 10;
            setVolume((float)volume_level * 0.1);
        }

        else if (ir_data.command == PLAY_PAUSE_CMD)
        {
            if (paused)
            {
                i2sDacResume();
                paused = false;
            }

            else
            {
                i2sDacPause();
                paused = true;
            }
        }
        ir_data.updated = false;
    }
}

void displayInfo()
{
    timeCnt = 0;
    hours = seconds / 3600;
    minutes = (seconds / 60) % 60;
    seconds = seconds % 60;

    sprintf(current_time, "%d:%d:%d", hours, minutes, seconds);
    clearDisplay();
    if (shuffle)
        shuffleIcon(110, 7);
    printString(current_time, 0, 40, 6, false);
    printString(total_time, 84, 40, 6, false);
    memset(songName, 0, sizeof(songName));
    for (uint8_t i = 0; i < 21; i++)
    {
        if ((i + scroll_cnt) < strlen(fileName) - 4)
            songName[i] = fileName[i + scroll_cnt];
    }
    scroll_cnt++;
    if (scroll_cnt == strlen(fileName) - 4)
        scroll_cnt = 0;
    songName[21] = '\0';
    printString(songName, 0, 0, 6, false);
    setBar(map(decoded / info.hz, 0, total_duration, 0, 125), 3);
    setSignal(map((uint8_t)(volume * 10.0), 1, 10, 0, 100));
    display();
}

void mp3DecodePlay()
{
    do
    {
        uint32_t samplesRead = 0;
        for (uint16_t i = 0; i < to_read; i++)
        {
            input_buf[buffered + i] = readByte(&songFile);
            fileTotalBytesRead++;
            samplesRead++;
            if (fileTotalBytesRead == fileSize(&songFile))
                break;
        }

        buffered += samplesRead;

        samples = mp3dec_decode_frame(&mp3d, input_buf, buffered, pcm, &info);
        /*
        Serial.printf("pcm decoded=%d\n", samples);
        Serial.printf("mp3 consumed=%d\n", info.frame_bytes);
        Serial.printf("bitrate=%d\n", info.bitrate_kbps);
        */

        // we've processed this may bytes from teh buffered data
        buffered -= info.frame_bytes;

        // shift the remaining data to the front of the buffer
        memmove(input_buf, input_buf + info.frame_bytes, buffered);

        // we need to top up the buffer from the file
        to_read = info.frame_bytes;

        // keep track of how many samples we've decoded
        decoded += samples;

        if (fast_forward)
        {
            mp3ReadIR(); // listen for IR data while in ff loop
            ff_loop_cnt++;
            if (ff_loop_cnt % 10 == 0)
                timeCnt++;
        }
        else
            timeCnt++;

        if (timeCnt == 10)
        {
            led_state = !led_state;
            digitalWrite(LED, led_state);
            seconds = (decoded / info.hz);
            displayInfo();
        }

    } while (fast_forward && (fileTotalBytesRead != fileSize(&songFile)));
    ff_loop_cnt = 0;

    if (fast_forward)
        fast_forward = false;

    if (samples > 0)
    {
        // if we haven't started the output yet we can do it now as we now know the sample rate and number of channels
        if (!is_output_started)
        {
            i2sDacStart(info.hz);
            is_output_started = true;
        }
        // if we've decoded a frame of mono samples convert it to stereo by duplicating the left channel
        // we can do this in place as our samples buffer has enough space
        if (info.channels == 1)
        {
            for (uint16_t i = samples - 1; i >= 0; i--)
            {
                pcm[i * 2] = pcm[i];
                pcm[i * 2 - 1] = pcm[i];
            }
        }
        // write the decoded samples to the I2S DAC output
        i2sDacWrite(pcm, samples);
    }
}

void mp3PlayerProcess()
{
    // mp3ReadIR(); /* executed from main loop so no need to execute here*/

    if (!paused)
    {
        if (!play_in_progress)
        {
            songFile = nextFile(&songFolder);
            if (isValidFile(&songFile) && !isDirectory(&songFile))
            {
                memset(songName, 0, sizeof(songName));
                Serial.print("Currently playing: ");
                Serial.println(fileName);
                for (uint8_t i = 0; i < strlen(fileName) - 4; i++)
                {
                    if (i < 21)
                        songName[i] = fileName[i];
                }
                songName[21] = '\0';
                scroll_cnt = 0;
                fileTotalBytesRead = 0;
                buffered = 0;
                decoded = 0;
                memset(current_time, 0, sizeof(current_time));
                play_in_progress = true;
                to_read = BUFFER_SIZE;
                prev = false;
                next = false;
                timeCnt = 0;

                // initially decode one frame to retrived mp3 info.
                mp3DecodePlay();

                total_duration = (fileSize(&songFile) - 128) / (((info.bitrate_kbps == 64 ? 128 : info.bitrate_kbps) * 1000) / 8);

                seconds = total_duration;
                hours = seconds / 3600;
                minutes = (seconds / 60) % 60;
                seconds = seconds % 60;
                sprintf(total_time, "%d:%d:%d", hours, minutes, seconds);
                seconds = 0;
                minutes = 0;
                hours = 0;
                displayInfo();
            }
        }
        else
        {
            mp3DecodePlay();

            if (next || fileTotalBytesRead >= fileSize(&songFile))
            {
                if (shuffle)
                {
                    songIndex = random(songsCount);
                }
                else
                {
                    songIndex++;
                    if (songIndex == songsCount)
                        songIndex = 0;
                }
                songFolder.entryIndex = songEntIndex[songIndex];
                i2sDacStop();
                is_output_started = false;
                play_in_progress = false;
                fileClose(&songFile);
            }

            if (prev)
            {
                songIndex--;

                if (songIndex > songsCount)
                    songIndex = songsCount - 1;

                songFolder.entryIndex = songEntIndex[songIndex];

                i2sDacStop();
                is_output_started = false;
                play_in_progress = false;
                fileClose(&songFile);
            }
        }
    }
}
