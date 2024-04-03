#include <BluetoothA2DPSource.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <mySdFat.h>
#include <myOled.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <IR_receiver.h>
#define MINIMP3_IMPLEMENTATION
#define MINIMP3_ONLY_MP3
#define MINIMP3_NO_STDIO
#include <minimp3.h>

#define NEXT_TRACK_CMD 31
#define PREV_TRACK_CMD 30
#define PLAY_PAUSE_CMD 64
#define VOLUME_UP_CMD 24
#define VOLUME_DOWN_CMD 23
#define SHUFFLE_CMD 37
#define FF_CMD 65

#define BUFFER_SIZE 1024

#define LED 2


const char* bt_device1 = "DR-BT101";
const char* bt_device2 = "Mi True Wireless EBs Basic_R";
static bool led_state = false;
BluetoothA2DPSource a2dp_source;

myFile songFolder;
myFile songFile;

uint32_t songEntIndex[255];
uint8_t songIndex = 0;
uint8_t songsCount;

uint16_t timeCnt = 0;
uint16_t total_duration;
uint8_t minutes, hours;
uint16_t seconds;
char current_time[8] = "0:0:0 \0";
char total_time[8] = "0:0:0 \0";
char songName[22] = {0};
uint8_t scroll_cnt = 0;

uint32_t fileTotalBytesRead;
uint32_t to_read = BUFFER_SIZE;
uint32_t buffered;
uint32_t decoded;
uint16_t samples = 0;
int16_t pcm[MINIMP3_MAX_SAMPLES_PER_FRAME];
uint8_t input_buf[BUFFER_SIZE];

mp3dec_t mp3d = {};
mp3dec_frame_info_t info = {};

volatile bool paused = false;
volatile bool next = false;
volatile bool prev = false;
volatile bool shuffle = false;
volatile bool play_in_progress = false;
volatile bool bt_started=false;
unsigned long timeout;

uint16_t ff_loop_cnt;
uint8_t Volume = 75;

float scale_factor = (48000.0 / 44100.0);

QueueHandle_t queue;
TaskHandle_t mp3TaskHandle = NULL;
SemaphoreHandle_t  dispSemaphore;



bool isMp3File(const char *filename)
{ //.mp3
  uint8_t len = strlen(filename);
  if (strcmp(filename + len - 3, "mp3") == 0)
    return true;
  return false;
}

void sample_rate_48k_44k() {
  uint16_t samples_cnt_44k;
  uint16_t right_sample_cnt = 0;
  uint16_t left_sample_cnt = 0;
  samples_cnt_44k = (uint16_t)((44100.0 / 48000.0) * (float)(samples));
  

  int16_t* p_48k_right = new int16_t[samples];
  int16_t* p_48k_left = new int16_t[samples];


  //copy right and left 48k samples in two separte buffers;
  for (uint16_t i = 0; i < (samples * 2); i++) {
    if (i % 2 == 0)
      p_48k_right[right_sample_cnt++] = pcm[i];
    else
      p_48k_left[left_sample_cnt++] = pcm[i];
  }
  memset(pcm, 0, sizeof(pcm));
  for (uint16_t i = 0; i < samples_cnt_44k; i++) {
    pcm[i * 2] = p_48k_right[(uint16_t)((float)i * scale_factor)];
    pcm[i * 2 + 1]  = p_48k_left[(uint16_t)((float)i * scale_factor)];
  }

  samples = samples_cnt_44k;

  delete[](p_48k_right);
  delete[](p_48k_left);
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
  setSignal(Volume);
  display();
}

void mp3Decode()
{

  uint32_t samplesRead = 0;
  for (uint16_t i = 0; i < to_read; i++)
  {

    if (fileTotalBytesRead == fileSize(&songFile))
      break;
    else {
      input_buf[buffered + i] = readByte(&songFile);
      fileTotalBytesRead++;
      samplesRead++;
    }
  }

  buffered += samplesRead;

  samples = mp3dec_decode_frame(&mp3d, input_buf, buffered, pcm, &info);

  // we've processed this may bytes from teh buffered data
  buffered -= info.frame_bytes;

  // shift the remaining data to the front of the buffer
  memmove(input_buf, input_buf + info.frame_bytes, buffered);

  // we need to top up the buffer from the file
  to_read = info.frame_bytes;

  // keep track of how many samples we've decoded
  decoded += samples;

  timeCnt++;
  if (timeCnt == 10)
    xSemaphoreGive(dispSemaphore);

  //if smaple rate is 48KHz, change it to 44.1KHz
  if (info.hz == 48000)
    sample_rate_48k_44k();

  for (int i = 0; i < (samples * 2); i++) {
    xQueueSend(queue, &pcm[i], portMAX_DELAY);
  }

  digitalWrite(LED, led_state);
  led_state = !led_state;

  if (fileTotalBytesRead == fileSize(&songFile)) {
    next = true;
    vTaskDelay(1);
  }
}


// The supported audio codec in ESP32 A2DP is SBC. SBC audio stream is encoded
// from PCM data normally formatted as 44.1kHz sampling rate, two-channel 16-bit sample data
int32_t get_data_frames(Frame *frame, int32_t frame_count) {
  static float m_time = 0.0;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  float m_deltaTime = 1.0 / info.hz;
  // fill the channel data


  for (int sample = 0; sample < frame_count; ++sample) {
    if (!paused) {
      xQueueReceiveFromISR(queue, &frame[sample].channel1, &xHigherPriorityTaskWoken);
      xQueueReceiveFromISR(queue, &frame[sample].channel2, &xHigherPriorityTaskWoken);
    }
    else
    {
      frame[sample].channel1 = 0;
      frame[sample].channel2 = 0;
    }
    m_time += m_deltaTime;
  }


  return frame_count;
}

bool isValid(const char* ssid, esp_bd_addr_t address, int rssi) {
  Serial.print("available SSID: ");
  Serial.println(ssid);
  if ((strcmp(bt_device1, ssid) == 0) || (strcmp(bt_device2, ssid) == 0))
    return true;
  else
    return false;
}

void mp3Task(void* vargs) {
  while (1) {
    if (!play_in_progress) {
nxt_file:  songFile = nextFile(&songFolder);
      if (isValidFile(&songFile) && !isDirectory(&songFile)) {
        songFile.entryIndex = 0;
        Serial.printf("currently playing=%s\n", fileName);
        fileTotalBytesRead = 0;
        buffered = 0;
        decoded = 0;
        to_read = BUFFER_SIZE;

        timeCnt = 0;
        scroll_cnt = 0;
        // initially decode one frame to retrive mp3 info.
        mp3Decode();

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

        play_in_progress = true;
      }
    }
    else
      mp3Decode();

  }

}



void setup() {
  Serial.begin(115200);
  initOled();
  IR_receiver_init(4);
  mp3dec_init(&mp3d);
  pinMode(LED, OUTPUT);
  if (!mySdFat_init())
  {
    Serial.println("SD init failed");
    while (1);
  }

  songFolder = pathExists("/mp3-songs");
  if (startCluster(&songFolder) == 0)
  {
    Serial.println("Invalid Path");
    while (1);
  }

  while (!isEndOfDir(&(songFile = nextFile(&songFolder))))
  {
    if (isValidFile(&songFile) && !isDirectory(&songFile) && isMp3File(fileName))
      songEntIndex[songIndex++] = songFolder.entryIndex - (fileLfnEntCnt(&songFile) + 1);
  }

  songFolder.entryIndex = songEntIndex[0];
  songsCount = songIndex;
  Serial.printf("Total mp3 files:%d\n", songsCount);
  songIndex = 0;

  a2dp_source.set_ssid_callback(isValid);
  a2dp_source.set_auto_reconnect(false);
  a2dp_source.start(get_data_frames);
  // a2dp_source.start({"Mi True Wireless EBs Basic_R", "DR-BT101"}, get_data_frames);
  a2dp_source.set_volume(75);
  dispSemaphore = xSemaphoreCreateBinary();
  queue = xQueueCreate(2304, sizeof (int16_t));
  xTaskCreatePinnedToCore(mp3Task, "mp3Task", 4096, NULL, 7, &mp3TaskHandle, 1);

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
      Volume -= 5;
      if (Volume <= 15)
        Volume = 15;
      a2dp_source.set_volume(Volume);
    }

    else if (ir_data.command == VOLUME_UP_CMD)
    {
      Volume += 5;
      if (Volume >= 100)
        Volume = 100;
      a2dp_source.set_volume(Volume);
    }

    else if (ir_data.command == PLAY_PAUSE_CMD)
    {
      if (paused)
      {

        paused = false;
      }

      else
      {

        paused = true;
      }
    }
    ir_data.updated = false;
  }
}

void loop() {

  mp3ReadIR();

  if (next)
  {
    if (shuffle)
    {
      songIndex = random(songsCount);
      songFolder.entryIndex = songEntIndex[songIndex];
    }
    else
    {
      songIndex++;
      if (songIndex == songsCount)
        songIndex = 0;
      songFolder.entryIndex = songEntIndex[songIndex];
    }

    fileClose(&songFile);
    next = false;
    play_in_progress = false;
  }

  if (prev)
  {
    songIndex--;

    if (songIndex > songsCount)
      songIndex = songsCount - 1;

    songFolder.entryIndex = songEntIndex[songIndex];

    fileClose(&songFile);
    prev = false;
    play_in_progress = false;
  }

  if (xSemaphoreTake(dispSemaphore, pdMS_TO_TICKS(1)) == pdPASS)
  {
    seconds = (decoded / info.hz);
    displayInfo();
    digitalWrite(3,1);
  }

}
