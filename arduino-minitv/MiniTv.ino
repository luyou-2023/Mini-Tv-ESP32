/***
 * Required libraries:
 * https://github.com/moononournation/Arduino_GFX.git
 * https://github.com/pschatzmann/arduino-libhelix.git
 * https://github.com/bitbank2/JPEGDEC.git
 */

//  Audio and video code

// 使用 FFmpeg 命令行工具进行音视频文件转换
// 音频转换命令，输出AAC格式，设置采样率、声道数、比特率，并进行音量调整。
// ffmpeg -i  office1.mp4 -ar 44100 -ac 1 -ab 24k -filter:a loudnorm -filter:a "volume=-5dB" office1.aac
// 视频转换命令，调整视频帧率、分辨率、裁剪区域，并设置压缩质量。
// ffmpeg -i office1.mp4 -vf "fps=25,scale=-1:240:flags=lanczos,crop=288:in_h:(in_w-288)/2:0" -q:v 11 office1.mjpeg

// 如果AAC文件不可用，自动回退到MP3文件。
#define AAC_FILENAME "/<FILENAME>.aac"  // 音频AAC文件路径
#define MP3_FILENAME "/<FILENAME>.mp3"  // 音频MP3文件路径
#define MJPEG_FILENAME "/<FILENAME>.mjpeg"  // 视频MJPEG文件路径

// 帧率设置为25fps
#define FPS 25
// MJPEG视频缓存大小，按帧宽高计算
#define MJPEG_BUFFER_SIZE (288 * 240 * 2 / 8)
// 定义不同核心的任务分配
#define AUDIOASSIGNCORE 1
#define DECODEASSIGNCORE 0
#define DRAWASSIGNCORE 1

// 包含必要的库和文件系统支持
#include <WiFi.h>
#include <FS.h>
#include <LittleFS.h>
#include <SPIFFS.h>
#include <FFat.h>
#include <SD.h>
#include <SD_MMC.h>

/**
Arduino_GFX_Library.h 是一个用于在不同显示屏（如LCD、OLED、TFT等）上进行图形显示的库，尤其适用于 Arduino 和 ESP32 平台。在你的代码中，它被用来控制和操作显示器，特别是用于控制 LCD 屏幕的显示。

主要功能：
显示文本：可以在屏幕上显示文本，如 gfx->println("Init I2S");。
绘制图形：支持绘制线条、矩形、圆形等基本图形，以及显示图像或位图。
颜色支持：能够设置前景色、背景色等，例如 fillScreen(BLACK)，这表示用黑色填充整个屏幕。
屏幕初始化：gfx->begin() 函数初始化显示器并设置一些基本参数，如分辨率、时钟频率等。
与硬件的关系：
Arduino_GFX_Library.h 实际上是一个硬件抽象层的库，支持多种不同的显示硬件。在你的代码中，它被用来控制一个特定的 LCD 显示屏，通常是通过 SPI 总线与 ESP32 进行通信。Arduino_ST7789 是该库的一部分，专门用于控制 ST7789 型号的显示器，这种显示器通常采用 SPI 接口。

ESP32 与 LCD 的控制：
I2S 和 SPI：在 ESP32 上，你可以使用 I2S（用于音频和数字信号处理）和 SPI（用于显示设备、传感器等）来与外部设备通信。Arduino_GFX_Library 主要是通过 SPI 接口来控制 LCD 屏幕。
显示控制：通过这个库，你能够发送绘图命令、文本信息到 LCD 屏幕，并能够通过图形绘制和颜色设置让屏幕显示你需要的内容。
在你的代码中，ESP32 控制 LCD 屏幕的流程大致如下：

使用 Arduino_GFX_Library 初始化显示器，设置屏幕分辨率。
利用显示库提供的 fillScreen()、draw16bitRGBBitmap() 等函数进行图形显示操作。
在屏幕上显示文本信息和视频图像等。
总的来说，Arduino_GFX_Library.h 提供了一种方便的方式来控制 ESP32 与各种 LCD 屏幕的交互，而不需要直接操作硬件细节，封装了大量底层的操作。
**/
// 包含显示和图形库
#include <Arduino_GFX_Library.h>
#define GFX_BL DF_GFX_BL // 默认背光引脚定义

// 初始化Arduino数据总线和显示库
Arduino_DataBus *bus = create_default_Arduino_DataBus();
// 使用ST7789显示器，并设置分辨率为240x288，启用IPS显示模式。
Arduino_GFX *gfx = new Arduino_ST7789(bus, DF_GFX_RST, 1 /* rotation */, true /* IPS */, 240 /* width */, 288 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 12 /* row offset 2 */);

/* variables */
static int next_frame = 0;  // 下一帧索引
static int skipped_frames = 0;  // 跳过的帧数
static unsigned long start_ms, curr_ms, next_frame_ms;  // 时间变量用于计算视频帧的显示时间

/* audio */
#include "esp32_audio_task.h"  // 包含音频任务处理文件

/* MJPEG Video */
#include "mjpeg_decode_draw_task.h"  // 包含MJPEG视频解码与显示文件

// 像素绘制回调函数
static int drawMCU(JPEGDRAW *pDraw)
{
  unsigned long s = millis();  // 获取当前时间
  // 使用显示库绘制像素数据到屏幕
  gfx->draw16bitRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  total_show_video_ms += millis() - s;  // 记录视频绘制时间
  return 1;
} /* drawMCU() */

// setup函数，用于初始化硬件和任务
void setup()
{
  disableCore0WDT();  // 禁用核心0的看门狗定时器

  WiFi.mode(WIFI_OFF);  // 关闭WiFi
  Serial.begin(115200);  // 启动串口通信
  // while (!Serial);  // 等待串口连接

  // 初始化显示器并填充为黑色
  gfx->begin(80000000);  // 初始化显示器，设置时钟速度
  gfx->fillScreen(BLACK);  // 填充黑色背景

  // 如果定义了背光引脚（GFX_BL），则开启背光
#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);  // 设置背光引脚为输出模式
  digitalWrite(GFX_BL, HIGH);  // 打开背光
#endif
  // 在LCD上显示初始化信息
  Serial.println("Init I2S");
  gfx->println("Init I2S");

  // 初始化I2S接口，设置音频输入输出的引脚和采样率
  #if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32)
    esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, 25 /* SCLK */, 26 /* LRCK */, 32 /* DOUT */, -1 /* DIN */);
  #elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S2)
    esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, 4 /* SCLK */, 5 /* LRCK */, 18 /* DOUT */, -1 /* DIN */);
  #elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)
    esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, 42 /* MCLK */, 46 /* SCLK */, 45 /* LRCK */, 43 /* DOUT */, 44 /* DIN */);
  #elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32C3)
    esp_err_t ret_val = i2s_init(I2S_NUM_0, 44100, -1 /* MCLK */, 10 /* SCLK */, 19 /* LRCK */, 18 /* DOUT */, -1 /* DIN */);
  #endif

  if (ret_val != ESP_OK)
  {
    Serial.printf("i2s_init failed: %d\n", ret_val);
  }
  i2s_zero_dma_buffer(I2S_NUM_0);  // 清空I2S的DMA缓冲区

  // 初始化文件系统，设置为SD卡
  Serial.println("Init FS");
  gfx->println("Init FS");
  SPIClass spi = SPIClass(HSPI);
  spi.begin(14 /* SCK */, 4 /* MISO */, 15 /* MOSI */, 13 /* CS */);
  if (!SD.begin(13, spi, 80000000))  // 初始化SD卡
  {
    Serial.println("ERROR: File system mount failed!");  // 如果初始化失败，输出错误信息
    gfx->println("ERROR: File system mount failed!");
  }
  else
  {
    bool aac_file_available = false;
    Serial.println("Open AAC file: " AAC_FILENAME);  // 打开AAC文件
    gfx->println("Open AAC file: " AAC_FILENAME);
    File aFile = SD.open(AAC_FILENAME);  // 尝试从SD卡中打开AAC文件
    if (aFile)  // 如果文件存在
    {
      aac_file_available = true;
    }
    else  // 如果AAC文件不存在，尝试打开MP3文件
    {
      Serial.println("Open MP3 file: " MP3_FILENAME);
      gfx->println("Open MP3 file: " MP3_FILENAME);
      aFile = SD.open(MP3_FILENAME);  // 打开MP3文件
    }

    if (!aFile || aFile.isDirectory())  // 如果打开失败，输出错误信息
    {
      Serial.println("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
      gfx->println("ERROR: Failed to open " AAC_FILENAME " or " MP3_FILENAME " file for reading");
    }
    else  // 如果音频文件成功打开，继续处理
    {
      Serial.println("Open MJPEG file: " MJPEG_FILENAME);
      gfx->println("Open MJPEG file: " MJPEG_FILENAME);
      File vFile = SD.open(MJPEG_FILENAME);  // 打开视频MJPEG文件
      if (!vFile || vFile.isDirectory())  // 如果视频文件不存在，输出错误信息
      {
        Serial.println("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
        gfx->println("ERROR: Failed to open " MJPEG_FILENAME " file for reading");
      }
      else
      {
        Serial.println("Init video");
        gfx->println("Init video");
        mjpeg_setup(&vFile, MJPEG_BUFFER_SIZE, drawMCU, false /* useBigEndian */, DECODEASSIGNCORE, DRAWASSIGNCORE);

        Serial.println("Start play audio task");
        gfx->println("Start play audio task");
        BaseType_t ret_val;
        if (aac_file_available)
        {
          ret_val = aac_player_task_start(&aFile, AUDIOASSIGNCORE);  // 启动AAC音频播放任务
        }
        else
        {
          ret_val = mp3_player_task_start(&aFile, AUDIOASSIGNCORE);  // 启动MP3音频播放任务
        }

        if (ret_val == pdPASS)
        {
          Serial.println("Start video playback");
          gfx->println("Start video playback");

          // 播放MJPEG视频
          mjpeg_play(FPS, &vFile);

          vFile.close();  // 关闭视频文件
        }
        else
        {
          Serial.println("Error in starting audio task");
          gfx->println("Error in starting audio task");
        }
      }
    }
  }
}
