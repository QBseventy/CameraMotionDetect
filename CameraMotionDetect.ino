#define DEBUG false // flag to turn on/off debugging
#define debug_begin(...) do { if (DEBUG) { Serial.begin(__VA_ARGS__); while(!Serial); }} while (0)
#define debug_print(...) do { if (DEBUG) Serial.print(__VA_ARGS__); } while (0)
#define debug_println(...) do { if (DEBUG) Serial.println(__VA_ARGS__); } while (0)
#define debug_printf(...) do { if (DEBUG) Serial.printf(__VA_ARGS__); } while (0)

#include "soc/soc.h"          // Disable brownout problems
#include "soc/rtc_cntl_reg.h" // Disable brownout problems
#include "driver/rtc_io.h"
#include <SD_MMC.h>
// TODO #1 change dimensions for taking pictures
// TODO #2 remove eloquent libraries
// TODO #3 refaktoring code
// TODO #ULTIMO code clean up

#include <FS.h>
#include <SPIFFS.h>
#include <EEPROM.h>

#define EEPROM_SIZE 2 //number of bytes to access

#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#include <EloquentArduino.h>
#include <eloquentarduino/io/serial_print.h>
#include <eloquentarduino/vision/camera/ESP32Camera.h>
#include <eloquentarduino/vision/io/decoders/Red565RandomAccessDecoder.h>
#include <eloquentarduino/vision/processing/downscaling/Center.h>
#include <eloquentarduino/vision/processing/downscaling/Downscaler.h>
#include <eloquentarduino/vision/processing/MotionDetector.h>
#include <eloquentarduino/vision/io/writers/JpegWriter.h>

#define FRAME_SIZE FRAMESIZE_QVGA
#define PIXFORMAT PIXFORMAT_RGB565
#define W 320
#define H 240
#define w 32
#define h 24
#define DIFF_THRESHOLD 15
#define MOTION_THRESHOLD 0.15

// delete the second definition if you want to turn on code benchmarking
//#define timeit(label, code) { uint32_t start = millis(); code; uint32_t duration = millis() - start; debug_printf("It took %s millis for %s\n", duration, label); }
#define timeit(label, code) code;

short pictureNumber = 0;

using namespace Eloquent::Vision;

camera_fb_t *frame;
Camera::ESP32Camera camera(PIXFORMAT);
uint8_t downscaled[w * h];
//IO::Decoders::GrayscaleRandomAccessDecoder decoder;
IO::Decoders::Red565RandomAccessDecoder decoder;
Processing::Downscaling::Center < W / w, H / h > strategy;
Processing::Downscaling::Downscaler<W, H, w, h> downscaler(&decoder, &strategy);
Processing::MotionDetector<w, h> motion;
IO::Writers::JpegWriter<W, H> jpegWriter;

void capture();
void takePicture();
void save();
void stream_downscaled();
void stream();

void setup()
{
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
  debug_begin(115200);
  SPIFFS.begin(true);
  delay(1000);
  Serial.println("Begin");

  camera.begin(FRAME_SIZE);
  // set how much a pixel value should differ to be considered as a change
  motion.setDiffThreshold(DIFF_THRESHOLD);
  // set how many pixels (in percent) should change to be considered as motion
  motion.setMotionThreshold(MOTION_THRESHOLD);
  // prevent consecutive triggers
  motion.setDebounceFrames(5);
  pinMode(4, INPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_dis(GPIO_NUM_4);

  //Serial.println("Starting SD Card");

  //  SD_MMC.begin("/sdcard", true)

  if (!SD_MMC.begin("/sdcard", true))
  {
    Serial.println("SD Card Mount Failed");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card attached");
    return;
  }
}

void loop()
{
  capture();
  debug_printf("%d pixels changed\n", motion.changes());

  if (motion.triggered())
  {
    Serial.println("Motion detected");

    takePicture();

    // uncomment to save to disk
    save();

    // uncomment to stream to the Python script for visualization
    // stream();

    // uncomment to stream downscaled imaged tp Python script
    // stream_downscaled();

    delay(1000);
  }

  delay(30);
}

void capture()
{
  timeit("capture frame", frame = camera.capture());

  // scale image from size H * W to size h * w
  timeit("downscale", downscaler.downscale(frame->buf, downscaled));

  // detect motion on the downscaled image
  timeit("motion detection", motion.detect(downscaled));
}

void save_old()
{
  File imageFile = SPIFFS.open("/capture.jpg", "wb");
  uint8_t quality = 30;

  debug_printf("The image will be saved as /capture.jpg\n");
  jpegWriter.write(imageFile, frame->buf, PIXFORMAT, quality);
  imageFile.close();
  debug_printf("Saved\n");
}
void save()
{
  // initialize EEPROM with predefined size
  EEPROM.begin(EEPROM_SIZE);
  pictureNumber = EEPROM.readShort(0) + 1;

  // Path where new picture will be saved in SD Card
  String path = "/cmd#" + String(pictureNumber) + ".jpg";

  fs::FS &fs = SD_MMC;
  debug_printf("Picture file name: %s\n", path.c_str());

  File imageFile = fs.open(path.c_str(), FILE_WRITE);
  if (!imageFile)
  {
    debug_printf("Failed to open file in writing mode\n");
  }
  else
  {
    uint8_t quality = 30;
    debug_printf("Picture file name: %s\n", path.c_str());
    jpegWriter.write(imageFile, frame->buf, PIXFORMAT, quality);
    debug_printf("Saved file to path: %s\n", path.c_str());
    EEPROM.writeShort(0, pictureNumber);
    EEPROM.commit();
  }
  imageFile.close();

  debug_printf("Saved\n");
  // esp_camera_fb_return(fb);

  // Turns off the flash
  pinMode(4, OUTPUT);
  digitalWrite(4, LOW);
  rtc_gpio_hold_en(GPIO_NUM_4);
}
void takePicture() {
  //here I want to take a high quality picture
  //change camera config to high quality
  //take a picture
  //reset camera config for motion detect
}
void stream()
{
  debug_printf("START OF FRAME\n");

  jpegWriter.write(Serial, frame->buf, PIXFORMAT, 30);

  debug_printf("END OF FRAME\n");
}

void stream_downscaled()
{
  debug_printf("START OF DOWNSCALED\n");

  //eloquent::io::print_array(downscaled, w * h);

  debug_printf("END OF DOWNSCALED\n");
}
