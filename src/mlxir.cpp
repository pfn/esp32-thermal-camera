#include <Arduino.h>

#include <TFT_eSPI.h>
#include <SPI.h>
#include <WiFiManager.h>
#include <Wire.h>
#include <vector>

#include "MLX90640_I2C_Driver.h"
#include "MLX90640_API.h"
#include "Font.h"
#include "bilinear.h"

#define BUTTON_U 23
#define BUTTON_D 19
#define BUTTON_L 18
#define BUTTON_R 5
#define BUTTON_C 17
#define BUTTON_A 16
#define BUTTON_B 4

#define FRAME_FREQ   16
#define REFRESH_4HZ  0x03
#define REFRESH_8HZ  0x04
#define REFRESH_16HZ 0x05
#define REFRESH_32HZ 0x06
#define REFRESH_64HZ 0x07
#define _REFRESH_RATE(x) REFRESH_ ## x ## HZ
#define __REFRESH_RATE(x) _REFRESH_RATE(x)
#define REFRESH_RATE __REFRESH_RATE(FRAME_FREQ)

#define RESOLUTION_18BIT 0x02
#define RESOLUTION_19BIT 0x03

#define SCALE_SIZE 256
#define SCALE_MAX (SCALE_SIZE - 1)

#define MLX_ADDR 0x33
#define TA_SHIFT 8 // ambient temperature adjustment

#define SENSOR_PIXELS 768

static float mlx90640To[SENSOR_PIXELS];

std::vector<uint16_t> selected_reticles;
volatile uint16_t reticle = SENSOR_PIXELS / 2 - 16;
volatile uint8_t emissivity = 95;
volatile bool freeze_frame = false;

TFT_eSPI tft = TFT_eSPI();
TFT_eSprite canvas16 = TFT_eSprite(&tft);
TFT_eSprite bottom_canvas = TFT_eSprite(&tft);
TFT_eSprite top_scale_canvas = TFT_eSprite(&tft);
TFT_eSprite bot_scale_canvas = TFT_eSprite(&tft);
paramsMLX90640 mlx90640;

uint16_t pm3d_scale[SCALE_SIZE];

TaskHandle_t render_task;
TaskHandle_t framegrabber_task;
TaskHandle_t processor_task;
SemaphoreHandle_t render_sync;
StaticSemaphore_t render_sync_buffer;
SemaphoreHandle_t frame_sync;
StaticSemaphore_t frame_sync_buffer;

volatile float tr;
volatile uint32_t frame_grabber_time = 0;
volatile bool frame_error = false;

#define TEMP_RANGES 5
volatile bool is_F;
volatile uint8_t temp_range = 0;
const float ranges[][2] = {
  {20.0, 32.22},
  {10.0, 32.22},
  {20.0, 50.0}, {-10.0, 200.0}, {0.0, 0.0},
};

void frame_grabber(void *arg) {
  static uint32_t last = millis();
  uint16_t mlx90640Frame[834] = {0};
  int status;

  const TickType_t xFrequency = pdMS_TO_TICKS(1000 / FRAME_FREQ);
  MLX90640_SynchFrame(MLX_ADDR);
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for (;;) {
    uint32_t now = millis();

    if (xSemaphoreTake(frame_sync, xFrequency)) {
      for (int i = 0; i < 2; ++i) {
        status = MLX90640_GetFrameData(MLX_ADDR, mlx90640Frame);
        if (status < 0) {
          i--;
          frame_error = true;
          continue;
        }
        float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
  
        tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature

        float e = emissivity / 100.0f;
        MLX90640_CalculateTo(mlx90640Frame, &mlx90640, e, tr, mlx90640To);
        if (i == 0)
          vTaskDelayUntil(&xLastWakeTime, xFrequency);
      }
      frame_error = false;
      xSemaphoreGive(frame_sync);
    } else {
      Serial.println("frame_sync semaphore timeout");
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

    frame_grabber_time = now - last;
    last = now;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

inline float display_unit(const float c) {
  return is_F ? (c * 9 / 5) + 32 : c;
}

inline bool is_autorange() {
  return ranges[temp_range][0] == ranges[temp_range][1];
}

inline float range_min(float c) {
  return is_autorange() ? c : ranges[temp_range][0];
}

inline float range_max(float c) {
  return is_autorange() ? c : ranges[temp_range][1];
}

inline float clamp(float c, float min_temp, float max_temp) {
  return max(range_min(min_temp), min(range_max(max_temp), c));
}

inline void analyze_px(float temp, uint16_t px, float &min_temp, float &max_temp,
  uint16_t &min_pt, uint16_t &max_pt) {
  if (temp < min_temp) {
    min_pt = px;
    min_temp = temp;
  }
  if (temp > max_temp) {
    max_pt = px;
    max_temp = temp;
  }
}

const char spinner[] = "-/|\\";
float average(float *buf, uint8_t size) {
  float sum = 0.0;
  uint8_t i;
  for (i = 0; i < size; ++i) {
    if (isnan(buf[i])) break;
    sum += buf[i];
  }
  return sum / i;
}

void process_frames(void *arg) {
  const uint8_t window_sz = FRAME_FREQ / 2;
  uint8_t window_i = 0;
  float min_temp_window[window_sz] = {NAN};
  float max_temp_window[window_sz] = {NAN};
  float reticle_windows[5][window_sz] = {{NAN}};
  TickType_t xFrequency = pdMS_TO_TICKS(1000 / 24);
  uint8_t spinner_counter = 0;
  uint32_t last = millis();
  TickType_t xLastWakeTime = xTaskGetTickCount();
  bool first_frame = true;

  float frame[SENSOR_PIXELS];
  float e = emissivity / 100.0f;
  for (;;) {
    uint32_t now = millis();
    bool new_frame = false;
  
    uint16_t min_pt, max_pt;
    float min_temp = 1000.0f, max_temp = -273.1f;
    if (xSemaphoreTake(frame_sync, pdMS_TO_TICKS(1000 / FRAME_FREQ) * 2)) {
      if (!freeze_frame)
        memcpy(frame, mlx90640To, sizeof(float) * SENSOR_PIXELS);
      e = emissivity / 100.0f;
      xSemaphoreGive(frame_sync);
      if (first_frame) goto process_cleanup;
      new_frame = true;
    } else {
      bottom_canvas.fillRect(0, 88, 240, 8, 0);
      bottom_canvas.setCursor(0, 88);
      bottom_canvas.printf("%c %-3.1f %-3.1f %s",
        spinner[spinner_counter = (spinner_counter + 1) % 4], 1000.0 / (now - last), 1000.0 / frame_grabber_time,
        frame_error ? "???" : "***");
      bottom_canvas.pushSprite(0, 6 * 24);
      goto process_cleanup;
    }

    if (new_frame || freeze_frame) {
      uint16_t frame_colors[SENSOR_PIXELS];
      for (uint16_t i = 0; i < SENSOR_PIXELS; ++i) {
        analyze_px(frame[i], i, min_temp, max_temp, min_pt, max_pt);
      }
      for (uint16_t x = 0; x < 32; ++x) {
        for (uint16_t y = 0; y < 24; ++y) {
          frame_colors[y * 32 + (31 - x)] = pm3d_scale[map(
              clamp(frame[y * 32 + x], range_min(min_temp), range_max(max_temp)) * 10.0f,
            range_min(min_temp) * 10.0f, range_max(max_temp) * 10.0f, 0, SCALE_MAX)];
        }
      }
      bilinear_scale(frame_colors, 32, 24, (uint16_t *) canvas16.getPointer(), 32 * 6, 24 * 6);

      min_temp_window[window_i] = min_temp;
      max_temp_window[window_i] = max_temp;
      reticle_windows[0][window_i] = frame[reticle];
      for (uint8_t i = 1; i < 5; ++i) {
        reticle_windows[i][window_i] = (i <= selected_reticles.size()) ? frame[selected_reticles[i - 1]] : NAN;
      }
      window_i = (window_i + 1) % window_sz;

      min_temp = average(min_temp_window, window_sz);
      max_temp = average(max_temp_window, window_sz);

      bottom_canvas.fillScreen(0);
      bottom_canvas.setFreeFont(&_FreeSans9pt7b);
      bottom_canvas.setCursor(0, 20);
      bottom_canvas.printf("| = %.1f~", display_unit(average(reticle_windows[0], window_sz)));
      bottom_canvas.setCursor(90, 20);
      bottom_canvas.printf("Ta = %.1f~", display_unit(tr));
      bottom_canvas.setCursor(190, 20);
      bottom_canvas.printf("~ = %c", is_F ? 'F' : 'C');
      bottom_canvas.setCursor(0, 40);
      "{"; // noop because of the following printf to fix brace matching for IDE
      bottom_canvas.printf("} = %.2f", e);
      bottom_canvas.setCursor(84, 40);
      bottom_canvas.printf("%.1f~ < T < %.1f~\n", display_unit(min_temp), display_unit(max_temp));
      if (freeze_frame) {
        bottom_canvas.setFreeFont();
        bottom_canvas.drawChar('*', 235, 88);
      }

      const uint8_t selected_locations[][2] = {
        {0, 60}, {120, 60}, {0, 80}, {120, 80}
      };
      uint8_t reticle_x = reticle % 32;
      uint8_t reticle_y = reticle / 32;
      canvas16.drawRect((31 - reticle_x) * 6 - 1, reticle_y * 6 - 1, 8, 8, TFT_BLACK);
      canvas16.drawRect((31 - reticle_x) * 6,     reticle_y * 6,     6, 6, TFT_WHITE);
      canvas16.drawRect((31 - reticle_x) * 6 + 1, reticle_y * 6 + 1, 4, 4, TFT_BLACK);
      for (uint8_t i = 0; i < selected_reticles.size(); ++i) {
        uint8_t x = selected_reticles[i] % 32;
        uint8_t y = selected_reticles[i] / 32;
        canvas16.drawRect((31 - x) * 6 - 1, y * 6 - 1, 8, 8, TFT_BLACK);
        canvas16.drawRect((31 - x) * 6,     y * 6,     6, 6, TFT_WHITE);
        canvas16.drawRect((31 - x) * 6 + 1, y * 6 + 1, 4, 4, TFT_BLACK);
        canvas16.drawChar(49 + i, (31 - x) * 6 + 7, y * 6 + 3);
        bottom_canvas.setFreeFont(&_FreeSans9pt7b);
        uint8_t tx = selected_locations[i][0];
        uint8_t ty = selected_locations[i][1];
        bottom_canvas.setCursor(tx, ty);
        bottom_canvas.printf("| = %.1f~", display_unit(average(reticle_windows[i + 1], window_sz)));
        bottom_canvas.setFreeFont();
        bottom_canvas.drawChar(49 + i, tx + 10, ty - 2);
      }
  
      top_scale_canvas.fillScreen(0);
      top_scale_canvas.setCursor(0, 14);
      top_scale_canvas.printf("%.1f~", display_unit(range_max(max_temp)));

      bot_scale_canvas.fillScreen(0);
      bot_scale_canvas.setCursor(0, 14);
      bot_scale_canvas.printf("%.1f~", display_unit(range_min(min_temp)));
    }
  
    xTaskNotify(render_task, 0, eSetBits);

  process_cleanup:
    first_frame = false;
    last = now;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void render_frames(void *arg) {
  TickType_t xFrequency = pdMS_TO_TICKS(1000 / 24);
  uint8_t spinner_counter = 0;
  uint32_t last = millis();

  for (int i = 0; i < 104; ++i) {
    tft.drawLine(8, i + 20, 24, i + 20, pm3d_scale[map(103 - i, 0, 103, 0, SCALE_MAX)]);
  }

  TickType_t xLastWakeTime = xTaskGetTickCount();
  uint32_t value;
  for (;;) {
    uint32_t now = millis();
    if (xTaskNotifyWait(0, 0, &value, xFrequency)) {
      canvas16.pushSprite(240 - 6 * 32, 0);

      bottom_canvas.setFreeFont();
      bottom_canvas.setCursor(0, 88);
      if (now - last > 0 && frame_grabber_time > 0)
        bottom_canvas.printf("%c %-3.1f %-3.1f %s",
          spinner[spinner_counter = (spinner_counter + 1) % 4], 1000.0 / (now - last), 1000.0 / frame_grabber_time,
          frame_error ? "???" : "");

      bottom_canvas.pushSprite(0, 6 * 24);
      top_scale_canvas.pushSprite(0, 0);
      bot_scale_canvas.pushSprite(0, 124);
  
    }
    last = now;
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

#define RGB888_TO_565(r, g, b) ((r & 0xf8) << 8) | ((g & 0xfc) << 3) | (b >> 3)

inline void pm3d(uint16_t *scale, size_t steps) {
  uint8_t r;
  uint8_t g;
  uint8_t b;

  for (int i = 0; i < steps; ++i) {
    float j = 1.0f * i / steps;
    r = round(255 * sqrt(j));
    g = round(255 * pow(j, 3));
    b = round(255 * max(sin(2 * PI * j), 0.0));
    scale[i] = RGB888_TO_565(r, g, b);
  }
}

bool is_connected() {
  Wire.beginTransmission(MLX_ADDR);
  return Wire.endTransmission() == 0;
}

#define DEBOUNCE_TIME 100
void IRAM_ATTR button_handler(void *arg) {
  static bool chord_A = false;
  static bool chord_B = false;
  static uint32_t last = 0;
  if (millis() - last < DEBOUNCE_TIME) {
    return;
  }
  last = millis();
  char c = ((char *)arg)[0];
  switch (c) {
    case 'U':
      if (!digitalRead(BUTTON_B)) {
        chord_B = true;
        emissivity = min(emissivity + 5, 100);
      } else {
        reticle = max(reticle - 32, 0);
      }
      break;
    case 'D':
      if (!digitalRead(BUTTON_B)) {
        chord_B = true;
        emissivity = max(emissivity - 5, 5);
      } else {
        reticle = min(reticle + 32, SENSOR_PIXELS);
      }
      break;
    case 'L':
      if (!digitalRead(BUTTON_B)) {
        chord_B = true;
        temp_range = max(temp_range - 1, 0);
      } else {
        reticle = min(reticle + 1, SENSOR_PIXELS);
      }
      break;
    case 'R':
      if (!digitalRead(BUTTON_B)) {
        chord_B = true;
        temp_range = min(temp_range + 1, TEMP_RANGES - 1);
      } else {
        reticle = max(reticle - 1, 0);
      }
      break;
    case 'A':
      if (!chord_A) {
        if (!digitalRead(BUTTON_B)) {
          tft.fillScreen(0);
          ESP.restart();
        } else {
          is_F = !is_F;
        }
      } else {
        chord_A = false;
      }
      break;
    case 'B':
      if (!chord_B) {
        if (!digitalRead(BUTTON_A)) {
          // A + B is open
          chord_A = true;
        } else {
          freeze_frame = !freeze_frame;
        }
      } else {
        chord_B = false;
      }
      break;
    case 'C':
      if (!digitalRead(BUTTON_A)) {
        selected_reticles.clear();
        chord_A = true;
      } else if (!digitalRead(BUTTON_B)) {
        chord_B = true;
      } else {
        uint16_t ret = reticle;
        auto r = std::find(selected_reticles.cbegin(), selected_reticles.cend(), ret);
        if (r != std::end(selected_reticles)) {
          selected_reticles.erase(r);
        } else {
          selected_reticles.push_back(ret);
        }
        while (selected_reticles.size() > 4)
          selected_reticles.erase(selected_reticles.begin());
      }
      break;
  }
}

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH);
  pinMode(BUTTON_U, INPUT_PULLUP);
  pinMode(BUTTON_D, INPUT_PULLUP);
  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(BUTTON_R, INPUT_PULLUP);
  pinMode(BUTTON_A, INPUT_PULLUP);
  pinMode(BUTTON_B, INPUT_PULLUP);
  pinMode(BUTTON_C, INPUT_PULLUP);
  attachInterruptArg(BUTTON_U, button_handler, (void *)"U", RISING);
  attachInterruptArg(BUTTON_D, button_handler, (void *)"D", RISING);
  attachInterruptArg(BUTTON_L, button_handler, (void *)"L", RISING);
  attachInterruptArg(BUTTON_R, button_handler, (void *)"R", RISING);
  attachInterruptArg(BUTTON_A, button_handler, (void *)"A", RISING);
  attachInterruptArg(BUTTON_B, button_handler, (void *)"B", RISING);
  attachInterruptArg(BUTTON_C, button_handler, (void *)"C", RISING);

  tft.setRotation(0);
  canvas16.setTextColor(0xffff);
  canvas16.setColorDepth(16);
  canvas16.createSprite(6 * 32, 6 * 24);
  bottom_canvas.setColorDepth(1);
  bottom_canvas.createSprite(240, 96);
  top_scale_canvas.setColorDepth(1);
  top_scale_canvas.createSprite(48, 20);
  bot_scale_canvas.setColorDepth(1);
  bot_scale_canvas.createSprite(48, 20);
  pm3d(pm3d_scale, SCALE_SIZE);
  Serial.begin(115200);
  Serial.println("Looking for MLX");

  Wire.begin();
  Wire.setClock(800000);
  if (is_connected()) Serial.println("MLX90640 found");
  int status;
  uint16_t eeMLX90640[832];
  MLX90640_I2CInit();
  status = MLX90640_DumpEE((uint8_t)MLX_ADDR, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");

  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.printf("Parameter extraction failed %d\n", status);

  MLX90640_SetRefreshRate(MLX_ADDR, REFRESH_RATE);
  MLX90640_SetResolution(MLX_ADDR, RESOLUTION_19BIT);
  MLX90640_SetChessMode(MLX_ADDR);

  tft.init();
  tft.fillScreen(TFT_BLACK);

  top_scale_canvas.setFreeFont(&_FreeSans9pt7b);
  bot_scale_canvas.setFreeFont(&_FreeSans9pt7b);
  frame_sync = xSemaphoreCreateMutexStatic(&frame_sync_buffer);
  render_sync = xSemaphoreCreateMutexStatic(&render_sync_buffer);
  xSemaphoreGive(frame_sync);
  xSemaphoreGive(render_sync);
  xTaskCreatePinnedToCore(frame_grabber, "FrameGrabber", 3500, NULL, 2, &framegrabber_task, 1);
  xTaskCreatePinnedToCore(process_frames, "FrameCalculator", 7000, NULL, 2, &processor_task, 1);
  xTaskCreatePinnedToCore(render_frames, "RenderTask", 4000, NULL, 2, &render_task, 1);
}

void loop() {
  vTaskDelete(NULL);
}
