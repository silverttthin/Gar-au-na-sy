//=========================== header ==========================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>

#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"

#include "driver/gpio.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/micro/system_setup.h"
#include "tensorflow/lite/schema/schema_generated.h"

#include "mfcc.h"
#include "model_manager.h"
#include "common_data.h"

#include "model.h"

#include "app_uart.h"

#include <esp_heap_caps.h>

//=========================== Settings ============================
// [사용자 설정]
// 0번 인덱스만 감지하고 싶음 (1번, 2번 등 나머지는 무시)
#define TARGET_CLASS_INDEX    0       // 감지할 타겟 인덱스 (0번)
#define CONFIDENCE_THRESHOLD  0.68f   // 70% 이상 확실할 때만 전송

//=========================== variables ===========================

uint8_t *tensor_arena = NULL;

const tflite::Model *model = nullptr;
tflite::MicroInterpreter *interpreter = nullptr;
TfLiteTensor *model_input = nullptr;
TfLiteTensor *model_output = nullptr;

const int kTensorArenaSize = 100 * 1024;

float quant_scale_for_input;
int32_t quant_zero_point_for_input;
float quant_scale_for_output;
int32_t quant_zero_point_for_output;

uint8_t speech_data_index = 0;

uint8_t max_index[1] = {0};
float max_value = 0;

int32_t data_head = 0;

int16_t *audio_data;
int16_t *inf_buf;

uint8_t led_data = 0;

QueueHandle_t xQueueSensorData = NULL;
send_data_t received_sensor_data;
//=========================== prototypes ==========================

//=========================== public ==============================
void model_setup()
{
  ESP_LOGI(MODEL_MANAGER_TAG, "model setup start");

  tensor_arena = (uint8_t *)heap_caps_malloc(kTensorArenaSize, MALLOC_CAP_SPIRAM);
  audio_data = (int16_t *)heap_caps_malloc(16000 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  inf_buf  = (int16_t *)heap_caps_malloc(16000 * sizeof(int16_t), MALLOC_CAP_SPIRAM);
  
  xQueueSensorData = xQueueCreate(10, sizeof(send_data_t));

  model = tflite::GetModel(g_model);

  if (model->version() != TFLITE_SCHEMA_VERSION)
  {
    MicroPrintf("Model schema mismatch!");
    return;
  }

  static tflite::MicroMutableOpResolver<11> micro_op_resolver;
  if (micro_op_resolver.AddFullyConnected() != kTfLiteOk) return;
  if (micro_op_resolver.AddRelu() != kTfLiteOk) return;
  if (micro_op_resolver.AddSoftmax() != kTfLiteOk) return;
  if (micro_op_resolver.AddConv2D() != kTfLiteOk) return;
  if (micro_op_resolver.AddDepthwiseConv2D() != kTfLiteOk) return;
  if (micro_op_resolver.AddMaxPool2D() != kTfLiteOk) return;
  if (micro_op_resolver.AddReshape() != kTfLiteOk) return;
  if (micro_op_resolver.AddQuantize() != kTfLiteOk) return;
  if (micro_op_resolver.AddDequantize() != kTfLiteOk) return;
  if (micro_op_resolver.AddMul() != kTfLiteOk) return;
  if (micro_op_resolver.AddAdd() != kTfLiteOk) return;

  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize);
  interpreter = &static_interpreter;

  TfLiteStatus allocate_status = interpreter->AllocateTensors();
  if (allocate_status != kTfLiteOk)
  {
    MicroPrintf("AllocateTensors() failed");
    return;
  }

  model_input = interpreter->input(0);
  quant_scale_for_input = model_input->params.scale;
  quant_zero_point_for_input = model_input->params.zero_point;

  model_output = interpreter->output(0);
  quant_scale_for_output = model_output->params.scale;
  quant_zero_point_for_output = model_output->params.zero_point;

  ESP_LOGI(MODEL_MANAGER_TAG, "Complete model setup!");
}

QueueHandle_t get_sensor_data_queue()
{
  return xQueueSensorData;
}

//=========================== tasks ===============================
void model_inference_task(void *arg)
{
  int64_t time_start, time_end;
  mfcc_init();
  ESP_LOGI(MODEL_MANAGER_TAG, "model_inference_task start!");

  while (1)
  {
    if (xQueueReceive(xQueueSensorData, &received_sensor_data, portMAX_DELAY))
    {
      switch (received_sensor_data.type)
      {
      case SPEECH_DATA:
      {
        // Step 1. 오디오 샘플 포맷 변환
        for (uint16_t i = 0; i < 400; i += 2)
        {
          uint8_t byte1 = received_sensor_data.speech_data.buf[i];
          uint8_t byte2 = received_sensor_data.speech_data.buf[i + 1];
          audio_data[speech_data_index * 200 + i / 2] = (byte2 << 8) | byte1;
        }
        speech_data_index++;

        // Step 2. 버퍼 관리
        if (speech_data_index == 80) speech_data_index = 0;

        // Step 3. 추론 수행 (125ms 주기)
        if (speech_data_index % 10 == 0)
        {
          time_start = esp_timer_get_time();

          // 버퍼 복사
          int remain_size = 80 - speech_data_index;
          memcpy(&inf_buf[0], &audio_data[speech_data_index*200], remain_size * 200 * sizeof(int16_t));
          memcpy(&inf_buf[remain_size*200], &audio_data[0], speech_data_index * 200 * sizeof(int16_t));

          // 전처리 버퍼 준비
          data_head = 0;
          float *freq_data = (float *)heap_caps_malloc(NUM_FRAMES * NUM_FBANK_BINS * sizeof(float), MALLOC_CAP_SPIRAM);
          memset(freq_data, 0, sizeof(float) * (NUM_FRAMES * NUM_FBANK_BINS));
          if (freq_data == NULL) {
            ESP_LOGE(MODEL_MANAGER_TAG, "freq malloc failed");
          }
          
          int16_t data_max = 0;
          for (int i = 0; i < 16000; i++){
            if (data_max < inf_buf[i]) data_max = inf_buf[i];
          }

          // MFCC 변환
          for (uint8_t i = 0; i < 49; i++)
          {
            mfcc_compute(inf_buf + (i * FRAME_SHIFT), &freq_data[data_head], data_max, 1);
            data_head += NUM_FBANK_BINS;
          }

          // Input 복사
          for (int16_t i = 0; i < model_input->bytes / 4; i++)
          {
            model_input->data.f[i] = freq_data[i];
          }
          heap_caps_free(freq_data);

          // 추론 실행
          if (interpreter->Invoke() != kTfLiteOk)
          {
            ESP_LOGE(MODEL_MANAGER_TAG, "Invoke failed!");
            while (1);
          }

          time_end = esp_timer_get_time();

          // -----------------------------------------------------------------
          // 결과 확인 및 로직 처리 (0번 인덱스 필터링)
          // -----------------------------------------------------------------
          
          max_index[0] = 0;
          max_value = 0;

          // 1. 전체 클래스 중 가장 높은 확률 찾기 (Winner 찾기)
          // 디버깅용: 터미널에는 현재 상태를 계속 출력
          // printf("DEBUG: ");
          for (int i = 0; i < model_output->bytes / 4; i++)
          {
            float _value = model_output->data.f[i];
            
            if (_value > max_value)
            {
              max_value = _value;
              max_index[0] = i;
            }
            printf("[%d]: %.2f  ", i, _value);
          }
          printf(" | Time: %.3fms\n", (time_end - time_start)/1000.f);

          // 2. [조건 검사] 
          // (1) Winner가 0번 인덱스인가? (TARGET_CLASS_INDEX)
          // (2) 확률이 70% 이상인가? (CONFIDENCE_THRESHOLD)
          
          if ((max_index[0] == TARGET_CLASS_INDEX) && (max_value >= CONFIDENCE_THRESHOLD))
          {
              // 조건 충족: 0번 명령어 전송
              printUART("Winner [%d]\n\r", max_index[0]);

              // LED 깜빡임 (전송 확인용)
              led_data = !led_data;
              gpio_set_level((gpio_num_t)GPIO_LED_BLUE, led_data);
              
              ESP_LOGI(MODEL_MANAGER_TAG, ">>> Command Sent: Index 0 detected (Score: %.2f)", max_value);
          }
          else 
          {
              // 1번이나 다른게 1등이거나, 0번이어도 확률이 낮으면 -> 무시(Silent)
          }
          // -----------------------------------------------------------------
        }
        break;
      }
      default:
        ESP_LOGE(MODEL_MANAGER_TAG, "data type error!");
        break;
      }
    }
  }
}
