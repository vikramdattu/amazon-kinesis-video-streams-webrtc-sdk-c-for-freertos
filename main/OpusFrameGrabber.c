/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <freertos/freeRTOS.h>
#include <freertos/task.h>

#include "driver/i2s.h"
#include "esp_log.h"

#include "esp_opus_enc.h"
#include "esp_audio_enc_def.h"
#include "esp_audio_def.h"

#include "allocators.h"
#include "OpusFrameGrabber.h"

static const char *TAG = "OpusFrameGrabber";

static uint8_t *inbuf = NULL;
static uint8_t *outbuf = NULL;
static int insize = 0;
static int outsize = 0;

static SemaphoreHandle_t frame_lock;
typedef struct audio_node audio_node_t;
typedef struct audio_node {
    void *data;
    audio_node_t *next;
} audio_node_t;

static audio_node_t *queue_head = NULL;

#define MAX_CNT 3
static volatile int curr_cnt = 0;

static void* frame_queue_get()
{
    void *data = NULL;
    audio_node_t *head = NULL;
    xSemaphoreTake(frame_lock, portMAX_DELAY);
    if (queue_head) {
        head = queue_head;
        data = head->data;
        queue_head = head->next;
        curr_cnt--;
    }
    xSemaphoreGive(frame_lock);
    if (head) {
        free(head);
    }
    return data;
}

static esp_err_t frame_queue_insert(void* data)
{
    while(curr_cnt >= MAX_CNT) {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
    audio_node_t *node = calloc(1, sizeof(audio_node_t));
    if (node == NULL) {
        ESP_LOGE(TAG, "Failed to allocate node");
        return ESP_FAIL;
    }
    node->data = data;
    node->next = NULL;
    xSemaphoreTake(frame_lock, portMAX_DELAY);
    if (!queue_head) {
        queue_head = node;
    } else {
        audio_node_t *pos = queue_head;
        while (pos->next) {
            pos = pos->next;
        }
        pos->next = node;
    }
    curr_cnt++;
    xSemaphoreGive(frame_lock);
    return ESP_OK;
}

static i2s_port_t i2s_port = I2S_NUM_0;
static void i2s_init(void)
{
    // Start listening for audio: MONO @ 16KHz
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = 16000,
        .bits_per_sample = (i2s_bits_per_sample_t) 16,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S,
        .intr_alloc_flags = 0,
        .dma_buf_count = 3,
        .dma_buf_len = 300,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = -1,
    };
#if CONFIG_IDF_TARGET_ESP32S3
    i2s_pin_config_t pin_config = {
        .bck_io_num = 41,    // IIS_SCLK
        .ws_io_num = 42,     // IIS_LCLK
        .data_out_num = -1,  // IIS_DSIN
        .data_in_num = 2,   // IIS_DOUT
    };
// #define READ_SAMPLE_SIZE_30  1
#if READ_SAMPLE_SIZE_30
    i2s_config.bits_per_sample = (i2s_bits_per_sample_t) 32;
#endif
#else
    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,    // IIS_SCLK
        .ws_io_num = 32,     // IIS_LCLK
        .data_out_num = -1,  // IIS_DSIN
        .data_in_num = 33,   // IIS_DOUT
    };
    i2s_port = I2S_NUM_1; // for esp32-eye
#endif

    esp_err_t ret = 0;
    ret = i2s_driver_install(i2s_port, &i2s_config, 0, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error in i2s_driver_install");
    }
    ret = i2s_set_pin(i2s_port, &pin_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error in i2s_set_pin");
    }

    ret = i2s_zero_dma_buffer(i2s_port);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error in initializing dma buffer with 0");
    }
}

/* returns handle */
static void *opus_encoder_init()
{
    void *enc_handle = NULL;
    esp_audio_err_t ret = ESP_AUDIO_ERR_OK;
    esp_opus_enc_config_t config = ESP_OPUS_ENC_CONFIG_DEFAULT();
    config.sample_rate = 16000;
    config.channel = 1;
    config.bitrate = 16000;
    ret = esp_opus_enc_open(&config, sizeof(esp_opus_enc_config_t), &enc_handle);

    return enc_handle;
}

static void audio_encoder_task(void *arg)
{
    static void *enc_handle = NULL;
    i2s_init();
    enc_handle = opus_encoder_init();
    esp_opus_enc_get_frame_size(enc_handle, &insize, &outsize);
#if READ_SAMPLE_SIZE_30
    inbuf = MEMALLOC(insize * 2); // 20ms mono
#else
    inbuf = MEMALLOC(insize); // 20ms mono
#endif
    outbuf = MEMALLOC(outsize);
    while (1) {
        // Encode process
        esp_audio_enc_in_frame_t in_frame = { 0 };
        esp_audio_enc_out_frame_t out_frame = { 0 };

        in_frame.buffer = inbuf;
        in_frame.len = insize;
        out_frame.buffer = outbuf;
        out_frame.len = outsize;

        size_t bytes_read = 0;
#if READ_SAMPLE_SIZE_30
        i2s_read(i2s_port, (void*)inbuf, insize * 2, &bytes_read, pdMS_TO_TICKS(20));
        // rescale the data (Actual data is 30 bits, use higher 16 bits out of those)
        for (int i = 0; i < bytes_read / 4; ++i) {
            ((uint16_t *) inbuf)[i] = (((uint32_t *) inbuf)[i] >> 14) & 0xffff;
        }
#else
        i2s_read(i2s_port, (void*)inbuf, insize, &bytes_read, pdMS_TO_TICKS(20));
#endif
        esp_opus_out_buf_t *opus_frame = NULL;
        esp_audio_err_t ret = ESP_AUDIO_ERR_OK;
        ret = esp_opus_enc_process(enc_handle, &in_frame, &out_frame);
        if (ret != ESP_AUDIO_ERR_OK) {
            printf("audio encoder process failed.\n");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        opus_frame = calloc(1, sizeof(esp_opus_out_buf_t));
        if (!opus_frame) {
            ESP_LOGE(TAG, "Failed to alloc opus_frame");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        opus_frame->len = out_frame.encoded_bytes;
        opus_frame->buffer = MEMALLOC(opus_frame->len);
        if (!opus_frame->buffer) {
            ESP_LOGE(TAG, "opus_frame->buffer alloc failed, size %d", (int) opus_frame->len);
            free(opus_frame);
            vTaskDelay(pdMS_TO_TICKS(50));
        }
        memcpy(opus_frame->buffer, out_frame.buffer, opus_frame->len);

        /* Insert it into the queue */
        if (frame_queue_insert(opus_frame) != ESP_OK) {
            free(opus_frame->buffer);
            free(opus_frame);
            vTaskDelay(50);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_opus_out_buf_t *get_opus_encoded_frame()
{
    static bool first_time = true;
    if (first_time) {
        frame_lock = xSemaphoreCreateMutex();

#define ENC_TASK_STACK_SIZE     (24 * 1024)
#define ENC_TASK_PRIO           (4) // lesser than the video `sender_task`
        StaticTask_t *task_buffer = heap_caps_calloc(1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
        void *task_stack = MEMALLOC(ENC_TASK_STACK_SIZE);
        assert(task_buffer && task_stack);

        /* the task never exits, so do not bother to free the buffers */
        xTaskCreateStatic(audio_encoder_task, "audio_encoder", ENC_TASK_STACK_SIZE,
                          NULL, ENC_TASK_PRIO, task_stack, task_buffer);

        printf("audio_encoder initialized\n");
        print_mem_stats();

        first_time = false;
    }

    esp_opus_out_buf_t *opus_frame = (esp_opus_out_buf_t *) frame_queue_get();
    if (opus_frame) {
        // printf("got a frame with size %d\n", (int) frame->len);
    }
    return opus_frame;
}
