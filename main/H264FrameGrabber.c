/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AppCommon.h"
#include "H264FrameGrabber.h"

#include "app_camera_esp.h"

static const char *TAG = "H264FrameGrabber";

#if CONFIG_IDF_TARGET_ESP32S3
// #define OLD_H264_ENCODER 1
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32P4
#include <esp_heap_caps.h>

#if CONFIG_IDF_TARGET_ESP32S3
#if OLD_H264_ENCODER
#include "esp_h264_enc.h"
#else
#include "esp_h264_enc_single_sw.h"
#include "esp_h264_enc_single.h"
#endif
#else
#include "esp_h264_hw_enc.h"
#include "esp_h264_enc_single.h"
extern void esp32p4_frame_grabber_init(void);
extern esp_h264_buf_t *esp32p4_grab_one_frame();
#endif

static int frame_count = 0;
#if OLD_H264_ENCODER
static esp_h264_enc_t handle = NULL;
static esp_h264_enc_frame_t out_frame = { 0 };
static esp_h264_raw_frame_t in_frame = { 0 };
static esp_h264_enc_cfg_t cfg = DEFAULT_H264_ENCODER_CONFIG();
static esp_h264_enc_t initialize_h264_encoder();
#else
static esp_h264_enc_handle_t handle = NULL;
static esp_h264_enc_out_frame_t out_frame = { 0 };
static esp_h264_enc_in_frame_t in_frame = { 0 };
static esp_h264_enc_cfg_t cfg = {0};
static esp_h264_enc_handle_t initialize_h264_encoder();
#endif

#ifdef ENCODER_TASK
#include <freertos/freeRTOS.h>
#include <freertos/task.h>

static SemaphoreHandle_t frame_lock;
typedef struct video_node video_node_t;
typedef struct video_node {
    void *data;
    video_node_t *next;
} video_node_t;

static video_node_t *queue_head = NULL;

#define MAX_CNT 3
static volatile int curr_cnt = 0;

static void* frame_queue_get()
{
    void *data = NULL;
    video_node_t *head = NULL;
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
    video_node_t *node = calloc(1, sizeof(video_node_t));
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
        video_node_t *pos = queue_head;
        while (pos->next) {
            pos = pos->next;
        }
        pos->next = node;
    }
    curr_cnt++;
    xSemaphoreGive(frame_lock);
    return ESP_OK;
}

static void video_encoder_task(void *arg)
{
    static uint8_t fill_val = 0;
    frame_count = 0;
#if OLD_H264_ENCODER
    int one_image_size = cfg.height * cfg.width * 2;
#else
    int one_image_size = cfg.res.height * cfg.res.width * 2;
#endif
    while(1) {
        camera_fb_t *fb = esp_camera_fb_get();
        if (fb) {
            memcpy(in_frame.raw_data.buffer, fb->buf, one_image_size);
            esp_camera_fb_return(fb);
        } else {
            memset(in_frame.raw_data.buffer, fill_val++, one_image_size);
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        in_frame.pts = frame_count++ * (1000 / cfg.fps);

        esp_h264_err_t ret = esp_h264_enc_process(handle, &in_frame, &out_frame);
        if (ret != ESP_H264_ERR_OK) {
            printf("Process failed. ret %d \r\n", ret);
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        esp_h264_out_buf_t *frame = calloc(1, sizeof(esp_h264_out_buf_t));
        if (!frame) {
            ESP_LOGE(TAG, "Failed to alloc frame");
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        /* Calculate the frame length */
#if OLD_H264_ENCODER
        frame->len = 0;
        for (size_t layer = 0; layer < out_frame.layer_num; layer++) {
            frame->len += out_frame.layer_data[layer].len;
        }
        frame->type = out_frame.frame_type_t;
#else
        frame->len = out_frame.length;
        frame->type = out_frame.frame_type;
#endif

        /* allocate the memory of size *frame_len */
        frame->buffer = (uint8_t *) MEMALLOC(frame->len);

        if (!frame->buffer) {
            ESP_LOGE(TAG, "frame->buffer alloc failed, size %d", (int) frame->len);
            free(frame);
            print_mem_stats();
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        } else {
            ESP_LOGI(TAG, "frame->len %d", (int) frame->len);
        }

        uint8_t *out_buf = frame->buffer;

        /* Copy the frame */
#if OLD_H264_ENCODER
        for (size_t layer = 0; layer < out_frame.layer_num; layer++) {
            memcpy(out_buf, out_frame.layer_data[layer].buffer, out_frame.layer_data[layer].len);
            out_buf += out_frame.layer_data[layer].len;
        }
#else
        memcpy(out_buf, out_frame.raw_data.buffer, frame->len);
#endif

        /* Insert it into the queue */
        if (frame_queue_insert(frame) != ESP_OK) {
            free(frame->buffer);
            free(frame);
            vTaskDelay(50);
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

esp_h264_out_buf_t *get_h264_encoded_frame()
{
    static bool is_first = true;
    if (is_first) {

#if CONFIG_IDF_TARGET_ESP32P4
        /* TODO: Breaking code... Needs to be fixed to use common code... */
        esp32p4_frame_grabber_init();
#else
        app_camera_init();
        printf("camera init done\n");

        initialize_h264_encoder();
        print_mem_stats();

        frame_lock = xSemaphoreCreateMutex();

#define ENC_TASK_STACK_SIZE     (12 * 1024)
#define ENC_TASK_PRIO           (4) // lesser than the video `sender_task`
        StaticTask_t *task_buffer = heap_caps_calloc(1, sizeof(StaticTask_t), MALLOC_CAP_INTERNAL);
        void *task_stack = MEMALLOC(ENC_TASK_STACK_SIZE);
        assert(task_buffer && task_stack);

        /* the task never exits, so do not bother to free the buffers */
        xTaskCreateStatic(video_encoder_task, "video_encoder", ENC_TASK_STACK_SIZE,
                          NULL, ENC_TASK_PRIO, task_stack, task_buffer);

        printf("encoder initialized\n");
#endif
        is_first = false;
    }
#if CONFIG_IDF_TARGET_ESP32P4
    return esp32p4_grab_one_frame();
#else
    esp_h264_out_buf_t *frame = (esp_h264_out_buf_t *) frame_queue_get();
    if (frame) {
        // printf("got a frame with size %d\n", (int) frame->len);
    }
    return frame;
#endif
}
#else
void get_h264_encoded_frame(uint8_t **out_buf, uint32_t *frame_len, uint32_t *frame_type)
{
    static bool is_first = true;
    static uint8_t fill_val = 0;
    if (is_first) {
#if CONFIG_IDF_TARGET_ESP32P4
        /* TODO: Breaking code... Needs to be fixed to use common code... */
        esp32p4_frame_grabber_init();
#else
        app_camera_init();
        printf("camera init done\n");
#endif
        initialize_encoder();

        print_mem_stats();

        printf("encoder initialized\n");
        is_first = false;
    }
    frame_count = 0;
#if OLD_H264_ENCODER
    int one_image_size = cfg.height * cfg.width * 2;
#else
    int one_image_size = cfg.res.height * cfg.res.width * 2;

#endif
#if CONFIG_IDF_TARGET_ESP32P4
    *frame_len = 0;
    esp_h264_buf_t *h264_buffer = esp32p4_grab_one_frame();
    if (h264_buffer) {
        if (out_buf) {
            *frame_len = h264_buffer->len;
            *out_buf = h264_buffer->buffer;
            *frame_type = (uint32_t) h264_buffer->type;
        } else {
            free(h264_buffer->buffer);
            h264_buffer->buffer = NULL;
        }
    }
    return;
#endif
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb) {
        memcpy(in_frame.raw_data.buffer, fb->buf, one_image_size);
        esp_camera_fb_return(fb);
    } else {
        memset(in_frame.raw_data.buffer, fill_val++, one_image_size);
    }

    *frame_len = 0;
    in_frame.pts = frame_count++ * (1000 / cfg.fps);

    esp_h264_err_t ret = esp_h264_enc_process(handle, &in_frame, &out_frame);
    if (ret != ESP_H264_ERR_OK) {
        printf("Process failed. ret %d \r\n", ret);
        return;
    }

    for (size_t layer = 0; layer < out_frame.layer_num; layer++) {
        memcpy(out_buf, out_frame.layer_data[layer].buffer, out_frame.layer_data[layer].len);
        out_buf += out_frame.layer_data[layer].len;
        *frame_len += out_frame.layer_data[layer].len;
    }
}
#endif
#else
void get_h264_encoded_frame(uint8_t *out_buf, uint32_t *frame_len)
{
    /* Dummy function which does nothing. Set the frame size to 0 */
    *frame_len = 0;
}
#endif

#if CONFIG_IDF_TARGET_ESP32S3
#if OLD_H264_ENCODER
static esp_h264_enc_t initialize_h264_encoder()
#else
static esp_h264_enc_handle_t initialize_h264_encoder()
#endif
{
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    int one_image_size = 0;
    cfg.fps = 10;//DEFAULT_FPS_VALUE;
#if OLD_H264_ENCODER
    cfg.width = 320;
    cfg.height = 240;
    // cfg.gop_size = 10;
    cfg.pic_type = ESP_H264_RAW_FMT_YUV422;
    one_image_size = cfg.height * cfg.width * 2; // 1.5 : Pixel is 1.5 on ESP_H264_RAW_FMT_I420.
#else
    cfg.gop = 30;
    cfg.res.width = 320;
    cfg.res.height = 240;
    cfg.rc.bitrate = cfg.res.width * cfg.res.height * cfg.fps / 20;
    cfg.rc.qp_min = 30;
    cfg.rc.qp_max = 30;
    cfg.pic_type = ESP_H264_RAW_FMT_YUYV;
    one_image_size = cfg.res.height * cfg.res.width * 2; // 1.5 : Pixel is 1.5 on ESP_H264_RAW_FMT_I420.
#endif
    in_frame.raw_data.buffer = (uint8_t *) MEMALIGNALLOC(one_image_size, 16);
    if (in_frame.raw_data.buffer == NULL) {
        printf("in_frame.raw_data.buffer allocation failed\n");
        goto h264_example_exit;
    }
#if OLD_H264_ENCODER
    ret = esp_h264_enc_open(&cfg, &handle);
#else
    out_frame.raw_data.len = one_image_size;
    out_frame.raw_data.buffer = (uint8_t *) MEMALIGNALLOC(one_image_size, 16);
    if (out_frame.raw_data.buffer == NULL) {
        printf("out_frame.raw_data.buffer allocation failed\n");
        goto h264_example_exit;
    }
#ifdef CONFIG_IDF_TARGET_ESP32P4
    ret = esp_h264_enc_hw_new(&cfg, &handle);
#else
    ret = esp_h264_enc_sw_new(&cfg, &handle);
#endif
    if (ret != ESP_H264_ERR_OK) {
        printf("esp_h264_enc_sw_new failed ret %d, handle %p\n", (int) ret, (void *) handle);
        goto h264_example_exit;
    }

    ret = esp_h264_enc_open(handle);
#endif
    if (ret != ESP_H264_ERR_OK) {
        printf("Open failed. ret %d, handle %p\n", (int) ret, (void *) handle);
        goto h264_example_exit;
    }

    return handle;

h264_example_exit:
    if (in_frame.raw_data.buffer) {
        heap_caps_free(in_frame.raw_data.buffer);
        in_frame.raw_data.buffer = NULL;
    }

    esp_h264_enc_close(handle);
#ifndef OLD_H264_ENCODER
    esp_h264_enc_del(handle);
#endif
    return NULL;
}
#endif
