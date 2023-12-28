/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "AppCommon.h"
#include "H264FrameGrabber.h"

#include "app_camera_esp.h"

const char *TAG = "H264FramerGrabber";

#if CONFIG_IDF_TARGET_ESP32S3
#include <esp_heap_caps.h>
#include "esp_h264_enc.h"

static esp_h264_enc_t handle = NULL;
static esp_h264_enc_frame_t out_frame = { 0 };
static esp_h264_raw_frame_t in_frame = { 0 };
static int frame_count = 0;
static esp_h264_enc_cfg_t cfg = DEFAULT_H264_ENCODER_CONFIG();

static esp_h264_enc_t initialize_h264_encoder();
{
    esp_h264_err_t ret = ESP_H264_ERR_OK;
    int one_image_size = 0;
    cfg.fps = 10;//DEFAULT_FPS_VALUE;
    cfg.width = 320;
    cfg.height = 240;
    // cfg.gop_size = 10;
    cfg.pic_type = ESP_H264_RAW_FMT_YUV422;
    one_image_size = cfg.height * cfg.width * 2; // 1.5 : Pixel is 1.5 on ESP_H264_RAW_FMT_I420.
    in_frame.raw_data.buffer = (uint8_t *) MEMALIGNALLOC(one_image_size, 16);
    if (in_frame.raw_data.buffer == NULL) {
        printf("Memory allocation failed\n");
        goto h264_example_exit;
    }
    ret = esp_h264_enc_open(&cfg, &handle);
    if (ret != ESP_H264_ERR_OK) {
        printf("Open failed. ret %d, handle %p\n", ret, handle);
        goto h264_example_exit;
    }

    return handle;

h264_example_exit:
    if (in_frame.raw_data.buffer) {
        heap_caps_free(in_frame.raw_data.buffer);
        in_frame.raw_data.buffer = NULL;
    }

    esp_h264_enc_close(handle);
    return NULL;
}

void get_h264_encoded_frame(uint8_t *out_buf, uint32_t *frame_len)
{
    static bool is_first = true;
    static uint8_t fill_val = 0;
    if (is_first) {
        app_camera_init();

        printf("camera init done\n");

        initialize_encoder();

        print_mem_stats();

        printf("encoder initialized\n");
        is_first = false;
    }
    frame_count = 0;
    int one_image_size = cfg.height * cfg.width * 2;

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

//
// TODO:
// Create an init function and a nice task which keeps grabbing frames from Camera and encode those
// Get encoded frame will simply do memcpy and release that frame for the encode task
//

#else
void get_h264_encoded_frame(uint8_t *out_buf, uint32_t *frame_len)
{
    /* Dummy function which does nothing. Set the frame size to 0 */
    *frame_len = 0;
}
#endif
