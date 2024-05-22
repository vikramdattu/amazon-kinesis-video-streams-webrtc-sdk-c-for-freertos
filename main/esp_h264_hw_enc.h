/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "H264FrameGrabber.h"
#include "esp_h264_types.h"

#define WIDTH               (1920)
#define HEIGHT              (1080)
#define ENCODER_TASK        (1)

typedef struct {
    uint8_t *buffer; /*<! Data buffer */
    uint32_t len;    /*<! It is buffer length in byte */
} esp_h264_buf_t;

// Data read callback to read raw data
typedef void data_read_cb_t(void *ctx, esp_h264_buf_t *in_data);

// Data write callback to output encoded frames
typedef void data_write_cb_t(void *ctx, esp_h264_out_buf_t *out_data);

typedef struct {
    data_read_cb_t *read_cb;
    data_write_cb_t *write_cb;
    esp_h264_enc_cfg_t enc_cfg;
} h264_enc_user_cfg_t;

/* setup encoder with given parameters */
esp_err_t esp_h264_setup_encoder(h264_enc_user_cfg_t *cfg);
esp_err_t esp_h264_hw_enc_process_one_frame();
esp_err_t esp_h264_hw_enc_set_bitrate(uint32_t bitrate);
void esp_h264_destroy_encoder();
