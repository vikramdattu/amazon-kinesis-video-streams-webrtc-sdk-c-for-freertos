/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief grab camera frames and encode them with h264 encoder
 *
 */

#pragma once

#include <stdint.h>
#include <esp_h264_types.h>

#define ENCODER_TASK        (1)

#if ENCODER_TASK
typedef struct {
    uint8_t *buffer; /*<! Data buffer */
    uint32_t len;    /*<! It is buffer length in byte */
    esp_h264_frame_type_t type; /* Frame type */
} esp_h264_out_buf_t;

esp_h264_out_buf_t *get_h264_encoded_frame();
#else
void get_h264_encoded_frame(uint8_t *out_buf, uint32_t *frame_len);
#endif
