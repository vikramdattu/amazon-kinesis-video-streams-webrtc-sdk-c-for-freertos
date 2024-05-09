/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief grab mic data encode them with opus encoder
 *
 */

#pragma once

#include <stdint.h>

typedef struct {
    uint8_t *buffer; /*<! Data buffer */
    uint32_t len;    /*<! It is buffer length in byte */
} esp_opus_out_buf_t;

esp_opus_out_buf_t *get_opus_encoded_frame();
