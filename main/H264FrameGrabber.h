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

void get_h264_encoded_frame(uint8_t *out_buf, uint32_t *frame_len);
