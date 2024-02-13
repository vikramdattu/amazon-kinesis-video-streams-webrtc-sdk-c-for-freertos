/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_dma_utils.h"

#include "bsp/esp-bsp.h"
#include "sensor.h"
#include "bsp/camera.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "AppCommon.h"
#include "instrumented_allocators.h"
#include "allocators.h"

static const char *TAG = "esp32p4_frame_grabber";

#define H264_ENCODE     1
// #define SDCARD_SAVE     1

#if H264_ENCODE
#include "esp_h264_hw_enc.h"
#include "esp_h264_alloc.h"
#include "esp_cache.h"
#endif

static uint8_t *jpeg_last_buf;
static uint8_t *jpeg_next_buf;

static uint8_t *camera_next_buf;
static uint8_t *camera_last_buf;
static esp_h264_buf_t h264_out_data;

#if SDCARD_SAVE
static FILE* f264 = NULL;
#endif

static void camera_stop_cb(void *cb_ctx)
{
    (void)cb_ctx;

    ESP_LOGI(TAG, "Camera Stop");
}

static volatile int frames_received_cnt = 0;

static bool camera_trans_done(void)
{
    jpeg_next_buf = camera_last_buf;

    bsp_camera_set_frame_buffer(camera_next_buf);
    camera_last_buf = camera_next_buf;
    frames_received_cnt++; // Update the frames received count
    return false;
}

static esp_err_t camera_init(void)
{
    size_t camera_fb_size = 0;
    bsp_camera_config_t camera_cfg = {
        .hor_res = WIDTH,
        .ver_res = HEIGHT,
        .fb_size_ptr = &camera_fb_size,
        .num_fbs = 3,
        .expect_color_width = sizeof(uint16_t) * 8,
        .color_mode = MIPI_CSI_YUV420_MODE,
#if WIDTH > 1280
        .clock_rate = OV5647_MIPI_IDI_CLOCK_RATE_1080P_22FPS,
        .csi_lane_rate = OV5647_MIPI_CSI_LINE_RATE_1080P_22FPS,
#else
        .clock_rate = OV5647_MIPI_IDI_CLOCK_RATE_720P_50FPS,
        .csi_lane_rate = OV5647_MIPI_CSI_LINE_RATE_720P_50FPS,
#endif
        .flags = {
            .use_external_fb = 0,
        },
    };
    const isp_config_t isp_cfg = ISP_CONFIG_DEFAULT(MIPI_CSI_RAW8_MODE, camera_cfg.hor_res, camera_cfg.ver_res);
    if(bsp_camera_new(&camera_cfg, &isp_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "camera initialization failed");
        print_mem_stats();
        return ESP_FAIL;
    }
    ESP_ERROR_CHECK(bsp_camera_register_trans_done_callback(camera_trans_done));
    ESP_ERROR_CHECK(bsp_camera_get_frame_buffer(3, &camera_last_buf, &camera_next_buf, &jpeg_last_buf));
    jpeg_next_buf = jpeg_last_buf;

    return ESP_OK;
}

#if H264_ENCODE
// Data read callback to read raw data
static void data_read_callback(void *ctx, esp_h264_buf_t *in_data)
{
    in_data->buffer = jpeg_next_buf;
    if (jpeg_next_buf != jpeg_last_buf) {
        camera_next_buf = jpeg_last_buf;
        jpeg_last_buf = jpeg_next_buf;
    }
    // print_mem_stats();
}

// Data write callback to output encoded frames
static void data_write_callback(void *ctx, esp_h264_buf_t *out_data)
{
    h264_out_data.buffer = out_data->buffer;
    h264_out_data.len = out_data->len;

#if SDCARD_SAVE
    if (f264) {
        int wr_len = fwrite(out_data->buffer, 1, out_data->len, f264);
        if (wr_len != out_data->len) {
            ESP_LOGW(TAG, "expected wr: %" PRIu32 " actual: %d", out_data->len);
        }
    }
#endif
}
#endif

static void init_chip()
{
    asm volatile("li t0, 0x2000\n"
                 "csrrs t0, mstatus, t0\n"); /* FPU_state = 1 (initial) */
    asm volatile("li t0, 0x1\n"
                 "csrrs t0, 0x7F1, t0\n"); /* HWLP_state = 1 (initial) */
    asm volatile("li t0, 0x1\n"
                 "csrrs t0, 0x7F2, t0\n"); /* AIA_state = 1 (initial) */
}

void init_clock(void)
{
#include "soc/hp_sys_clkrst_reg.h"

    uint32_t rd;

    // rd = REG_READ(HP_SYS_CLKRST_REF_CLK_CTRL0_REG);
    // REG_WRITE(HP_SYS_CLKRST_REF_CLK_CTRL0_REG, rd & (~HP_SYS_CLKRST_REG_REF_240M_CLK_DIV_NUM_M));

    // rd = REG_READ(HP_SYS_CLKRST_REF_CLK_CTRL0_REG);
    // rd &= 0xffffff;
    // rd |= 0x1<<24;
    // REG_WRITE(HP_SYS_CLKRST_REF_CLK_CTRL1_REG, rd);

#if 1
    REG_SET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL02_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_CLK_SRC_SEL, 1);
    REG_SET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_CSI_DPHY_CLK_SRC_SEL, 1);
    REG_CLR_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_CFG_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_CFG_CLK_EN);
    REG_CLR_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_PLL_REFCLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_PLL_REFCLK_EN);
    REG_CLR_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_CSI_DPHY_CFG_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_CSI_DPHY_CFG_CLK_EN);
    REG_CLR_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_PLL_REFCLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPHY_PLL_REFCLK_EN);

    REG_CLR_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_DSI_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_DSI_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_DSI_BRG);
    REG_CLR_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_DSI_BRG);

    REG_CLR_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_CSI_HOST_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_CSI_HOST_SYS_CLK_EN);
    REG_CLR_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_CSI_BRG_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_CSI_BRG_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_CSI_HOST);
    REG_CLR_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_CSI_HOST);
    REG_SET_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_CSI_BRG);
    REG_CLR_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_CSI_BRG);

    // REG_SET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPICLK_DIV_NUM, (480000000 / MIPI_DPI_CLOCK_RATE) - 1);
    REG_SET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPICLK_SRC_SEL, 1);
    REG_SET_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL03_REG, HP_SYS_CLKRST_REG_MIPI_DSI_DPICLK_EN);
#endif

    REG_CLR_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_GDMA_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, HP_SYS_CLKRST_REG_GDMA_SYS_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_GDMA);
    REG_CLR_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_GDMA);

    REG_SET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL26_REG, HP_SYS_CLKRST_REG_ISP_CLK_DIV_NUM, 1 - 1);
    REG_SET_FIELD(HP_SYS_CLKRST_PERI_CLK_CTRL25_REG, HP_SYS_CLKRST_REG_ISP_CLK_SRC_SEL, 1);
    REG_CLR_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL25_REG, HP_SYS_CLKRST_REG_ISP_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_PERI_CLK_CTRL25_REG, HP_SYS_CLKRST_REG_ISP_CLK_EN);
    REG_SET_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_ISP);
    REG_CLR_BIT(HP_SYS_CLKRST_HP_RST_EN0_REG, HP_SYS_CLKRST_REG_RST_EN_ISP);

    rd = REG_READ(HP_SYS_CLKRST_REF_CLK_CTRL1_REG);
    REG_WRITE(HP_SYS_CLKRST_REF_CLK_CTRL1_REG, rd | HP_SYS_CLKRST_REG_REF_240M_CLK_EN);

    rd = REG_READ(HP_SYS_CLKRST_REF_CLK_CTRL2_REG);
    REG_WRITE(HP_SYS_CLKRST_REF_CLK_CTRL2_REG, rd | HP_SYS_CLKRST_REG_REF_160M_CLK_EN);

    rd = REG_READ(HP_SYS_CLKRST_HP_RST_EN2_REG);
    REG_WRITE(HP_SYS_CLKRST_HP_RST_EN2_REG, rd & (~HP_SYS_CLKRST_REG_RST_EN_H264));

    rd = REG_READ(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG);
    rd = rd | HP_SYS_CLKRST_REG_H264_SYS_CLK_EN | HP_SYS_CLKRST_REG_GPSPI3_SYS_CLK_EN | HP_SYS_CLKRST_REG_AXI_PDMA_SYS_CLK_EN | HP_SYS_CLKRST_REG_GDMA_SYS_CLK_EN;
    rd = rd | HP_SYS_CLKRST_REG_CSI_HOST_SYS_CLK_EN | HP_SYS_CLKRST_REG_CSI_BRG_SYS_CLK_EN;
    REG_WRITE(HP_SYS_CLKRST_SOC_CLK_CTRL1_REG, rd);
    // H264_DMA
    rd = REG_READ(HP_SYS_CLKRST_PERI_CLK_CTRL26_REG);
    rd |= HP_SYS_CLKRST_REG_H264_CLK_EN;
    rd |= HP_SYS_CLKRST_REG_H264_CLK_SRC_SEL;
    REG_WRITE(HP_SYS_CLKRST_PERI_CLK_CTRL26_REG, rd);
}

void esp32p4_frame_grabber_cleanup(void)
{
#if SDCARD_SAVE
    if (f264) {
        fflush(f264);
        fclose(f264);
    }
#endif
}

void esp32p4_frame_grabber_init(void)
{
    printf("Initializing chip\n");
    init_chip();
    printf("Initializing clock\n");
    init_clock();

    if (camera_init() != ESP_OK) {
        // cannot continue... Abort..!
        vTaskDelay(portMAX_DELAY);
    }

#if SDCARD_SAVE
    sdmmc_card_t *sdcard = bsp_sdcard_mount();
    if (sdcard == NULL) {
        ESP_LOGE(TAG, "sdcard initialization failed!");
        return;
    }

    printf("Filesystem mounted\n");
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, sdcard);

    char h264_name[100] = "/sdcard/encoded_file.h264"; // CONFIG_BSP_SD_MOUNT_POINT
    // sprintf(h264_name, "/eMMC/res_%d_%d.264", 1920, 1080);
    printf("h264_name %s \n", h264_name);

    f264 = fopen(h264_name, "wb+");
    if (f264 == NULL) {
        printf("H264 file create failed\n");
        esp32p4_frame_grabber_cleanup();
        return;
    }
#endif

#if H264_ENCODE
    h264_enc_user_cfg_t cfg = {
        .read_cb = &data_read_callback,
        .write_cb = &data_write_callback,
    };
    esp_h264_setup_encoder(&cfg);
#else
    while (1) {
        ESP_LOGI(TAG, "frames_received_cnt %d", frames_received_cnt);
        vTaskDelay(pdMS_TO_TICKS(1000)); // Test: 1fps
        // print_mem_stats();
    }
#endif
}

esp_h264_buf_t *esp32p4_grab_one_frame()
{
    static int print_cnt = 100;
    if (print_cnt == 100) {
        ESP_LOGI(TAG, "frames_received_cnt %d, curr frame len %" PRIu32, frames_received_cnt, h264_out_data.len);
        print_mem_stats();
        print_cnt = 0;
    }
    print_cnt++;

    esp_err_t ret = esp_h264_hw_enc_process_one_frame();
    if (ret != ESP_OK) {
        return NULL;
    }
    return &h264_out_data;
}
