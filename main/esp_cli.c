/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_console.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_cli.h"
#include "allocators.h"
#include "AppMain.h"

static const char *TAG = "[esp_cli]";

static int task_dump_cli_handler(int argc, char *argv[])
{
    /* Just to go to the next line */
    printf("\n");
#ifndef CONFIG_FREERTOS_USE_TRACE_FACILITY
    printf("%s: To use this utility enable: Component config > FreeRTOS > Kernel > configUSE_TRACEFACILITY\n", TAG);
#else
    int num_of_tasks = uxTaskGetNumberOfTasks();
    TaskStatus_t *task_array = calloc(1, num_of_tasks * sizeof(TaskStatus_t));
    if (!task_array) {
        ESP_LOGE(TAG, "Memory not allocated for task list.");
        return 0;
    }
    num_of_tasks = uxTaskGetSystemState(task_array, num_of_tasks, NULL);
    printf("\tName\tNumber\tPriority\tStackWaterMark\n");
    for (int i = 0; i < num_of_tasks; i++) {
        printf("%16s\t%u\t%u\t%u\n",
               task_array[i].pcTaskName,
               (unsigned) task_array[i].xTaskNumber,
               (unsigned) task_array[i].uxCurrentPriority,
               (unsigned) task_array[i].usStackHighWaterMark);
    }
    free(task_array);
#endif
    return 0;
}

static int cpu_dump_cli_handler(int argc, char *argv[])
{
    /* Just to go to the next line */
    printf("\n");
#ifndef CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS
    printf("%s: To use this utility enable: Component config > FreeRTOS > Kernel > configGENERATE_RUN_TIME_STATS\n", TAG);
#else
    char *buf = calloc(1, 2 * 1024);
    vTaskGetRunTimeStats(buf);
    printf("%s: Run Time Stats:\n%s\n", TAG, buf);
    free(buf);
#endif
    return 0;
}

static int mem_dump_cli_handler(int argc, char *argv[])
{
    /* Just to go to the next line */
    printf("\n");
    print_mem_stats();
    return 0;
}

static int reboot_cli_handler(int argc, char *argv[])
{
    /* Just to go to the next line */
    printf("\n");
    esp_restart();
    return 0;
}

static int crash_device_handler(int argc, char** argv)
{
    printf("Crashing the device now...\n");

    // Writing at invalid address
    *(int *) (0x0) = 0;
    return ESP_OK;
}

static esp_console_cmd_t diag_cmds[] = {
    {
        .command = "crash",
        .help = "Crash the device writing at invalid address",
        .func = &crash_device_handler,
    },
    {
        .command = "reboot",
        .help = "Reboot the device",
        .func = reboot_cli_handler,
    },
    {
        .command = "mem-dump",
        .help = "Prints memory stats",
        .func = mem_dump_cli_handler,
    },
    {
        .command = "task-dump",
        .help = "Print task snapshots",
        .func = task_dump_cli_handler,
    },
    {
        .command = "cpu-dump",
        .help = "Print CPU consumption data at the moment",
        .func = cpu_dump_cli_handler,
    },
};

int esp_cli_register_cmds()
{
    int cmds_num = sizeof(diag_cmds) / sizeof(esp_console_cmd_t);
    int i;
    for (i = 0; i < cmds_num; i++) {
        ESP_LOGI(TAG, "Registering command: %s", diag_cmds[i].command);
        esp_console_cmd_register(&diag_cmds[i]);
    }
    return 0;
}

int esp_cli_start()
{
    static int cli_started;
    if (cli_started) {
        return 0;
    }

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

    esp_console_register_help_command();
    esp_cli_register_cmds();
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif
    ESP_ERROR_CHECK(esp_console_start_repl(repl));
    cli_started = 1;
    return 0;
}
