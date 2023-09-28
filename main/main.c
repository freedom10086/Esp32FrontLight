#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "math.h"

#include "iot_button.h"
#include "driver/ledc.h"

#define TAG "font_light"

#define GPIO_LED_EN     GPIO_NUM_18
#define GPIO_LED_FAULT  GPIO_NUM_19
#define GPIO_CAN_RX     GPIO_NUM_6
#define GPIO_CAN_TX     GPIO_NUM_7

#define GPIO_SPK_SD     GPIO_NUM_0
#define GPIO_SPK_LR     GPIO_NUM_1
#define GPIO_SPK_BCLK   GPIO_NUM_10
#define GPIO_SPK_DIN    GPIO_NUM_11

static void button_single_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "BUTTON_SINGLE_CLICK");
}

static void button_double_click_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "BUTTON_DOUBLE_CLICK");
}

static void button_long_press_1_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "BUTTON_LONG_PRESS_START_1");
}

static void button_long_press_2_cb(void *arg, void *usr_data) {
    ESP_LOGI(TAG, "BUTTON_LONG_PRESS_START_2");
}

static IRAM_ATTR bool cb_ledc_fade_end_event(const ledc_cb_param_t *param, void *user_arg) {
//    ESP_LOGI(TAG, "LED fade end, current duty %ld", param->duty);
//    BaseType_t taskAwoken = pdFALSE;
//
//    if (param->event == LEDC_FADE_END_EVT) {
//        SemaphoreHandle_t counting_sem = (SemaphoreHandle_t) user_arg;
//        xSemaphoreGiveFromISR(counting_sem, &taskAwoken);
//    }
//
//    return (taskAwoken == pdTRUE);

    return false;
}

void app_main(void) {
    printf("Hello world!\n");

    /* Print chip information */
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    printf("This is %s chip with %d CPU core(s), %s%s%s%s, ",
           CONFIG_IDF_TARGET,
           chip_info.cores,
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi/" : "",
           (chip_info.features & CHIP_FEATURE_BT) ? "BT" : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE" : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? ", 802.15.4 (Zigbee/Thread)" : "");

    unsigned major_rev = chip_info.revision / 100;
    unsigned minor_rev = chip_info.revision % 100;
    printf("silicon revision v%d.%d, ", major_rev, minor_rev);
    if (esp_flash_get_size(NULL, &flash_size) != ESP_OK) {
        printf("Get flash size failed");
        return;
    }

    printf("%" PRIu32 "MB %s flash\n", flash_size / (uint32_t) (1024 * 1024),
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "embedded" : "external");

    printf("Minimum free heap size: %" PRIu32 " bytes\n", esp_get_minimum_free_heap_size());

    // create gpio button
    button_config_t gpio_btn_cfg = {
            .type = BUTTON_TYPE_GPIO,
            .long_press_time = CONFIG_BUTTON_LONG_PRESS_TIME_MS,
            .short_press_time = CONFIG_BUTTON_SHORT_PRESS_TIME_MS,
            .gpio_button_config = {
                    .gpio_num = GPIO_NUM_9,
                    .active_level = 0,
            },
    };
    button_handle_t gpio_btn = iot_button_create(&gpio_btn_cfg);
    if (NULL == gpio_btn) {
        ESP_LOGE(TAG, "Button create failed");
    }

    button_event_config_t cfg = {
            .event = BUTTON_LONG_PRESS_START,
            .event_data.long_press.press_time = 2000,
    };

    iot_button_register_event_cb(gpio_btn, cfg, button_long_press_1_cb, NULL);

    cfg.event_data.long_press.press_time = 5000;
    iot_button_register_event_cb(gpio_btn, cfg, button_long_press_2_cb, NULL);

    cfg.event = BUTTON_SINGLE_CLICK;
    iot_button_register_event_cb(gpio_btn, cfg, button_single_click_cb, NULL);

    cfg.event = BUTTON_DOUBLE_CLICK;
    iot_button_register_event_cb(gpio_btn, cfg, button_double_click_cb, NULL);

    // led
    gpio_config_t io_conf = {
            .pin_bit_mask = (1ULL << GPIO_LED_FAULT),
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = 1,
            .pull_down_en = 0,
    };
    gpio_config(&io_conf);

    int led_fault_level = gpio_get_level(GPIO_LED_FAULT);
    ESP_LOGI(TAG, "Led Fault Level %d", led_fault_level);

    ledc_timer_config_t ledc_timer = {
            .duty_resolution = LEDC_TIMER_13_BIT, // resolution of PWM duty
            .freq_hz = 5000,                      // frequency of PWM signal
            .speed_mode = LEDC_LOW_SPEED_MODE,    // timer mode
            .timer_num = LEDC_TIMER_0,            // timer index
            .clk_cfg = LEDC_AUTO_CLK,             // Auto select the source clock
    };
    // Set configuration of timer0 for high speed channels
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
            .channel    = LEDC_CHANNEL_0,
            .duty       = 0,
            .gpio_num   = GPIO_LED_EN,
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .hpoint     = 0,
            .timer_sel  = LEDC_TIMER_0,
            .flags.output_invert = 0
    };
    ledc_channel_config(&ledc_channel);

    // Initialize fade service.
    ledc_fade_func_install(0);
    ledc_cbs_t callbacks = {
            .fade_cb = cb_ledc_fade_end_event
    };
    ledc_cb_register(ledc_channel.speed_mode, ledc_channel.channel, &callbacks, NULL);

    // [0, (2**duty_resolution) - 1]
    // 50% 0.5 * (2**duty_resolution)
    int duty = 50 * ((1 << ledc_timer.duty_resolution) - 1) / 100;
    ledc_set_fade_with_time(ledc_channel.speed_mode, ledc_channel.channel, duty, 200);
    ledc_fade_start(ledc_channel.speed_mode,ledc_channel.channel, LEDC_FADE_NO_WAIT);
    ESP_LOGI(TAG, "Done!");

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
