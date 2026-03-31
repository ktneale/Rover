#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

static volatile int32_t count = 0;

static constexpr gpio_num_t ENC_A = GPIO_NUM_18;
static constexpr gpio_num_t ENC_B = GPIO_NUM_17;

static void IRAM_ATTR enc_a_isr(void *arg)
{
    int a = gpio_get_level(ENC_A);
    int b = gpio_get_level(ENC_B);

    if (a == b) {
        count++;
    } else {
        count--;
    }
}

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << ENC_A) | (1ULL << ENC_B);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(ENC_A, enc_a_isr, nullptr);

    int32_t last = 0;

    while (true) {
        int32_t now = count;
        printf("count=%ld delta=%ld\n", (long)now, (long)(now - last));
        last = now;
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}