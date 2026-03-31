#include <stdio.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static volatile int32_t left_count = 0;
static volatile int32_t right_count = 0;

static constexpr gpio_num_t LEFT_A  = GPIO_NUM_17;
static constexpr gpio_num_t LEFT_B  = GPIO_NUM_18;
static constexpr gpio_num_t RIGHT_A = GPIO_NUM_19;
static constexpr gpio_num_t RIGHT_B = GPIO_NUM_21;

static void IRAM_ATTR left_isr(void *arg)
{
    int a = gpio_get_level(LEFT_A);
    int b = gpio_get_level(LEFT_B);
    left_count += (a == b) ? 1 : -1;
}

static void IRAM_ATTR right_isr(void *arg)
{
    int a = gpio_get_level(RIGHT_A);
    int b = gpio_get_level(RIGHT_B);
    right_count += (a == b) ? 1 : -1;
}

extern "C" void app_main(void)
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_ANYEDGE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask =
        (1ULL << LEFT_A) |
        (1ULL << LEFT_B) |
        (1ULL << RIGHT_A) |
        (1ULL << RIGHT_B);
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    gpio_config(&io_conf);

    gpio_install_isr_service(0);
    gpio_isr_handler_add(LEFT_A, left_isr, nullptr);
    gpio_isr_handler_add(RIGHT_A, right_isr, nullptr);

    int32_t last_left = 0;
    int32_t last_right = 0;

    while (true) {
        int32_t l = left_count;
        int32_t r = right_count;

        printf("L=%ld dL=%ld | R=%ld dR=%ld\n",
               (long)l, (long)(l - last_left),
               (long)r, (long)(r - last_right));

        last_left = l;
        last_right = r;

        vTaskDelay(pdMS_TO_TICKS(200));
    }
}