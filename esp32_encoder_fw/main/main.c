#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"

#include "driver/pulse_cnt.h"
/*
ESP32 Pulse Counter PCNT with ESP-IDF and Rotary Encoder ExampleThe ESP32 Pulse Counter (PCNT) 
is a hardware peripheral designed to count the rising and/or falling edges of input signals 
independently of the main CPU. It features multiple 16-bit signed counter units (8 on ESP32) 
that can increment, decrement, or act as a quadrature decoder for rotary encoders. 

 
Key Features & Capabilities:

Independent Units: The ESP32 has 8 independent units (0-7), each with two channels.
Signal Filtering: Each unit includes a programmable glitch filter to remove noise 
from signals.

Speed: Capable of handling high-frequency signals, up to 40 MHz.
Control Signals: Each channel has a control input to enable/disable counting based on 
another GPIO level, useful for direction control.
Interrupts: Generates interrupts on specific conditions, such as reaching a set maximum, minimum, 
or threshold values. 


Common Use Cases:
Rotary Encoders: Ideal for decoding quadrature signals to determine speed and direction.
Flow Meters/RPM Calculation: Counting pulses from sensors over specific time intervals.
Pulse Width Measurements: Detecting the edge behavior of complex signals. 

The PCNT is highly efficient for real-time applications, often utilizing esp-idf 
(Espressif IoT Development Framework) for configuration. 
*/

static const char *TAG = "ENCODER";

// -----------------------------------------------------------------------------
// GPIO CONFIGURATION
// -----------------------------------------------------------------------------
#define ENC_L_A_GPIO     17
#define ENC_L_B_GPIO     18
#define ENC_R_A_GPIO     21
#define ENC_R_B_GPIO     20

// -----------------------------------------------------------------------------
// ENCODER / ODOM CONFIGURATION
// Tune these to match your rover
// -----------------------------------------------------------------------------

// Set these so that when the rover moves FORWARD, both wheel deltas are positive.
#define LEFT_SIGN                   1
#define RIGHT_SIGN                  1

// Hardware PCNT limits
#define PCNT_LOW_LIMIT             (-30000)
#define PCNT_HIGH_LIMIT            (30000)

// Glitch filter in ns
#define PCNT_GLITCH_NS             1000

// Sampling / reporting period
#define ENCODER_SAMPLE_MS          200

// Optional periodic hardware clear
#define ENABLE_PERIODIC_HW_CLEAR   1
#define HW_CLEAR_EVERY_SAMPLES     25      // 25 * 200 ms = 5 s

// --- ODOMETRY PARAMETERS ---
// Replace with your actual values
#define COUNTS_PER_WHEEL_REV       2100.0f     // example only
#define WHEEL_DIAMETER_M           0.08f      // example only (65 mm wheel)
#define TRACK_WIDTH_M              0.130f      // example only (150 mm between wheel centers)

// Enable raw JSON odometry output over UART/stdout
#define ENABLE_ODOM_JSON_OUTPUT    1

// Enable simple serial command parser from stdin
// Supported command: reset
#define ENABLE_STDIN_RESET_TASK    1

typedef struct
{
    const char *name;
    int gpio_a;
    int gpio_b;
    int sign;

    pcnt_unit_handle_t unit;
    pcnt_channel_handle_t chan;

    int32_t total_count;
    int16_t last_hw_count;

    int32_t last_delta;
    float cps;
} encoder_t;

typedef struct
{
    float x_m;
    float y_m;
    float theta_rad;

    float v_mps;         // linear velocity
    float w_radps;       // angular velocity

    uint32_t seq;
    int64_t last_ms;
} odom_t;

static encoder_t enc_left = {
    .name = "LEFT",
    .gpio_a = ENC_L_A_GPIO,
    .gpio_b = ENC_L_B_GPIO,
    .sign = LEFT_SIGN,
};

static encoder_t enc_right = {
    .name = "RIGHT",
    .gpio_a = ENC_R_A_GPIO,
    .gpio_b = ENC_R_B_GPIO,
    .sign = RIGHT_SIGN,
};

static odom_t g_odom = {0};

static volatile bool g_reset_requested = false;

// -----------------------------------------------------------------------------
// HELPERS
// -----------------------------------------------------------------------------
static float normalize_angle_rad(float a)
{
    while (a > (float)M_PI) {
        a -= 2.0f * (float)M_PI;
    }
    while (a < -(float)M_PI) {
        a += 2.0f * (float)M_PI;
    }
    return a;
}

static float wheel_circumference_m(void)
{
    return (float)M_PI * WHEEL_DIAMETER_M;
}

static float meters_per_count(void)
{
    return wheel_circumference_m() / COUNTS_PER_WHEEL_REV;
}

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

// -----------------------------------------------------------------------------
// RESET API
// Call this from elsewhere in your app if you want to zero odometry on demand.
// -----------------------------------------------------------------------------
void encoder_request_reset(void)
{
    g_reset_requested = true;
}

static esp_err_t encoder_reset_one(encoder_t *enc)
{
    esp_err_t err = pcnt_unit_clear_count(enc->unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_clear_count failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    enc->total_count = 0;
    enc->last_hw_count = 0;
    enc->last_delta = 0;
    enc->cps = 0.0f;

    return ESP_OK;
}

static esp_err_t odom_reset_all(void)
{
    esp_err_t err;

    err = encoder_reset_one(&enc_left);
    if (err != ESP_OK) {
        return err;
    }

    err = encoder_reset_one(&enc_right);
    if (err != ESP_OK) {
        return err;
    }

    g_odom.x_m = 0.0f;
    g_odom.y_m = 0.0f;
    g_odom.theta_rad = 0.0f;
    g_odom.v_mps = 0.0f;
    g_odom.w_radps = 0.0f;
    g_odom.seq = 0;
    g_odom.last_ms = now_ms();

    ESP_LOGI(TAG, "Odometry reset");

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// PCNT INITIALISATION
// -----------------------------------------------------------------------------
static esp_err_t encoder_init(encoder_t *enc)
{
    esp_err_t err;

    pcnt_unit_config_t unit_config = {
        .low_limit = PCNT_LOW_LIMIT,
        .high_limit = PCNT_HIGH_LIMIT,
    };

    err = pcnt_new_unit(&unit_config, &enc->unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_new_unit failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = PCNT_GLITCH_NS,
    };

    err = pcnt_unit_set_glitch_filter(enc->unit, &filter_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_set_glitch_filter failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    pcnt_chan_config_t chan_config = {
        .edge_gpio_num = enc->gpio_a,
        .level_gpio_num = enc->gpio_b,
    };

    err = pcnt_new_channel(enc->unit, &chan_config, &enc->chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_new_channel failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    err = pcnt_channel_set_edge_action(
        enc->chan,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE,
        PCNT_CHANNEL_EDGE_ACTION_DECREASE
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_channel_set_edge_action failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    err = pcnt_channel_set_level_action(
        enc->chan,
        PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE
    );
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_channel_set_level_action failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    err = pcnt_unit_enable(enc->unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_enable failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    err = pcnt_unit_clear_count(enc->unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_clear_count failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    err = pcnt_unit_start(enc->unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_start failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    enc->total_count = 0;
    enc->last_hw_count = 0;
    enc->last_delta = 0;
    enc->cps = 0.0f;

    ESP_LOGI(TAG, "%s ready: A=%d B=%d sign=%d",
             enc->name, enc->gpio_a, enc->gpio_b, enc->sign);

    return ESP_OK;
}

// -----------------------------------------------------------------------------
// ENCODER UPDATE
// -----------------------------------------------------------------------------
static esp_err_t encoder_update(encoder_t *enc, float sample_period_s)
{
    int hw_count = 0;
    esp_err_t err = pcnt_unit_get_count(enc->unit, &hw_count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_get_count failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    int32_t raw_delta = (int32_t)hw_count - (int32_t)enc->last_hw_count;
    int32_t signed_delta = raw_delta * enc->sign;

    enc->last_hw_count = (int16_t)hw_count;
    enc->last_delta = signed_delta;
    enc->total_count += signed_delta;
    enc->cps = (sample_period_s > 0.0f) ? ((float)signed_delta / sample_period_s) : 0.0f;

    return ESP_OK;
}

static esp_err_t encoder_clear_hw_counter_preserve_total(encoder_t *enc)
{
    esp_err_t err = pcnt_unit_clear_count(enc->unit);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s: pcnt_unit_clear_count failed: %s", enc->name, esp_err_to_name(err));
        return err;
    }

    enc->last_hw_count = 0;
    return ESP_OK;
}

// -----------------------------------------------------------------------------
// ODOMETRY UPDATE
// -----------------------------------------------------------------------------
static void odom_update_from_counts(int32_t dL_counts, int32_t dR_counts, float dt_s)
{
    const float m_per_count = meters_per_count();

    const float dL_m = (float)dL_counts * m_per_count;
    const float dR_m = (float)dR_counts * m_per_count;

    const float ds = 0.5f * (dR_m + dL_m);
    const float dtheta = (dR_m - dL_m) / TRACK_WIDTH_M;

    const float theta_mid = g_odom.theta_rad + 0.5f * dtheta;

    g_odom.x_m += ds * cosf(theta_mid);
    g_odom.y_m += ds * sinf(theta_mid);
    g_odom.theta_rad = normalize_angle_rad(g_odom.theta_rad + dtheta);

    g_odom.v_mps = (dt_s > 0.0f) ? (ds / dt_s) : 0.0f;
    g_odom.w_radps = (dt_s > 0.0f) ? (dtheta / dt_s) : 0.0f;
}

// -----------------------------------------------------------------------------
// JSON OUTPUT
// This goes to stdout/UART and is easy for the Pi side to parse.
// -----------------------------------------------------------------------------
static void odom_print_json(int32_t dL, int32_t dR)
{
#if ENABLE_ODOM_JSON_OUTPUT
    int64_t ms = now_ms();
    g_odom.seq++;

    printf("{\"T\":\"odom\",\"seq\":%" PRIu32 ",\"ms\":%" PRId64
           ",\"dL\":%" PRId32 ",\"dR\":%" PRId32
           ",\"tL\":%" PRId32 ",\"tR\":%" PRId32
           ",\"x\":%.6f,\"y\":%.6f,\"th\":%.6f"
           ",\"v\":%.6f,\"w\":%.6f}\n",
           g_odom.seq, ms,
           dL, dR,
           enc_left.total_count, enc_right.total_count,
           g_odom.x_m, g_odom.y_m, g_odom.theta_rad,
           g_odom.v_mps, g_odom.w_radps);
#endif
}

// -----------------------------------------------------------------------------
// OPTIONAL: SIMPLE SERIAL COMMAND TASK
// Type "reset" in serial monitor and press enter.
// -----------------------------------------------------------------------------
#if ENABLE_STDIN_RESET_TASK
static void reset_command_task(void *arg)
{
    (void)arg;

    char line[64];

    while (1) {
        if (fgets(line, sizeof(line), stdin) != NULL) {
            size_t len = strlen(line);
            while (len > 0 && (line[len - 1] == '\n' || line[len - 1] == '\r')) {
                line[len - 1] = '\0';
                len--;
            }

            if (strcmp(line, "reset") == 0) {
                encoder_request_reset();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
#endif

// -----------------------------------------------------------------------------
// MAIN ENCODER / ODOM TASK
// -----------------------------------------------------------------------------
static void encoder_task(void *arg)
{
    (void)arg;

    const TickType_t period_ticks = pdMS_TO_TICKS(ENCODER_SAMPLE_MS);
    const float dt_s = ((float)ENCODER_SAMPLE_MS) / 1000.0f;

    uint32_t sample_count = 0;

    g_odom.last_ms = now_ms();

    while (1) {
        if (g_reset_requested) {
            g_reset_requested = false;
            if (odom_reset_all() != ESP_OK) {
                ESP_LOGE(TAG, "Failed to reset odometry");
            }
        }

        if (encoder_update(&enc_left, dt_s) != ESP_OK ||
            encoder_update(&enc_right, dt_s) != ESP_OK) {
            vTaskDelay(period_ticks);
            continue;
        }

        const int32_t dL = enc_left.last_delta;
        const int32_t dR = enc_right.last_delta;
        const int32_t totalL = enc_left.total_count;
        const int32_t totalR = enc_right.total_count;
        const float cpsL = enc_left.cps;
        const float cpsR = enc_right.cps;

        const int32_t diff_mag = abs(abs(dL) - abs(dR));
        float ratio = NAN;
        if (dR != 0) {
            ratio = fabsf((float)dL) / fabsf((float)dR);
        }

        odom_update_from_counts(dL, dR, dt_s);

        if (isnan(ratio)) {
            ESP_LOGI(TAG,
                     "dL=%" PRId32 " totalL=%" PRId32 " cpsL=%.2f | "
                     "dR=%" PRId32 " totalR=%" PRId32 " cpsR=%.2f | "
                     "diff_mag=%" PRId32 " ratio=NA | "
                     "x=%.3f y=%.3f th=%.3f",
                     dL, totalL, cpsL,
                     dR, totalR, cpsR,
                     diff_mag,
                     g_odom.x_m, g_odom.y_m, g_odom.theta_rad);
        } else {
            ESP_LOGI(TAG,
                     "dL=%" PRId32 " totalL=%" PRId32 " cpsL=%.2f | "
                     "dR=%" PRId32 " totalR=%" PRId32 " cpsR=%.2f | "
                     "diff_mag=%" PRId32 " ratio=%.3f | "
                     "x=%.3f y=%.3f th=%.3f",
                     dL, totalL, cpsL,
                     dR, totalR, cpsR,
                     diff_mag, ratio,
                     g_odom.x_m, g_odom.y_m, g_odom.theta_rad);
        }

        odom_print_json(dL, dR);

#if ENABLE_PERIODIC_HW_CLEAR
        sample_count++;
        if (sample_count >= HW_CLEAR_EVERY_SAMPLES) {
            if (encoder_clear_hw_counter_preserve_total(&enc_left) == ESP_OK &&
                encoder_clear_hw_counter_preserve_total(&enc_right) == ESP_OK) {
                ESP_LOGI(TAG, "Hardware PCNT counters cleared, software totals preserved");
            }
            sample_count = 0;
        }
#endif

        vTaskDelay(period_ticks);
    }
}

// -----------------------------------------------------------------------------
// APP MAIN
// -----------------------------------------------------------------------------
void app_main(void)
{
    ESP_ERROR_CHECK(encoder_init(&enc_left));
    ESP_ERROR_CHECK(encoder_init(&enc_right));
    ESP_ERROR_CHECK(odom_reset_all());

    xTaskCreate(
        encoder_task,
        "encoder_task",
        4096,
        NULL,
        5,
        NULL
    );

#if ENABLE_STDIN_RESET_TASK
    xTaskCreate(
        reset_command_task,
        "reset_command_task",
        4096,
        NULL,
        4,
        NULL
    );
#endif
}
