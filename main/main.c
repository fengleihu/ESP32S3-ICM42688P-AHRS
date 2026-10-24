#include <math.h>
#include <stdio.h>
#include <string.h>

#include "ahrs.h"
#include "att_i2c_bus.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "mag.h"
#include "mag_axis.h"
#include "icm42688p.h"
#include "nvs_flash.h"
#include "oled_ui.h"
#include "sdkconfig.h"
#include "ssd1306.h"

static const char *TAG = "att9axis";

#define SAMPLE_HZ 100
#define MAG_READ_DIV 4
#define GYRO_CAL_SAMPLES 200
#define GYRO_CAL_DELAY_MS 5
#define MAG_CAL_MIN_QUALITY 0.4f
#define MAG_NVS_NAMESPACE "att9axis"
#define MAG_RECAL_KEY "recal2"
#define MAG_AXIS_DETECT_KEY "mapdet1"
#define DISPLAY_TASK_STACK_SIZE 6144
#define MAG_AXIS_DETECT_SECONDS 20
#define DEG_TO_RAD 0.017453292519943295f
#define MAG_RECAL_COMMAND "RECAL_MAG"

typedef struct {
    float roll;
    float pitch;
    float yaw;
    float mag_progress;
    float rot[9]; /* 机体 -> 世界 旋转矩阵（3D 姿态显示用） */
    bool display_ready;
    bool mag_present;
    bool mag_calibrated;
    bool mag_calibrating;
    bool mag_axis_detecting;
    bool mag_used;
} display_state_t;

static icm42688p_t s_imu;
static ahrs_t s_ahrs;
static display_state_t s_display;
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

/*
 * Board/body frame from the user's photo:
 * +X points to the photo's top (forward), +Y to the left, +Z out of the board.
 *
 * ICM42688P module mounting: the module's printed +X points forward, +Y points
 * left and +Z points up, i.e. the module frame already equals the body frame.
 * That is the old MPU6050 module rotated 90 degrees clockwise about +Z, so no
 * rotation (and no sign flip) is needed here any more.
 *
 * The BMM350 is a separate chip and still needs its own axis map; the saved map
 * (idx=19, body = (+my, +mx, -mz)) already flips the magnetometer's raw +Z, so
 * magnetometer and IMU agree in the body frame.
 */
static void imu_to_body(icm42688p_sample_t *sample)
{
    (void)sample; /* 模块坐标 = 机体坐标，无需变换 */
}

static void set_display_state(const display_state_t *state)
{
    portENTER_CRITICAL(&s_state_mux);
    s_display = *state;
    portEXIT_CRITICAL(&s_state_mux);
}

static display_state_t get_display_state(void)
{
    display_state_t state;
    portENTER_CRITICAL(&s_state_mux);
    state = s_display;
    portEXIT_CRITICAL(&s_state_mux);
    return state;
}

static void render_display(const display_state_t *state)
{
    static uint8_t fb[OLED_UI_FB_SIZE];
    oled_ui_clear(fb);

    char line[24];
    if (state->mag_axis_detecting) {
        oled_ui_text(fb, 0, 0, "MAG AXIS", 2);
#if OLED_UI_H >= 64
        oled_ui_text(fb, 0, 22, "TILT BOARD", 2);
        oled_ui_text(fb, 0, 48, "ALL DIRECTIONS", 1);
#else
        oled_ui_text(fb, 0, 16, "TILT BOARD", 1);
        oled_ui_text(fb, 0, 24, "ALL DIRS", 1);
#endif
    } else if (state->mag_calibrating) {
        oled_ui_text(fb, 0, 0, "MAG CAL", 2);
#if OLED_UI_H >= 64
        oled_ui_text(fb, 0, 20, "ROTATE SLOWLY", 1);
        snprintf(line, sizeof(line), "%3.0f%%",
                 (double)(state->mag_progress * 100.0f));
        oled_ui_text(fb, 0, 32, line, 2);
        oled_ui_text(fb, 0, 53, "ALL DIRECTIONS", 1);
#else
        oled_ui_text(fb, 0, 16, "ROTATE", 1);
        snprintf(line, sizeof(line), "%3.0f%%",
                 (double)(state->mag_progress * 100.0f));
        oled_ui_text(fb, 0, 24, line, 1);
        oled_ui_text(fb, 42, 24, "ALL DIRS", 1);
#endif
    } else {
        /* 主体：3D 姿态立方体；底部一行小字：航向 + 磁力计来源 */
        oled_ui_render_attitude(fb, state->rot);

        /* 右侧三行：Roll / Pitch / Yaw（1 倍字，固定宽度便于竖向对齐） */
#if OLED_UI_H >= 64
        const int tx = 76, ty0 = 8, tstep = 18;
#else
        const int tx = 72, ty0 = 2, tstep = 10;
#endif
        snprintf(line, sizeof(line), "R%+06.1f", (double)state->roll);
        oled_ui_text(fb, tx, ty0, line, 1);
        snprintf(line, sizeof(line), "P%+06.1f", (double)state->pitch);
        oled_ui_text(fb, tx, ty0 + tstep, line, 1);
        snprintf(line, sizeof(line), "Y%+06.1f", (double)state->yaw);
        oled_ui_text(fb, tx, ty0 + 2 * tstep, line, 1);
    }

    esp_err_t err = ssd1306_flush(fb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "OLED refresh failed: %s", esp_err_to_name(err));
    }
}

static void display_task(void *arg)
{
    (void)arg;
    const TickType_t period = pdMS_TO_TICKS(100);
    TickType_t last = xTaskGetTickCount();
    for (;;) {
        vTaskDelayUntil(&last, period);
        display_state_t state = get_display_state();
        if (state.display_ready) {
            render_display(&state);
        }
    }
}

static void publish_angles(bool mag_used)
{
    display_state_t state = get_display_state();
    ahrs_get_euler_deg(&s_ahrs, &state.roll, &state.pitch, &state.yaw);
    ahrs_quat_to_matrix(s_ahrs.q, state.rot);
    state.mag_used = mag_used;
    set_display_state(&state);
}

static void calibrate_magnetometer(void)
{
    const uint32_t duration_ms = (uint32_t)CONFIG_APP_MAG_CAL_SECONDS * 1000U;
    mag_cal_start(duration_ms);

    display_state_t state = get_display_state();
    state.mag_calibrating = true;
    state.mag_progress = 0.0f;
    set_display_state(&state);

    int64_t next_read = esp_timer_get_time();
    int reads = 0;
    while (mag_cal_is_running()) {
        int64_t now = esp_timer_get_time();
        state = get_display_state();
        state.mag_progress = mag_cal_progress();
        set_display_state(&state);

        if (now >= next_read) {
            next_read = now + 40000;
            float raw[3];
            if (mag_read(raw, NULL, NULL) == ESP_OK) {
                mag_cal_process(raw);
                reads++;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    state = get_display_state();
    state.mag_calibrating = false;
    state.mag_progress = 1.0f;
    state.mag_calibrated = mag_is_calibrated();
    set_display_state(&state);

    if (state.mag_calibrated) {
        mag_cal_t cal;
        mag_cal_get(&cal);
        ESP_LOGI(TAG, "Mag calibration OK: samples=%d quality=%.2f radius=%.1f uT",
                 reads, cal.quality, cal.radius);
    } else {
        ESP_LOGE(TAG, "Mag calibration failed after %d reads; continuing in 6-axis mode",
                 reads);
    }
}

static bool sample_for_mag_axis_detection(float accel[3], float mag[3])
{
    icm42688p_sample_t sample;
    if (icm42688p_read(&s_imu, &sample) != ESP_OK) {
        return false;
    }
    imu_to_body(&sample);
    if (mag_read_unmapped(mag) != ESP_OK) {
        return false;
    }
    memcpy(accel, sample.accel_g, sizeof(sample.accel_g));
    return true;
}

static esp_err_t nvs_flag_pending(const char *key, bool *pending)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MAG_NVS_NAMESPACE, NVS_READONLY, &handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *pending = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    uint8_t completed = 0;
    err = nvs_get_u8(handle, key, &completed);
    nvs_close(handle);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        *pending = true;
        return ESP_OK;
    }
    if (err != ESP_OK) {
        return err;
    }

    *pending = completed == 0;
    return ESP_OK;
}

static esp_err_t mark_nvs_flag_complete(const char *key)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MAG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_u8(handle, key, 1);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    return err;
}

static void request_mag_recalibration(void)
{
    nvs_handle_t handle;
    esp_err_t err = nvs_open(MAG_NVS_NAMESPACE, NVS_READWRITE, &handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not open NVS to request magnetometer recalibration: %s",
                 esp_err_to_name(err));
        return;
    }

    err = nvs_set_u8(handle, MAG_RECAL_KEY, 0);
    if (err == ESP_OK) {
        err = nvs_commit(handle);
    }
    nvs_close(handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Could not save magnetometer recalibration request: %s",
                 esp_err_to_name(err));
        return;
    }

    ESP_LOGW(TAG, "Magnetometer recalibration requested; rebooting. "
                  "Keep the board still for gyro calibration, then rotate it "
                  "slowly through all orientations when prompted.");
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
}

static void serial_command_task(void *arg)
{
    (void)arg;
    char command[sizeof(MAG_RECAL_COMMAND)];
    size_t length = 0;
    bool overflow = false;

    ESP_LOGI(TAG, "Serial command enabled: send %s followed by newline to recalibrate BMM350",
             MAG_RECAL_COMMAND);
    printf("CMD_READY,%s\n", MAG_RECAL_COMMAND);
    for (;;) {
        int ch = getchar();
        if (ch == EOF) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (ch == '\r') {
            continue;
        }
        if (ch == '\n') {
            command[length] = '\0';
            if (length > 0) {
                ESP_LOGI(TAG, "serial command received: %s", command);
            }
            if (!overflow && strcmp(command, MAG_RECAL_COMMAND) == 0) {
                request_mag_recalibration();
            }
            length = 0;
            overflow = false;
            continue;
        }
        if (length < sizeof(command) - 1) {
            command[length++] = (char)ch;
        } else {
            overflow = true;
        }
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32-S3 ICM42688P + BMM350 AHRS");
    ESP_LOGI(TAG, "Body axes: +X=photo top/forward, +Y=left, +Z=out of board");

    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGE(TAG, "NVS needs erase; refusing to erase it automatically");
        return;
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(err));
        return;
    }

    err = app_i2c_bus_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }
    app_i2c_scan();

    bool display_ready = (ssd1306_init() == ESP_OK);
    display_state_t state = {
        .display_ready = display_ready,
        .rot = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    };
    set_display_state(&state);
    if (xTaskCreate(display_task, "display", DISPLAY_TASK_STACK_SIZE, NULL, 3, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not start OLED task");
    }

        err = icm42688p_init(&s_imu, app_i2c_bus_get(), ICM42688P_ADDR_AD0_LOW,
                       SAMPLE_HZ, 3);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "0x68 no ACK, retrying 0x69 (AP_AD0 = VDD)");
        err = icm42688p_init(&s_imu, app_i2c_bus_get(), ICM42688P_ADDR_AD0_HIGH,
                             SAMPLE_HZ, 3);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "ICM42688P init failed: %s", esp_err_to_name(err));
        return;
    }

    err = icm42688p_calibrate_gyro(&s_imu, GYRO_CAL_SAMPLES, GYRO_CAL_DELAY_MS, NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Gyroscope bias calibration failed: %s", esp_err_to_name(err));
    }

    bool have_mag = false;
    bool mag_mapping_ready = true;
#if CONFIG_APP_MAG_ENABLE
    if (mag_init(app_i2c_bus_get()) == ESP_OK && mag_present()) {
        have_mag = true;
        state = get_display_state();
        state.mag_present = true;
        set_display_state(&state);

        int previous_map = mag_axis_get();
        bool detect_axis = false;
        err = nvs_flag_pending(MAG_AXIS_DETECT_KEY, &detect_axis);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not read one-time magnetometer axis-detection flag: %s",
                     esp_err_to_name(err));
            detect_axis = true;
        }

        if (detect_axis) {
            state = get_display_state();
            state.mag_axis_detecting = true;
            set_display_state(&state);
            ESP_LOGW(TAG, "Starting one-time magnetometer axis detection. "
                          "Slowly tilt the board forward, backward, left, right, "
                          "then turn it component-side down for %d seconds",
                     MAG_AXIS_DETECT_SECONDS);
            int detected_map = mag_axis_detect(MAG_AXIS_DETECT_SECONDS,
                                               sample_for_mag_axis_detection);
            state = get_display_state();
            state.mag_axis_detecting = false;
            set_display_state(&state);

            if (detected_map < 0) {
                ESP_LOGE(TAG, "Magnetometer axis detection failed; using 6-axis mode "
                              "until detection succeeds");
                mag_mapping_ready = false;
            } else {
                err = mag_axis_set(detected_map);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Could not save detected magnetometer axis map: %s",
                             esp_err_to_name(err));
                    mag_mapping_ready = false;
                } else if (mag_is_calibrated()) {
                    err = mag_cal_clear();
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Could not clear calibration for detected axis map: %s",
                                 esp_err_to_name(err));
                        mag_mapping_ready = false;
                    }
                }
                if (mag_mapping_ready) {
                    err = mark_nvs_flag_complete(MAG_AXIS_DETECT_KEY);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Could not save axis-detection completion flag: %s",
                                 esp_err_to_name(err));
                    }
                    ESP_LOGI(TAG, "Using detected BMM350 axis map idx=%d (previous idx=%d)",
                             detected_map, previous_map);
                }
            }
        }

        bool force_recalibration = false;
        err = nvs_flag_pending(MAG_RECAL_KEY, &force_recalibration);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Could not read one-time magnetometer recalibration flag: %s",
                     esp_err_to_name(err));
            force_recalibration = true;
        }

        if (!mag_mapping_ready) {
            state = get_display_state();
            state.mag_calibrated = false;
            set_display_state(&state);
        } else if (force_recalibration) {
            ESP_LOGW(TAG, "One-time magnetometer recalibration requested; "
                          "calibrate away from nearby magnetic interference");
            err = mag_cal_clear();
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Could not clear magnetometer calibration before recalibration: %s",
                         esp_err_to_name(err));
            }
        } else if (mag_is_calibrated()) {
            mag_cal_t cal;
            mag_cal_get(&cal);
            if (cal.quality < MAG_CAL_MIN_QUALITY) {
                ESP_LOGW(TAG, "Stored magnetometer calibration quality %.2f is too low; "
                              "clearing it and recalibrating",
                         cal.quality);
                err = mag_cal_clear();
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Could not clear low-quality magnetometer calibration: %s",
                             esp_err_to_name(err));
                }
            }
        }

        if (mag_mapping_ready && !mag_is_calibrated()) {
            ESP_LOGW(TAG, "Magnetometer calibration: rotate the board slowly through all "
                          "orientations for %d seconds",
                     CONFIG_APP_MAG_CAL_SECONDS);
            calibrate_magnetometer();
            if (force_recalibration && mag_is_calibrated()) {
                mag_cal_t cal;
                mag_cal_get(&cal);
                if (cal.quality >= MAG_CAL_MIN_QUALITY) {
                    err = mark_nvs_flag_complete(MAG_RECAL_KEY);
                    if (err != ESP_OK) {
                        ESP_LOGE(TAG, "Could not save one-time magnetometer recalibration flag: %s",
                                 esp_err_to_name(err));
                    }
                } else {
                    ESP_LOGW(TAG, "Magnetometer calibration quality %.2f is below %.2f; "
                                  "the one-time recalibration will retry on the next boot",
                             cal.quality, MAG_CAL_MIN_QUALITY);
                }
            } else if (force_recalibration) {
                ESP_LOGW(TAG, "One-time magnetometer recalibration did not produce valid "
                              "calibration; it will retry on the next boot");
            }
        }
    } else {
        ESP_LOGW(TAG, "BMM350 unavailable; continuing in 6-axis mode");
    }
#endif

    state = get_display_state();
    state.mag_present = have_mag;
    state.mag_calibrated = have_mag && mag_mapping_ready && mag_is_calibrated();
    set_display_state(&state);

    ahrs_init(&s_ahrs, 2.0f, 0.005f);
    icm42688p_sample_t sample = {0};
    if (icm42688p_read(&s_imu, &sample) == ESP_OK) {
        imu_to_body(&sample);
        float mag0[3];
        const float *initial_mag = NULL;
        if (have_mag && mag_mapping_ready && mag_is_calibrated() &&
            mag_read(NULL, mag0, NULL) == ESP_OK) {
            initial_mag = mag0;
        }
        if (!ahrs_init_from_accel_mag(&s_ahrs, sample.accel_g, initial_mag)) {
            ESP_LOGE(TAG, "Initial attitude alignment failed");
        }
    } else {
        ESP_LOGE(TAG, "Initial ICM42688P read failed");
    }
    publish_angles(false);

    if (xTaskCreate(serial_command_task, "serial_cmd", 4096, NULL, 4, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Could not start serial command task; magnetometer recalibration "
                      "command is unavailable");
    }

    TickType_t period = pdMS_TO_TICKS(1000 / SAMPLE_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_time = esp_timer_get_time();
    int mag_div = 0;
    int64_t next_stream = last_time;

    for (;;) {
        vTaskDelayUntil(&last_wake, period);
        int64_t now = esp_timer_get_time();
        float dt = (float)(now - last_time) * 1e-6f;
        last_time = now;

        bool mag_fresh = false;
        static float mag_raw_body[3];
        static float mag_body[3];
        if (have_mag && mag_mapping_ready && mag_is_calibrated() &&
            ++mag_div >= MAG_READ_DIV) {
            mag_div = 0;
            if (mag_read(mag_raw_body, mag_body, NULL) == ESP_OK) {
                mag_fresh = true;
            }
        }

        if (icm42688p_read(&s_imu, &sample) != ESP_OK) {
            ESP_LOGW(TAG, "ICM42688P sample read failed");
            continue;
        }
        imu_to_body(&sample);

        const float gyro_rad[3] = {
            sample.gyro_dps[0] * DEG_TO_RAD,
            sample.gyro_dps[1] * DEG_TO_RAD,
            sample.gyro_dps[2] * DEG_TO_RAD,
        };
        if (have_mag && mag_mapping_ready && mag_is_calibrated()) {
            ahrs_update_mag(&s_ahrs, gyro_rad, sample.accel_g, mag_body, dt,
                            1.0f, mag_fresh ? 1.0f : 0.0f);
        } else {
            ahrs_update(&s_ahrs, gyro_rad, sample.accel_g, dt, 1.0f);
        }
        publish_angles(have_mag && mag_mapping_ready && mag_is_calibrated());

        if (now >= next_stream) {
            next_stream = now + 100000;
            display_state_t pose = get_display_state();
            bool mag_heading_valid = false;
            float mag_heading = 0.0f;
            if (have_mag && mag_mapping_ready && mag_is_calibrated()) {
                mag_heading = ahrs_mag_heading_deg(sample.accel_g, mag_body,
                                                   &mag_heading_valid);
            }
            printf("ATT,%lld,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,"
                   "%d,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                   (long long)(now / 1000), (double)pose.roll, (double)pose.pitch,
                   (double)pose.yaw, have_mag && mag_mapping_ready ? 1 : 0,
                   have_mag && mag_mapping_ready && mag_is_calibrated() ? 1 : 0,
                   (int)(sample.accel_g[0] * 1000.0f),
                   (int)(sample.accel_g[1] * 1000.0f),
                   (int)(sample.accel_g[2] * 1000.0f),
                   mag_fresh ? 1 : 0, mag_heading_valid ? 1 : 0,
                   (double)mag_heading, (double)mag_body[0],
                   (double)mag_body[1], (double)mag_body[2],
                   (double)mag_raw_body[0], (double)mag_raw_body[1],
                   (double)mag_raw_body[2]);
        }
    }
}
