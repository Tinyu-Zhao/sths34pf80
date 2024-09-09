/**
 ******************************************************************************
 * @file           : sths34pf80.c
 * @author         : Mauricio Barroso Benavides
 * @date           : Jun 18, 2024
 * @brief          : todo: write brief
 ******************************************************************************
 * @attention
 *
 * MIT License
 *
 * Copyright (c) 2024 Mauricio Barroso Benavides
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "sths34pf80.h"

/* Private macros ------------------------------------------------------------*/
#define NOP() asm volatile("nop")

#define STHS34PF80_I2C_BUFFER_LEN_MAX (256)
#define STHS34PF80_BOOT_TIME_MS       (10)

/* External variables --------------------------------------------------------*/

/* Private typedef -----------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/
static const char *TAG = "sths34pf80";
i2c_port_t lp_i2c_num;
/* Private function prototypes -----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 *
 * @param handle   : Pointer to the interface descriptor
 * @param reg_addr : Register address to be read
 * @param reg_data : Pointer to the data to be read from reg_addr
 * @param data_len : Length of the data transfer
 *
 * @return 0 if successful, non-zero otherwise
 */
static esp_err_t i2c_read(void *handle, uint8_t reg_addr, uint8_t *reg_data, uint16_t data_len);
/**
 * @brief Function that implements the default I2C write transaction
 *
 * @param handle   : Pointer to the interface descriptor
 * @param reg_addr : Register address to be written
 * @param reg_data : Pointer to the data to be written to reg_addr
 * @param data_len : Length of the data transfer
 *
 * @return 0 if successful, non-zero otherwise
 */
static esp_err_t i2c_write(void *handle, uint8_t reg_addr, const uint8_t *reg_data, uint16_t data_len);
/**
 * @brief Function that implements a micro seconds delay
 *
 * @param period_us: Time in us to delay
 */
static void delay_ms(uint32_t period_ms);

/* Exported functions definitions --------------------------------------------*/
/**
 * @brief Function to initialize a STHS34PF80 instance
 */
esp_err_t sths34pf80_init(sths34pf80_t *const me, i2c_bus_handle_t i2c_bus_handle, uint8_t dev_addr)
{
    ESP_LOGI(TAG, "Initializing STHS34PF80 instance...");

    /* Add device to I2C bus */

    me->i2c_dev = i2c_bus_device_create(i2c_bus_handle, dev_addr, 400000);
    if (me->i2c_dev == NULL) {
        ESP_LOGE(TAG, "Failed to add device to I2C bus");
    } else {
        ESP_LOGI(TAG, "sths34pf80 create success");
    }

    // i2c_device_config_t i2c_dev_conf = {.scl_speed_hz = 400000, .device_address = dev_addr >> 1};
    // if (i2c_master_bus_add_device(i2c_bus_handle, &i2c_dev_conf, &me->i2c_dev) != ESP_OK) {
    //     ESP_LOGE(TAG, "Failed to add device to I2C bus");
    //     return ret;
    // }

    /* Fill driver interface */
    me->stmdev_ctx.write_reg = i2c_write;
    me->stmdev_ctx.read_reg  = i2c_read;
    me->stmdev_ctx.handle    = me->i2c_dev;
    me->stmdev_ctx.mdelay    = delay_ms;

    /* Wait for sensor boot time */
    delay_ms(STHS34PF80_BOOT_TIME_MS);

    /* Check device ID */
    uint8_t whoami;
    sths34pf80_device_id_get(&me->stmdev_ctx, &whoami);

    if (whoami != STHS34PF80_ID) {
        ESP_LOGE(TAG, "Invalid device ID");
        return ESP_FAIL;
    } else {
        ESP_LOGI(TAG, "Device ID: %02X", whoami);
    }

    /* Set averages (AVG_TAMB = 8, AVG_TMOS = 32) */
    int32_t avgErr  = sths34pf80_avg_tobject_num_set(&me->stmdev_ctx, STHS34PF80_AVG_TMOS_32);
    int32_t tAmbErr = sths34pf80_avg_tambient_num_set(&me->stmdev_ctx, STHS34PF80_AVG_T_8);

    /* Read filters */
    sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;
    sths34pf80_lpf_m_bandwidth_get(&me->stmdev_ctx, &lpf_m);
    sths34pf80_lpf_p_bandwidth_get(&me->stmdev_ctx, &lpf_p);
    sths34pf80_lpf_p_m_bandwidth_get(&me->stmdev_ctx, &lpf_p_m);
    sths34pf80_lpf_a_t_bandwidth_get(&me->stmdev_ctx, &lpf_a_t);

    ESP_LOGI(TAG, "lpf_m: %02d, lpf_p: %02d, lpf_p_m: %02d, lpf_a_t: %02d", lpf_m, lpf_p, lpf_p_m, lpf_a_t);

    /* Set BDU */
    int32_t blockErr = sths34pf80_block_data_update_set(&me->stmdev_ctx, 1);

    int32_t odrErr = sths34pf80_set_odr(&me->stmdev_ctx, STHS34PF80_ODR_AT_1Hz);

    if (avgErr != 0) {
        return avgErr;
    } else if (tAmbErr != 0) {
        return tAmbErr;
    } else if (blockErr != 0) {
        return blockErr;
    } else if (odrErr != 0) {
        return odrErr;
    }

    // If no errors, return 0
    return ESP_OK;
}

/**
 * @brief Function to set the ODR (Output Data Rate)
 */
esp_err_t sths34pf80_set_odr(sths34pf80_t *const me, sths34pf80_odr_t val)
{
    /* Variable to return error code */
    esp_err_t ret = ESP_OK;

    /**/
    if (sths34pf80_odr_set(&me->stmdev_ctx, val) != 0) {
        ESP_LOGE(TAG, "Failed to set ODR");
        return ESP_FAIL;
    }

    /* Return ESP_OK */
    return ret;
}

/* Private function definitions ----------------------------------------------*/
/**
 * @brief Function that implements the default I2C read transaction
 */
// static int32_t i2c_read(void *handle, uint8_t reg_addr, uint8_t *reg_data, uint16_t data_len) {
//     i2c_master_dev_handle_t i2c_dev = (i2c_master_dev_handle_t)handle;

//     if (i2c_master_transmit_receive(i2c_dev, &reg_addr, 1, reg_data, data_len, -1) != ESP_OK) {
//         return -1;
//     }

//     return 0;
// }

static esp_err_t i2c_read(void *handle, uint8_t reg_addr, uint8_t *reg_data, uint16_t data_len)
{
    i2c_bus_device_handle_t i2c_dev = (i2c_bus_device_handle_t)handle;
    return i2c_bus_read_bytes(i2c_dev, reg_addr, data_len, reg_data);
}

static esp_err_t i2c_write(void *handle, uint8_t reg_addr, const uint8_t *reg_data, uint16_t data_len)
{
    i2c_bus_device_handle_t i2c_dev = (i2c_bus_device_handle_t)handle;
    return i2c_bus_write_bytes(i2c_dev, reg_addr, data_len, reg_data);
}

/**
 * @brief Function that implements a micro seconds delay
 */
static void delay_ms(uint32_t period_ms)
{
    uint32_t period_us = period_ms * 1000;

    uint64_t m = (uint64_t)esp_timer_get_time();

    if (period_us) {
        uint64_t e = (m + period_us);

        if (m > e) { /* overflow */
            while ((uint64_t)esp_timer_get_time() > e) {
                NOP();
            }
        }

        while ((uint64_t)esp_timer_get_time() < e) {
            NOP();
        }
    }
}

/***************************** END OF FILE ************************************/
