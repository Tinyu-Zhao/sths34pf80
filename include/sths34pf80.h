/**
 ******************************************************************************
 * @file           : sths34pf80.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef STHS34PF80_H_
#define STHS34PF80_H_

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include "sths34pf80_reg.h"
#include "driver/i2c_master.h"
#include "../../main/i2c_bus.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_err.h"

/* Exported Macros -----------------------------------------------------------*/

/* Exported typedef ----------------------------------------------------------*/
typedef void (*sths34pf80_cb_t)(void *);

typedef struct {
    stmdev_ctx_t stmdev_ctx;
    i2c_bus_device_handle_t i2c_dev; /*!< I2C device handle */
    sths34pf80_cb_t isr_cb;
    int32_t isr_gpio;
} sths34pf80_t;

/* Exported variables --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
/**
 * @brief Function to initialize a STHS34PF80 instance
 *
 * @param me             : Pointer to a sths34pf80_t instance
 * @param i2c_bus_handle : Handle to the I2C bus to add this device
 * @param dev_addr       : I2C device address
 *
 * @return ESP_OK on success
 */
esp_err_t sths34pf80_init(sths34pf80_t *const me, i2c_bus_handle_t i2c_bus_handle, uint8_t dev_addr);

/**
 * @brief Function to set the ODR (Output Data Rate)
 *
 * @param me  : Pointer to a sths34pf80_t instance
 * @param val : ODR value, can be 0 Hz, 0.25 Hz, 0.5 Hz, 1 Hz, 2 Hz, 4 Hz, 8 Hz,
 *              15 Hz or 30 Hz
 *
 * @return ESP_OK on success
 */
esp_err_t sths34pf80_set_odr(sths34pf80_t *const me, sths34pf80_odr_t val);

/**
 * @brief Function to set the ODR (Output Data Rate)
 *
 * @param me  : Pointer to a sths34pf80_t instance
 * @param val : ODR value, can be 0 Hz, 0.25 Hz, 0.5 Hz, 1 Hz, 2 Hz, 4 Hz, 8 Hz,
 *              15 Hz or 30 Hz
 *
 * @return ESP_OK on success
 */
esp_err_t sths34pf80_set_odr(sths34pf80_t *const me, sths34pf80_odr_t val);

#ifdef __cplusplus
}
#endif

#endif /* STHS34PF80_H_ */

/***************************** END OF FILE ************************************/
