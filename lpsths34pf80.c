/* Includes ------------------------------------------------------------------*/
#include "lpsths34pf80.h"

#include "esp_err.h"
#include "ulp_lp_core_print.h"

/* Private macros ------------------------------------------------------------*/
#define NOP() asm volatile("nop")

#define STHS34PF80_I2C_BUFFER_LEN_MAX (256)
#define STHS34PF80_BOOT_TIME_MS       (10)
#define LP_I2C_TRANS_TIMEOUT_CYCLES   5000
#define LP_I2C_TRANS_WAIT_FOREVER     -1
#define STHS34PF80_I2C_ADDR           0x5A

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
esp_err_t sths34pf80_init(lpsths34pf80_t *const me, i2c_port_t i2c_num, uint8_t dev_addr)
{
    lp_core_printf("Initializing STHS34PF80 instance...\n");

    /* Add device to I2C bus */

    me->i2c_dev = NULL;

    /* Fill driver interface */
    me->stmdev_ctx.write_reg = i2c_write;
    me->stmdev_ctx.read_reg  = i2c_read;
    me->stmdev_ctx.handle    = me->i2c_dev;
    me->stmdev_ctx.mdelay    = delay_ms;

    lp_i2c_num = i2c_num;

    /* Wait for sensor boot time */
    delay_ms(STHS34PF80_BOOT_TIME_MS);

    /* Check device ID */
    uint8_t whoami;
    sths34pf80_device_id_get(&me->stmdev_ctx, &whoami);

    if (whoami != STHS34PF80_ID) {
        lp_core_printf("Invalid device ID\n");
        return ESP_FAIL;
    } else {
        lp_core_printf("Device ID: %02X\n", whoami);
    }

    /* Set averages (AVG_TAMB = 8, AVG_TMOS = 32) */
    sths34pf80_avg_tobject_num_t testData;
    sths34pf80_avg_tobject_num_get(&me->stmdev_ctx, &testData);
    lp_core_printf("testData: %d\n", testData);
    int32_t avgErr = sths34pf80_avg_tobject_num_set(&me->stmdev_ctx, STHS34PF80_AVG_TMOS_32);
    sths34pf80_avg_tobject_num_get(&me->stmdev_ctx, &testData);
    lp_core_printf("testData: %d\n", testData);

    sths34pf80_avg_tambient_num_t testData2;
    sths34pf80_avg_tambient_num_get(&me->stmdev_ctx, &testData2);
    lp_core_printf("testData2: %d\n", testData2);

    int32_t tAmbErr = sths34pf80_avg_tambient_num_set(&me->stmdev_ctx, STHS34PF80_AVG_T_8);
    sths34pf80_avg_tambient_num_get(&me->stmdev_ctx, &testData2);
    lp_core_printf("testData2: %d\n", testData2);

    lp_core_printf("avgErr: %d, tAmbErr: %d\n", avgErr, tAmbErr);
    return ESP_OK;

    /* Read filters */
    sths34pf80_lpf_bandwidth_t lpf_m, lpf_p, lpf_p_m, lpf_a_t;
    sths34pf80_lpf_m_bandwidth_get(&me->stmdev_ctx, &lpf_m);
    sths34pf80_lpf_p_bandwidth_get(&me->stmdev_ctx, &lpf_p);
    sths34pf80_lpf_p_m_bandwidth_get(&me->stmdev_ctx, &lpf_p_m);
    sths34pf80_lpf_a_t_bandwidth_get(&me->stmdev_ctx, &lpf_a_t);

    lp_core_printf("lpf_m: %02d, lpf_p: %02d, lpf_p_m: %02d, lpf_a_t: %02d\n", lpf_m, lpf_p, lpf_p_m, lpf_a_t);

    /* Set BDU */
    int32_t blockErr = sths34pf80_block_data_update_set(&me->stmdev_ctx, 1);
    lp_core_printf("blockErr: %d\n", blockErr);
    ulp_lp_core_delay_us(10);

    // 这一行有问题
    int32_t odrErr = sths34pf80_odr_set(&me->stmdev_ctx, STHS34PF80_ODR_AT_1Hz);

    lp_core_printf("blockErr: %d, odrErr: %d\n", blockErr, odrErr);
    /////////int32_t odrErr = sths34pf80_set_odr(&me->stmdev_ctx, STHS34PF80_ODR_AT_1Hz);
    if (avgErr != 0) {
        return avgErr;
    } else if (tAmbErr != 0) {
        return tAmbErr;
    } else if (blockErr != 0) {
        return blockErr;
    } else if (odrErr != 0) {
        return odrErr;
    }
    lp_core_printf(("avgErr: %d, tAmbErr: %d, blockErr: %d, odrErr: %d\n"), avgErr, tAmbErr, blockErr, odrErr);

    // If no errors, return 0
}

/**
 * @brief Function to set the ODR (Output Data Rate)
 */
esp_err_t sths34pf80_set_odr(lpsths34pf80_t *const me, sths34pf80_odr_t val)
{
    /* Variable to return error code */
    esp_err_t ret = ESP_OK;

    /**/
    if (sths34pf80_odr_set(&me->stmdev_ctx, val) != 0) {
        lp_core_printf("Failed to set ODR\n");
        return ESP_FAIL;
    }

    /* Return ESP_OK */
    return ret;
}

static esp_err_t i2c_read(void *handle, uint8_t reg_addr, uint8_t *reg_data, uint16_t data_len)
{
    // lp_core_printf("lp_i2c_num: %d, reg_addr: %d, data_len: %d\n", lp_i2c_num, reg_addr, data_len);
    uint8_t write_data[1];
    write_data[0] = reg_addr;
    // lp_core_printf("submem_addr: %d\n", write_data[0]);
    esp_err_t ret = lp_core_i2c_master_write_read_device(lp_i2c_num, STHS34PF80_I2C_ADDR, write_data, 1, reg_data,
                                                         data_len, LP_I2C_TRANS_TIMEOUT_CYCLES);

    // lp_core_printf("ret: %d\n", ret);
    return ret;
}

// static esp_err_t i2c_write(void *handle, uint8_t reg_addr, const uint8_t *reg_data, uint16_t data_len)
// {
//     lp_core_printf("lp_i2c_num: %d, reg_addr: %d, data_len: %d\n", lp_i2c_num, reg_addr, data_len);
//     return lp_core_i2c_master_write_to_device(lp_i2c_num, reg_addr, reg_data, data_len, LP_I2C_TRANS_TIMEOUT_CYCLES);
// }

static esp_err_t i2c_write(void *handle, uint8_t reg_addr, const uint8_t *reg_data, uint16_t data_len)
{
    lp_core_printf("reg_addr: %d, reg_data:%d, data_len: %d\n", reg_addr, *reg_data, data_len);
    uint8_t buffer[data_len + 1];
    buffer[0] = reg_addr;
    memcpy(&buffer[1], reg_data, data_len);
    lp_core_printf("buffer[0]: %d buffer[1]:%d\n", buffer[0], buffer[1]);
    esp_err_t ret = lp_core_i2c_master_write_to_device(lp_i2c_num, STHS34PF80_I2C_ADDR, buffer, sizeof(buffer),
                                                       LP_I2C_TRANS_WAIT_FOREVER);
    lp_core_printf("ret: %d\n", ret);
    return ret;
}
/**
 * @brief Function that implements a micro seconds delay
 */
static void delay_ms(uint32_t period_ms)
{
    uint32_t period_us = period_ms * 1000;

    ulp_lp_core_delay_us(period_us);
}

/***************************** END OF FILE ************************************/
