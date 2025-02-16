#ifndef DRV_ACC_H
#define DRV_ACC_H

#include <stdbool.h>
#include <stdint.h>

/** Module ID */
#define DRV_ACC_ID (0x00A2)

/**< Device ID */
typedef uint8_t drv_acc_device_id_t;

typedef enum _ERROR_E {
    APP_SUCCESS,
    APP_ERROR_INVALID_PARAM,
    APP_ERROR_INVALID_STATE,
    APP_ERROR_TIMEOUT,
    APP_ERROR_INTERNAL,
} error_t;

typedef enum _DRV_ACC_FS_MODE_E { DRV_ACC_FS_2G, DRV_ACC_FS_4G, DRV_ACC_FS_8G, DRV_ACC_FS_16G } drv_acc_fs_mode_t;

typedef enum _DRV_ACC_RES_MODE_E {
    DRV_ACC_RES_MODE_NORMAL,
    DRV_ACC_RES_MODE_LOW_POWER,
    DRV_ACC_RES_MODE_HIGH
} drv_acc_res_mode_t;

typedef enum _DRV_ACC_INT_ROUTE_E {
    DRV_ACC_INT_ROUTE_CLICK,
    DRV_ACC_INT_ROUTE_INT_ACTY_1,
    DRV_ACC_INT_ROUTE_INT_ACTY_2,
    DRV_ACC_INT_ROUTE_ZYX_DATA,
    DRV_ACC_INT_ROUTE_WATERMARK,
    DRV_ACC_INT_ROUTE_OVERRUN,
    DRV_ACC_INT_ROUTE_BOOT,
    DRV_ACC_INT_ROUTE_ACTIVITY
} drv_acc_int_route_t;

typedef enum _DRV_ACC_ODR_E {
    DRV_ACC_ODR_POWER_DOWN,
    DRV_ACC_ODR_1HZ,
    DRV_ACC_ODR_10HZ,
    DRV_ACC_ODR_25HZ,
    DRV_ACC_ODR_50HZ,
    DRV_ACC_ODR_100HZ,
    DRV_ACC_ODR_200HZ,
    DRV_ACC_ODR_400HZ,
    DRV_ACC_ODR_1_6KHZ,
    DRV_ACC_ODR_1_344KHZ
} drv_acc_odr_t;

/**< XYZ Data Struct */
typedef struct _DRV_ACC_XYZ_DATA {
    signed short x;
    signed short y;
    signed short z;
} drv_acc_xyz_data_t;

typedef enum _DRV_ACC_FIFO_MODE_E {
    DRV_ACC_FIFO_MODE_BYPASS,
    DRV_ACC_FIFO_MODE_FIFO,
    DRV_ACC_FIFO_MODE_STREAM,
    DRV_ACC_FIFO_MODE_STREAM_FIFO
} drv_acc_fifo_mode_t;

/* ---------------------------------------------- */

error_t drv_acc_full_scale_set(drv_acc_fs_mode_t fs);
error_t drv_acc_axis_enable(bool x, bool y, bool z);
error_t drv_acc_get_device_id(drv_acc_device_id_t* dev_id);

error_t drv_acc_odr_set(uint8_t odr);
error_t drv_acc_res_mode_set(drv_acc_res_mode_t mode);

error_t drv_acc_int1_route(drv_acc_int_route_t int_route);

error_t drv_acc_fifo_enable(bool fifo_en);
error_t drv_acc_fifo_mode_set(drv_acc_fifo_mode_t mode);
error_t drv_acc_fifo_threshold_set(uint8_t ths);
error_t drv_acc_fifo_data_get(drv_acc_xyz_data_t* data, uint8_t samples);

/* ---------------------------------------------- */

/*	You don't need to implement these functions.
    These functions it's only to comunicate with accelerometer.
*/
error_t acc_tx_data(uint8_t reg_addr, uint8_t* tx_data, uint16_t tx_data_len);
error_t acc_rx_data(uint8_t reg_addr, uint8_t* rx_data, uint16_t rx_data_len);

#endif  // DRV_ACC_H
