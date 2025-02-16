#include "drv_acc.h"

#include <stddef.h>

/* Register Addresses */
#define LIS2DH12_CTRL_REG1     0x20
#define LIS2DH12_CTRL_REG2     0x21
#define LIS2DH12_CTRL_REG3     0x22
#define LIS2DH12_CTRL_REG4     0x23
#define LIS2DH12_CTRL_REG5     0x24
#define LIS2DH12_CTRL_REG6     0x25
#define LIS2DH12_WHO_AM_I      0x0F
#define LIS2DH12_REFERENCE     0x26
#define LIS2DH12_FIFO_CTRL_REG 0x2E
#define LIS2DH12_FIFO_SRC_REG  0x2F
#define LIS2DH12_OUT_X_L       0x28

/* Bit Masks */
#define LIS2DH12_CTRL_REG4_FS_MASK      0x30
#define LIS2DH12_CTRL_REG1_ODR_MASK     0xF0
#define LIS2DH12_FIFO_CTRL_REG_FM_MASK  0xC0
#define LIS2DH12_FIFO_CTRL_REG_FTH_MASK 0x1F
#define FIFO_SRC_REG_FSS_MASK           0x1F
#define LIS2DH12_CTRL_REG1_LPEN         (1 << 3)
#define LIS2DH12_CTRL_REG4_HR           (1 << 3)
#define LIS2DH12_CTRL_REG3_I1_CLICK     (1 << 7)
#define LIS2DH12_CTRL_REG3_I1_IA1       (1 << 6)
#define LIS2DH12_CTRL_REG3_I1_IA2       (1 << 5)
#define LIS2DH12_CTRL_REG3_I1_ZYXDA     (1 << 4)
#define LIS2DH12_CTRL_REG3_I1_WTM       (1 << 2)
#define LIS2DH12_CTRL_REG3_I1_OVERRUN   (1 << 1)
#define LIS2DH12_CTRL_REG5_FIFO_EN      (1 << 7)

/* Full-Scale Selection Values */
#define LIS2DH12_FS_2G  0x00
#define LIS2DH12_FS_4G  0x10
#define LIS2DH12_FS_8G  0x20
#define LIS2DH12_FS_16G 0x30

/* Axis Enable values */
#define LIS2DH12_CTRL_REG1_XEN (1 << 0)
#define LIS2DH12_CTRL_REG1_YEN (1 << 1)
#define LIS2DH12_CTRL_REG1_ZEN (1 << 2)

/* Output Data Rate (ODR) Selection Values */
#define LIS2DH12_ODR_POWER_DOWN 0x00
#define LIS2DH12_ODR_1HZ        0x10
#define LIS2DH12_ODR_10HZ       0x20
#define LIS2DH12_ODR_25HZ       0x30
#define LIS2DH12_ODR_50HZ       0x40
#define LIS2DH12_ODR_100HZ      0x50
#define LIS2DH12_ODR_200HZ      0x60
#define LIS2DH12_ODR_400HZ      0x70
#define LIS2DH12_ODR_1_6KHZ     0x80
#define LIS2DH12_ODR_1_344KHZ   0x90

/* FIFO Mode Selection Values */
#define LIS2DH12_FIFO_CTRL_REG_FM_BYPASS         0x00
#define LIS2DH12_FIFO_CTRL_REG_FM_FIFO           0x40
#define LIS2DH12_FIFO_CTRL_REG_FM_STREAM         0x80
#define LIS2DH12_FIFO_CTRL_REG_FM_STREAM_TO_FIFO 0xC0

/* Simulate communication with accelerometer */
error_t acc_tx_data(uint8_t reg_addr, uint8_t* tx_data, uint16_t tx_data_len) {
    return APP_SUCCESS;
}

/* Simulate communication with accelerometer */
error_t acc_rx_data(uint8_t reg_addr, uint8_t* rx_data, uint16_t rx_data_len) {
    return APP_SUCCESS;
}

/* Sets the full scale of the accelerometer */
error_t drv_acc_full_scale_set(drv_acc_fs_mode_t fs) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of CTRL_REG4
    err = acc_rx_data(LIS2DH12_CTRL_REG4, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the FS bits
    reg_value &= ~LIS2DH12_CTRL_REG4_FS_MASK;

    // Set the FS bits based on the desired full-scale range
    switch (fs) {
        case DRV_ACC_FS_2G:
            reg_value |= LIS2DH12_FS_2G;
            break;
        case DRV_ACC_FS_4G:
            reg_value |= LIS2DH12_FS_4G;
            break;
        case DRV_ACC_FS_8G:
            reg_value |= LIS2DH12_FS_8G;
            break;
        case DRV_ACC_FS_16G:
            reg_value |= LIS2DH12_FS_16G;
            break;
        default:
            return APP_ERROR_INVALID_PARAM;
    }

    // Write the updated value back to CTRL_REG4
    return acc_tx_data(LIS2DH12_CTRL_REG4, &reg_value, sizeof(reg_value));
}

/* Enables the specified axes of the accelerometer */
error_t drv_acc_axis_enable(bool x, bool y, bool z) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of CTRL_REG1
    err = acc_rx_data(LIS2DH12_CTRL_REG1, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the axis enable bits
    reg_value &= ~(LIS2DH12_CTRL_REG1_XEN | LIS2DH12_CTRL_REG1_YEN | LIS2DH12_CTRL_REG1_ZEN);

    // Set the axis enable bits based on function parameters
    reg_value |= (x == true ? LIS2DH12_CTRL_REG1_XEN : 0);
    reg_value |= (y == true ? LIS2DH12_CTRL_REG1_YEN : 0);
    reg_value |= (z == true ? LIS2DH12_CTRL_REG1_ZEN : 0);

    // Write the updated value back to CTRL_REG1
    return acc_tx_data(LIS2DH12_CTRL_REG1, &reg_value, sizeof(reg_value));
}

/* Read device id register */
error_t drv_acc_get_device_id(drv_acc_device_id_t* dev_id) {
    error_t err;
    uint8_t reg_value = 0;

    // Check if the provided pointer is valid
    if (dev_id == NULL) {
        return APP_ERROR_INVALID_PARAM;
    }

    // Read who_am_i register
    err = acc_rx_data(LIS2DH12_WHO_AM_I, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Store the device ID in the provided pointer
    *dev_id = reg_value;

    return err;
}

/* Sets the output data rate of the accelerometer */
error_t drv_acc_odr_set(uint8_t odr) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of CTRL_REG1
    err = acc_rx_data(LIS2DH12_CTRL_REG1, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the ODR bits
    reg_value &= ~LIS2DH12_CTRL_REG1_ODR_MASK;

    // Set the ODR bits based on the desired output data rate
    switch (odr) {
        case DRV_ACC_ODR_POWER_DOWN:
            reg_value |= LIS2DH12_ODR_POWER_DOWN;
            break;
        case DRV_ACC_ODR_1HZ:
            reg_value |= LIS2DH12_ODR_1HZ;
            break;
        case DRV_ACC_ODR_10HZ:
            reg_value |= LIS2DH12_ODR_10HZ;
            break;
        case DRV_ACC_ODR_25HZ:
            reg_value |= LIS2DH12_ODR_25HZ;
            break;
        case DRV_ACC_ODR_50HZ:
            reg_value |= LIS2DH12_ODR_50HZ;
            break;
        case DRV_ACC_ODR_100HZ:
            reg_value |= LIS2DH12_ODR_100HZ;
            break;
        case DRV_ACC_ODR_200HZ:
            reg_value |= LIS2DH12_ODR_200HZ;
            break;
        case DRV_ACC_ODR_400HZ:
            reg_value |= LIS2DH12_ODR_400HZ;
            break;
        case DRV_ACC_ODR_1_6KHZ:
            reg_value |= LIS2DH12_ODR_1_6KHZ;
            break;
        case DRV_ACC_ODR_1_344KHZ:
            reg_value |= LIS2DH12_ODR_1_344KHZ;
            break;
        default:
            return APP_ERROR_INVALID_PARAM;
    }

    // Write the updated value back to CTRL_REG1
    err = acc_tx_data(LIS2DH12_CTRL_REG1, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // If switching to power-down mode from high-resolution mode, read the REFERENCE register
    if (odr == DRV_ACC_ODR_POWER_DOWN) {
        uint8_t ctrl_reg4_value;

        // Read the current value of CTRL_REG4
        err = acc_rx_data(LIS2DH12_CTRL_REG4, &ctrl_reg4_value, sizeof(ctrl_reg4_value));
        if (err != APP_SUCCESS) {
            return err;
        }

        // Check if high-resolution mode is enabled
        if (ctrl_reg4_value & 0x08) {  // HR bit is set
            uint8_t reference_value;

            // Read the REFERENCE register to reset the filtering block
            err = acc_rx_data(LIS2DH12_REFERENCE, &reference_value, sizeof(reference_value));
            if (err != APP_SUCCESS) {
                return err;
            }
        }
    }

    return err;
}

/* Sets the resolution mode of the accelerometer */
error_t drv_acc_res_mode_set(drv_acc_res_mode_t mode) {
    error_t err;
    uint8_t ctrl_reg1_value;
    uint8_t ctrl_reg4_value;

    // Read the current value of CTRL_REG1
    err = acc_rx_data(LIS2DH12_CTRL_REG1, &ctrl_reg1_value, sizeof(ctrl_reg1_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Read the current value of CTRL_REG4
    err = acc_rx_data(LIS2DH12_CTRL_REG4, &ctrl_reg4_value, sizeof(ctrl_reg4_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the LPen bit in CTRL_REG1 and the HR bit in CTRL_REG4
    ctrl_reg1_value &= ~LIS2DH12_CTRL_REG1_LPEN;
    ctrl_reg4_value &= ~LIS2DH12_CTRL_REG4_HR;

    // Set the bits according to the desired mode
    switch (mode) {
        case DRV_ACC_RES_MODE_LOW_POWER:
            ctrl_reg1_value |= LIS2DH12_CTRL_REG1_LPEN;  // Enable low-power mode
            break;
        case DRV_ACC_RES_MODE_NORMAL:
            // Both LPen and HR bits are cleared for normal mode
            break;
        case DRV_ACC_RES_MODE_HIGH:
            ctrl_reg4_value |= LIS2DH12_CTRL_REG4_HR;  // Enable high-resolution mode
            break;
        default:
            return APP_ERROR_INVALID_PARAM;
    }

    // Write the updated values back to the registers
    err = acc_tx_data(LIS2DH12_CTRL_REG1, &ctrl_reg1_value, sizeof(ctrl_reg1_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    return acc_tx_data(LIS2DH12_CTRL_REG4, &ctrl_reg4_value, sizeof(ctrl_reg4_value));
}

/* Sets the interrupt routing for INT1 */
error_t drv_acc_int1_route(drv_acc_int_route_t int_route) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of CTRL_REG3
    err = acc_rx_data(LIS2DH12_CTRL_REG3, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the relevant bits for interrupt routing
    reg_value &=
        ~(LIS2DH12_CTRL_REG3_I1_CLICK | LIS2DH12_CTRL_REG3_I1_IA1 | LIS2DH12_CTRL_REG3_I1_IA2 | LIS2DH12_CTRL_REG3_I1_ZYXDA |
          LIS2DH12_CTRL_REG3_I1_WTM | LIS2DH12_CTRL_REG3_I1_OVERRUN);

    // Set the bits according to the desired interrupt routing
    switch (int_route) {
        case DRV_ACC_INT_ROUTE_CLICK:
            reg_value |= LIS2DH12_CTRL_REG3_I1_CLICK;
            break;
        case DRV_ACC_INT_ROUTE_INT_ACTY_1:
            reg_value |= LIS2DH12_CTRL_REG3_I1_IA1;
            break;
        case DRV_ACC_INT_ROUTE_INT_ACTY_2:
            reg_value |= LIS2DH12_CTRL_REG3_I1_IA2;
            break;
        case DRV_ACC_INT_ROUTE_ZYX_DATA:
            reg_value |= LIS2DH12_CTRL_REG3_I1_ZYXDA;
            break;
        case DRV_ACC_INT_ROUTE_WATERMARK:
            reg_value |= LIS2DH12_CTRL_REG3_I1_WTM;
            break;
        case DRV_ACC_INT_ROUTE_OVERRUN:
            reg_value |= LIS2DH12_CTRL_REG3_I1_OVERRUN;
            break;
        case DRV_ACC_INT_ROUTE_BOOT:
            // Boot interrupt is routed through INT2, not INT1
            return APP_ERROR_INVALID_PARAM;
        case DRV_ACC_INT_ROUTE_ACTIVITY:
            // Activity interrupt is routed through INT2, not INT1
            return APP_ERROR_INVALID_PARAM;
        default:
            return APP_ERROR_INVALID_PARAM;
    }

    // Write the updated value back to CTRL_REG3
    return acc_tx_data(LIS2DH12_CTRL_REG3, &reg_value, sizeof(reg_value));
}

/* Enables or disables the FIFO functionality */
error_t drv_acc_fifo_enable(bool fifo_en) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of CTRL_REG5
    err = acc_rx_data(LIS2DH12_CTRL_REG5, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Set or clear the FIFO_EN bit based on fifo_en parameter
    if (fifo_en) {
        reg_value |= LIS2DH12_CTRL_REG5_FIFO_EN;
    } else {
        reg_value &= ~LIS2DH12_CTRL_REG5_FIFO_EN;
    }

    // Write the updated value back to CTRL_REG5
    return acc_tx_data(LIS2DH12_CTRL_REG5, &reg_value, sizeof(reg_value));
}

/* Sets the mode of the FIFO */
error_t drv_acc_fifo_mode_set(drv_acc_fifo_mode_t mode) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of FIFO_CTRL_REG
    err = acc_rx_data(LIS2DH12_FIFO_CTRL_REG, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the existing FIFO mode bits
    reg_value &= ~LIS2DH12_FIFO_CTRL_REG_FM_MASK;

    // Set the FIFO mode based on the provided mode
    switch (mode) {
        case DRV_ACC_FIFO_MODE_BYPASS:
            reg_value |= LIS2DH12_FIFO_CTRL_REG_FM_BYPASS;
            break;
        case DRV_ACC_FIFO_MODE_FIFO:
            reg_value |= LIS2DH12_FIFO_CTRL_REG_FM_FIFO;
            break;
        case DRV_ACC_FIFO_MODE_STREAM:
            reg_value |= LIS2DH12_FIFO_CTRL_REG_FM_STREAM;
            break;
        case DRV_ACC_FIFO_MODE_STREAM_FIFO:
            reg_value |= LIS2DH12_FIFO_CTRL_REG_FM_STREAM_TO_FIFO;
            break;
        default:
            return APP_ERROR_INVALID_PARAM;
    }

    // Write the updated value back to FIFO_CTRL_REG
    return acc_tx_data(LIS2DH12_FIFO_CTRL_REG, &reg_value, sizeof(reg_value));
}

/* Sets the threshold for the FIFO */
error_t drv_acc_fifo_threshold_set(uint8_t ths) {
    error_t err;
    uint8_t reg_value;

    // Read the current value of FIFO_CTRL_REG
    err = acc_rx_data(LIS2DH12_FIFO_CTRL_REG, &reg_value, sizeof(reg_value));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Clear the existing FIFO threshold bits
    reg_value &= ~LIS2DH12_FIFO_CTRL_REG_FTH_MASK;

    // Set the new FIFO threshold
    reg_value |= (ths & LIS2DH12_FIFO_CTRL_REG_FTH_MASK);

    // Write the updated value back to FIFO_CTRL_REG
    return acc_tx_data(LIS2DH12_FIFO_CTRL_REG, &reg_value, sizeof(reg_value));
}

/* Gets the data from the FIFO */
error_t drv_acc_fifo_data_get(drv_acc_xyz_data_t* data, uint8_t samples) {
    error_t err;
    uint8_t fifo_status = 0;
    uint8_t ctrl_reg1 = 0, ctrl_reg4 = 0;
    uint8_t out_reg[6];  // Buffer to store raw output data
    uint8_t shift_bits;

    // Read CTRL_REG1 and CTRL_REG4 to determine the operating mode
    err = acc_rx_data(LIS2DH12_CTRL_REG1, &ctrl_reg1, sizeof(ctrl_reg1));
    if (err != APP_SUCCESS) {
        return err;
    }

    err = acc_rx_data(LIS2DH12_CTRL_REG4, &ctrl_reg4, sizeof(ctrl_reg4));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Determine the number of bits to shift based on the operating mode
    bool lpen = ctrl_reg1 & LIS2DH12_CTRL_REG1_LPEN;
    bool hr = ctrl_reg4 & LIS2DH12_CTRL_REG4_HR;

    if (lpen) {
        // Low-Power Mode: 8-bit resolution
        shift_bits = 8;
    } else if (hr) {
        // High-Resolution Mode: 12-bit resolution
        shift_bits = 4;
    } else {
        // Normal Mode: 10-bit resolution
        shift_bits = 6;
    }

    // Read the FIFO status register to check the number of available samples
    err = acc_rx_data(LIS2DH12_FIFO_SRC_REG, &fifo_status, sizeof(fifo_status));
    if (err != APP_SUCCESS) {
        return err;
    }

    // Extract the number of samples available in FIFO
    uint8_t available_samples = fifo_status & 0x1F;  // FIFO_SRC_REG[4:0] indicates available samples

    // Ensure we do not read more samples than available
    uint8_t samples_to_read = (samples < available_samples) ? samples : available_samples;

    for (uint8_t i = 0; i < samples_to_read; i++) {
        // Read the output registers for X, Y, Z axes
        err = acc_rx_data(LIS2DH12_OUT_X_L, out_reg, sizeof(out_reg));
        if (err != APP_SUCCESS) {
            return err;
        }

        // Combine low and high bytes for each axis
        int16_t raw_x = (int16_t)((out_reg[1] << 8) | out_reg[0]);
        int16_t raw_y = (int16_t)((out_reg[3] << 8) | out_reg[2]);
        int16_t raw_z = (int16_t)((out_reg[5] << 8) | out_reg[4]);

        // Right-shift to align the data based on the operating mode
        data[i].x = raw_x >> shift_bits;
        data[i].y = raw_y >> shift_bits;
        data[i].z = raw_z >> shift_bits;
    }

    return err;
}