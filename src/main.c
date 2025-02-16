/*
 * Copyright (c) 2021 Bosch Sensortec GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>

#include "bmi270plus.h"

int main(void) {
    int ret = 0;
    const struct device *const dev = DEVICE_DT_GET_ONE(bosch_bmi270plus);
    struct sensor_value acc[3], gyr[3], step_counter;
    struct sensor_value full_scale, sampling_freq, oversampling;

    if (!device_is_ready(dev)) {
        printf("Device %s is not ready\n", dev->name);
        return 0;
    }

    full_scale.val1 = 2; /* G */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100; /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 1; /* Normal mode */
    oversampling.val2 = 0;

    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    /* Setting scale in degrees/s to match the sensor scale */
    full_scale.val1 = 500; /* dps */
    full_scale.val2 = 0;
    sampling_freq.val1 = 100; /* Hz. Performance mode */
    sampling_freq.val2 = 0;
    oversampling.val1 = 1; /* Normal mode */
    oversampling.val2 = 0;

    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_FULL_SCALE, &full_scale);
    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_OVERSAMPLING, &oversampling);
    sensor_attr_set(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &sampling_freq);

    sensor_attr_set(dev, SENSOR_CHAN_STEP_CNT, SENSOR_ATTR_OFFSET, &step_counter);

    while (1) {
        ret = sensor_sample_fetch(dev);
        if (ret == 0) {
            sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, acc);
            sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyr);
            sensor_channel_get(dev, SENSOR_CHAN_STEP_CNT, &step_counter);

            printf(
                "AX: %d.%06d; AY: %d.%06d; AZ: %d.%06d; "
                "GX: %d.%06d; GY: %d.%06d; GZ: %d.%06d; "
                "Step counter: %d\n",
                acc[0].val1, acc[0].val2, acc[1].val1, acc[1].val2, acc[2].val1, acc[2].val2, gyr[0].val1, gyr[0].val2,
                gyr[1].val1, gyr[1].val2, gyr[2].val1, gyr[2].val2, step_counter.val1);
        } else {
            printf("Error: %d\n", ret);
        }

        k_sleep(K_SECONDS(1));
    }

    return 0;
}