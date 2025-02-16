#
# Copyright (c) 2021 Bosch Sensortec GmbH
# Copyright (c) 2022 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: Apache-2.0
#

zephyr_include_directories(.)

zephyr_library_sources(bmi270plus.c)
zephyr_library_sources_ifdef(CONFIG_BMI270_PLUS_BUS_I2C bmi270plus_i2c.c)
zephyr_library_sources_ifdef(CONFIG_BMI270_PLUS_BUS_SPI bmi270plus_spi.c)
zephyr_library_sources_ifdef(CONFIG_BMI270_PLUS_TRIGGER bmi270plus_trigger.c)
