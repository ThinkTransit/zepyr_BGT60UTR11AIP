/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>

#define SPIOP SPI_WORD_SET(8) | SPI_TRANSFER_MSB

struct spi_dt_spec spispec = SPI_DT_SPEC_GET(DT_NODELABEL(gendev), SPIOP, 0);

void spi_write_register(uint8_t reg, uint32_t value)
{
    uint8_t tx_buf[4] = {
            (reg << 1) | 0x01,
            (value >> 16) & 0xFF,
            (value >> 8) & 0xFF,
            value & 0xFF
    };

    struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };

    int err = spi_write_dt(&spispec, &tx_set);
    if (err) {
        printk("SPI write reg 0x%02X failed: %d\n", reg, err);
    } else {
        printk("Wrote 0x%06X to reg 0x%02X\n", value, reg);
    }
}

void test_spi_write_readback(void)
{
    if (!spi_is_ready_dt(&spispec)) {
        printk("SPI device not ready\n");
        return;
    }

    const uint8_t reg = 0x06;  // ADC_CONFIG register
    const uint32_t test_value = 0x005500;  // Example test pattern

    spi_write_register(reg, test_value);

    uint8_t tx_buf[4] = { (reg << 1) & 0xFE, 0x00, 0x00, 0x00 };
    uint8_t rx_buf[4] = { 0 };

    struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
    struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };

    struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

    int err = spi_transceive_dt(&spispec, &tx_set, &rx_set);
    if (err) {
        printk("SPI readback failed: %d\n", err);
        return;
    }

    uint32_t read_value = (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
    printk("Read back reg 0x%02X = 0x%06X (expected 0x%06X)\n",
           reg, read_value, test_value);
}

void test_spi_read_main_reg(void)
{
    if (!spi_is_ready_dt(&spispec)) {
        printk("SPI device not ready\n");
        return;
    }

    for (uint8_t reg = 0x00; reg <= 0x02; reg++) {
        uint8_t tx_buf[4] = { (reg << 1) & 0xFE, 0x00, 0x00, 0x00 };
        uint8_t rx_buf[4] = { 0 };

        struct spi_buf tx = { .buf = tx_buf, .len = sizeof(tx_buf) };
        struct spi_buf rx = { .buf = rx_buf, .len = sizeof(rx_buf) };

        struct spi_buf_set tx_set = { .buffers = &tx, .count = 1 };
        struct spi_buf_set rx_set = { .buffers = &rx, .count = 1 };

        int err = spi_transceive_dt(&spispec, &tx_set, &rx_set);
        if (err) {
            printk("SPI read reg 0x%02X failed: %d\n", reg, err);
            continue;
        }

        uint32_t value = (rx_buf[1] << 16) | (rx_buf[2] << 8) | rx_buf[3];
        printk("Read reg 0x%02X = 0x%06X (GSR0: 0x%02X)\n", reg, value, rx_buf[0]);
    }
}

int main(void)
{
    printk("Starting\n");
    test_spi_write_readback();
    test_spi_read_main_reg();

    return 0;
}
