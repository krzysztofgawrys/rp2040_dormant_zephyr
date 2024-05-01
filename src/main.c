/*
 * Copyright (c) 2022 Mr. Green's Workshop https://www.MrGreensWorkshop.com
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/drivers/gpio.h>
#include <inttypes.h>
#include <stddef.h>
#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/adc.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zephyr/pm/pm.h>
#include <zephyr/drivers/clock_control.h>

#include <hardware/clocks.h>
#include <hardware/xosc.h>
#include <hardware/structs/rosc.h>
#include <hardware/pll.h>
#include <hardware/watchdog.h>
#include <hardware/resets.h>

#include "sleep.h"
#include "gpio.h"
#include "stdio.h"

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define CLK_NODE DT_ALIAS(clocks)

static const struct gpio_dt_spec led0 = GPIO_DT_SPEC_GET(LED0_NODE, gpios);
static const struct device *const dev = DEVICE_DT_GET(CLK_NODE);

static uint scb_orig;
static uint clock0_orig;
static uint clock1_orig;
static uint syscfg_hw_orig;

void recover_from_sleep(uint scb_orig, uint clock0_orig, uint clock1_orig)
{

    // syscfg_hw->mempowerdown = syscfg_hw_orig;

    // Re-enable ring Oscillator control
    rosc_write(&rosc_hw->ctrl, ROSC_CTRL_ENABLE_BITS);

    // reset procs back to default
    scb_hw->scr = scb_orig;
    clocks_hw->sleep_en0 = clock0_orig;
    clocks_hw->sleep_en1 = clock1_orig;

    // reset clocks
    clocks_init();

    if (!gpio_is_ready_dt(&led0))
    {
        return 0;
    }

    int ret;
    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return 0;
    }
    // stdio_init_all();
    // i2c_init(i2c1, 100 * 1000);
    // gpio_set_function(PICO__SDA_PIN, GPIO_FUNC_I2C);
    // gpio_set_function(PICO__SCL_PIN, GPIO_FUNC_I2C);
    // bi_decl(bi_2pins_with_func(PICO__SDA_PIN, PICO__SCL_PIN, GPIO_FUNC_I2C));
    gpio_init(26);
    gpio_set_dir(26, GPIO_IN);
    gpio_set_input_hysteresis_enabled(0, 0);
    gpio_pull_up(26);

    return;
}

int main(void)
{
    int ret;
    bool led_state = true;

    if (!gpio_is_ready_dt(&led0))
    {
        return 0;
    }

    ret = gpio_pin_configure_dt(&led0, GPIO_OUTPUT_ACTIVE);
    if (ret < 0)
    {
        return 0;
    }

    ret = gpio_pin_toggle_dt(&led0);
    if (ret < 0)
    {
        return 0;
    }
    k_sleep(K_MSEC(125));
    ret = gpio_pin_toggle_dt(&led0);
    if (ret < 0)
    {
        return 0;
    }
    k_sleep(K_MSEC(125));
    ret = gpio_pin_toggle_dt(&led0);
    if (ret < 0)
    {
        return 0;
    }
    k_sleep(K_MSEC(125));
    ret = gpio_pin_toggle_dt(&led0);
    if (ret < 0)
    {
        return 0;
    }
    k_sleep(K_MSEC(125));
    ret = gpio_pin_toggle_dt(&led0);
    if (ret < 0)
    {
        return 0;
    }
    k_sleep(K_MSEC(125));
    ret = gpio_pin_toggle_dt(&led0);
    if (ret < 0)
    {
        return 0;
    }
    k_sleep(K_MSEC(125));

    gpio_init(26);
    gpio_set_dir(26, GPIO_IN);
    gpio_set_input_hysteresis_enabled(0, 0);
    gpio_pull_up(26);

    while (1)
    {
        led_state = !led_state;
        printf("LED state: %s\n", led_state ? "ON" : "OFF");

        enum clock_control_status clkstat;
        clkstat = clock_control_get_status(dev, 5);

        uint32_t rate = 0U;
        clock_control_get_rate(dev, 5, &rate);

        printf("Clock state: %i, %u\n", clkstat, rate);

        k_sleep(K_MSEC(250));

        ret = gpio_pin_toggle_dt(&led0);
        if (ret < 0)
        {
            return 0;
        }

        k_sleep(K_MSEC(250));

        scb_orig = scb_hw->scr;
        clock0_orig = clocks_hw->sleep_en0;
        clock1_orig = clocks_hw->sleep_en1;

        sleep_run_from_rosc();
        sleep_goto_dormant_until_pin(26, GPIO_IRQ_EDGE_FALL, false);
        recover_from_sleep(scb_orig, clock0_orig, clock1_orig);
    }

    return 0;
}

void k_sys_fatal_error_handler(unsigned int reason,
                               const z_arch_esf_t *esf)
{
    ARG_UNUSED(esf);
    ARG_UNUSED(reason);

    sys_reboot(1);
    CODE_UNREACHABLE;
}