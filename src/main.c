/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <drivers/gpio.h>
#include <stdio.h>
#include <stdlib.h>
#include <devicetree.h>
#include <dk_buttons_and_leds.h>
#include <cJSON.h>
#include <cJSON_os.h>

static struct k_delayed_work read_sensor_data_work;

uint8_t interval = 10;
double threshold = 24;
bool isOn = true;
#define RELAY_PIN 17

static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	if ((has_changed & button_states & DK_BTN1_MSK)) {
		threshold = threshold - 1;
		printf("Button - pressed: Threshold is now %d\n", (int)threshold);
	}
	if ((has_changed & button_states & DK_BTN2_MSK)) {
		threshold = threshold + 1;
		printf("Button + pressed: Threshold is now %d\n", (int)threshold);
	}
}

static void read_sensor_data_work_fn(struct k_work *work)
{
	dk_set_led(DK_LED2, 1); // LED2 on while working

	// Initialize DHT sensor
	const char *const label = DT_LABEL(DT_INST(0, aosong_dht));
	struct device *dht22 = device_get_binding(label);
	if (!dht22) {
		printf("Failed to find sensor %s!\n", label);
		return;
	}

	// Initialize GPIO for Relay
	struct device *dev = device_get_binding("GPIO_0");
	if (!dev) {
		printf("Failed get binding for GPIO_0!\n");
		return;
	}
	gpio_pin_configure(dev, RELAY_PIN, GPIO_OUTPUT);
	
	int rc = sensor_sample_fetch(dht22);
	if (rc != 0) {
		printf("Sensor fetch failed: %d\n", rc);
		isOn = true; // Safety fallback
	} else {
		struct sensor_value temperature;
		rc = sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		if (rc != 0) {
			printf("get failed: %d\n", rc);
		} else {
			printf("Temp: %.1f\n", sensor_value_to_double(&temperature));
		}
		if (sensor_value_to_double(&temperature) < threshold) {
			if (isOn) {
				printf("Turning off...\n");
			}
			isOn = false;
		} else {
			if (!isOn) {
				printf("Turning on...\n");
			}
			isOn = true;
		}
	}
	dk_set_led(DK_LED1, (int)isOn); // LED1 status
	gpio_pin_set(dev, RELAY_PIN, (int)(!isOn));
	dk_set_led(DK_LED2, 0); // LED2 off

	printk("Next sensor read in %d seconds\n", interval);

	k_delayed_work_submit(&read_sensor_data_work, K_SECONDS(interval));
}

static void work_init(void)
{
	k_delayed_work_init(&read_sensor_data_work, read_sensor_data_work_fn);
	k_delayed_work_submit(&read_sensor_data_work, K_NO_WAIT);
}

void main(void) {
	// Initialize Buttons
	int buttonError = dk_buttons_init(button_handler);
	if (buttonError != 0) {
		printf("dk_buttons_init, error: %d", buttonError);
		return;
	}

	// Initialize LEDs
	int ledError = dk_leds_init();
	if (ledError != 0) {
		printf("ledError, error: %d", ledError);
		return;
	}

	// LEDs
	dk_set_led(DK_LED3, 0); // LED3 static off
	dk_set_led(DK_LED4, 0); // LED3 static off

	printf("Version: %s\n", CONFIG_APP_VERSION);
	printf("Reading temperature every %d seconds\n", interval);
	printf("Temperature threshold: %d\n", (int)threshold);

	cJSON_Init();

	work_init();
}
