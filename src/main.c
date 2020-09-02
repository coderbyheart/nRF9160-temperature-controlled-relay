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
#include <net/aws_iot.h>
#include <power/reboot.h>
#include <date_time.h>
#include <dfu/mcuboot.h>
#include <modem/lte_lc.h>
#include <modem/bsdlib.h>
#include <modem/at_cmd.h>
#include <modem/at_notif.h>
#include <modem/modem_info.h>
#include <bsd.h>

static struct k_delayed_work read_sensor_data_work;
static struct k_delayed_work report_state_work;

K_SEM_DEFINE(lte_connected, 0, 1);

uint16_t sensorUpdateIntervalSeconds = 10;
uint16_t cloudPublishIntervalSeconds = 60;
double cloudPublishThreshold = 0.1;
double threshold = 24;
double currentTemperature;
int64_t currentTemperatureTs;
#define RELAY_PIN 17

bool switchState = true;
int64_t switchStateTs;
uint16_t sensorErrorCount = 0;
uint16_t maxSensorErrorCount = 3;

#define MIN_VALID_TS 1500000000000

bool reportedVersion = false;
double reportedCurrentTemperature = -127;
double reportedThreshold = -127;
bool reportedSwitchState = false;

static void button_handler(uint32_t button_states, uint32_t has_changed)
{
	bool triggerPublication = false;
	if ((has_changed & button_states & DK_BTN1_MSK)) {
		threshold = threshold - 1;
		printf("Button - pressed: Threshold is now %d\n", (int)threshold);
		triggerPublication = true;
	}
	if ((has_changed & button_states & DK_BTN2_MSK)) {
		threshold = threshold + 1;
		printf("Button + pressed: Threshold is now %d\n", (int)threshold);
		triggerPublication = true;
	}
	if (triggerPublication) {
		// Trigger sensor read
		k_delayed_work_cancel(&read_sensor_data_work);
		k_delayed_work_submit(&read_sensor_data_work, K_SECONDS(1));
		// Sensor read might cancel this, but if not, schedule report of new threshold
		k_delayed_work_cancel(&report_state_work);
		k_delayed_work_submit(&report_state_work, K_SECONDS(5));
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

	int dtError = date_time_now(&currentTemperatureTs);
	if (dtError) {
		printk("Failed to get current time: error %d\n", dtError);
	}
	
	int rc = sensor_sample_fetch(dht22);
	if (rc != 0) {
		printf("Sensor fetch failed: %d\n", rc);
		sensorErrorCount += 1;
		printf("Sensor fetch error count: %d\n", sensorErrorCount);
		if (sensorErrorCount > maxSensorErrorCount) {
			switchState = true; // Safety fallback
			switchStateTs = currentTemperatureTs;
			printf("Safety fallback triggered.\n");
		}
	} else {
		bool triggerPublication = false;
		sensorErrorCount = 0;
		struct sensor_value temperature;
		rc = sensor_channel_get(dht22, SENSOR_CHAN_AMBIENT_TEMP, &temperature);
		if (rc != 0) {
			printf("get failed: %d\n", rc);
		} else {
			currentTemperature = sensor_value_to_double(&temperature);
		}
		if (currentTemperature < threshold) {
			if (switchState) {
				printf("Turning off...\n");
				switchStateTs = currentTemperatureTs;
				switchState = false;
				triggerPublication = true;
			}
		} else {
			if (!switchState) {
				printf("Turning on...\n");
				switchStateTs = currentTemperatureTs;
				switchState = true;
				triggerPublication = true;
			}
		}
		if (triggerPublication) {
			k_delayed_work_cancel(&report_state_work);
			k_delayed_work_submit(&report_state_work, K_SECONDS(1));
		}
		printf("[%lld] Temp: %.1f | Threshold: %.1f | Switch: %s\n", currentTemperatureTs, currentTemperature, threshold, switchState ? "On" : "Off");
	}
	dk_set_led(DK_LED1, (int)switchState); // LED1 status
	gpio_pin_set(dev, RELAY_PIN, (int)(!switchState));
	dk_set_led(DK_LED2, 0); // LED2 off
	// Schedule next read
	k_delayed_work_submit(&read_sensor_data_work, K_SECONDS(sensorUpdateIntervalSeconds));
}

static int json_add_obj(cJSON *parent, const char *str, cJSON *object)
{
	cJSON_AddItemToObject(parent, str, object);

	return 0;
}

static int json_add_str(cJSON *parent, const char *str, const char *value)
{
	cJSON *json_str = cJSON_CreateString(value);
	if (json_str == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_str);
}

static int json_add_number(cJSON *parent, const char *str, double value)
{
	cJSON *json_num = cJSON_CreateNumber(value);
	if (json_num == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, json_num);
}

static int json_add_boolean(cJSON *parent, const char *str, bool value)
{
	cJSON *b = value ? cJSON_CreateTrue() : cJSON_CreateFalse();
	if (b == NULL) {
		return -ENOMEM;
	}

	return json_add_obj(parent, str, b);
}

static int report_state()
{
	int err;
	char *message;

	cJSON *root_obj = cJSON_CreateObject();
	cJSON *state_obj = cJSON_CreateObject();
	cJSON *reported_obj = cJSON_CreateObject();

	if (root_obj == NULL || state_obj == NULL || reported_obj == NULL) {
		cJSON_Delete(root_obj);
		cJSON_Delete(state_obj);
		cJSON_Delete(reported_obj);
		err = -ENOMEM;
		return err;
	}

	err = 0;

	if (!reportedVersion) {
		err += json_add_str(reported_obj, "app_version", CONFIG_APP_VERSION);
	}

	bool pendingReportedSwitchState = switchState;
	if (reportedSwitchState != pendingReportedSwitchState) {
		err += json_add_boolean(reported_obj, "switchState", pendingReportedSwitchState);
		if (switchStateTs > MIN_VALID_TS) {
			err += json_add_number(reported_obj, "switchStateTs", switchStateTs);
		}
	}

	double pendingReportedCurrentTemperature = currentTemperature;
	if (reportedCurrentTemperature != pendingReportedCurrentTemperature) {
		err += json_add_number(reported_obj, "temp", pendingReportedCurrentTemperature);
		if (currentTemperatureTs > MIN_VALID_TS) {
			err += json_add_number(reported_obj, "tempTs", currentTemperatureTs);
		}
	}

	double pendingReportedThreshold = threshold;
	if (reportedThreshold != pendingReportedThreshold) {
		err += json_add_number(reported_obj, "threshold", pendingReportedThreshold);
	}

	err += json_add_obj(state_obj, "reported", reported_obj);
	err += json_add_obj(root_obj, "state", state_obj);

	if (err < 0) {
		printk("json_add, error: %d\n", err);
		goto cleanup;
	}

	message = cJSON_Print(root_obj);
	if (message == NULL) {
		printk("cJSON_Print, error: returned NULL\n");
		err = -ENOMEM;
		goto cleanup;
	}

	struct aws_iot_data tx_data = {
		.qos = MQTT_QOS_0_AT_MOST_ONCE,
		.topic.type = AWS_IOT_SHADOW_TOPIC_UPDATE,
		.ptr = message,
		.len = strlen(message)
	};

	printk("Publishing: %s to AWS IoT broker\n", message);

	err = aws_iot_send(&tx_data);
	if (err) {
		printk("aws_iot_send, error: %d\n", err);
	} else {
		reportedVersion = true;
		reportedSwitchState = pendingReportedSwitchState;
		reportedCurrentTemperature = pendingReportedCurrentTemperature;
		reportedThreshold = pendingReportedThreshold;
	}

	k_free(message);

cleanup:

	cJSON_Delete(root_obj);

	return err;
}

static bool needsPublish() {
	if (!reportedVersion) return true;
	if (reportedSwitchState != switchState) return true;
	if (reportedThreshold != threshold) return true;
	if (reportedCurrentTemperature > currentTemperature && reportedCurrentTemperature - currentTemperature > cloudPublishThreshold) return true;
	if (reportedCurrentTemperature < currentTemperature && currentTemperature - reportedCurrentTemperature > cloudPublishThreshold) return true;
	return false;
}

static void report_state_work_fn(struct k_work *work)
{
	if (!needsPublish()) {
		printk("No updates to report.\n");
	} else {
		int err;
		err = report_state();
		if (err) {
			printk("report_state, error: %d\n", err);
		}
	}

	// Schedule next publication
	k_delayed_work_submit(&report_state_work, K_SECONDS(cloudPublishIntervalSeconds));
}

void aws_iot_event_handler(const struct aws_iot_evt *const evt)
{
	int err;

	switch (evt->type) {
	case AWS_IOT_EVT_CONNECTING:
		printf("Connecting to AWS IoT...\n");
		break;
	case AWS_IOT_EVT_CONNECTED:
		printf("Connected to AWS IoT.\n");
		dk_set_led(DK_LED3, 1); // LED3 on while connected

		if (evt->data.persistent_session) {
			printk("Persistent session enabled\n");
		}

		/** Successfully connected to AWS IoT broker, mark image as
		 *  working to avoid reverting to the former image upon reboot.
		 */
		boot_write_img_confirmed();

		/** Send version number to AWS IoT broker to verify that the
		 *  FOTA update worked.
		 */
		k_delayed_work_submit(&report_state_work, K_NO_WAIT);

		err = lte_lc_psm_req(true);
		if (err) {
			printk("Requesting PSM failed, error: %d\n", err);
		}

		break;
	case AWS_IOT_EVT_READY:
		// Ignore
		break;
	case AWS_IOT_EVT_DISCONNECTED:
		printf("Disconnected from AWS IoT.\n");
		k_delayed_work_cancel(&report_state_work);
		dk_set_led(DK_LED3, 0); // LED3 off while connected
		break;
	case AWS_IOT_EVT_DATA_RECEIVED:
		printk("AWS_IOT_EVT_DATA_RECEIVED\n");
		break;
	case AWS_IOT_EVT_FOTA_START:
		printk("AWS_IOT_EVT_FOTA_START\n");
		break;
	case AWS_IOT_EVT_FOTA_ERASE_PENDING:
		printk("AWS_IOT_EVT_FOTA_ERASE_PENDING\n");
		printk("Disconnect LTE link or reboot\n");
		err = lte_lc_offline();
		if (err) {
			printk("Error disconnecting from LTE\n");
		}
		break;
	case AWS_IOT_EVT_FOTA_ERASE_DONE:
		printk("AWS_FOTA_EVT_ERASE_DONE\n");
		printk("Reconnecting the LTE link");
		err = lte_lc_connect();
		if (err) {
			printk("Error connecting to LTE\n");
		}
		break;
	case AWS_IOT_EVT_FOTA_DONE:
		printk("AWS_IOT_EVT_FOTA_DONE\n");
		printk("FOTA done, rebooting device\n");
		aws_iot_disconnect();
		sys_reboot(0);
		break;
	case AWS_IOT_EVT_FOTA_DL_PROGRESS:
		printk("AWS_IOT_EVT_FOTA_DL_PROGRESS, (%d%%)", evt->data.fota_progress);
	case AWS_IOT_EVT_ERROR:
		printk("AWS_IOT_EVT_ERROR, %d\n", evt->data.err);
		break;
	default:
		printk("Unknown AWS IoT event type: %d\n", evt->type);
		break;
	}
}

static void lte_handler(const struct lte_lc_evt *const evt)
{
	switch (evt->type) {
	case LTE_LC_EVT_NW_REG_STATUS:
		if ((evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_HOME) &&
		     (evt->nw_reg_status != LTE_LC_NW_REG_REGISTERED_ROAMING)) {
			break;
		}

		printk("Network registration status: %s\n",
			evt->nw_reg_status == LTE_LC_NW_REG_REGISTERED_HOME ?
			"Connected - home network" : "Connected - roaming");

		k_sem_give(&lte_connected);
		break;
	case LTE_LC_EVT_PSM_UPDATE:
		printk("PSM parameter update: TAU: %d, Active time: %d\n",
			evt->psm_cfg.tau, evt->psm_cfg.active_time);
		break;
	case LTE_LC_EVT_EDRX_UPDATE: {
		char log_buf[60];
		ssize_t len;

		len = snprintf(log_buf, sizeof(log_buf),
			       "eDRX parameter update: eDRX: %f, PTW: %f",
			       evt->edrx_cfg.edrx, evt->edrx_cfg.ptw);
		if (len > 0) {
			printk("%s\n", log_buf);
		}
		break;
	}
	case LTE_LC_EVT_RRC_UPDATE:
		// printk("RRC mode: %s\n", evt->rrc_mode == LTE_LC_RRC_MODE_CONNECTED ? "Connected" : "Idle");
		break;
	case LTE_LC_EVT_CELL_UPDATE:
		printk("LTE cell changed: Cell ID: %d, Tracking area: %d\n",
			evt->cell.id, evt->cell.tac);
		break;
	default:
		break;
	}
}

static void modem_configure(void)
{
	int err = lte_lc_init_and_connect_async(lte_handler);
	if (err) {
		printk("Modem could not be configured, error: %d\n", err);
	}
}

static void at_configure(void)
{
	int err;

	err = at_notif_init();
	__ASSERT(err == 0, "AT Notify could not be initialized.");
	err = at_cmd_init();
	__ASSERT(err == 0, "AT CMD could not be established.");
}

static void bsd_lib_modem_dfu_handler(void)
{
	int err;

	err = bsdlib_init();

	switch (err) {
	case MODEM_DFU_RESULT_OK:
		printk("Modem update suceeded, reboot\n");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case MODEM_DFU_RESULT_UUID_ERROR:
	case MODEM_DFU_RESULT_AUTH_ERROR:
		printk("Modem update failed, error: %d\n", err);
		printk("Modem will use old firmware\n");
		sys_reboot(SYS_REBOOT_COLD);
		break;
	case MODEM_DFU_RESULT_HARDWARE_ERROR:
	case MODEM_DFU_RESULT_INTERNAL_ERROR:
		printk("Modem update malfunction, error: %d, reboot\n", err);
		sys_reboot(SYS_REBOOT_COLD);
		break;
	default:
		break;
	}

	at_configure();
}

static void work_init(void)
{
	k_delayed_work_init(&read_sensor_data_work, read_sensor_data_work_fn);
	k_delayed_work_submit(&read_sensor_data_work, K_SECONDS(15));
	k_delayed_work_init(&report_state_work, report_state_work_fn);
}

void main(void) {
	int err;

	// Initialize Buttons
	err = dk_buttons_init(button_handler);
	if (err != 0) {
		printf("dk_buttons_init, error: %d", err);
		return;
	}

	// Initialize LEDs
	err = dk_leds_init();
	if (err != 0) {
		printf("ledError, error: %d", err);
		return;
	}

	printf("##########################################################################################\n");
	printf("Version:                   %s\n", CONFIG_APP_VERSION);
	printf("Temperature read interval: %d seconds\n", sensorUpdateIntervalSeconds);
	printf("Max sensor read error:     %d\n", maxSensorErrorCount);
	printf("Temperature threshold:     %d degree\n", (int)threshold);
	printf("AWS IoT Client ID:         %s\n", CONFIG_AWS_IOT_CLIENT_ID_STATIC);
	printf("AWS IoT broker hostname:   %s\n", CONFIG_AWS_IOT_BROKER_HOST_NAME);
	printf("Publish min interval:      %d seconds\n", cloudPublishIntervalSeconds);
	printf("Publish threshold:         %.1f degree\n", cloudPublishThreshold);
	printf("##########################################################################################\n");

	cJSON_Init();

	bsd_lib_modem_dfu_handler();

	work_init();

	int awsErr = aws_iot_init(NULL, aws_iot_event_handler);
	if (awsErr) {
		printk("AWS IoT library could not be initialized, error: %d\n", awsErr);
		return;
	}

	modem_configure();

	printk("Initializing modem ...\n");
	err = modem_info_init();
	if (err) {
		printk("Failed initializing modem info module, error: %d\n",
			err);
	}
	k_sem_take(&lte_connected, K_FOREVER);

	date_time_update_async();
	// Sleep to ensure that time has been obtained before communication with AWS IoT.
	printf("Waiting 15 seconds before attempting to connect...\n");
	k_sleep(K_SECONDS(15));
	
	err = aws_iot_connect(NULL);
	if (err) {
		printk("aws_iot_connect failed: %d\n", err);
	}
}
