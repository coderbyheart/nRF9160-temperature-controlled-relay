#include <cloud.h>
#include <cJSON.h>
#include <zephyr.h>
#include <stdbool.h>
#include <stdlib.h>
#include <net/aws_iot.h>

#define MIN_VALID_TS 1500000000000

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

static cJSON *json_object_decode(cJSON *obj, const char *str)
{
	return obj ? cJSON_GetObjectItem(obj, str) : NULL;
}

int cloud_decode_response(char *input, struct desired_state *cfg)
{
	char *string = NULL;
	cJSON *root_obj = NULL;
	cJSON *state_obj = NULL;
	cJSON *threshold = NULL;

	if (input == NULL) {
		return -EINVAL;
	}

	root_obj = cJSON_Parse(input);
	if (root_obj == NULL) {
		return -ENOENT;
	}

	string = cJSON_Print(root_obj);
	if (string == NULL) {
		printk("Failed to print message.");
		goto exit;
	}

	printk("Decoded message: %s\n", string);

	state_obj = json_object_decode(root_obj, "state");
	if (state_obj == NULL) {
		goto exit;
	}

	threshold = cJSON_GetObjectItem(state_obj, "threshold");

	if (threshold != NULL) {
		cfg->threshold = threshold->valuedouble;
	}
exit:
	cJSON_Delete(root_obj);
	return 0;
}

int cloud_report_state(struct current_state *currentState, struct track_reported *trackReported)
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

	if (!trackReported->version) {
		err += json_add_str(reported_obj, "app_version", CONFIG_APP_VERSION);
	}

	bool prendingReportedSwitch = currentState->switchState;
	if (trackReported->switchState != prendingReportedSwitch) {
		err += json_add_boolean(reported_obj, "switchState", prendingReportedSwitch);
		if (currentState->switchStateTs > MIN_VALID_TS) {
			err += json_add_number(reported_obj, "switchStateTs", currentState->switchStateTs);
		}
	}

	double pendingReportedCurrentTemperature = currentState->temperature;
	if (trackReported->temperature != pendingReportedCurrentTemperature) {
		err += json_add_number(reported_obj, "temp", pendingReportedCurrentTemperature);
		if (currentState->temperatureTs > MIN_VALID_TS) {
			err += json_add_number(reported_obj, "tempTs", currentState->temperatureTs);
		}
	}

	double pendingReportedThreshold = currentState->threshold;
	if (trackReported->threshold != pendingReportedThreshold) {
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
		trackReported->version = true;
		trackReported->switchState = prendingReportedSwitch;
		trackReported->temperature = pendingReportedCurrentTemperature;
		trackReported->threshold = pendingReportedThreshold;
	}

	k_free(message);

cleanup:

	cJSON_Delete(root_obj);

	return err;
}