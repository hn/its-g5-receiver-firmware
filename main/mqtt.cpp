#include "sdkconfig.h"

#include <cmath>
#include <cstring>

#include "ArduinoJson.h"

#include "esp_app_desc.h"
#include "esp_console.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_timer.h"
#include "mqtt_client.h"

#include "cmd_sniffer.h"
#include "config.h"
#include "lte_ppp.h"
#include "ethernet.h"
#include "events.h"
#include "temperature.h"

#include "mqtt.h"

constexpr char TAG[] = "MQTT";

static esp_mqtt_client_handle_t client;
static bool connected;

static char topic_prefix[96];
static char command_topic[128];
static char result_topic[128];
static char packet_topic[128];
static char status_topic[128];
static char stats_topic[128];
static char info_topic[128];
static char position_topic[128];

static esp_timer_handle_t stats_timer_handle;

static void mqtt_set_connected(bool new_connected)
{
    if (new_connected != connected)
    {
        connected = new_connected;
        esp_event_post(MQTT_EVENT_BASE, new_connected ? MQTT_CONNECTED : MQTT_DISCONNECTED, NULL, 0, 0);
    }
}

static void publish_node_info(void)
{
    char info[128];
    JsonDocument doc;

    char mac[6*2+5+1];
    {
        uint8_t eth_mac[6];
        ethernet_get_mac(eth_mac);

        snprintf(mac, sizeof(mac), "%02hhx:%02hhx:%02hhx:%02hhx:%02hhx:%02hhx",
                 eth_mac[0], eth_mac[1], eth_mac[2], eth_mac[3], eth_mac[4], eth_mac[5]);
    }

    doc["emac"] = mac;

    const esp_app_desc_t *app_desc = esp_app_get_description();
    doc["ver"] = app_desc->version;

    doc["hwv"] = CONFIG_HW_VARIANT;

    size_t written = serializeJson(doc, info);
    esp_mqtt_client_publish(client, info_topic, info, written, 0, 0);
}

/*
 * Publish the last GNSS position, if the board has GNSS.
 *
 * Unreachable on purpose, and not behind a Kconfig option: neither the topic
 * nor the payload schema is agreed with upstream, and an option would promise a
 * working feature.  A `static const bool` rather than #if 0 so the compiler
 * keeps type-checking this against lte_gnss_get_fix() and the JSON document --
 * it cannot rot silently -- while the optimiser drops the branch, so a shipped
 * image pays nothing for it.
 *
 * Pulled from the stats timer rather than pushed from the LTE task: that reuses
 * an existing 60 s cadence, cannot attempt a publish while MQTT is down, and
 * keeps the send rate independent of the modem loop.  The age of the fix goes
 * into the payload so a stale position is visible to the receiver instead of
 * looking current.
 */
static void publish_gnss_position(void)
{
#if defined(CONFIG_USE_LTE_PPP) && defined(CONFIG_LTE_SIMCOM_GNSS)
    static const bool gnss_publish_enabled = false;

    /* Early, so the optimiser drops everything below and a shipped image pays
     * nothing -- while the compiler still parses and type-checks it. */
    if (!gnss_publish_enabled)
        return;

    lte_gnss_fix_t fix;

    if (!lte_gnss_get_fix(&fix))
        return;

    char payload[160];
    JsonDocument doc;

    doc["lat"] = fix.lat;
    doc["ns"] = fix.ns;
    doc["lon"] = fix.lon;
    doc["ew"] = fix.ew;
    doc["alt"] = fix.alt;
    doc["hdop"] = fix.hdop;
    doc["sats"] = fix.sats_gps + fix.sats_glonass + fix.sats_galileo +
                  fix.sats_beidou;
    doc["age"] = (uint32_t)((esp_timer_get_time() - fix.uptime_us) / 1000000);

    size_t written = serializeJson(doc, payload);
    esp_mqtt_client_publish(client, position_topic, payload, written, 0, 0);
#endif
}

static void publish_stats_timer_cb(void *)
{
    char stats[128];
    JsonDocument doc;

#ifdef CONFIG_ENABLE_TEMPERATURE
    float temp_f = temperature_get();
    if (!isnanf(temp_f))
    {
        temp_f = roundf(temp_f * 10.f) / 10.f;
        doc["temp"] = temp_f;
    }
#endif

    uint64_t seconds_since_boot = esp_timer_get_time() / 1000000ull;
    doc["rbt"] = seconds_since_boot;

    size_t written = serializeJson(doc, stats);
    esp_mqtt_client_publish(client, stats_topic, stats, written, 0, 0);

    publish_gnss_position();
}

static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
    ESP_LOGD(TAG, "Event dispatched from event loop base=%s, event_id=%d", base, event_id);

    esp_mqtt_event_handle_t event = static_cast<esp_mqtt_event_handle_t>(event_data);
    esp_mqtt_client_handle_t client = event->client;
    int msg_id;
    // your_context_t *context = event->context;
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            mqtt_set_connected(true);
            ESP_LOGI(TAG, "MQTT_EVENT_CONNECTED");
            msg_id = esp_mqtt_client_subscribe(client, command_topic, 0);
            ESP_LOGI(TAG, "sent subscribe to '%s', msg_id=%d", command_topic, msg_id);

            esp_mqtt_client_publish(client, status_topic, "online", sizeof("online") - 1, 0, 1);

            publish_node_info();
            publish_stats_timer_cb(NULL);

            esp_timer_start_periodic(stats_timer_handle, 60000000);

            break;
        case MQTT_EVENT_DISCONNECTED:
            mqtt_set_connected(false);
            ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");
            break;
        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_UNSUBSCRIBED:
            ESP_LOGI(TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id);
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "MQTT_EVENT_DATA");
            ESP_LOGD(TAG, "TOPIC=%.*s\r\n", event->topic_len, event->topic);
            ESP_LOGD(TAG, "DATA=%.*s\r\n", event->data_len, event->data);
            if (strlen(command_topic) == event->topic_len && !strncmp(event->topic, command_topic, event->topic_len))
            {
                std::string cmd{event->data, static_cast<size_t>(event->data_len)};
                ESP_LOGI(TAG, "Running command '%s'", cmd.c_str());

                int ret;
                int res = esp_console_run(cmd.c_str(), &ret);
                char result_buf[128];
                if (res != ESP_OK)
                {
                    snprintf(result_buf, sizeof(result_buf), "esp_console_run failed: %s", esp_err_to_name(res));
                    ESP_LOGE(TAG, "%s", result_buf);
                    break;
                }
                else
                {
                    snprintf(result_buf, sizeof(result_buf), "%d", ret);
                }

                esp_mqtt_client_publish(client, result_topic, result_buf, strlen(result_buf), 0, 0);
            }
            break;
        case MQTT_EVENT_ERROR:
            mqtt_set_connected(false);
            ESP_LOGI(TAG, "MQTT_EVENT_ERROR");
            if (event->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                ESP_LOGI(TAG, "Last error code reported from esp-tls: 0x%x", event->error_handle->esp_tls_last_esp_err);
                ESP_LOGI(TAG, "Last tls stack error number: 0x%x", event->error_handle->esp_tls_stack_err);
                ESP_LOGI(TAG, "Last captured errno : %d (%s)",  event->error_handle->esp_transport_sock_errno,
                                                                strerror(event->error_handle->esp_transport_sock_errno));
            } else if (event->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                ESP_LOGI(TAG, "Connection refused error: 0x%x", event->error_handle->connect_return_code);
            } else {
                ESP_LOGW(TAG, "Unknown error type: 0x%x", event->error_handle->error_type);
            }
            break;
        default:
            ESP_LOGI(TAG, "Other event id:%d", event->event_id);
            break;
    }
}

static int make_topic_prefix()
{
    char nodeid[CONFIG_NODEID_BUFFER_SIZE];
    size_t size = sizeof(nodeid);

    esp_err_t res = config_get_str(CONFIG_INDEX_NODEID, nodeid, &size);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get node ID from config: %s", esp_err_to_name(res));
        return res;
    }

    snprintf(topic_prefix, sizeof(topic_prefix), "its/%.*s/", size, nodeid);

    return ESP_OK;
}

void mqtt_start(void)
{
    if (client)
    {
        ESP_LOGW(TAG, "MQTT already started");
        return;
    }

    if (make_topic_prefix() != ESP_OK)
    {
        ESP_LOGW(TAG, "make_topic_prefix failed");
        return;
    }

    strcpy(command_topic, topic_prefix);
    strcat(command_topic, "command");

    strcpy(result_topic, topic_prefix);
    strcat(result_topic, "command_result");

    strcpy(packet_topic, topic_prefix);
    strcat(packet_topic, "packet");

    strcpy(status_topic, topic_prefix);
    strcat(status_topic, "status");

    strcpy(info_topic, topic_prefix);
    strcat(info_topic, "info");

    strcpy(stats_topic, topic_prefix);
    strcat(stats_topic, "stats");

    strcpy(position_topic, topic_prefix);
    strcat(position_topic, "position");

    esp_mqtt_client_config_t mqtt_cfg{};

    char mqtt_uri[CONFIG_MQTT_URI_BUFFER_SIZE];
    size_t size = sizeof(mqtt_uri);
    esp_err_t res = config_get_str(CONFIG_INDEX_MQTT_URI, mqtt_uri, &size);
    if (res != ESP_OK)
    {
        ESP_LOGE(TAG, "Could not get MQTT URI from config: %s", esp_err_to_name(res));
        return;
    }

    mqtt_cfg.broker.address.uri = mqtt_uri;
    mqtt_cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
    mqtt_cfg.session.last_will = {
        .topic = status_topic,
        .msg = "offline",
        .msg_len = sizeof("offline") - 1,
        .qos = 0,
        .retain = 1
    };

    esp_mqtt_client_handle_t client_ = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client_, static_cast<esp_mqtt_event_id_t>(ESP_EVENT_ANY_ID), mqtt_event_handler, client_);
    esp_mqtt_client_start(client_);

    client = client_;
    ESP_LOGI(TAG, "MQTT started");
}

void mqtt_stop(void)
{
    if (!client)
    {
        ESP_LOGW(TAG, "Attempted to stop MQTT while not running");
        return;
    }

    esp_timer_stop_blocking(stats_timer_handle, 1000);

    esp_mqtt_client_stop(client);
    mqtt_set_connected(false);

    esp_mqtt_client_handle_t client_ = client;
    client = NULL;
    esp_mqtt_client_destroy(client_);

    ESP_LOGI(TAG, "MQTT stopped");
}

static void app_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case APP_ETHERNET_MGMT_INTERFACE_GOT_IP:
        mqtt_start();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_LOST_IP:
        mqtt_stop();
        break;
    }
}

extern "C" void mqtt_handle_packet(sniffer_packet_info_t *packet)
{
    if (!client || !connected)
        return;

    esp_mqtt_client_publish(client, packet_topic, static_cast<const char *>(packet->payload), packet->length, 0, 0);
}

extern "C" void mqtt_init(void)
{
    esp_event_handler_register(APP_EVENT_BASE, ESP_EVENT_ANY_ID, app_event_handler, NULL);

    esp_timer_create_args_t create_args = {
        .callback = publish_stats_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "mqtt_stats",
        .skip_unhandled_events = true
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &stats_timer_handle));
}
