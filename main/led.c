#include "sdkconfig.h"

#include <stdint.h>
#include <stdbool.h>

#include "esp_event.h"
#include "esp_timer.h"

#include "led_indicator.h"
#include "led_indicator_strips.h"

#include "config.h"
#include "ethernet.h"
#include "events.h"

#include "led.h"

led_indicator_handle_t led_handle;
bool sniffer_running;
bool mqtt_connected;

#define LED_IRGB(i, r, g, b) SET_IRGB(i, r, g, b)

static uint8_t led_brightness;

static void set_led_with_brightness(led_indicator_handle_t handle, uint32_t irgb)
{
    uint8_t i = GET_INDEX(irgb);
    uint8_t r = GET_RED(irgb);
    uint8_t g = GET_GREEN(irgb);
    uint8_t b = GET_BLUE(irgb);

    r = ((uint32_t)r) * ((uint32_t)led_brightness) / 255u;
    g = ((uint32_t)g) * ((uint32_t)led_brightness) / 255u;
    b = ((uint32_t)b) * ((uint32_t)led_brightness) / 255u;

    led_indicator_set_rgb(handle, SET_IRGB(i, r, g, b));
}

#if CONFIG_LEDSTRIP_COUNT == 1

/*
 * Single-LED mode: one WS2812 shows overall system state via color.
 * Priority (highest first):
 *   - Sniffer stopped:    Red
 *   - No IP:              Yellow
 *   - MQTT disconnected:  Green blink
 *   - MQTT connected:     Green solid
 *   - C-ITS packet flash: Blue (with MQTT) / Orange (without MQTT), 50ms
 */

typedef enum {
    LED_ST_SNIFFER_STOPPED,
    LED_ST_NO_IP,
    LED_ST_MQTT_DISCONNECTED,
    LED_ST_MQTT_CONNECTED,
} led_state_t;

static led_state_t current_state = LED_ST_SNIFFER_STOPPED;
static bool cits_flash_active = false;
static bool blink_on = true;
static bool has_ip = false;

static esp_timer_handle_t cits_timer_handle;
static esp_timer_handle_t blink_timer_handle;

static void update_single_led(void)
{
    if (cits_flash_active) {
        uint32_t color = mqtt_connected ?
            LED_IRGB(0, 0, 0, 0xFF) :       // Blue
            LED_IRGB(0, 0xFF, 0xA0, 0);     // Orange
        set_led_with_brightness(led_handle, color);
        return;
    }

    switch (current_state) {
    case LED_ST_SNIFFER_STOPPED:
        set_led_with_brightness(led_handle, LED_IRGB(0, 0xFF, 0, 0));      // Red
        break;
    case LED_ST_NO_IP:
        set_led_with_brightness(led_handle, LED_IRGB(0, 0xFF, 0xFF, 0));   // Yellow
        break;
    case LED_ST_MQTT_DISCONNECTED:
        if (blink_on)
            set_led_with_brightness(led_handle, LED_IRGB(0, 0, 0xFF, 0));  // Green
        else
            set_led_with_brightness(led_handle, LED_IRGB(0, 0, 0, 0));    // Off
        break;
    case LED_ST_MQTT_CONNECTED:
        set_led_with_brightness(led_handle, LED_IRGB(0, 0, 0xFF, 0));     // Green solid
        break;
    }
}

static void recalc_state(void)
{
    if (!sniffer_running) {
        current_state = LED_ST_SNIFFER_STOPPED;
    } else if (!has_ip) {
        current_state = LED_ST_NO_IP;
    } else if (!mqtt_connected) {
        current_state = LED_ST_MQTT_DISCONNECTED;
    } else {
        current_state = LED_ST_MQTT_CONNECTED;
    }
    update_single_led();
}

static void cits_timer_cb(void *)
{
    cits_flash_active = false;
    update_single_led();
}

static void blink_timer_cb(void *)
{
    blink_on = !blink_on;
    update_single_led();
}

static void app_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id) {
    case APP_ETHERNET_MGMT_INTERFACE_GOT_IP:
        has_ip = true;
        recalc_state();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_LOST_IP:
    case APP_ETHERNET_MGMT_INTERFACE_DISCONNECTED:
        has_ip = false;
        mqtt_connected = false;
        recalc_state();
        break;
    default:
        break;
    }
}

static void sniffer_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    switch (event_id) {
    case SNIFFER_RECEIVED_PACKET:
        cits_flash_active = true;
        update_single_led();
        if (esp_timer_restart(cits_timer_handle, 50000) == ESP_ERR_INVALID_STATE)
            esp_timer_start_once(cits_timer_handle, 50000);
        break;
    case SNIFFER_STARTED:
        sniffer_running = true;
        recalc_state();
        break;
    case SNIFFER_STOPPED:
        sniffer_running = false;
        recalc_state();
        break;
    }
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id) {
    case MQTT_CONNECTED:
        mqtt_connected = true;
        recalc_state();
        break;
    case MQTT_DISCONNECTED:
        mqtt_connected = false;
        recalc_state();
        break;
    }
}

void led_update(void)
{
    uint8_t brightness;
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;
    update_single_led();
}

void led_init(void)
{
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_LEDSTRIP_PIN,
            .max_leds = 1,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .led_model = LED_MODEL_WS2812
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {0}
    };
    led_indicator_config_t led_config = {
        .blink_lists = NULL,
        .blink_list_num = 0
    };
    ESP_ERROR_CHECK(led_indicator_new_strips_device(&led_config, &strips_config, &led_handle));

    esp_timer_create_args_t create_args = {
        .callback = cits_timer_cb,
        .arg = NULL,
        .name = "cits_led"
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &cits_timer_handle));

    create_args.callback = blink_timer_cb;
    create_args.name = "blink_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &blink_timer_handle));

    ESP_ERROR_CHECK(esp_event_handler_register(SNIFFER_EVENT_BASE, ESP_EVENT_ANY_ID, sniffer_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(MQTT_EVENT_BASE, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_EVENT_BASE, ESP_EVENT_ANY_ID, app_event_handler, NULL));

    uint8_t brightness;
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    // Start red (sniffer stopped)
    set_led_with_brightness(led_handle, LED_IRGB(0, 0xFF, 0, 0));

    // Start blink timer (used when in MQTT_DISCONNECTED state)
    esp_timer_start_periodic(blink_timer_handle, 500000);
}

#else /* CONFIG_LEDSTRIP_COUNT > 1 */

#define LED_SYSTEM  0
#define LED_SNIFFER 1
#define LED_ETH     2
#define LED_MQTT    3
#define LED_CITS    4

#define LED_ETH_COLOR_100M LED_IRGB(LED_ETH,    0, 0xFF, 0)
#define LED_ETH_COLOR_10M  LED_IRGB(LED_ETH, 0xFF, 0xA0, 0)

static uint32_t system_led_state  = LED_IRGB(LED_SYSTEM,  0xFF, 0xFF, 0xFF);
static uint32_t sniffer_led_state = LED_IRGB(LED_SNIFFER, 0xFF,    0,    0);
static uint32_t eth_led_state     = LED_IRGB(LED_ETH,        0,    0,    0);
static uint32_t cits_led_state    = LED_IRGB(LED_CITS,       0,    0,    0);
static uint32_t mqtt_led_state    = LED_IRGB(LED_MQTT,       0,    0,    0);

static bool system_led_blink_state;

static esp_timer_handle_t system_led_timer_handle;
static esp_timer_handle_t cits_led_timer_handle;
static esp_timer_handle_t eth_led_timer_handle;

static eth_speed_t eth_speed;
static bool eth_link_state;
static bool eth_led_blink_state;

static void set_eth_led_disconnected(void)
{
    esp_timer_stop_blocking(eth_led_timer_handle, 10 / portTICK_PERIOD_MS);

    eth_led_state = LED_IRGB(LED_ETH, 0, 0, 0);
    set_led_with_brightness(led_handle, eth_led_state);
}

static void set_eth_led_connected(void)
{
    esp_timer_stop_blocking(eth_led_timer_handle, 10 / portTICK_PERIOD_MS);

    eth_led_blink_state = false;
    eth_led_state = LED_IRGB(LED_ETH, 0, 0, 0);
    set_led_with_brightness(led_handle, eth_led_state);

    esp_timer_start_periodic(eth_led_timer_handle, 500000);
}

static void set_eth_led_connected_with_ip(void)
{
    esp_timer_stop_blocking(eth_led_timer_handle, 10 / portTICK_PERIOD_MS);

    eth_led_state = eth_speed == ETH_SPEED_100M ? LED_ETH_COLOR_100M : LED_ETH_COLOR_10M;
    set_led_with_brightness(led_handle, eth_led_state);
}

static void set_mqtt_led_destroyed(void)
{
    mqtt_led_state = LED_IRGB(LED_MQTT, 0, 0, 0);
    set_led_with_brightness(led_handle, mqtt_led_state);
}

static void set_mqtt_led_disconnected(void)
{
    mqtt_led_state = LED_IRGB(LED_MQTT, 0xFF, 0xFF, 0);
    set_led_with_brightness(led_handle, mqtt_led_state);
}

static void set_mqtt_led_connected(void)
{
    mqtt_led_state = LED_IRGB(LED_MQTT, 0, 0xFF, 0);
    set_led_with_brightness(led_handle, mqtt_led_state);
}

static void set_cits_led_idle(void)
{
    cits_led_state = LED_IRGB(LED_CITS, 0, 0, 0);
    set_led_with_brightness(led_handle, cits_led_state);
}

static void set_cits_led_active(void)
{
    cits_led_state = mqtt_connected ? LED_IRGB(LED_CITS, 0, 0, 0xFF) : LED_IRGB(LED_CITS, 0xFF, 0xA0, 0);
    set_led_with_brightness(led_handle, cits_led_state);
}

static void set_sniffer_led_stopped(void)
{
    sniffer_led_state = LED_IRGB(LED_SNIFFER, 0xFF, 0, 0);
    set_led_with_brightness(led_handle, sniffer_led_state);
}

static void set_sniffer_led_running(void)
{
    sniffer_led_state = LED_IRGB(LED_SNIFFER, 0, 0xFF, 0);
    set_led_with_brightness(led_handle, sniffer_led_state);
}

static void app_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case APP_ETHERNET_MGMT_INTERFACE_CONNECTED:
        eth_link_state = true;
        eth_speed = ethernet_get_mgmt_if_link_speed();
        set_eth_led_connected();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_GOT_IP:
        set_eth_led_connected_with_ip();
        set_mqtt_led_disconnected();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_LOST_IP:
        if (eth_link_state)
            set_eth_led_connected();
        set_mqtt_led_destroyed();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_DISCONNECTED:
        eth_link_state = false;
        set_eth_led_disconnected();
        set_mqtt_led_destroyed();
        break;
    }
}

static void sniffer_event_handler(void* arg, esp_event_base_t event_base,
                                  int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case SNIFFER_RECEIVED_PACKET:
        set_cits_led_active();
        if (esp_timer_restart(cits_led_timer_handle, 50000) == ESP_ERR_INVALID_STATE)
        {
            esp_timer_start_once(cits_led_timer_handle, 50000);
        }
        break;
    case SNIFFER_STARTED:
        sniffer_running = true;
        set_sniffer_led_running();
        set_cits_led_idle();
        break;
    case SNIFFER_STOPPED:
        sniffer_running = false;
        set_sniffer_led_stopped();
        set_cits_led_idle();
        break;
    }
}

static void mqtt_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    switch (event_id)
    {
    case MQTT_CONNECTED:
        mqtt_connected = true;
        set_mqtt_led_connected();
        break;
    case MQTT_DISCONNECTED:
        mqtt_connected = false;
        if (eth_link_state)
            set_mqtt_led_disconnected();
        else
            set_mqtt_led_destroyed();
        break;
    }
}

static void system_led_timer_cb(void *)
{
    set_led_with_brightness(led_handle, system_led_blink_state ?
                              LED_IRGB(LED_SYSTEM, 0xFF, 0xFF, 0xFF) :
                              LED_IRGB(LED_SYSTEM, 0, 0, 0));
    system_led_blink_state = !system_led_blink_state;
}

static void eth_led_timer_cb(void *)
{
    eth_led_state = eth_led_blink_state ?
                (eth_speed == ETH_SPEED_100M ? LED_ETH_COLOR_100M : LED_ETH_COLOR_10M) :
                LED_IRGB(LED_ETH, 0, 0, 0);
    set_led_with_brightness(led_handle, eth_led_state);

    eth_led_blink_state = !eth_led_blink_state;
}

static void cits_led_timer_cb(void *)
{
    set_cits_led_idle();
}

void led_update(void)
{
    uint8_t brightness;
    // brightness receives the default value of 255 in led.c, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    set_led_with_brightness(led_handle, system_led_state);
    set_led_with_brightness(led_handle, sniffer_led_state);
    set_led_with_brightness(led_handle, eth_led_state);
    set_led_with_brightness(led_handle, mqtt_led_state);
    set_led_with_brightness(led_handle, cits_led_state);

    esp_timer_start_periodic(system_led_timer_handle, 1000000);
}

void led_init(void)
{
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_LEDSTRIP_PIN,
            .max_leds = CONFIG_LEDSTRIP_COUNT,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .led_model = LED_MODEL_WS2812
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {0}
    };
    led_indicator_config_t led_config = {
        .blink_lists = NULL,
        .blink_list_num = 0
    };
    ESP_ERROR_CHECK(led_indicator_new_strips_device(&led_config, &strips_config, &led_handle));

    esp_timer_create_args_t create_args = {
        .callback = cits_led_timer_cb,
        .arg = NULL,
        .name = "cits_led"
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &cits_led_timer_handle));

    create_args.callback = eth_led_timer_cb;
    create_args.name = "eth_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &eth_led_timer_handle));

    create_args.callback = system_led_timer_cb;
    create_args.name = "system_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &system_led_timer_handle));

    ESP_ERROR_CHECK(esp_event_handler_register(SNIFFER_EVENT_BASE, ESP_EVENT_ANY_ID, sniffer_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(MQTT_EVENT_BASE, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_EVENT_BASE, ESP_EVENT_ANY_ID, app_event_handler, NULL));

    uint8_t brightness;
    // brightness receives the default value of 255 in led.c, and thus cannot fail
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &brightness));
    led_brightness = brightness;

    set_led_with_brightness(led_handle, LED_IRGB(0, 0xFF,    0,    0));
    set_led_with_brightness(led_handle, LED_IRGB(1, 0xFF, 0xFF,    0));
    set_led_with_brightness(led_handle, LED_IRGB(2,    0, 0xFF,    0));
    set_led_with_brightness(led_handle, LED_IRGB(3,    0,    0, 0xFF));
    set_led_with_brightness(led_handle, LED_IRGB(4, 0xFF,    0, 0xFF));
}

#endif /* CONFIG_LEDSTRIP_COUNT */
