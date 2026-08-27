/*
 * Backend: WS2812 strip with one LED per subsystem.
 *
 *   0 system   white, blinking at 1 Hz
 *   1 sniffer  red = stopped, green = running
 *   2 network  off = no link, blinking = link but no IP, solid = link + IP;
 *              colour encodes link speed (green 100M, orange 10M)
 *   3 MQTT     off = no IP, yellow = disconnected, green = connected
 *   4 C-ITS    off = idle, blue = packet (MQTT up), orange = packet (MQTT down)
 *
 * The strip stays dark until APP_BOOT_FINISHED, so the boot rainbow in main.c
 * is not overwritten.
 *
 * The system LED is driven from an event rather than straight from its timer:
 * the timer only posts LED_HEARTBEAT, and the handler does the work.  That way
 * a wedged event loop shows up as a frozen system LED (commit ba3b0c9).
 */

#include "sdkconfig.h"

#ifdef CONFIG_LED_TYPE_WS2812_STRIP

#include "freertos/FreeRTOS.h"

#include "esp_event.h"
#include "esp_timer.h"
#include "led_indicator.h"
#include "led_indicator_strips.h"

#include "events.h"
#include "led_backend.h"

#define LED_SYSTEM  0
#define LED_SNIFFER 1
#define LED_ETH     2
#define LED_MQTT    3
#define LED_CITS    4

#define ETH_COLOR_100M      SET_IRGB(LED_ETH, 0, 0xFF, 0)
#define ETH_COLOR_10M       SET_IRGB(LED_ETH, 0xFF, 0xA0, 0)

#define SYSTEM_PERIOD_US    1000000
#define ETH_BLINK_PERIOD_US 500000

static led_indicator_handle_t handle;
static esp_timer_handle_t system_timer;
static esp_timer_handle_t eth_timer;

static bool system_phase;
static bool eth_phase;
static led_state_t last_state;

static void show(uint32_t irgb, const led_state_t *s)
{
    if (!s->booted)
        return;

    led_indicator_set_rgb(handle, led_backend_scale(irgb, s));
}

void led_backend_render(const led_state_t *s)
{
    last_state = *s;

    if (!s->booted)
        return;

    /* The network LED blinks only while the link is up without an address, so
     * its timer is started and stopped from here rather than from the event
     * handlers. */
    bool want_eth_blink = s->eth_link_up && !s->has_ip;
    if (want_eth_blink)
    {
        if (!esp_timer_is_active(eth_timer))
            esp_timer_start_periodic(eth_timer, ETH_BLINK_PERIOD_US);
    }
    else if (esp_timer_is_active(eth_timer))
    {
        esp_timer_stop(eth_timer);
        eth_phase = false;
    }

    if (!esp_timer_is_active(system_timer))
        esp_timer_start_periodic(system_timer, SYSTEM_PERIOD_US);

    uint32_t eth_color = (s->eth_speed == ETH_SPEED_100M) ? ETH_COLOR_100M
                                                          : ETH_COLOR_10M;

    show(system_phase ? SET_IRGB(LED_SYSTEM, 0xFF, 0xFF, 0xFF)
                      : SET_IRGB(LED_SYSTEM, 0, 0, 0), s);

    show(s->sniffer_running ? SET_IRGB(LED_SNIFFER, 0, 0xFF, 0)
                            : SET_IRGB(LED_SNIFFER, 0xFF, 0, 0), s);

    if (s->has_ip)
        show(eth_color, s);
    else if (s->eth_link_up)
        show(eth_phase ? eth_color : SET_IRGB(LED_ETH, 0, 0, 0), s);
    else
        show(SET_IRGB(LED_ETH, 0, 0, 0), s);

    if (!s->has_ip)
        show(SET_IRGB(LED_MQTT, 0, 0, 0), s);
    else if (s->mqtt_connected)
        show(SET_IRGB(LED_MQTT, 0, 0xFF, 0), s);
    else
        show(SET_IRGB(LED_MQTT, 0xFF, 0xFF, 0), s);

    if (s->cits_flash)
        show(s->mqtt_connected ? SET_IRGB(LED_CITS, 0, 0, 0xFF)
                               : SET_IRGB(LED_CITS, 0xFF, 0xA0, 0), s);
    else
        show(SET_IRGB(LED_CITS, 0, 0, 0), s);
}

static void system_timer_cb(void *arg)
{
    /* Deliberately routed through the event loop, see file comment. */
    esp_event_post(LED_EVENT_BASE, LED_HEARTBEAT, NULL, 0, 0);
}

static void heartbeat_handler(void *arg, esp_event_base_t base,
                              int32_t event_id, void *event_data)
{
    system_phase = !system_phase;
    led_backend_render(&last_state);
}

static void eth_timer_cb(void *arg)
{
    eth_phase = !eth_phase;
    led_backend_render(&last_state);
}

void led_backend_init(void)
{
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_LED_PIN,
            .max_leds = 5,
            .color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB,
            .led_model = LED_MODEL_WS2812,
        },
        .led_strip_driver = LED_STRIP_RMT,
        .led_strip_rmt_cfg = {0},
    };
    led_indicator_config_t led_config = {
        .blink_lists = NULL,
        .blink_list_num = 0,
    };
    ESP_ERROR_CHECK(led_indicator_new_strips_device(&led_config, &strips_config, &handle));

    esp_timer_create_args_t create_args = {
        .callback = system_timer_cb,
        .arg = NULL,
        .name = "system_led",
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &system_timer));

    create_args.callback = eth_timer_cb;
    create_args.name = "eth_led";
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &eth_timer));

    ESP_ERROR_CHECK(esp_event_handler_register(LED_EVENT_BASE, LED_HEARTBEAT,
                                               heartbeat_handler, NULL));
}

#endif /* CONFIG_LED_TYPE_WS2812_STRIP */
