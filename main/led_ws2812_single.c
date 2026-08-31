/*
 * Backend: one WS2812 showing overall system state by colour.
 *
 * Priority, highest first:
 *   C-ITS packet flash   Blue (MQTT up) / Orange (MQTT down), 50 ms
 *   Sniffer stopped      Red
 *   No IP                Yellow
 *   MQTT disconnected    Green, blinking
 *   MQTT connected       Green, solid
 *
 * Unlike the strip backend this one shows state from the very start rather
 * than staying dark until APP_BOOT_FINISHED -- on a single-LED board that is
 * the only boot feedback there is.
 */

#include "sdkconfig.h"

#ifdef CONFIG_LED_TYPE_WS2812_SINGLE

#include "esp_timer.h"
#include "led_indicator.h"
#include "led_indicator_strips.h"

#include "led_backend.h"

#define BLINK_PERIOD_US 500000

static led_indicator_handle_t handle;
static esp_timer_handle_t blink_timer;
static bool blink_on = true;
static led_state_t last_state;

static void show(uint32_t irgb, const led_state_t *s)
{
    led_indicator_set_rgb(handle, led_backend_scale(irgb, s));
}

void led_backend_render(const led_state_t *s)
{
    last_state = *s;

    if (s->cits_flash)
    {
        show(s->mqtt_connected ? SET_IRGB(0, 0, 0, 0xFF)      /* blue */
                               : SET_IRGB(0, 0xFF, 0xA0, 0),  /* orange */
             s);
        return;
    }

    if (!s->sniffer_running)
        show(SET_IRGB(0, 0xFF, 0, 0), s);                     /* red */
    else if (!s->has_ip)
        show(LED_YELLOW(0), s);
    else if (!s->mqtt_connected)
        show(blink_on ? SET_IRGB(0, 0, 0xFF, 0)               /* green */
                      : SET_IRGB(0, 0, 0, 0), s);             /* off */
    else
        show(SET_IRGB(0, 0, 0xFF, 0), s);                     /* green solid */
}

static void blink_timer_cb(void *arg)
{
    blink_on = !blink_on;

    /* Only the MQTT-disconnected state blinks; re-rendering in the other
     * states is harmless and keeps this callback free of state logic. */
    led_backend_render(&last_state);
}

void led_backend_init(void)
{
    led_indicator_strips_config_t strips_config = {
        .led_strip_cfg = {
            .strip_gpio_num = CONFIG_LED_PIN,
            .max_leds = 1,
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

    const esp_timer_create_args_t create_args = {
        .callback = blink_timer_cb,
        .arg = NULL,
        .name = "blink_led",
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &blink_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(blink_timer, BLINK_PERIOD_US));
}

#endif /* CONFIG_LED_TYPE_WS2812_SINGLE */
