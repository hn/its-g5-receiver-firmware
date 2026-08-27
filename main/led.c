/*
 * Status LED state machine.
 *
 * This file owns the *logical* state only: it subscribes to the event bases,
 * keeps a led_state_t up to date and asks the active backend to show it.  What
 * that looks like -- a colour, a blink pattern, one LED per subsystem -- is a
 * backend's business, see led_backend.h.
 */

#include "sdkconfig.h"

#include <stdint.h>
#include <stdbool.h>

#include "esp_event.h"
#include "esp_timer.h"

#include "led_indicator.h"

#include "config.h"
#include "ethernet.h"
#include "events.h"

#include "led.h"
#include "led_backend.h"

/* A received packet lights the C-ITS indication for this long. */
#define CITS_FLASH_US   50000

static led_state_t state = {
    .eth_speed = ETH_SPEED_10M,
};

static esp_timer_handle_t cits_timer_handle;

/* ------------------------------------------------------------------ */

#ifdef CONFIG_LED_TYPE_NONE
/* No status LED on this board; keep the rest of the file linkable. */
void led_backend_init(void) {}
void led_backend_render(const led_state_t *s) { (void)s; }
#endif

uint32_t led_backend_scale(uint32_t irgb, const led_state_t *s)
{
    if (s->override_color && s->booted)
        irgb = s->override_color;

    uint8_t i = GET_INDEX(irgb);
    uint8_t r = GET_RED(irgb);
    uint8_t g = GET_GREEN(irgb);
    uint8_t b = GET_BLUE(irgb);

    r = ((uint32_t)r) * ((uint32_t)s->brightness) / 255u;
    g = ((uint32_t)g) * ((uint32_t)s->brightness) / 255u;
    b = ((uint32_t)b) * ((uint32_t)s->brightness) / 255u;

    return SET_IRGB(i, r, g, b);
}

static void render(void)
{
    led_backend_render(&state);
}

/* ------------------------------------------------------------------ */
/*  Events                                                            */
/* ------------------------------------------------------------------ */

static void cits_timer_cb(void *arg)
{
    state.cits_flash = false;
    render();
}

static void app_event_handler(void *handler_args, esp_event_base_t base,
                              int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case APP_BOOT_FINISHED:
        state.booted = true;
        break;
    case APP_ETHERNET_MGMT_INTERFACE_CONNECTED:
        /* Only hardware Ethernet reports link state; PPP and EPPP never post
         * this, so backends must not require it before showing anything. */
        state.eth_link_up = true;
        state.eth_speed = ethernet_get_mgmt_if_link_speed();
        break;
    case APP_ETHERNET_MGMT_INTERFACE_GOT_IP:
        state.has_ip = true;
        break;
    case APP_ETHERNET_MGMT_INTERFACE_LOST_IP:
        state.has_ip = false;
        state.mqtt_connected = false;
        break;
    case APP_ETHERNET_MGMT_INTERFACE_DISCONNECTED:
        state.eth_link_up = false;
        state.has_ip = false;
        state.mqtt_connected = false;
        break;
    default:
        return;
    }

    render();
}

static void sniffer_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case SNIFFER_RECEIVED_PACKET:
        state.cits_flash = true;
        render();
        if (esp_timer_restart(cits_timer_handle, CITS_FLASH_US) == ESP_ERR_INVALID_STATE)
            esp_timer_start_once(cits_timer_handle, CITS_FLASH_US);
        return;
    case SNIFFER_STARTED:
        state.sniffer_running = true;
        break;
    case SNIFFER_STOPPED:
        state.sniffer_running = false;
        break;
    default:
        return;
    }

    render();
}

static void mqtt_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case MQTT_CONNECTED:
        state.mqtt_connected = true;
        break;
    case MQTT_DISCONNECTED:
        state.mqtt_connected = false;
        break;
    default:
        return;
    }

    render();
}

static void config_event_handler(void *arg, esp_event_base_t event_base,
                                 int32_t event_id, void *event_data)
{
    led_update();
}

/* ------------------------------------------------------------------ */

void led_update(void)
{
    /* Both values have defaults in config.c and therefore cannot fail. */
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &state.brightness));
    ESP_ERROR_CHECK(config_get_u32(CONFIG_INDEX_LED_OVERRIDE_COLOR, &state.override_color));

    render();
}

void led_init(void)
{
    ESP_ERROR_CHECK(config_get_u8(CONFIG_INDEX_LED_BRIGHTNESS, &state.brightness));
    ESP_ERROR_CHECK(config_get_u32(CONFIG_INDEX_LED_OVERRIDE_COLOR, &state.override_color));

    led_backend_init();

    const esp_timer_create_args_t create_args = {
        .callback = cits_timer_cb,
        .arg = NULL,
        .name = "cits_led",
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &cits_timer_handle));

    ESP_ERROR_CHECK(esp_event_handler_register(SNIFFER_EVENT_BASE, ESP_EVENT_ANY_ID,
                                               sniffer_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(MQTT_EVENT_BASE, ESP_EVENT_ANY_ID,
                                               mqtt_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(APP_EVENT_BASE, ESP_EVENT_ANY_ID,
                                               app_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(CONFIG_EVENT_BASE, CONFIG_INDEX_LED_BRIGHTNESS,
                                               config_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(CONFIG_EVENT_BASE, CONFIG_INDEX_LED_OVERRIDE_COLOR,
                                               config_event_handler, NULL));

    render();
}
