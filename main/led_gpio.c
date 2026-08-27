/*
 * Backend: a single plain, non-addressable LED on a GPIO.
 *
 * With no colour available the states are encoded as blink patterns on a
 * 2 s cycle (100 ms tick):
 *
 *   sniffer stopped      off
 *   no IP                1 Hz symmetric blink
 *   MQTT disconnected    two short blinks per cycle
 *   MQTT connected       on, with one short dip per cycle
 *
 * The "connected" state deliberately dips instead of being steady: a steady
 * LED cannot be told apart from a hung firmware, which is the one thing a
 * status LED must never be ambiguous about.
 *
 * A received C-ITS packet inverts whatever the pattern currently shows.  That
 * makes it visible both on a dark LED (it lights up) and on a lit one (it
 * dips), without needing a second colour.
 *
 * Limitations of the hardware, not of this code: brightness cannot be scaled
 * without PWM, so CONFIG_INDEX_LED_BRIGHTNESS only acts as an on/off switch at
 * 0.  The override colour ("Flugsicherungslampenmodus") cannot be shown as a
 * colour either and simply forces the LED on.
 */

#include "sdkconfig.h"

#ifdef CONFIG_LED_TYPE_GPIO

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "led_backend.h"

#define TICK_US         100000      /* 100 ms */
#define CYCLE_TICKS     20          /* 2 s */

static const char TAG[] = "led";

static esp_timer_handle_t tick_timer;
static int phase;
static led_state_t last_state;

static void write_led(bool on)
{
#ifdef CONFIG_LED_ACTIVE_LOW
    gpio_set_level(CONFIG_LED_PIN, on ? 0 : 1);
#else
    gpio_set_level(CONFIG_LED_PIN, on ? 1 : 0);
#endif
}

static bool pattern_on(const led_state_t *s)
{
    if (s->brightness == 0)
        return false;

    if (s->override_color && s->booted)
        return true;

    if (!s->sniffer_running)
        return false;

    if (!s->has_ip)
        return phase < CYCLE_TICKS / 2;

    if (!s->mqtt_connected)
        return phase < 2 || (phase >= 4 && phase < 6);

    return phase != 0;
}

void led_backend_render(const led_state_t *s)
{
    last_state = *s;

    bool on = pattern_on(s);

    /* Invert on a packet: visible whether the pattern is currently lit or not. */
    if (s->cits_flash)
        on = !on;

    write_led(on);
}

static void tick_cb(void *arg)
{
    phase = (phase + 1) % CYCLE_TICKS;
    led_backend_render(&last_state);
}

void led_backend_init(void)
{
    const gpio_config_t io = {
        .pin_bit_mask = 1ULL << CONFIG_LED_PIN,
        .mode = GPIO_MODE_OUTPUT,
    };

    /* Not ESP_ERROR_CHECK: a failing status LED must not take down the device
     * it is supposed to report on. */
    esp_err_t err = gpio_config(&io);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "gpio_config(%d) failed: %s",
                 CONFIG_LED_PIN, esp_err_to_name(err));
        return;
    }

    write_led(false);

    const esp_timer_create_args_t create_args = {
        .callback = tick_cb,
        .arg = NULL,
        .name = "led_tick",
    };
    ESP_ERROR_CHECK(esp_timer_create(&create_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, TICK_US));
}

#endif /* CONFIG_LED_TYPE_GPIO */
