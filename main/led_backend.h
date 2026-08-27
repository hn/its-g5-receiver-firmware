#pragma once

/*
 * Internal interface between the shared LED state machine in led.c and the
 * hardware backends (led_gpio.c, led_ws2812_single.c, led_ws2812_strip.c).
 *
 * led.c owns the logical state: it subscribes to the event bases, keeps the
 * struct below up to date and calls led_backend_render() whenever something
 * changes.  A backend only translates that state into whatever its hardware
 * can show, and owns any blink timers it needs for that.
 *
 * Before this split, led.c held both jobs twice over -- one 200-line and one
 * 280-line #ifdef branch, each with its own copy of the event handlers.
 */

#include <stdbool.h>
#include <stdint.h>

#include "ethernet.h"   /* eth_speed_t */

typedef struct {
    bool booted;             /* APP_BOOT_FINISHED seen */
    bool sniffer_running;
    bool mqtt_connected;
    bool has_ip;
    bool eth_link_up;        /* only reported by hardware Ethernet */
    eth_speed_t eth_speed;
    bool cits_flash;         /* set on a received packet, cleared after 50 ms */
    uint8_t brightness;
    uint32_t override_color; /* "Flugsicherungslampenmodus", 0 = off */
} led_state_t;

/* Create the hardware and any backend-owned timers. */
void led_backend_init(void);

/* Show the given state.  Called on every state change and from led_update(). */
void led_backend_render(const led_state_t *state);

/*
 * Apply brightness scaling and the override colour to an indexed RGB value.
 * Shared so the two WS2812 backends do not each carry a copy of the same
 * arithmetic.  The override deliberately only takes effect after boot.
 */
uint32_t led_backend_scale(uint32_t irgb, const led_state_t *state);
