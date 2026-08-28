/*
 * LTE uplink over PPP — provides IP connectivity via a SIMCom A7670E modem on
 * a local UART, instead of a hardware Ethernet chip or the SPI EPPP gateway.
 *
 * Non-blocking on purpose.  Registering on a mobile network takes 10-60 s and
 * can fail permanently: no SIM, no coverage, no credit, no modem.  In app_main
 * the console, the sniffer, OTA and APP_BOOT_FINISHED all come *after* the
 * uplink block, and a device that cannot reach the network still has to be
 * usable over the console -- so unlike initialize_spi_eppp(), which blocks
 * indefinitely in eppp_connect(), this only builds the netif and the modem DCE
 * and hands bring-up and recovery to a background task.
 *
 * That task is the single writer of the application's link state: it posts
 * APP_ETHERNET_MGMT_INTERFACE_GOT_IP once an address and DNS are in place, and
 * _LOST_IP once the link is gone, strictly alternating.  The netif's own
 * IP_EVENT_PPP_LOST_IP is deliberately not the trigger -- measured, it arrives
 * up to two minutes after the modem stops answering, while the netif's address
 * goes to 0.0.0.0 within ten seconds.
 *
 * Only the following are specific to SIMCom, should a second vendor ever be
 * added: the DCE profile (ESP_MODEM_DCE_SIM7600), the network mode command
 * (AT+CNMP), and the service cross-check (AT+CPSI?).  They are collected in one
 * block in lte_ppp.c.  Everything else -- the recovery ladder, the CMUX exit,
 * the PPP teardown, the link state -- is esp_modem and lwIP level and portable.
 */

#pragma once

#include "sdkconfig.h"

/*
 * Belt and braces against a hand-edited sdkconfig.  Kconfig already enforces
 * this with "depends on !USE_SPI_EPPP", which has to be one-way: two symbols
 * each depending on the other's negation is a recursive dependency Kconfig
 * rejects.
 */
#if defined(CONFIG_USE_LTE_PPP) && defined(CONFIG_USE_SPI_EPPP)
#error "CONFIG_USE_LTE_PPP and CONFIG_USE_SPI_EPPP are mutually exclusive uplinks"
#endif

#ifdef CONFIG_USE_LTE_PPP

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Create the PPP netif and the modem DCE, then start the background task that
 * owns bring-up and recovery.  Returns immediately, whether or not a modem is
 * attached, and does not talk to the modem itself.
 */
void initialize_lte_ppp(void);

#ifdef CONFIG_LTE_SIMCOM_GNSS

#include <stdbool.h>
#include <stdint.h>

/*
 * The last position the modem reported.  Deliberately not the raw CGNSSINFO
 * field array: that layout is a property of one A76xx manual variant and has no
 * business crossing a header.
 */
typedef struct {
    bool valid;              /* false until a position has been computed once */
    int64_t uptime_us;       /* esp_timer_get_time() when this was parsed */
    char mode[4];            /* "2" or "3" -- a 2D or 3D solution */
    int sats_gps, sats_glonass, sats_galileo, sats_beidou;
    char lat[16], lon[16];   /* decimal degrees as reported, e.g. "50.7152863" */
    char ns[2], ew[2];
    char alt[12];            /* metres; reference surface unclear, see the parser */
    double hdop;
} lte_gnss_fix_t;

/** Copy the last position.  Returns false while there has never been one. */
bool lte_gnss_get_fix(lte_gnss_fix_t *out);

#endif /* CONFIG_LTE_SIMCOM_GNSS */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_USE_LTE_PPP */
