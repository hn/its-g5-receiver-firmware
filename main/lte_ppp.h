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

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_USE_LTE_PPP */
