/*
 * SPI EPPP client — provides IP connectivity via an ESP32 running
 * the EPPP SPI slave (NAT gateway) instead of a hardware Ethernet chip.
 */

#pragma once

#ifdef CONFIG_USE_SPI_EPPP

/**
 * Register IP event handlers, configure the EPPP SPI master and connect
 * to the EPPP server.  Blocks until the PPP link is up and an IP address
 * has been assigned.
 *
 * On success, posts APP_ETHERNET_MGMT_INTERFACE_GOT_IP so that the rest
 * of the application (MQTT, etc.) starts normally.
 */
void initialize_spi_eppp(void);

#endif /* CONFIG_USE_SPI_EPPP */
