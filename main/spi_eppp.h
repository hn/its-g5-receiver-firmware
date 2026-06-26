/*
 * SPI EPPP client — provides IP connectivity via an ESP32 running
 * the EPPP SPI slave (NAT gateway) instead of a hardware Ethernet chip.
 */

#pragma once

#ifdef CONFIG_USE_SPI_EPPP

/**
 * Initialize the EPPP SPI master link and bring its TUN netif up.
 *
 * This does NOT block: it returns immediately whether or not the peer
 * ESP32 (SPI slave / gateway) is present.  A background task pumps the
 * self-healing SPI transport, and a permanent ICMP health-check pings the
 * gateway, posting APP_ETHERNET_MGMT_INTERFACE_GOT_IP once it becomes
 * reachable and APP_ETHERNET_MGMT_INTERFACE_LOST_IP if it later drops —
 * so MQTT and the LED follow the real link state and recover on their own
 * (no reboot, no link teardown).
 */
void initialize_spi_eppp(void);

#endif /* CONFIG_USE_SPI_EPPP */
