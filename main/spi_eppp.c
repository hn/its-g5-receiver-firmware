/*
 * SPI EPPP client — provides IP connectivity via an ESP32 running
 * the EPPP SPI slave (NAT gateway) instead of a hardware Ethernet chip.
 */

#include "sdkconfig.h"

#ifdef CONFIG_USE_SPI_EPPP

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/spi_common.h"
#include "eppp_link.h"
#include "lwip/dns.h"
#include "lwip/inet.h"

#include "config.h"
#include "events.h"
#include "spi_eppp.h"

static const char TAG[] = "SPI_EPPP";

/* ------------------------------------------------------------------ */
/*  DNS configuration (EPPP TUN mode does not relay DNS from server)  */
/* ------------------------------------------------------------------ */

static void configure_dns(void)
{
    const config_index_t dns_keys[] = {
        CONFIG_INDEX_ETH_DNS0, CONFIG_INDEX_ETH_DNS1, CONFIG_INDEX_ETH_DNS2
    };

    bool any_configured = false;

    for (int i = 0; i < 3; i++)
    {
        char dns_str[CONFIG_IPV4_BUFFER_SIZE] = {0};
        size_t dns_size = sizeof(dns_str);

        esp_err_t res = config_get_str(dns_keys[i], dns_str, &dns_size);
        if (res != ESP_OK || dns_size == 0)
            continue;

        ip_addr_t dns_addr = IPADDR4_INIT(0);
        ipaddr_aton(dns_str, &dns_addr);

        dns_setserver(i, &dns_addr);

        ESP_LOGI(TAG, "DNS%d: %s", i, dns_str);
        any_configured = true;
    }

    /* Default to 8.8.8.8 if no DNS server configured in NVS */
    if (!any_configured)
    {
        ip_addr_t dns_addr = IPADDR4_INIT(0);
        ipaddr_aton("8.8.8.8", &dns_addr);
        dns_setserver(0, &dns_addr);
        ESP_LOGI(TAG, "DNS0: 8.8.8.8 (default)");
    }
}

/* ------------------------------------------------------------------ */
/*  IP event handler — bridge PPP events to APP_EVENT_BASE            */
/* ------------------------------------------------------------------ */

static void ip_event_handler(void *arg, esp_event_base_t base,
                             int32_t event_id, void *event_data)
{
    if (event_id == IP_EVENT_PPP_GOT_IP)
    {
        ESP_LOGI(TAG, "SPI EPPP: got IP");
        configure_dns();
        esp_event_post(APP_EVENT_BASE, APP_ETHERNET_MGMT_INTERFACE_GOT_IP,
                       NULL, 0, 0);
    }
    else if (event_id == IP_EVENT_PPP_LOST_IP)
    {
        ESP_LOGI(TAG, "SPI EPPP: lost IP");
        esp_event_post(APP_EVENT_BASE, APP_ETHERNET_MGMT_INTERFACE_LOST_IP,
                       NULL, 0, 0);
    }
}

/* ------------------------------------------------------------------ */
/*  Public: initialize EPPP SPI master and connect                    */
/* ------------------------------------------------------------------ */

void initialize_spi_eppp(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_GOT_IP,
                                               ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_PPP_LOST_IP,
                                               ip_event_handler, NULL));

    eppp_config_t config = EPPP_DEFAULT_CLIENT_CONFIG();
    config.transport = EPPP_TRANSPORT_SPI;
    config.spi.is_master = true;
    config.spi.host = SPI2_HOST;
    config.spi.mosi = CONFIG_SPI_EPPP_PIN_MOSI;
    config.spi.miso = CONFIG_SPI_EPPP_PIN_MISO;
    config.spi.sclk = CONFIG_SPI_EPPP_PIN_SCLK;
    config.spi.cs   = CONFIG_SPI_EPPP_PIN_CS;
    config.spi.intr = CONFIG_SPI_EPPP_PIN_INT;
    config.spi.freq = CONFIG_SPI_EPPP_FREQ_MHZ * 1000 * 1000;

    ESP_LOGI(TAG, "Connecting to SPI EPPP server...");
    esp_netif_t *eppp_netif = eppp_connect(&config);
    if (eppp_netif == NULL)
    {
        ESP_LOGE(TAG, "SPI EPPP connection failed");
        return;
    }
    ESP_LOGI(TAG, "SPI EPPP connected");
}

#endif /* CONFIG_USE_SPI_EPPP */
