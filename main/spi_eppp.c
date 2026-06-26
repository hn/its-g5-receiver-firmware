/*
 * SPI EPPP client — provides IP connectivity via an ESP32 running
 * the EPPP SPI slave (NAT gateway) instead of a hardware Ethernet chip.
 *
 * Design notes
 * ------------
 * The EPPP high-level API (eppp_connect/eppp_open) blocks until the link
 * is up.  In TUN mode "up" means a one-shot ping to the gateway has
 * succeeded, and eppp_open() only ever reacts to CONNECTION_FAILED — which
 * the TUN path never raises.  So if the peer ESP32 (the SPI slave / NAT
 * gateway) is missing at boot, app_main() would block here forever and the
 * device would hang: red LED, no console, no sniffer.
 *
 * Instead we drive the link with the low-level public API and never block:
 *   - eppp_init()                       init transport + netif (task-less)
 *   - esp_netif_action_start/_connected bring the static TUN netif up
 *   - a private pump task running eppp_perform() moves SPI frames; the SPI
 *     master is self-healing, so it keeps retrying transparently while the
 *     slave is absent and recovers automatically once it comes back.
 *   - a permanent ICMP health-check pings the gateway and posts
 *     APP_ETHERNET_MGMT_INTERFACE_GOT_IP / _LOST_IP so the rest of the
 *     application (MQTT, LED) tracks the real link state — without ever
 *     rebooting or tearing the link down.
 */

#include "sdkconfig.h"

#ifdef CONFIG_USE_SPI_EPPP

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "driver/spi_common.h"
#include "eppp_link.h"
#include "ping/ping_sock.h"
#include "lwip/dns.h"
#include "lwip/inet.h"

#include "config.h"
#include "events.h"
#include "spi_eppp.h"

static const char TAG[] = "SPI_EPPP";

/* Health-check tuning: link-level ICMP ping to the EPPP gateway.
 * After HEALTH_FAIL_THRESHOLD consecutive misses the link is declared
 * down; a single reply brings it back up. */
#define HEALTH_PING_INTERVAL_MS 5000
#define HEALTH_PING_TIMEOUT_MS  1000
#define HEALTH_FAIL_THRESHOLD   3

static bool s_link_up = false;
static int  s_fail_count = 0;

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
/*  Health check: permanent ping to the EPPP gateway                  */
/* ------------------------------------------------------------------ */

static void health_on_success(esp_ping_handle_t hdl, void *args)
{
    (void)hdl;
    (void)args;

    s_fail_count = 0;
    if (!s_link_up)
    {
        s_link_up = true;
        ESP_LOGI(TAG, "EPPP gateway reachable: link up");
        esp_event_post(APP_EVENT_BASE, APP_ETHERNET_MGMT_INTERFACE_GOT_IP,
                       NULL, 0, 0);
    }
}

static void health_on_timeout(esp_ping_handle_t hdl, void *args)
{
    (void)hdl;
    (void)args;

    if (s_link_up && ++s_fail_count >= HEALTH_FAIL_THRESHOLD)
    {
        s_link_up = false;
        ESP_LOGW(TAG, "EPPP gateway unreachable (%d misses): link down",
                 s_fail_count);
        esp_event_post(APP_EVENT_BASE, APP_ETHERNET_MGMT_INTERFACE_LOST_IP,
                       NULL, 0, 0);
    }
}

static void start_health_check(esp_netif_t *netif)
{
    esp_ping_config_t cfg = ESP_PING_DEFAULT_CONFIG();
    cfg.count = ESP_PING_COUNT_INFINITE;
    cfg.interval_ms = HEALTH_PING_INTERVAL_MS;
    cfg.timeout_ms = HEALTH_PING_TIMEOUT_MS;

    /* Target the EPPP gateway (server IP, baked into the netif's static
     * ip_info.gw at creation: 192.168.11.1). */
    esp_netif_ip_info_t ip = {0};
    esp_netif_get_ip_info(netif, &ip);
    ip_addr_t target_addr = {0};
#if LWIP_IPV6
    target_addr.u_addr.ip4.addr = ip.gw.addr;
#else
    target_addr.addr = ip.gw.addr;
#endif
    cfg.target_addr = target_addr;

    esp_ping_callbacks_t cbs = {
        .cb_args = netif,
        .on_ping_success = health_on_success,
        .on_ping_timeout = health_on_timeout,
        .on_ping_end = NULL,
    };

    esp_ping_handle_t ping = NULL;
    if (esp_ping_new_session(&cfg, &cbs, &ping) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to create health-check ping session");
        return;
    }
    if (esp_ping_start(ping) != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to start health-check ping session");
        esp_ping_delete_session(ping);
        return;
    }

    ESP_LOGI(TAG, "Health-check ping started (gw " IPSTR ", every %d ms)",
             IP2STR(&ip.gw), HEALTH_PING_INTERVAL_MS);
}

/* ------------------------------------------------------------------ */
/*  Transport pump task (task-less eppp API + our own task)           */
/* ------------------------------------------------------------------ */

static void eppp_pump_task(void *arg)
{
    esp_netif_t *netif = arg;

    /* eppp_perform() returns ESP_ERR_TIMEOUT only when asked to stop
     * (handle->stop), which we never do.  It pumps SPI frames forever and
     * self-heals across slave reboots: a missing slave just yields failed
     * transactions that the loop retries. */
    while (eppp_perform(netif) != ESP_ERR_TIMEOUT)
    {
    }

    ESP_LOGW(TAG, "EPPP pump task exited");
    vTaskDelete(NULL);
}

/* ------------------------------------------------------------------ */
/*  Public: initialize EPPP SPI master (non-blocking)                 */
/* ------------------------------------------------------------------ */

void initialize_spi_eppp(void)
{
    ESP_ERROR_CHECK(esp_netif_init());

    eppp_config_t config = EPPP_DEFAULT_CLIENT_CONFIG();
    config.transport = EPPP_TRANSPORT_SPI;
    config.task.run_task = false; /* we run our own pump task */
    config.spi.is_master = true;
    config.spi.host = SPI2_HOST;
    config.spi.mosi = CONFIG_SPI_EPPP_PIN_MOSI;
    config.spi.miso = CONFIG_SPI_EPPP_PIN_MISO;
    config.spi.sclk = CONFIG_SPI_EPPP_PIN_SCLK;
    config.spi.cs   = CONFIG_SPI_EPPP_PIN_CS;
    config.spi.intr = CONFIG_SPI_EPPP_PIN_INT;
    config.spi.freq = CONFIG_SPI_EPPP_FREQ_MHZ * 1000 * 1000;

    esp_netif_t *netif = eppp_init(EPPP_CLIENT, &config);
    if (netif == NULL)
    {
        ESP_LOGE(TAG, "Failed to initialize SPI EPPP link");
        return;
    }

    /* Bring the TUN netif up (static ip 192.168.11.2, gw 192.168.11.1).
     * Unlike eppp_netif_start() we deliberately skip the library's
     * one-shot connection check; our permanent health-check replaces it. */
    esp_netif_action_start(netif, 0, 0, 0);
    esp_netif_action_connected(netif, 0, 0, 0);

    configure_dns();

    if (xTaskCreate(eppp_pump_task, "eppp_pump", config.task.stack_size,
                    netif, config.task.priority, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to create EPPP pump task");
        return;
    }

    start_health_check(netif);

    ESP_LOGI(TAG, "SPI EPPP link initialized (non-blocking); waiting for gateway");
}

#endif /* CONFIG_USE_SPI_EPPP */
