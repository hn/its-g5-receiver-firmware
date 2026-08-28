/*
 * LTE uplink over PPP for a SIMCom A7670E modem.  See lte_ppp.h for the design
 * and for what is vendor-specific here.
 *
 * Wiring on the reference board (NiceMCU ESP32-C5 Mini <-> BKP-A7670 CN101):
 *   CONFIG_LTE_PIN_TX -> "R" (modem RXD), CONFIG_LTE_PIN_RX <- "T" (modem TXD),
 *   GND -> "G", and "V" left unconnected -- the modem runs off its own
 *   micro-USB jack so LTE transmit bursts never load the MCU supply.
 *
 * RTS/CTS are not brought out on that breakout, so there is no hardware flow
 * control.  The A7670 is AT-compatible with esp_modem's SIM7600 profile; the
 * component's own "A7670" entry is USB-only and unusable here.
 */

#include "sdkconfig.h"

#ifdef CONFIG_USE_LTE_PPP

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_netif.h"
#include "esp_netif_ppp.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "lwip/dns.h"
#include "lwip/inet.h"

#include "config.h"
#include "events.h"
#include "lte_ppp.h"

static const char TAG[] = "LTE_PPP";

/*
 * UART1, not configurable.  The ESP32-C5 has two high-power UARTs: UART0
 * carries the console and the log on every board variant here, and the LP UART
 * is unusable for PPP (16-byte FIFO, fixed to GPIO4/5).  So there is exactly
 * one valid answer, and an option would only be an invitation to kill the
 * console.
 */
#define MODEM_UART UART_NUM_1

/*
 * The rate the modem comes up at is its stored AT+IPREX value, not a choice --
 * the scan needs it as a fallback.  CONFIG_LTE_UART_TARGET_BAUD is the rate we
 * switch it to with AT+IPR, which is a per-board property of the wiring.
 */
#define MODEM_BAUD_BOOT 115200

/* Seconds between "still online" log lines. */
#define POLL_PERIOD_S 30

#define LTE_TASK_STACK 4096
#define LTE_TASK_PRIO  3

#define GOT_IP_BIT   BIT0
#define LOST_IP_BIT  BIT1
#define PPP_FAIL_BIT BIT2

static EventGroupHandle_t s_events;
static esp_netif_t *s_ppp_netif;
static esp_modem_dce_t *s_dce;

static volatile bool s_link_up;
static bool s_ip_announced;

/* Exposed to the console command added later; written only by the LTE task. */
static const char *volatile s_stage = "init";
static volatile int s_active_baud;

/* ------------------------------------------------------------------ */
/*  PPP events                                                        */
/* ------------------------------------------------------------------ */

/*
 * lwIP reports a failed dial-up long before our own timeout would: CMUX can
 * come up while PPP never negotiates, and NETIF_PPP_ERRORPEERDEAD then arrives
 * within a second.  Waiting on that alongside the address keeps the decision
 * sub-second instead of burning the full timeout.
 *
 * ERRORUSER is excluded: it is posted by our own teardown and must not look
 * like a peer failure on the next attempt.
 */
static void ppp_status_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (id == NETIF_PPP_ERRORNONE || id == NETIF_PPP_ERRORUSER)
        return;

    ESP_LOGW(TAG, "PPP: error %d, giving up on this attempt", (int)id);
    xEventGroupSetBits(s_events, PPP_FAIL_BIT);
}

static void ip_event_handler(void *arg, esp_event_base_t base,
                             int32_t id, void *data)
{
    if (id == IP_EVENT_PPP_GOT_IP)
    {
        ip_event_got_ip_t *ev = data;

        ESP_LOGI(TAG, "PPP: got IP " IPSTR " gw " IPSTR " mask " IPSTR,
                 IP2STR(&ev->ip_info.ip), IP2STR(&ev->ip_info.gw),
                 IP2STR(&ev->ip_info.netmask));

        s_link_up = true;
        xEventGroupSetBits(s_events, GOT_IP_BIT);
    }
    else if (id == IP_EVENT_PPP_LOST_IP)
    {
        ESP_LOGW(TAG, "PPP: lost IP");
        s_link_up = false;
        xEventGroupSetBits(s_events, LOST_IP_BIT);
    }
}

/*
 * Announce the link to the rest of the application, edge-triggered.
 *
 * mqtt.cpp reacts to these by creating and destroying an MQTT client, so they
 * have to alternate.  A repeated event is caught there and only logs a warning,
 * but a *missed* transition leaves MQTT permanently on the wrong side, and each
 * pair throws away a TLS session and pays for a fresh handshake on a metered
 * SIM.
 *
 * Called only from the LTE task, never from the event handlers above, so the
 * state has exactly one writer and needs no lock.  File-static rather than
 * task-local so that the announce_link(false) at the top of the bring-up loop
 * restores consistency even if the task is ever recreated.
 */
static void announce_link(bool up)
{
    if (up == s_ip_announced)
        return;

    s_ip_announced = up;
    esp_event_post(APP_EVENT_BASE,
                   up ? APP_ETHERNET_MGMT_INTERFACE_GOT_IP
                      : APP_ETHERNET_MGMT_INTERFACE_LOST_IP,
                   NULL, 0, 0);
}

/*
 * DNS.  The ethdns0..2 override wins, otherwise whatever IPCP handed out, and
 * 8.8.8.8 only if that was empty too: PPP/IPCP does not always supply
 * resolvers, and without them name resolution fails while plain IP routing
 * works -- a confusing failure mode.
 *
 * Called on every bring-up, not once at init: lwIP applies the IPCP-supplied
 * resolvers afresh on every new PPP session, so an override set once would be
 * silently replaced after the first reconnect.
 */
static void configure_dns(void)
{
    const config_index_t dns_keys[] = {
        CONFIG_INDEX_ETH_DNS0, CONFIG_INDEX_ETH_DNS1, CONFIG_INDEX_ETH_DNS2
    };

    bool overridden = false;

    for (int i = 0; i < 3; i++)
    {
        char dns_str[CONFIG_IPV4_BUFFER_SIZE] = {0};
        size_t dns_size = sizeof(dns_str);

        if (config_get_str(dns_keys[i], dns_str, &dns_size) != ESP_OK ||
            dns_str[0] == '\0')
            continue;

        ip_addr_t dns_addr = IPADDR4_INIT(0);
        ipaddr_aton(dns_str, &dns_addr);
        dns_setserver(i, &dns_addr);

        ESP_LOGI(TAG, "DNS%d: %s (from configuration)", i, dns_str);
        overridden = true;
    }

    if (overridden)
        return;

    esp_netif_dns_info_t dns;

    if (esp_netif_get_dns_info(s_ppp_netif, ESP_NETIF_DNS_MAIN, &dns) == ESP_OK &&
        dns.ip.u_addr.ip4.addr != 0)
    {
        ESP_LOGI(TAG, "DNS: from IPCP " IPSTR, IP2STR(&dns.ip.u_addr.ip4));
        return;
    }

    ESP_LOGW(TAG, "DNS: none from IPCP, falling back to 8.8.8.8");
    ip_addr_t fallback = IPADDR4_INIT_BYTES(8, 8, 8, 8);
    dns_setserver(0, &fallback);
}

/* ------------------------------------------------------------------ */
/*  Bringing the modem to a known state                               */
/* ------------------------------------------------------------------ */

static bool try_sync(esp_modem_dce_t *dce, int baud, int attempts);

/*
 * Put our own side of the link back to a raw UART terminal, without needing
 * the modem to cooperate.  Required after CMUX: setup_cmux() replaces the DTE's
 * primary terminal with a CMuxInstance, and DTE::send() writes to
 * primary_term -- so every probe, including the raw ones MODE_DETECT sends,
 * goes out CMUX framed, and a modem answering plain AT cannot be found on any
 * rate.
 *
 * Both return values are ignored.  MODE_UNDEF is free and only needed because
 * the DCE and DTE guards admit CMUX_MANUAL_EXIT from CMUX_MANUAL_MODE or UNDEF
 * alone, while automatic CMUX leaves us in CMUX_MODE.  CMUX_MANUAL_EXIT reports
 * failure when the modem does not acknowledge the disconnect, but exit_cmux()
 * restores the real terminal either way.  Bounded at ~1 s.
 */
static void reset_local_link_state(esp_modem_dce_t *dce)
{
    esp_modem_set_mode(dce, ESP_MODEM_MODE_UNDEF);
    esp_modem_set_mode(dce, ESP_MODEM_MODE_CMUX_MANUAL_EXIT);
}

/*
 * Shut the PPP session down before another bring-up attempt.
 *
 * Not optional, however wasteful it looks: esp_modem_set_mode(COMMAND) is the
 * only route to Netif::stop(), which is the only thing that clears the
 * PPP_STARTED signal.  Nothing clears it when PPP dies by itself, and
 * Netif::start() is guarded by that flag -- so a stale PPP_STARTED turns every
 * later start() into a silent no-op and the uplink never returns.
 *
 * Costs 2 s, up to 12 s after a half-open CMUX setup: lwIP closes PPP properly
 * and waits for a Terminate-Ack that a dead peer never sends.
 *
 * reset_local_link_state() is a complement, not a substitute -- that one
 * restores the raw terminal, which is a separate problem.
 */
static void teardown_ppp(esp_modem_dce_t *dce)
{
    esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND);
}

/*
 * Bring the modem to command mode at the given UART rate, whatever it was
 * doing before.
 *
 * MODE_DETECT does the work: guess_unsafe() writes its probes straight to the
 * DTE, bypassing the DCE's mode bookkeeping, and tries an LCP echo request
 * (200 ms), a plain AT (500 ms) and a CMUX SABM on DLCI 0 (200 ms).  One call
 * covers every state for at most 900 ms, and on success resets the DCE to
 * modem_mode::UNDEF, which is what makes the follow-up transitions legal.
 *
 * Per candidate rate, not once: the probes only reach a modem listening at the
 * rate the UART is set to, and when all three time out DETECT returns before
 * resetting anything.  Stale bookkeeping would then wrap a later plain AT in
 * CMUX framing, so no rate would answer.
 */
static bool probe_at_baud(esp_modem_dce_t *dce, int baud)
{
    /* Not ESP_ERROR_CHECK: while scanning candidate rates, a rate the UART
     * cannot generate from the current clock source is an expected outcome and
     * must be skipped -- aborting would reboot the board on every scan. */
    esp_err_t err = uart_set_baudrate(MODEM_UART, baud);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "cannot set UART to %d baud: %s", baud, esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(150));

    if (esp_modem_set_mode(dce, ESP_MODEM_MODE_DETECT) != ESP_OK)
        return false;

    /*
     * DETECT resumed whichever mode it found, so command mode still has to be
     * reached explicitly, and which mode it was is not reported back.
     *
     * CMUX_MANUAL_EXIT must come first even though a plain sync looks cheaper:
     * AT succeeds over a CMUX channel too, so syncing first would report
     * success while the modem is still muxed, and set_mode(DATA) is then
     * rejected for mode >= CMUX_MANUAL.  From any other mode the exit is
     * refused without touching the line.
     */
    if (esp_modem_set_mode(dce, ESP_MODEM_MODE_CMUX_MANUAL_EXIT) == ESP_OK)
        /* No settle delay: setting CMUX up again right after leaving it fails,
         * but waiting 500 ms only moved the failure 500 ms later.  Not a timing
         * problem, and the failure is cheap -- one teardown and a retry. */
        ESP_LOGW(TAG, "modem was in CMUX at %d baud -- left it cleanly", baud);
    else if (esp_modem_sync(dce) == ESP_OK)
    {
        s_active_baud = baud;
        return true;
    }
    else if (esp_modem_set_mode(dce, ESP_MODEM_MODE_COMMAND) == ESP_OK)
        ESP_LOGW(TAG, "modem was in data mode at %d baud -- escaped it", baud);
    else
        return false;

    return try_sync(dce, baud, 3);
}

static bool try_sync(esp_modem_dce_t *dce, int baud, int attempts)
{
    esp_err_t err = uart_set_baudrate(MODEM_UART, baud);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "cannot set UART to %d baud: %s", baud, esp_err_to_name(err));
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(150));

    for (int i = 1; i <= attempts; i++)
    {
        if (esp_modem_sync(dce) == ESP_OK)
        {
            s_active_baud = baud;
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    return false;
}

/*
 * The modem keeps whatever AT+IPR set until it loses power, while the MCU
 * restarts far more often -- and after a restart mid-session it sits in data
 * mode or CMUX, silent to plain AT on every rate.  Hence the ladder: reset our
 * own side, plain AT on all candidate rates, then the protocol probes, then a
 * slow wait at the board default for a modem that is still booting.
 */
static bool establish_link(esp_modem_dce_t *dce)
{
    static const int candidates[] = { CONFIG_LTE_UART_TARGET_BAUD, MODEM_BAUD_BOOT,
                                      230400, 460800, 921600 };
    int found = 0;

    reset_local_link_state(dce);

    /*
     * Two passes over the candidate rates, the cheap one first.
     *
     * Plain AT has to lead for correctness, not just speed: guess_unsafe()
     * opens with an LCP echo request and accepts any reply carrying a PPP frame
     * delimiter followed by the LCP protocol id -- which a command-mode modem
     * satisfies by itself, since ATE1 is the default and it echoes the probe
     * back.  MODE_DETECT then resumes a PPP session that never existed and
     * unwinding that costs 15 s.
     */
    for (int pass = 0; pass < 2 && !found; pass++)
    {
        s_stage = pass == 0 ? "scan-at" : "scan-modes";

        for (size_t i = 0; i < sizeof(candidates) / sizeof(candidates[0]); i++)
        {
            bool dup = false;
            for (size_t j = 0; j < i; j++)
                if (candidates[j] == candidates[i])
                    dup = true;
            if (dup)
                continue;

            if (pass == 0)
            {
                ESP_LOGI(TAG, "probing UART at %d baud", candidates[i]);
                if (try_sync(dce, candidates[i], 1))
                    found = candidates[i];
            }
            else
            {
                ESP_LOGI(TAG, "probing %d baud for data/CMUX mode", candidates[i]);
                if (probe_at_baud(dce, candidates[i]))
                    found = candidates[i];
            }

            if (found)
                break;
        }
    }

    /* Nothing answered.  A modem that just lost power needs roughly 8 s before
     * its UART responds and comes back at its stored IPREX rate, answering
     * plain AT. */
    if (!found)
    {
        s_stage = "modem-booting";
        ESP_LOGI(TAG, "still nothing; waiting for a booting modem, UART at %d baud",
                 MODEM_BAUD_BOOT);
        if (try_sync(dce, MODEM_BAUD_BOOT, 20))
            found = MODEM_BAUD_BOOT;
    }

    if (!found)
    {
        ESP_LOGE(TAG, "no answer at any UART baud rate -- check wiring and power");
        return false;
    }

    ESP_LOGI(TAG, "modem found on UART at %d baud", found);

    /* A data call may still be up even though AT works again; dialling would
     * then fail.  Failure here is fine -- usually there is nothing to hang up. */
    if (esp_modem_hang_up(dce) == ESP_OK)
        ESP_LOGI(TAG, "hung up a leftover data call");

    /* Kill the command echo: it is what lets the LCP probe above misdetect
     * command mode as PPP.  Session-only, so it is resent after every modem
     * restart. */
    char echo_reply[32];
    esp_modem_at(dce, "ATE0", echo_reply, 1000);

    if (found == CONFIG_LTE_UART_TARGET_BAUD)
        return true;

    /* AT+IPR (what esp_modem_set_baud sends) is session-only, unlike AT+IPREX,
     * so a rate the wiring cannot carry is undone by power-cycling the modem
     * rather than leaving it unreachable. */
    s_stage = "baud-switch";
    ESP_LOGI(TAG, "negotiating UART %d -> %d baud", found,
             CONFIG_LTE_UART_TARGET_BAUD);

    if (esp_modem_set_baud(dce, CONFIG_LTE_UART_TARGET_BAUD) != ESP_OK)
    {
        ESP_LOGW(TAG, "AT+IPR rejected, UART stays at %d baud", found);
        return true;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
    if (try_sync(dce, CONFIG_LTE_UART_TARGET_BAUD, 8))
    {
        ESP_LOGI(TAG, "UART link now at %d baud", CONFIG_LTE_UART_TARGET_BAUD);
        return true;
    }

    ESP_LOGE(TAG, "no answer at %d baud -- the wiring cannot carry that UART rate",
             CONFIG_LTE_UART_TARGET_BAUD);
    ESP_LOGE(TAG, "the modem is at %d now; power-cycle it to get back to %d",
             CONFIG_LTE_UART_TARGET_BAUD, MODEM_BAUD_BOOT);
    return false;
}

/* ------------------------------------------------------------------ */
/*  SIMCom-specific commands                                          */
/*                                                                    */
/*  The only vendor-dependent AT in this file, collected here so a     */
/*  port to another module has one place to look.  Everything else is  */
/*  3GPP 27.007 or esp_modem level.                                   */
/* ------------------------------------------------------------------ */

/*
 * LTE only.  The notorious 2 A bursts with 577 us slots are a GSM/GPRS
 * phenomenon; more importantly for this device, 2G transmits at +30 dBm and its
 * harmonics land in the 5 GHz band this board exists to listen to.  Not a
 * configuration option for that reason.
 *
 * Quectel modules use AT+QCFG="nwscanmode" instead.
 */
static void set_lte_only(esp_modem_dce_t *dce)
{
    char out[64];

    if (esp_modem_at(dce, "AT+CNMP=38", out, 3000) != ESP_OK)
        ESP_LOGW(TAG, "AT+CNMP=38 rejected -- 2G may still be used");
}

/*
 * Cross-check the registration.  Right after power-on the modem was observed
 * reporting "+CEREG: 0,1" (registered) while "+CPSI: NO SERVICE" and rssi=99
 * said otherwise, so CEREG alone is too optimistic to gate dialling on.
 *
 * Quectel modules answer AT+QNWINFO instead.
 */
static bool has_service(esp_modem_dce_t *dce)
{
    char out[192];

    if (esp_modem_at(dce, "AT+CPSI?", out, 3000) != ESP_OK)
        return false;

    return strstr(out, "NO SERVICE") == NULL;
}

/* ------------------------------------------------------------------ */
/*  Registration and link health                                      */
/* ------------------------------------------------------------------ */

/*
 * Parse the <stat> field of "+CEREG: <n>,<stat>[,...]".  A plain
 * strstr(reply, ",1") also matches "+CEREG: 1,0" -- notifications enabled,
 * *not* registered -- which is exactly the case the check exists to catch.
 */
static bool cereg_registered(const char *reply)
{
    const char *p = strstr(reply, "+CEREG:");

    if (!p)
        return false;

    p = strchr(p, ',');
    if (!p)
        return false;

    int stat = atoi(p + 1);
    return stat == 1 || stat == 5;   /* home / roaming */
}

/*
 * Health signal.  Measured after powering the modem off, the netif's IP goes to
 * 0.0.0.0 after ~10 s while IP_EVENT_PPP_LOST_IP only arrives after ~2 min --
 * lwIP keeps renegotiating and reports once the session is fully closed.  So
 * the IP is the timely signal, and it is what the application needs anyway.
 */
static bool link_alive(void)
{
    esp_netif_ip_info_t info;

    if (!s_link_up)
        return false;

    if (esp_netif_get_ip_info(s_ppp_netif, &info) != ESP_OK || info.ip.addr == 0)
    {
        ESP_LOGW(TAG, "netif lost its address -- treating link as down");
        s_link_up = false;
        return false;
    }

    return true;
}

/*
 * SIM PIN, at most once per boot.
 *
 * Three wrong PINs lock the SIM behind a PUK, and the bring-up loop retries
 * every few seconds -- so it would burn all three attempts in under a minute.
 * Hence: send only on a literal "+CPIN: SIM PIN", never on "SIM PUK", and never
 * twice.  An unset ltesimpin is not an error; most data SIMs have no PIN.
 */
static bool sim_ready(esp_modem_dce_t *dce)
{
    static bool pin_sent;
    char out[192];

    if (esp_modem_at(dce, "AT+CPIN?", out, 5000) != ESP_OK)
    {
        ESP_LOGW(TAG, "AT+CPIN? failed -- no SIM inserted?");
        return false;
    }

    if (strstr(out, "READY"))
        return true;

    if (strstr(out, "SIM PUK"))
    {
        ESP_LOGE(TAG, "SIM is PUK-locked -- unlock it in a phone, not here");
        return false;
    }

    if (!strstr(out, "SIM PIN"))
    {
        ESP_LOGW(TAG, "SIM not ready: %s", out);
        return false;
    }

    if (pin_sent)
    {
        ESP_LOGE(TAG, "SIM still wants a PIN after one attempt -- not retrying, "
                      "two more wrong tries would need a PUK");
        return false;
    }

    char pin[CONFIG_SIM_PIN_BUFFER_SIZE] = {0};
    size_t pin_size = sizeof(pin);

    if (config_get_str(CONFIG_INDEX_LTE_SIM_PIN, pin, &pin_size) != ESP_OK ||
        pin[0] == '\0')
    {
        ESP_LOGE(TAG, "SIM wants a PIN but ltesimpin is not set");
        return false;
    }

    pin_sent = true;
    ESP_LOGI(TAG, "sending the SIM PIN (once per boot)");

    if (esp_modem_set_pin(dce, pin) != ESP_OK)
    {
        ESP_LOGE(TAG, "the SIM rejected the PIN -- check ltesimpin");
        return false;
    }

    vTaskDelay(pdMS_TO_TICKS(3000));
    return esp_modem_at(dce, "AT+CPIN?", out, 5000) == ESP_OK &&
           strstr(out, "READY") != NULL;
}

/* ------------------------------------------------------------------ */
/*  Bring-up and recovery                                             */
/* ------------------------------------------------------------------ */

static void lte_task(void *arg)
{
    esp_modem_dce_t *dce = s_dce;
    int attempt = 0;

    /* Never returns: every failure path loops back here, so the uplink keeps
     * trying instead of going silent for the rest of the boot. */
    while (1)
    {
        attempt++;
        ESP_LOGI(TAG, "=== bring-up attempt %d ===", attempt);

        s_link_up = false;
        /* One place covers every "continue" below, so no future failure path
         * can forget to tell the application the link is gone. */
        announce_link(false);
        s_stage = "link-reset";

        if (!establish_link(dce))
        {
            s_stage = "no-modem";
            vTaskDelay(pdMS_TO_TICKS(15000));
            continue;
        }

        s_stage = "modem-info";
        char buf[192];
        if (esp_modem_get_module_name(dce, buf) == ESP_OK)
            ESP_LOGI(TAG, "module: %s", buf);

        int rssi = 0, ber = 0;
        if (esp_modem_get_signal_quality(dce, &rssi, &ber) == ESP_OK)
            ESP_LOGI(TAG, "signal: rssi=%d ber=%d", rssi, ber);

        s_stage = "sim-check";
        if (!sim_ready(dce))
        {
            /* Long wait on purpose: a misconfigured PIN should produce a loud
             * log, not a stream of attempts. */
            s_stage = "sim-blocked";
            vTaskDelay(pdMS_TO_TICKS(60000));
            continue;
        }

        set_lte_only(dce);

        char apn[CONFIG_APN_BUFFER_SIZE] = {0};
        size_t apn_size = sizeof(apn);
        if (config_get_str(CONFIG_INDEX_LTE_APN, apn, &apn_size) == ESP_OK)
            esp_modem_set_apn(dce, apn);

        /* Dialling before the network is attached just fails, and a minute of
         * pointless retries buries the real errors in the log. */
        s_stage = "waiting-registration";
        bool attached = false;
        for (int i = 0; i < 30 && !attached; i++)
        {
            if (esp_modem_at(dce, "AT+CEREG?", buf, 3000) == ESP_OK &&
                cereg_registered(buf) && has_service(dce))
                attached = true;
            else
                vTaskDelay(pdMS_TO_TICKS(5000));
        }
        if (!attached)
        {
            ESP_LOGE(TAG, "not registered after 60 s");
            s_stage = "not-registered";
            vTaskDelay(pdMS_TO_TICKS(10000));
            continue;
        }
        ESP_LOGI(TAG, "registered: %s", buf);

#ifdef CONFIG_LTE_USE_CMUX
        const esp_modem_dce_mode_t target_mode = ESP_MODEM_MODE_CMUX;
        const char *mode_name = "CMUX";
#else
        const esp_modem_dce_mode_t target_mode = ESP_MODEM_MODE_DATA;
        const char *mode_name = "DATA";
#endif

        s_stage = "mode-switch";
        ESP_LOGI(TAG, "switching to %s mode...", mode_name);
        xEventGroupClearBits(s_events, GOT_IP_BIT | LOST_IP_BIT | PPP_FAIL_BIT);

        /*
         * The first mode switch after a fresh sync often fails while the next
         * outer-loop attempt succeeds.  Retrying in place does not help, so
         * fail fast and let the loop come round with a short wait.
         */
        vTaskDelay(pdMS_TO_TICKS(300));

        if (esp_modem_set_mode(dce, target_mode) != ESP_OK)
        {
            ESP_LOGE(TAG, "set_mode(%s) failed", mode_name);
            s_stage = "mode-switch-failed";
            teardown_ppp(dce);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        s_stage = "waiting-ip";
        /*
         * 20 s, not 60: every successful negotiation produced an address within
         * 6.6 s, so a longer wait only delays the retry that fixes it.
         */
        EventBits_t bits = xEventGroupWaitBits(s_events, GOT_IP_BIT | PPP_FAIL_BIT,
                                               pdFALSE, pdFALSE,
                                               pdMS_TO_TICKS(20000));
        if (!(bits & GOT_IP_BIT))
        {
            ESP_LOGE(TAG, "no IP (%s)",
                     (bits & PPP_FAIL_BIT) ? "PPP reported an error"
                                           : "timeout after 20 s");
            s_stage = "no-ip";
            teardown_ppp(dce);
            vTaskDelay(pdMS_TO_TICKS(2000));
            continue;
        }

        /* Before the announcement, not after: mqtt_start() resolves the broker
         * hostname, so DNS has to be in place first. */
        configure_dns();
        announce_link(true);

        s_stage = "online";
        while (link_alive())
        {
            ESP_LOGI(TAG, "UPLINK: online, UART %d baud", s_active_baud);

            /* Sleep in 1 s steps rather than one long block, otherwise the log
             * cadence dictates how fast an outage is noticed. */
            for (int i = 0; i < POLL_PERIOD_S && link_alive(); i++)
                vTaskDelay(pdMS_TO_TICKS(1000));
        }

        ESP_LOGW(TAG, "PPP: link went down (UART at %d baud)", s_active_baud);
        /* Announce before the teardown, which blocks for seconds: MQTT gets
         * told while this task is busy closing PPP. */
        announce_link(false);
        s_stage = "link-lost";

        teardown_ppp(dce);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void initialize_lte_ppp(void)
{
    /* Probing the wrong baud rate produces framing errors that esp_modem's
     * terminal reports as breaks -- hundreds of lines, on the same UART0 the
     * console lives on. */
    esp_log_level_set("uart_terminal", ESP_LOG_ERROR);

    /* When the modem vanishes, the CMUX layer emits "Restarting CMUX state
     * machine" hundreds of times per second until the link is declared down.
     * The message adds nothing beyond "the peer is gone". */
    esp_log_level_set("CMUX", ESP_LOG_ERROR);

    ESP_ERROR_CHECK(esp_netif_init());

    s_events = xEventGroupCreate();
    if (!s_events)
    {
        ESP_LOGE(TAG, "out of memory -- no uplink on this boot");
        return;
    }

    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID,
                                               ip_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(NETIF_PPP_STATUS, ESP_EVENT_ANY_ID,
                                               ppp_status_handler, NULL));

    esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_PPP();
    esp_netif_config_t netif_cfg = {
        .base = &base_cfg,
        .driver = NULL,
        .stack = ESP_NETIF_NETSTACK_DEFAULT_PPP,
    };

    s_ppp_netif = esp_netif_new(&netif_cfg);
    if (!s_ppp_netif)
    {
        ESP_LOGE(TAG, "esp_netif_new failed -- no uplink on this boot");
        return;
    }

    esp_modem_dte_config_t dte_cfg = ESP_MODEM_DTE_DEFAULT_CONFIG();
    dte_cfg.uart_config.port_num = MODEM_UART;
    dte_cfg.uart_config.tx_io_num = CONFIG_LTE_PIN_TX;
    dte_cfg.uart_config.rx_io_num = CONFIG_LTE_PIN_RX;
    dte_cfg.uart_config.baud_rate = MODEM_BAUD_BOOT;
    dte_cfg.uart_config.flow_control = ESP_MODEM_FLOW_CONTROL_NONE;
    dte_cfg.uart_config.rx_buffer_size = 4096;
    dte_cfg.uart_config.tx_buffer_size = 1024;

    /* esp_modem copies the APN into its own std::string, so a stack buffer is
     * fine here. */
    char apn[CONFIG_APN_BUFFER_SIZE] = {0};
    size_t apn_size = sizeof(apn);
    ESP_ERROR_CHECK(config_get_str(CONFIG_INDEX_LTE_APN, apn, &apn_size));

    esp_modem_dce_config_t dce_cfg = ESP_MODEM_DCE_DEFAULT_CONFIG(apn);

    ESP_LOGI(TAG, "UART%d TX=IO%d RX=IO%d 8N1, no flow control, APN '%s'",
             MODEM_UART, CONFIG_LTE_PIN_TX, CONFIG_LTE_PIN_RX, apn);
    ESP_LOGI(TAG, "local UART baud: boot %d, target %d (not the LTE rate)",
             MODEM_BAUD_BOOT, CONFIG_LTE_UART_TARGET_BAUD);

    /* SIM7600 profile: the A7670 is AT-compatible with it, and esp_modem's own
     * "A7670" entry is USB-only.  Synchronous, but it only installs the UART
     * driver and allocates buffers -- it sends nothing to the modem. */
    s_dce = esp_modem_new_dev(ESP_MODEM_DCE_SIM7600, &dte_cfg, &dce_cfg,
                              s_ppp_netif);
    if (!s_dce)
    {
        ESP_LOGE(TAG, "esp_modem_new_dev failed -- no uplink on this boot");
        return;
    }

    if (xTaskCreate(lte_task, "lte", LTE_TASK_STACK, NULL, LTE_TASK_PRIO,
                    NULL) != pdPASS)
        ESP_LOGE(TAG, "failed to create the LTE task -- no uplink on this boot");
}

#endif /* CONFIG_USE_LTE_PPP */
