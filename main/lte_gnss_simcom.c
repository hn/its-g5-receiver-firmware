/*
 * GNSS on a SIMCom A76xx, read over CMUX while PPP is up.
 *
 * Everything in this file is vendor-specific -- the AT commands, the reply
 * layout and the mode table all differ on other modules.  A Quectel port would
 * use AT+QGPS=1 and AT+QGPSLOC? and need its own parser, which is why this is a
 * separate file rather than a branch inside lte_ppp.c.
 *
 * Log convention, as in lte_ppp.c: "GNSS: " is a subsystem prefix, so the
 * message after it carries no further colon, and raw modem replies go in quotes
 * because they bring a colon of their own.
 */

#include "sdkconfig.h"

#if defined(CONFIG_USE_LTE_PPP) && defined(CONFIG_LTE_SIMCOM_GNSS)

#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_log.h"
#include "esp_modem_api.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "lte_ppp.h"

static const char TAG[] = "LTE_PPP";

/*
 * The last parsed position, for the console command and anything else that
 * wants it without talking to the modem.  A spinlock, not a mutex: the copy is
 * under a hundred bytes and the writer is a task that is otherwise idle.
 */
static portMUX_TYPE s_fix_lock = portMUX_INITIALIZER_UNLOCKED;
static lte_gnss_fix_t s_fix;

enum { F_MODE, F_GPS, F_BEIDOU, F_GLONASS, F_GALILEO, F_LAT, F_NS, F_LON,
       F_EW, F_DATE, F_UTC, F_ALT, F_SPEED, F_COURSE, F_PDOP, F_HDOP,
       F_VDOP, F_COUNT };

static void store_fix(char (*fields)[24]);

/*
 * AT+CGNSSMODE's numbers are not portable: the A76XX AT Command Manual V1.09
 * section 24.2.7 gives three different meanings for the same values.  The range
 * reported by the test command tells them apart, so it is checked before this
 * table is applied.
 *
 * The A7670E is the European variant, so the "foreign module" table applies.
 * SBAS is an augmentation service (EGNOS over Europe), QZSS a Japanese
 * regional system; neither adds coverage here.
 */
static const char *gnss_mode_name(int mode)
{
    switch (mode)
    {
    case 1: return "GPS + SBAS + QZSS";
    case 2: return "BeiDou";
    case 3: return "GPS + GLONASS + Galileo + SBAS + QZSS";
    case 4: return "GPS + BeiDou + Galileo";
    default: return NULL;
    }
}

void lte_gnss_enable(esp_modem_dce_t *dce)
{
    char out[192];

    if (esp_modem_at(dce, "AT+CGNSSPWR=1", out, 5000) == ESP_OK)
        ESP_LOGI(TAG, "GNSS: receiver powered on");
    else
        ESP_LOGW(TAG, "GNSS: power-on not accepted");

}

/*
 * Report which constellations are in use, once.  Which ones this module
 * receives is not in the board manual, so it is asked rather than inferred.
 *
 * Read only, never set: several constellations enabled give better solutions
 * than a narrowed selection.
 *
 * Called from the reporting path, not from power-on: the manual makes
 * AT+CGNSSMODE valid only after the "+CGNSSPWR: READY!" URC, and asking
 * earlier returns ERROR immediately.  Here a CGNSSINFO reply has just proven
 * the receiver is up, and a failed round simply retries on the next one.
 */
static void gnss_log_mode_once(esp_modem_dce_t *dce)
{
    static bool done;
    char out[192];

    if (done)
        return;

    if (esp_modem_at(dce, "AT+CGNSSMODE=?", out, 3000) != ESP_OK)
        return;

    ESP_LOGI(TAG, "GNSS: mode range reply \"%s\"", out);

    /*
     * The range discriminates between the manual's three variants, so it is checked
     * rather than merely printed.  Everything decoded here -- the mode names and
     * the 17-field CGNSSINFO layout -- is the "foreign module" variant, the one
     * that answers (1-4).  On a different module the values would look plausible
     * and be wrong, so the verdict belongs in the log.
     */
    const bool foreign = strstr(out, "(1-4)") != NULL;

    if (foreign)
        ESP_LOGI(TAG, "GNSS: decoding as the manual's \"foreign module\" variant "
                 "(mode range 1-4, CGNSSINFO with 17 fields)");
    else
        ESP_LOGW(TAG, "GNSS: mode range is not (1-4), so this is NOT the "
                 "\"foreign module\" variant that this code decodes -- "
                 "constellation names and the CGNSSINFO field order do not "
                 "apply to it");

    if (esp_modem_at(dce, "AT+CGNSSMODE?", out, 3000) != ESP_OK)
        return;

    const char *colon = strchr(out, ':');
    int mode = colon ? atoi(colon + 1) : -1;
    const char *name = foreign ? gnss_mode_name(mode) : NULL;

    if (name)
        ESP_LOGI(TAG, "GNSS: using %s (mode %d per the \"foreign module\" "
                 "table)", name, mode);
    else
        ESP_LOGW(TAG, "GNSS: mode %d not decoded, raw reply \"%s\"", mode, out);

    done = true;
}

/*
 * Adopt the GNSS time as the system clock.
 *
 * The ESP32 has a POSIX clock (settimeofday()/gettimeofday(), with
 * time()/strftime() on top), and this board has no battery-backed RTC, so
 * without a source the clock starts at the epoch on every boot.  Nothing else
 * in this firmware sets it -- there is no SNTP client -- and GNSS supplies a
 * time that needs no network at all, which matters when the uplink is the thing
 * that is down.
 *
 * Accurate to tens of milliseconds, limited by the reply's trip across the UART
 * and the CMUX channel.  The modem's 1PPS output would be nanosecond-grade but
 * is not brought out on the breakout.
 *
 * timegm() rather than mktime(): the reply is UTC, and mktime() would apply the
 * local timezone on top of it.
 */
static void set_system_time(const char *d, const char *t)
{
    int day, mon, year, hour, min, sec;

    if (sscanf(d, "%2d%2d%2d", &day, &mon, &year) != 3 ||
        sscanf(t, "%2d%2d%2d", &hour, &min, &sec) != 3)
        return;

    struct tm tm = {
        .tm_mday = day,
        .tm_mon  = mon - 1,          /* struct tm counts months from 0 */
        .tm_year = 2000 + year - 1900,
        .tm_hour = hour,
        .tm_min  = min,
        .tm_sec  = sec,
    };

    time_t epoch = timegm(&tm);
    if (epoch <= 0)
        return;

    struct timeval before;
    gettimeofday(&before, NULL);
    long long offset = (long long)epoch - (long long)before.tv_sec;

    struct timeval tv = { .tv_sec = epoch, .tv_usec = 0 };
    if (settimeofday(&tv, NULL) != 0)
    {
        ESP_LOGW(TAG, "GNSS: could not set the system clock");
        return;
    }

    /*
     * Set on every position, log only a real correction.  This SoC has no
     * temperature-compensated crystal and drifts by tens of ppm, while GNSS
     * offers a fresh time every round for free.
     *
     * No first-time special case needed: an unset clock is billions of seconds
     * off and trips the threshold by itself, and the printed offset says which
     * case it was.  The threshold is 2 s because the reply's own latency
     * already accounts for a fraction of a second.
     */
    if (offset > 2 || offset < -2)
    {
        char buf[32];
        time_t now = time(NULL);
        strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", gmtime(&now));
        ESP_LOGI(TAG, "GNSS: system clock set to %s UTC, was off by %lld s",
                 buf, offset);
    }
}

/*
 * Put a word to an HDOP value.
 *
 * HDOP -- horizontal dilution of precision -- says how favourably the
 * satellites are spread across the sky, not how accurate the position is.  It
 * is dimensionless: the horizontal error is roughly HDOP times the ranging
 * error, itself a few metres.  Buckets are the usual ones.
 */
static const char *hdop_quality(double hdop)
{
    if (hdop <= 0)  return "unknown";
    if (hdop < 1)   return "ideal";
    if (hdop < 2)   return "excellent";
    if (hdop < 5)   return "good";
    if (hdop < 10)  return "moderate";
    if (hdop < 20)  return "fair";
    return "poor";
}

void lte_gnss_report(esp_modem_dce_t *dce)
{
    char out[192];

    /*
     * One retry: right after a data burst the CMUX AT channel can swallow a
     * reply, which the component's own help predicts ("might need to retry AT
     * commands").  2 s instead of 5 so a dead link is noticed sooner.
     */
    if (esp_modem_at(dce, "AT+CGNSSINFO", out, 2000) != ESP_OK &&
        esp_modem_at(dce, "AT+CGNSSINFO", out, 2000) != ESP_OK)
    {
        ESP_LOGW(TAG, "GNSS: no answer after retry (AT channel unavailable)");
        return;
    }

    gnss_log_mode_once(dce);

    const char *p = strstr(out, "+CGNSSINFO:");
    if (!p)
    {
        ESP_LOGW(TAG, "GNSS: unexpected reply");
        return;
    }

    /*
     * Field layout from the A76XX AT Command Manual V1.09 section 24.2.12 -- from
     * its closing NOTE, not the main table, which describes a different variant:
     *
     *   foreign version (17 fields, used here):
     *     <mode>,<GPS-SVs>,<BEIDOU-SVs>,<GLONASS-SVs>,<GALILEO-SVs>,
     *     <lat>,<N/S>,<log>,<E/W>,<date>,<UTC-time>,<alt>,<speed>,
     *     <course>,<PDOP>,<HDOP>,<VDOP>
     *
     * BeiDou comes second, ahead of GLONASS, and Galileo has a field of its own.
     * A wrong layout is not visible in the output -- the main table's 16 fields
     * print the UTC time as an altitude and swap two satellite counts -- hence the
     * field-count check below rather than trusting the layout.
     *
     * Coordinates are decimal degrees (dd.ddddd), usable as they are.
     *
     * The altitude is probably not the documented quantity.  <alt> is described as
     * "MSL Altitude", but against a known location it reads ~50 m high, which is
     * about the geoid separation from the WGS84 ellipsoid at this latitude.  Left
     * as reported: confirming it needs the geoid offset from an NMEA GGA sentence,
     * and NMEA is off limits because it would be routed into the PPP link.  The
     * horizontal position is unaffected.
     *
     * With no position, every field is empty including the satellite counts, so
     * "how many can it see" is not in this reply at all.
     *
     * <mode> reports the GNSS "fix" (2=2D, 3=3D); the log spells that out, since
     * "no fix" reads like an unresolved bug in a software project.
     */
    char fields[F_COUNT][24] = {{0}};
    const char *cur = strchr(p, ':');
    cur = cur ? cur + 1 : p;
    int parsed = 0;

    for (int i = 0; i < F_COUNT && cur; i++)
    {
        while (*cur == ' ')
            cur++;
        const char *comma = strchr(cur, ',');
        size_t len = comma ? (size_t)(comma - cur) : strcspn(cur, "\r\n");
        if (len >= sizeof(fields[0]))
            len = sizeof(fields[0]) - 1;
        memcpy(fields[i], cur, len);
        fields[i][len] = '\0';
        cur = comma ? comma + 1 : NULL;
        parsed = i + 1;
    }

    if (fields[F_MODE][0] == '\0')
    {
        ESP_LOGI(TAG, "GNSS: no position computed yet -- the module reports no "
                 "satellite count either until it has one");
        return;
    }

    if (parsed != F_COUNT)
    {
        ESP_LOGW(TAG, "GNSS: %d fields, expected %d for the \"foreign module\" "
                 "layout this code decodes -- another variant assigns them "
                 "differently, so nothing is printed rather than mislabelled, "
                 "raw reply \"%s\"", parsed, F_COUNT, out);
        return;
    }

    /*
     * atoi(): the module zero-pads counts to two digits and leaves unused
     * constellations empty, and atoi("") is 0.
     */
    ESP_LOGI(TAG, "GNSS: data computed (%sD) from %d GPS + %d GLONASS + "
             "%d Galileo + %d BeiDou satellites",
             fields[F_MODE],
             atoi(fields[F_GPS]), atoi(fields[F_GLONASS]),
             atoi(fields[F_GALILEO]), atoi(fields[F_BEIDOU]));
    /*
     * The altitude label carries its caveat -- a bare one would read as a height
     * above sea level, which this is measured not to be.  "datum" is avoided
     * despite being the correct term: it is also German for "date", printed just
     * below.
     */
    ESP_LOGI(TAG, "GNSS: position %s %s, %s %s, altitude %s m "
             "(reference surface unclear)",
             fields[F_LAT], fields[F_NS], fields[F_LON], fields[F_EW],
             fields[F_ALT][0] ? fields[F_ALT] : "?");

    /*
     * Date is ddmmyy and time hhmmss.ss, so both are reordered.  Guarded on length
     * because every field is empty until there is a position.
     */
    const char *d = fields[F_DATE], *t = fields[F_UTC];
    char when[48] = "";

    if (strlen(d) >= 6 && strlen(t) >= 6)
    {
        snprintf(when, sizeof(when), ", time 20%.2s-%.2s-%.2s %.2s:%.2s:%.2s UTC",
                 d + 4, d + 2, d, t, t + 2, t + 4);
        set_system_time(d, t);
    }

    ESP_LOGI(TAG, "GNSS: horizontal geometry HDOP %s (%s)%s",
             fields[F_HDOP][0] ? fields[F_HDOP] : "?",
             hdop_quality(atof(fields[F_HDOP])), when);

    store_fix(fields);
}

bool lte_gnss_get_fix(lte_gnss_fix_t *out)
{
    portENTER_CRITICAL(&s_fix_lock);
    *out = s_fix;
    portEXIT_CRITICAL(&s_fix_lock);

    return out->valid;
}

/* ------------------------------------------------------------------ */

/* ------------------------------------------------------------------ */

static void copy_field(char *dst, size_t dst_size, const char *src)
{
    snprintf(dst, dst_size, "%s", src);
}

static void store_fix(char (*fields)[24])
{
    lte_gnss_fix_t fix = { 0 };

    fix.valid = true;
    fix.uptime_us = esp_timer_get_time();

    copy_field(fix.mode, sizeof(fix.mode), fields[F_MODE]);
    fix.sats_gps     = atoi(fields[F_GPS]);
    fix.sats_glonass = atoi(fields[F_GLONASS]);
    fix.sats_galileo = atoi(fields[F_GALILEO]);
    fix.sats_beidou  = atoi(fields[F_BEIDOU]);
    copy_field(fix.lat, sizeof(fix.lat), fields[F_LAT]);
    copy_field(fix.lon, sizeof(fix.lon), fields[F_LON]);
    copy_field(fix.ns, sizeof(fix.ns), fields[F_NS]);
    copy_field(fix.ew, sizeof(fix.ew), fields[F_EW]);
    copy_field(fix.alt, sizeof(fix.alt), fields[F_ALT]);
    fix.hdop = atof(fields[F_HDOP]);

    portENTER_CRITICAL(&s_fix_lock);
    s_fix = fix;
    portEXIT_CRITICAL(&s_fix_lock);
}

#endif /* CONFIG_USE_LTE_PPP && CONFIG_LTE_SIMCOM_GNSS */
