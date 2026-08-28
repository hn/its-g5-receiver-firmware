/*
 * Internal interface between the LTE task in lte_ppp.c and the GNSS
 * implementation.  Separate from lte_ppp.h so that main.c, which only needs
 * initialize_lte_ppp(), does not pull esp_modem into its include graph.
 *
 * The implementation is vendor-specific (lte_gnss_simcom.c); a second module
 * would provide its own file behind the same two calls.
 */

#pragma once

#include "sdkconfig.h"

#if defined(CONFIG_USE_LTE_PPP) && defined(CONFIG_LTE_SIMCOM_GNSS)

#include "esp_modem_api.h"

/* Power the receiver up.  Call once per bring-up, after PPP is up. */
void lte_gnss_enable(esp_modem_dce_t *dce);

/* Query and log the position, and update the snapshot lte_gnss_get_fix()
 * returns.  Costs one AT exchange over the CMUX command channel. */
void lte_gnss_report(esp_modem_dce_t *dce);

#endif
