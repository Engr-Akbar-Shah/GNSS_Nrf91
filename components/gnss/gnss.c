/*
Name : gnss.c

Description :  
    This source file implements GNSS functionality using Nordic's nRF modem GNSS API.
    It handles GNSS initialization, configuration, event processing, satellite tracking,
    and periodic fix reporting. It also includes utilities for calculating distances
    and logging GNSS data in a terminal-friendly format.

Developer : Engr Akbar Shah

Date : May 16, 2025
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrf_modem_at.h>
#include <modem/lte_lc.h>
#include <nrf_modem_gnss.h>
#include "gnss.h"

LOG_MODULE_REGISTER(GNSS);

#define PI 3.14159265358979323846
#define EARTH_RADIUS_METERS (6371.0 * 1000.0)

#define DEG_TO_RAD (PI / 180.0)

static const char update_indicator[] = {'\\', '|', '/', '-'};
static uint32_t fix_timestamp;

static struct nrf_modem_gnss_pvt_data_frame last_pvt;

/* Reference position. */
bool ref_used;
double ref_latitude;
double ref_longitude;

uint8_t cnt = 0;
struct nrf_modem_gnss_nmea_data_frame *nmea_data;

K_MSGQ_DEFINE(nmea_queue, sizeof(struct nrf_modem_gnss_nmea_data_frame *), 10, 4);
static K_SEM_DEFINE(pvt_data_sem, 0, 1);

static struct k_poll_event events[2] = {
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_SEM_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &pvt_data_sem, 0),
    K_POLL_EVENT_STATIC_INITIALIZER(K_POLL_TYPE_MSGQ_DATA_AVAILABLE,
                                    K_POLL_MODE_NOTIFY_ONLY,
                                    &nmea_queue, 0),
};

/*
Function : distance_calculate

Description : 
    Calculates the great-circle distance (in meters) between two GPS coordinates
    using the Haversine formula.

Parameter : 
    double lat1 - Latitude of the first point (in degrees)
    double lon1 - Longitude of the first point (in degrees)
    double lat2 - Latitude of the second point (in degrees)
    double lon2 - Longitude of the second point (in degrees)

Return : 
    double - Distance in meters between the two points

Example Call : 
    double dist = distance_calculate(59.3293, 18.0686, 60.1695, 24.9354);
*/
static double distance_calculate(double lat1, double lon1,
                                 double lat2, double lon2)
{
    double d_lat_rad = (lat2 - lat1) * DEG_TO_RAD;
    double d_lon_rad = (lon2 - lon1) * DEG_TO_RAD;

    double lat1_rad = lat1 * DEG_TO_RAD;
    double lat2_rad = lat2 * DEG_TO_RAD;

    double a = pow(sin(d_lat_rad / 2), 2) +
               pow(sin(d_lon_rad / 2), 2) *
                   cos(lat1_rad) * cos(lat2_rad);

    double c = 2 * asin(sqrt(a));

    return EARTH_RADIUS_METERS * c;
}

/*
Function : print_distance_from_reference

Description : 
    Calculates and logs the distance between the current GNSS fix and a stored
    reference position, if set.

Parameter : 
    struct nrf_modem_gnss_pvt_data_frame *pvt_data - Pointer to current PVT data

Return : 
    void

Example Call : 
    print_distance_from_reference(&last_pvt);
*/
static void print_distance_from_reference(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
    if (!ref_used)
    {
        return;
    }

    double distance = distance_calculate(pvt_data->latitude, pvt_data->longitude,
                                         ref_latitude, ref_longitude);

    LOG_INF("Distance from reference: %.01f\n\r", distance);
}

/*
Function : gnss_event_handler

Description : 
    GNSS event callback registered with the modem to handle PVT and NMEA data.
    Reads PVT or NMEA data into appropriate structures and queues or signals them.

Parameter : 
    int event - GNSS event type (e.g., NRF_MODEM_GNSS_EVT_PVT, EVT_NMEA)

Return : 
    void

Example Call : 
    nrf_modem_gnss_event_handler_set(gnss_event_handler);
*/
static void gnss_event_handler(int event)
{
    int retval;
    struct nrf_modem_gnss_nmea_data_frame *nmea_data;

    switch (event)
    {
    case NRF_MODEM_GNSS_EVT_PVT:
        retval = nrf_modem_gnss_read(&last_pvt, sizeof(last_pvt), NRF_MODEM_GNSS_DATA_PVT);
        if (retval == 0)
        {
            k_sem_give(&pvt_data_sem);
        }
        break;

    case NRF_MODEM_GNSS_EVT_NMEA:
        nmea_data = k_malloc(sizeof(struct nrf_modem_gnss_nmea_data_frame));
        if (nmea_data == NULL)
        {
            LOG_ERR("Failed to allocate memory for NMEA");
            break;
        }

        retval = nrf_modem_gnss_read(nmea_data,
                                     sizeof(struct nrf_modem_gnss_nmea_data_frame),
                                     NRF_MODEM_GNSS_DATA_NMEA);
        if (retval == 0)
        {
            retval = k_msgq_put(&nmea_queue, &nmea_data, K_NO_WAIT);
        }

        if (retval != 0)
        {
            k_free(nmea_data);
        }
        break;
    default:
        break;
    }
}

/*
Function : print_satellite_stats

Description : 
    Logs the number of satellites tracked, used in fix, and unhealthy
    from the GNSS PVT data.

Parameter : 
    struct nrf_modem_gnss_pvt_data_frame *pvt_data - Pointer to PVT data

Return : 
    void

Example Call : 
    print_satellite_stats(&last_pvt);
*/
static void print_satellite_stats(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
    uint8_t tracked = 0;
    uint8_t in_fix = 0;
    uint8_t unhealthy = 0;

    for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; ++i)
    {
        if (pvt_data->sv[i].sv > 0)
        {
            tracked++;

            if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX)
            {
                in_fix++;
            }

            if (pvt_data->sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_UNHEALTHY)
            {
                unhealthy++;
            }
        }
    }

    LOG_INF("Tracking: %2d Using: %2d Unhealthy: %d", tracked, in_fix, unhealthy);
}

/*
Function : print_flags

Description : 
    Logs debug information about GNSS flags such as LTE blocking, scheduled download,
    or sleep conditions.

Parameter : 
    struct nrf_modem_gnss_pvt_data_frame *pvt_data - Pointer to PVT data

Return : 
    void

Example Call : 
    print_flags(&last_pvt);
*/
static void print_flags(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
    if (pvt_data->flags & NRF_MODEM_GNSS_PVT_FLAG_DEADLINE_MISSED)
    {
        LOG_WRN("GNSS operation blocked by LTE\n");
    }
    if (pvt_data->flags & NRF_MODEM_GNSS_PVT_FLAG_NOT_ENOUGH_WINDOW_TIME)
    {
        LOG_WRN("Insufficient GNSS time windows\n");
    }
    if (pvt_data->flags & NRF_MODEM_GNSS_PVT_FLAG_SLEEP_BETWEEN_PVT)
    {
        LOG_INF("Sleep period(s) between PVT notifications\n");
    }
    if (pvt_data->flags & NRF_MODEM_GNSS_PVT_FLAG_SCHED_DOWNLOAD)
    {
        LOG_INF("Scheduled navigation data download\n");
    }
}

/*
Function : print_fix_data

Description : 
    Logs all available GNSS fix data including position, speed, heading, DOP values,
    and UTC timestamp.

Parameter : 
    struct nrf_modem_gnss_pvt_data_frame *pvt_data - Pointer to valid fix data

Return : 
    void

Example Call : 
    print_fix_data(&last_pvt);
*/
static void print_fix_data(struct nrf_modem_gnss_pvt_data_frame *pvt_data)
{
    LOG_INF("Latitude:          %.06f", pvt_data->latitude);
    LOG_INF("Longitude:         %.06f", pvt_data->longitude);
    LOG_INF("Accuracy:          %.01f m", (double)pvt_data->accuracy);
    LOG_INF("Altitude:          %.01f m", (double)pvt_data->altitude);
    LOG_INF("Altitude accuracy: %.01f m", (double)pvt_data->altitude_accuracy);
    LOG_INF("Speed:             %.01f m/s", (double)pvt_data->speed);
    LOG_INF("Speed accuracy:    %.01f m/s", (double)pvt_data->speed_accuracy);
    LOG_INF("V. speed:          %.01f m/s", (double)pvt_data->vertical_speed);
    LOG_INF("V. speed accuracy: %.01f m/s", (double)pvt_data->vertical_speed_accuracy);
    LOG_INF("Heading:           %.01f deg", (double)pvt_data->heading);
    LOG_INF("Heading accuracy:  %.01f deg", (double)pvt_data->heading_accuracy);
    LOG_INF("Date:              %04u-%02u-%02u",
            pvt_data->datetime.year,
            pvt_data->datetime.month,
            pvt_data->datetime.day);
    LOG_INF("Time (UTC):        %02u:%02u:%02u.%03u",
            pvt_data->datetime.hour,
            pvt_data->datetime.minute,
            pvt_data->datetime.seconds,
            pvt_data->datetime.ms);
    LOG_INF("PDOP:              %.01f", (double)pvt_data->pdop);
    LOG_INF("HDOP:              %.01f", (double)pvt_data->hdop);
    LOG_INF("VDOP:              %.01f", (double)pvt_data->vdop);
    LOG_INF("TDOP:              %.01f\n", (double)pvt_data->tdop);
}

/*
Function : gnss_init_and_start

Description : 
    Initializes the GNSS module by configuring use cases, enabling NMEA output,
    setting power modes, and starting GNSS tracking.

Parameter : 
    void

Return : 
    int - 0 on success, -1 on failure

Example Call : 
    if (gnss_init_and_start() != 0) {
        LOG_ERR("GNSS failed to initialize");
    }
*/
int gnss_init_and_start(void)
{
    if (lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS) != 0)
    {
        LOG_ERR("Failed to activate GNSS functional mode");
        return -1;
    }
    /* Configure GNSS. */
    if (nrf_modem_gnss_event_handler_set(gnss_event_handler) != 0)
    {
        LOG_ERR("Failed to set GNSS event handler");
        return -1;
    }

    /* Enable all supported NMEA messages. */
    uint16_t nmea_mask = NRF_MODEM_GNSS_NMEA_RMC_MASK |
                         NRF_MODEM_GNSS_NMEA_GGA_MASK |
                         NRF_MODEM_GNSS_NMEA_GLL_MASK |
                         NRF_MODEM_GNSS_NMEA_GSA_MASK |
                         NRF_MODEM_GNSS_NMEA_GSV_MASK;

    int err = nrf_modem_gnss_nmea_mask_set(nmea_mask);

    if (err != 0)
    {
        LOG_ERR("Failed to set GNSS NMEA mask %d", err);
        return -1;
    }

    /* Make QZSS satellites visible in the NMEA output. */
    if (nrf_modem_gnss_qzss_nmea_mode_set(NRF_MODEM_GNSS_QZSS_NMEA_MODE_CUSTOM) != 0)
    {
        LOG_WRN("Failed to enable custom QZSS NMEA mode");
    }

    /* This use case flag should always be set. */
    uint8_t use_case = NRF_MODEM_GNSS_USE_CASE_MULTIPLE_HOT_START;

    if (IS_ENABLED(CONFIG_GNSS_SAMPLE_MODE_PERIODIC) &&
        !IS_ENABLED(CONFIG_GNSS_SAMPLE_ASSISTANCE_NONE))
    {
        /* Disable GNSS scheduled downloads when assistance is used. */
        use_case |= NRF_MODEM_GNSS_USE_CASE_SCHED_DOWNLOAD_DISABLE;
    }

    if (IS_ENABLED(CONFIG_GNSS_SAMPLE_LOW_ACCURACY))
    {
        use_case |= NRF_MODEM_GNSS_USE_CASE_LOW_ACCURACY;
    }

    if (nrf_modem_gnss_use_case_set(use_case) != 0)
    {
        LOG_WRN("Failed to set GNSS use case");
    }

#if defined(CONFIG_NRF_CLOUD_AGNSS_ELEVATION_MASK)
    if (nrf_modem_gnss_elevation_threshold_set(CONFIG_NRF_CLOUD_AGNSS_ELEVATION_MASK) != 0)
    {
        LOG_ERR("Failed to set elevation threshold");
        return -1;
    }
    LOG_DBG("Set elevation threshold to %u", CONFIG_NRF_CLOUD_AGNSS_ELEVATION_MASK);
#endif

#if defined(CONFIG_GNSS_SAMPLE_MODE_CONTINUOUS)
    /* Default to no power saving. */
    uint8_t power_mode = NRF_MODEM_GNSS_PSM_DISABLED;

#if defined(CONFIG_GNSS_SAMPLE_POWER_SAVING_MODERATE)
    power_mode = NRF_MODEM_GNSS_PSM_DUTY_CYCLING_PERFORMANCE;
#elif defined(CONFIG_GNSS_SAMPLE_POWER_SAVING_HIGH)
    power_mode = NRF_MODEM_GNSS_PSM_DUTY_CYCLING_POWER;
#endif

    if (nrf_modem_gnss_power_mode_set(power_mode) != 0)
    {
        LOG_ERR("Failed to set GNSS power saving mode");
        return -1;
    }
#endif /* CONFIG_GNSS_SAMPLE_MODE_CONTINUOUS */

    /* Default to continuous tracking. */
    uint16_t fix_retry = 0;
    uint16_t fix_interval = 1;

#if defined(CONFIG_GNSS_SAMPLE_MODE_PERIODIC)
    fix_retry = CONFIG_GNSS_SAMPLE_PERIODIC_TIMEOUT;
    fix_interval = CONFIG_GNSS_SAMPLE_PERIODIC_INTERVAL;
#endif

    if (nrf_modem_gnss_fix_retry_set(fix_retry) != 0)
    {
        LOG_ERR("Failed to set GNSS fix retry");
        return -1;
    }

    if (nrf_modem_gnss_fix_interval_set(fix_interval) != 0)
    {
        LOG_ERR("Failed to set GNSS fix interval");
        return -1;
    }

    if (nrf_modem_gnss_start() != 0)
    {
        LOG_ERR("Failed to start GNSS");
        return -1;
    }
    fix_timestamp = k_uptime_get();
    return 0;
}

/*
Function : refresh_display

Description : 
    Clears the previous terminal output to refresh the printed GNSS data.
    Uses ANSI escape codes to move and clear lines.

Parameter : 
    bool has_fix - Indicates if a GNSS fix is currently available

Return : 
    void

Example Call : 
    refresh_display(last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID);
*/
static void refresh_display(bool has_fix)
{
    int lines_to_clear = has_fix ? 20 : 4;

    // Move up the number of lines to refresh
    printf("\033[%dA", lines_to_clear);

    // Clear and step down each line
    for (int i = 0; i < lines_to_clear; i++)
    {
        printf("\033[2K"); // Clear entire line
        if (i < lines_to_clear - 1)
        {
            printf("\033[1B"); // Move cursor down, except last
        }
    }

    // Move back up to the starting line
    printf("\033[%dA", lines_to_clear - 1);
}

/*
Function : gnss_start_searching

Description : 
    Waits for GNSS events using k_poll. Handles and displays new PVT and NMEA data.
    Shows fix or search status updates in the terminal.

Parameter : 
    void

Return : 
    int - Always returns 0

Example Call : 
    while (1) {
        gnss_start_searching();
    }
*/
int gnss_start_searching(void)
{
    (void)k_poll(events, 2, K_FOREVER);

    if (events[0].state == K_POLL_STATE_SEM_AVAILABLE &&
        k_sem_take(events[0].sem, K_NO_WAIT) == 0)
    {
        static bool first_display = true;

        if (!first_display)
        {
            refresh_display(last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID);
        }

        first_display = false;

        // Now reprint only the 4 lines
        print_satellite_stats(&last_pvt);
        print_flags(&last_pvt);
        printf("-----------------------------------\n");

        if (last_pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID)
        {
            fix_timestamp = k_uptime_get();
            print_fix_data(&last_pvt);
            print_distance_from_reference(&last_pvt);
        }
        else
        {
            LOG_INF("Seconds since last fix: %d",
                    (uint32_t)((k_uptime_get() - fix_timestamp) / 1000));
            cnt++;
            LOG_INF("Searching [%c]", update_indicator[cnt % 4]);
        }
    }

    if (events[1].state == K_POLL_STATE_MSGQ_DATA_AVAILABLE &&
        k_msgq_get(events[1].msgq, &nmea_data, K_NO_WAIT) == 0)
    {
        k_free(nmea_data);
    }

    events[0].state = K_POLL_STATE_NOT_READY;
    events[1].state = K_POLL_STATE_NOT_READY;

    return 0;
}
