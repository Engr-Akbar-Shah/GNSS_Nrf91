#include <stdio.h>
#include <stdlib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "nrf91_modem.h"
#include "gnss.h"

LOG_MODULE_REGISTER(MAIN);

extern bool ref_used;
extern double ref_latitude;
extern double ref_longitude;

int main(void)
{

	LOG_INF("Starting GNS Based Location Tracking\n\r");

	/* Initialize reference coordinates (if used). */
	if (sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE) > 1 &&
		sizeof(CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE) > 1)
	{
		ref_used = true;
		ref_latitude = atof(CONFIG_GNSS_SAMPLE_REFERENCE_LATITUDE);
		ref_longitude = atof(CONFIG_GNSS_SAMPLE_REFERENCE_LONGITUDE);
	}

	if (modem_init() != 0)
	{
		LOG_ERR("Failed to initialize modem");
		return -1;
	}

	if (gnss_init_and_start() != 0)
	{
		LOG_ERR("Failed to initialize and start GNSS");
		return -1;
	}

	while (1)
	{
		gnss_start_searching();
	}

	return 0;
}
