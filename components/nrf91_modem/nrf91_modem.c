/*
Name        : lte.c

Description : Implementation of LTE modem handling for nRF91 using nRF Connect SDK.
              Contains logic for LTE event handling, modem and LTE initialization,
              deinitialization, and information extraction via AT commands.

Developer   : Engr. Akbar Shah

Date        : May 13, 2025
*/

#include <stdio.h>
#include <ncs_version.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <modem/nrf_modem_lib.h>
#include <modem/modem_info.h>
#include <nrf_modem_at.h>
#include "nrf91_modem.h"

#define MAX_MODEM_INFO_LEN 30

char MODEM_FW_VERSION[MAX_MODEM_INFO_LEN];
char MODEM_IMEI[MAX_MODEM_INFO_LEN];
char MODEM_ICCID[MAX_MODEM_INFO_LEN];

static K_SEM_DEFINE(lte_connected, 0, 1);
static K_SEM_DEFINE(time_sem, 0, 1);

struct modem_param_info mdm_param;

LOG_MODULE_REGISTER(LTE_Nrf91);

int get_modem_info_fw_version(char *fw_version, size_t len)
{
    if (!fw_version || len == 0)
    {
        return -EINVAL;
    }

    if (modem_info_string_get(MODEM_INFO_FW_VERSION, fw_version, len) <= 0)
    {
        LOG_WRN("Failed to get modem FW version");
        return -EIO;
    }
    return 0;
}

int get_modem_info_imei(char *imei, size_t len)
{
    if (!imei || len == 0)
    {
        return -EINVAL;
    }

    char response[128];
    int err = nrf_modem_at_cmd(response, sizeof(response), "AT+CGSN=1");
    if (err)
    {
        LOG_ERR("Couldn't get IMEI, error: %d", err);
        return err;
    }

    char *start = strchr(response, '"');
    char *end = start ? strchr(start + 1, '"') : NULL;
    if (!start || !end)
    {
        LOG_ERR("Failed to parse IMEI.");
        return -EBADMSG;
    }

    size_t imei_len = end - (start + 1);
    if (imei_len >= len)
        imei_len = len - 1;

    strncpy(imei, start + 1, imei_len);
    imei[imei_len] = '\0';

    return 0;
}

int get_modem_info_iccid(char *iccid, size_t len)
{
    if (!iccid || len == 0)
    {
        return -EINVAL;
    }

    char response[128];
    int err = nrf_modem_at_cmd(response, sizeof(response), "AT+CCID");
    if (err)
    {
        LOG_ERR("Couldn't get ICCID, error: %d", err);
        return err;
    }

    char *ccid = strchr(response, ':');
    if (!ccid)
    {
        LOG_ERR("Failed to parse ICCID.");
        return -EBADMSG;
    }

    ccid += 1;
    while (*ccid == ' ')
        ccid++; // skip whitespace

    size_t iccid_len = strcspn(ccid, "\r\n");
    if (iccid_len >= len)
        iccid_len = len - 1;

    strncpy(iccid, ccid, iccid_len);
    iccid[iccid_len] = '\0';
    return 0;
}

int modem_deinit(void)
{
    int err;

    // Give the modem time to settle
    k_msleep(8000);

    err = nrf_modem_lib_shutdown();
    if (err)
    {
        LOG_ERR("Modem library shutdown failed, error: %d", err);
    }
    else
    {
        LOG_INF("Modem library successfully shut down");
    }
    return err;
}

int modem_init(void)
{
    int err;

    LOG_INF("Initializing modem library");
    err = nrf_modem_lib_init();
    if (err)
    {
        LOG_ERR("Failed to initialize the modem library, error: %d", err);
        return err;
    }
    if ((err = modem_info_init()))
    {
        LOG_ERR("Modem info init failed, error: %d", err);
        return err;
    }

    if ((err = modem_info_params_init(&mdm_param)))
    {
        LOG_ERR("Modem info param init failed, error: %d", err);
        return err;
    }

    err = get_modem_info_imei(MODEM_IMEI, sizeof(MODEM_IMEI));
    if (!err)
    {
        LOG_INF("IMEI: [ %s ]", MODEM_IMEI);
    }

    err = get_modem_info_iccid(MODEM_ICCID, sizeof(MODEM_ICCID));
    if (!err)
    {
        LOG_INF("ICCID: [ %s ]", MODEM_ICCID);
    }

    err = get_modem_info_fw_version(MODEM_FW_VERSION, sizeof(MODEM_FW_VERSION));
    if (!err)
    {
        LOG_INF("Modem FW version: %s\n\r", MODEM_FW_VERSION);
    }

    return err;
}