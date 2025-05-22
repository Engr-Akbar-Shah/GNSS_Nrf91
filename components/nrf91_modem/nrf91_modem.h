/*
Name        : lte.h

Description : Header file for LTE modem control and information retrieval APIs.
              Declares functions to initialize/deinitialize LTE and modem systems,
              and retrieve modem-specific details like IMEI, ICCID, and firmware version.

Developer   : Engr. Akbar Shah

Date        : May 13, 2025
*/

#ifndef _NRF91_MODEM_H
#define _NRF91_MODEM_H

/*
Function    : get_modem_info_fw_version

Description : Retrieves the firmware version of the modem and stores it in the provided buffer.

Parameter   : char *fw_version - Destination buffer for the firmware version string.
              size_t len       - Length of the buffer.

Return      : int - 0 on success, negative error code on failure.

Example Call: get_modem_info_fw_version(MODEM_FW_VERSION, sizeof(MODEM_FW_VERSION));
*/
int get_modem_info_fw_version(char *fw_version, size_t len);

/*
Function    : get_modem_info_imei

Description : Retrieves the modem IMEI using the AT+CGSN=1 command and parses the response.

Parameter   : char *imei - Destination buffer for the IMEI string.
              size_t len - Length of the buffer.

Return      : int - 0 on success, negative error code on failure.

Example Call: get_modem_info_imei(MODEM_IMEI, sizeof(MODEM_IMEI));
*/
int get_modem_info_imei(char *imei, size_t len);

/*
Function    : get_modem_info_iccid

Description : Retrieves the SIM card ICCID using the AT+CCID command and parses the response.

Parameter   : char *iccid - Destination buffer for the ICCID string.
              size_t len  - Length of the buffer.

Return      : int - 0 on success, negative error code on failure.

Example Call: get_modem_info_iccid(MODEM_ICCID, sizeof(MODEM_ICCID));
*/
int get_modem_info_iccid(char *iccid, size_t len);

/*
Function    : modem_deinit

Description : Shuts down the modem library safely.

Parameter   : void

Return      : int - 0 on success, negative error code on failure.

Example Call: modem_deinit();
*/
int modem_deinit(void);

/*
Function    : modem_init

Description : Initializes the modem library and retrieves IMEI, ICCID, and FW version.

Parameter   : void

Return      : int - 0 on success, negative error code on failure.

Example Call: modem_init();
*/
int modem_init(void);

#endif