/*
Name : gnss.h

Description :  
    Header file for GNSS-related functions. Declares interfaces for initializing
    the GNSS subsystem and starting the GNSS tracking/searching loop.

Developer : Engr Akbar Shah

Date : May 16, 2025
*/

#ifndef _GNSS_H
#define _GNSS_H

int gnss_start_searching(void);

int gnss_init_and_start(void);

#endif