/*
 * Copyright (C) 2001-2006 by egnite Software GmbH
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * For additional information see http://www.ethernut.de/
 */

/*!
 * \file net/confnet.c
 * \brief Persistent storage of network configuration.
 *
 * \verbatim
 * $Id: confnet.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

#include <string.h>
#include <arpa/inet.h>
#include <netinet/if_ether.h>
#include <sys/confnet.h>
#include <dev/nvmem.h>

#ifdef CONFNET_VIRGIN_MAC
#define VIRGIN_MAC  ether_aton(CONFNET_VIRGIN_MAC)
#else
static uint8_t virgin_mac[6] = { 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
#define VIRGIN_MAC  virgin_mac
#endif

#ifdef CONFNET_VIRGIN_NETMASK
#define VIRGIN_NETMASK  inet_addr(CONFNET_VIRGIN_NETMASK)
#else
#define VIRGIN_NETMASK  0x00FFFFFFUL
#endif


/*!
 * \addtogroup xgConfNet
 */
/*@{*/

/*!
 * \brief Global network configuration structure.
 *
 * Contains the current network configuration. Nut/Net will load
 * this structure from non-volatile memory during initialization.
 */
CONFNET confnet;

/*!
 * \brief Load network configuration from non-volatile memory.
 *
 * If no configuration is available in EEPROM, all configuration
 * parameters are cleared to zero. Except the MAC address, which
 * is set to the locally administered address 02-00-00-00-00-00.
 *
 * \param name Name of the device.
 *
 * \return 0 if configuration has been read. Otherwise the
 *         return value is -1.
 */
int NutNetLoadConfig(const char *name)
{
#ifndef CONFNET_HARDCODED
    /* Read non-volatile memory. */
    if (NutNvMemLoad(CONFNET_EE_OFFSET, &confnet, sizeof(CONFNET)) == 0) {
        /* Sanity check. */
        if (confnet.cd_size == sizeof(CONFNET) && strcmp(confnet.cd_name, name) == 0) {
            /* Got a (hopefully) valid configuration. */
            return 0;
        }
    }
#endif
    memset(&confnet, 0, sizeof(CONFNET));
    memcpy(confnet.cdn_mac, VIRGIN_MAC, sizeof(confnet.cdn_mac));

    /* Set local IP and the gate's IP, if configured. */
#ifdef CONFNET_VIRGIN_IP
    confnet.cdn_cip_addr = inet_addr(CONFNET_VIRGIN_IP);
#endif
#ifdef CONFNET_VIRGIN_GATE
    confnet.cdn_gateway = inet_addr(CONFNET_VIRGIN_GATE);
#endif

#ifdef CONFNET_HARDCODED
    /* If hard coded, set the default mask and return success. */
    confnet.cdn_ip_mask = VIRGIN_NETMASK;
    return 0;
#else
    /* Reading from non-volatile memory failed. */
    return -1;
#endif
}

/*!
 * \brief Save network configuration in non-volatile memory.
 *
 * \return 0 if OK, -1 on failures.
 */
int NutNetSaveConfig(void)
{
#if !defined (__NUT_EMULATION__) && !defined (CONFNET_HARDCODED)
    confnet.cd_size = sizeof(CONFNET);
    if (NutNvMemSave(CONFNET_EE_OFFSET, &confnet, sizeof(CONFNET))) {
        return -1;
    }
#endif
    return 0;
}

/*@}*/
