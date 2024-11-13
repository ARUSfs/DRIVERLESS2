/*
**             Copyright 2023 by Kvaser AB, Molndal, Sweden
**                         http://www.kvaser.com
**
** This software is dual licensed under the following two licenses:
** BSD-new and GPLv2. You may use either one. See the included
** COPYING file for details.
**
** License: BSD-new
** ==============================================================================
** Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are met:
**     * Redistributions of source code must retain the above copyright
**       notice, this list of conditions and the following disclaimer.
**     * Redistributions in binary form must reproduce the above copyright
**       notice, this list of conditions and the following disclaimer in the
**       documentation and/or other materials provided with the distribution.
**     * Neither the name of the <organization> nor the
**       names of its contributors may be used to endorse or promote products
**       derived from this software without specific prior written permission.
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
** AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
** ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
** LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
** CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
** SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
** BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
** IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
** ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
** POSSIBILITY OF SUCH DAMAGE.
**
**
** License: GPLv2
** ==============================================================================
** This program is free software; you can redistribute it and/or modify
** it under the terms of the GNU General Public License as published by
** the Free Software Foundation; either version 2 of the License, or
** (at your option) any later version.
**
** This program is distributed in the hope that it will be useful,
** but WITHOUT ANY WARRANTY; without even the implied warranty of
** MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
** GNU General Public License for more details.
**
** You should have received a copy of the GNU General Public License
** along with this program; if not, write to the Free Software
** Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
**
**
** IMPORTANT NOTICE:
** ==============================================================================
** This source code is made available for free, as an open license, by Kvaser AB,
** for use with its applications. Kvaser AB does not accept any liability
** whatsoever for any third party patent or other immaterial property rights
** violations that may result from any usage of this source code, regardless of
** the combination of source code and various applications that it can be used
** in, or with.
**
** -----------------------------------------------------------------------------
*/

/*
 * Kvaser Linux Canlib
 * Get channel(s) for a Kvaser CAN device given EAN code and serial no.
 */

#include <stdio.h>
#include <errno.h>
#include <limits.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include "canlib.h"
#include "canstat.h"

#define KVASER_EAN "7330130"

#define ERROR_INVALID_ARGS 1
#define ERROR_CANLIB       2
#define ERROR_NOT_FOUND    3

static bool quiet = false;

static void PrintError(char *pFunc, canStatus status)
{
    if (!quiet) {
        char buf[50] = { 0 };

        canGetErrorText(status, buf, sizeof(buf));
        fprintf(stderr, "%s: failed, stat=%d (%s)\n", pFunc, (int)status, buf);
    }
}

static void PrintUsage(char *pPrgName)
{
    printf("Get channel(s) for a Kvaser CAN device given EAN code and serial no.\n");
    printf("Usage: %s [-q[1]] <EAN code without dashes> <serial no.>\n", pPrgName);
}

int main(int argc, char *argv[])
{
    uint32_t partialEanToMatch;
    uint64_t serialNoToMatch;
    char *pEnd;
    int i, status, noOfChannels, arg;
    bool firstOnly = false;
    uint32_t ean[2], serial[2];
    int firstChannel = -1;
    int lastChannel = -1;

    if (3 == argc) {
        arg = 1;
    } else if (4 == argc) {
        arg = 2;
        if (strcmp(argv[1], "-q") == 0) {
            quiet = true;
        } else if (strcmp(argv[1], "-q1") == 0) {
            quiet = true;
            firstOnly = true;
        } else {
            PrintUsage(argv[0]);
            return ERROR_INVALID_ARGS;
        }
    } else {
        PrintUsage(argv[0]);
        return ERROR_INVALID_ARGS;
    }

    // parse EAN (full or partial)
    errno = 0;
    if (0 == memcmp(argv[arg], KVASER_EAN, sizeof(KVASER_EAN) - 1)) {
        // skip initial Kvaser EAN if found
        partialEanToMatch = strtoul(&argv[arg][sizeof(KVASER_EAN) - 1], &pEnd, 16);
    } else {
        // accept partial EAN (the part after 7330130)
        partialEanToMatch = strtoul(argv[arg], &pEnd, 16);
    }
    if (errno != 0 || (*pEnd != '\0')) {
        PrintUsage(argv[0]);
        return ERROR_INVALID_ARGS;
    }

    // parse serial number
    errno = 0;
    arg++;
    serialNoToMatch = strtoul(argv[arg], &pEnd, 10);
    if (errno != 0 || (*pEnd != '\0')) {
        PrintUsage(argv[0]);
        return ERROR_INVALID_ARGS;
    }

    canInitializeLibrary();

    status = canGetNumberOfChannels(&noOfChannels);
    if (status != canOK) {
        PrintError("canGetNumberOfChannels", status);
        canUnloadLibrary();
        return ERROR_CANLIB;
    }

    for (i = 0; i < noOfChannels; ++i) {
        status = canGetChannelData(i, canCHANNELDATA_CARD_UPC_NO, &ean, sizeof(ean));
        if (status != canOK) {
            PrintError("canGetChannelData: CARD_UPC_NO", status);
            canUnloadLibrary();
            return ERROR_CANLIB;
        }
        // only test the unique part
        if ((ean[0] & 0x00FFFFFF) != partialEanToMatch) {
            if (firstChannel >= 0) {
                break; // new EAN -> we are done
            } else {
                continue; // EAN mismatch, try next channel
            }
        }

        status = canGetChannelData(i, canCHANNELDATA_CARD_SERIAL_NO, &serial, sizeof(serial));
        if (status != canOK) {
            PrintError("canGetChannelData: CARD_SERIAL_NO", status);
            canUnloadLibrary();
            return ERROR_CANLIB;
        }
        if ((serial[0] + ((uint64_t)serial[1] << 32)) != serialNoToMatch) {
            if (firstChannel >= 0) {
                break; // new serial no. -> we are done
            } else {
                continue; // serial no. mismatch, try next channel
            }
        }

        // match!
        if (firstChannel < 0) {
            firstChannel = i;
        }
        lastChannel = i;
    }

    canUnloadLibrary();

    if (firstChannel < 0) {
        if (!quiet) {
            printf("No channels.\n");
        }
        return ERROR_NOT_FOUND;
    }

    if (quiet) {
        if (firstOnly) {
            printf("%i\n", firstChannel);
        } else {
            printf("%i %i\n", firstChannel, lastChannel);
        }
    } else if (firstChannel != lastChannel) {
        printf("Channels: %i - %i\n", firstChannel, lastChannel);
    } else {
        printf("Channels: %i\n", firstChannel);
    }

    return 0; // Success
}
