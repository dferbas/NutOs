/*
 * Copyright (C) 2006 by egnite Software GmbH. All rights reserved.
 * Copyright (C) 2008 by egnite GmbH. All rights reserved.
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
 *
 */

#include <cfg/os.h>

#include <sys/timer.h>
#include <sys/nutdebug.h>
#include <dev/spibus.h>

#include <string.h>
#include <stdlib.h>

static int SpiDacCommandWrite(NUTSPINODE * node, uint16_t *cmd)
{
    int rc = -1;
    NUTSPIBUS *bus;

    NUTASSERT(node != NULL);
    bus = (NUTSPIBUS *) node->node_bus;
    NUTASSERT(bus != NULL);
    NUTASSERT(bus->bus_alloc != NULL);
    NUTASSERT(bus->bus_transfer != NULL);
    NUTASSERT(bus->bus_release != NULL);

    /* write data */
    rc = (*bus->bus_alloc) (node, 1000);
    if (rc == 0) {
        rc = (*bus->bus_transfer) (node, cmd, NULL, 2);

        (*bus->bus_release) (node);
    }
    return rc;
}

/*!
 * \brief Initialize dataflash at specified interface and chip select.
 *
 * \param spibas Interface base address. For ARM MCUs this may be the
 *               I/O base address of the hardware SPI.
 * \param spipcs Device chip select.
 *
 * \return Device descriptor or -1 in case of an error.
 */
int SpiDacInit(NUTDEVICE* dev)
{
    return 0;
}

int SpiDacWrite (NUTFILE * fp, const void *data, int len)
{
	NUTDEVICE * dev = fp->nf_dev;
    NUTSPINODE *node;
    uint16_t val;

    NUTASSERT( dev != NULL);

    if (len != 2) {
        return 0;
    }
    val = *(uint16_t *)data;

	//set config bits
	val &= ~0xF000;

//	val |= 0x8000; // ~DACA if 0 else DACB
	val |= 0x4000 | // Vref Input buffer Control bit
		   0x2000 | // ~GA Output gain select bit. Gain of 1 V/V (GA = 1) or a gain of 2 V/V(GA = 0)
		   0x1000; // ~SHDN Output Power down Control bit.

    node = (NUTSPINODE *) dev->dev_icb;

    return SpiDacCommandWrite(node, &val);
};

/*!
 * \brief Generate File Handle for 7-Sefgment Display.
 *
 * \param dev Specifies the 7seg device.
 *
 * \return 0 on success or -1 if no valid 7seg was found.
 */
NUTFILE *SpiDacOpen(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    NUTFILE *fp;

    NUTASSERT( dev != NULL);

    if ((fp = malloc(sizeof(NUTFILE))) == 0) {
        return NUTFILE_EOF;
    }

    fp->nf_fcb = 0;
    fp->nf_dev = dev;
    fp->nf_next = 0;

    return fp;

}

/*!
 * \brief Close 7-Segment Device.
 *
 * \return 0 if closed and was opened before, else -1.
 */
static int SpiDacClose(NUTFILE * fp)
{
    if( fp != NULL) {
        free( fp);
        return 0;
    }
    return -1;
}
int SpiDacIOCtl (NUTDEVICE * dev, int req, void *conf){

    return 0;
};


NUTSPINODE dacMcp4921 = {
    NULL,   /* SPI bus */
    NULL,   /* additional parameters (dcb) */
    1000000,/* SPI data rate TODO: в каких величинах? */
    0,  /* SPI mode */
    8,  /* data bits */
    0   /* chip select index */
};

NUTDEVICE devDacMcp4921 = {
    0,                  /* Pointer to next device, dev_next. */
    {'m', 'c', 'p', '4', '9', '2', '1', 0, 0},    /* Unique device name, dev_name. */
    IFTYP_CHAR,         /* Type of device, dev_type. */
    0,                  /* Codec number, dev_base. */
    0,                  /* First interrupt number, dev_irq (not used). */
    &dacMcp4921,            /* Interface control block, dev_icb (not used). */
    0,       			/* Driver control block, dev_dcb. */
    0,       			/* Driver initialization routine, dev_init. */
    SpiDacIOCtl,       	/* Driver specific control function, dev_ioctl. */
    0,        			/* Read from device, dev_read. */
    SpiDacWrite,       	/* Write to device, dev_write. */
#ifdef __HARVARD_ARCH__
    0,     				/* Write data from program space to device, dev_write_P. */
#endif
    SpiDacOpen,        	/* Open a device or file, dev_open. */
    SpiDacClose,       	/* Close a device or file, dev_close. */
    0                	/* Request file size, dev_size. */
};
