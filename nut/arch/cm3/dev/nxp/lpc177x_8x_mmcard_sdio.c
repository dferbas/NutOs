/*
 * Copyright (C) 2012 by Rob van Lieshout (info@pragmalab.nl)
 * Copyright (C) 2008 by egnite GmbH.
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
 * \brief Basic block device driver for multimedia cards.
 *
 * The driver uses SDIO mode, 4-bit bus
 *
 */

#include <cfg/mmci.h>
#include <errno.h>
#include <string.h>
#include <stdlib.h>

#include <sys/heap.h>
#include <sys/timer.h>
#include <sys/event.h>
#include <fs/dospart.h>
#include <fs/fs.h>

#include <dev/blockdev.h>

#include "arch/cm3/nxp/lpc177x_8x_mmcard_sdio.h"
#include "arch/cm3/nxp/lpc177x_8x_mci.h"

/*!
 * \addtogroup Card
 */
/*@{*/

typedef struct _MCIFC
{
    uint32_t ifc_config;        /*! \brief Configuration flags. */
    uint32_t ifc_opcond;        /*! \brief Operating conditions. */
    uint32_t ifc_reladdr;       /*! \brief Relative card address. */
    uint8_t *ifc_buff;          /*! \brief Pointer to sector buffer. */
    uint32_t ifc_resp[4];       /*! \brief MMC response. */
    uint32_t ifc_cid[4];        /*! \brief Card identification. */
    uint8_t  ifc_admode;        /*! \brief adressing mode */
} MCIFC;


/*!
 * \brief Local multimedia card mount information.
 */
typedef struct _MMCFCB
{
    /*! \brief Attached file system device.
     */
    NUTDEVICE *fcb_fsdev;

    /*! \brief Partition table entry of the currently mounted partition.
     */
    DOSPART fcb_part;

    /*! \brief Next block number to read.
     *
     * The file system driver will send a NUTBLKDEV_SEEK control command
     * to set this value before calling the read or the write routine.
     *
     * The number is partition relative.
     */
    uint32_t fcb_blknum;

    /*! \brief Internal block buffer.
     *
     * A file system driver may use this one or optionally provide it's
     * own buffers.
     *
     * Minimal systems may share their external bus interface with
     * device I/O lines, in which case the buffer must be located
     * in internal memory.
     */
    u_char fcb_blkbuf[MMC_BLOCK_SIZE];
} MMCFCB;

/*
 * Several routines call NutSleep, which results in a context switch.
 * This mutual exclusion semaphore takes care, that multiple threads
 * do not interfere with each other.
 */
static HANDLE mutex;
static MCIFC mci0_ifc;

/* local routines */
static uint32_t Lpc177x_8x_MmcardWriteData(uint8_t*, int, int);
static uint32_t Lpc177x_8x_MmcardReadData(uint8_t*, int, int);

/*-------------------------------------------------------------------------*/
/*                         start of code                                   */
/*-------------------------------------------------------------------------*/


/*!
 * \brief read n-blocks of data, starting at blocknum. Wait for ending!
 *
 * \param buffer Pointer to the data buffer to fill.
 * \param blk    first blocknumber to read
 * \param num    Maximum number of blocks to write. Please note the buffer
 *               should be able to contain the data. No boundery test is performed
 *
 * \return A return value of <0 indicates an error.
 */
static uint32_t Lpc177x_8x_MmcardReadData(uint8_t* buffer, int blk, int num)
{
    int32_t retVal;
    uint32_t errorState;


    /* Gain mutex access. */
    NutEventWait(&mutex, 0);

    retVal = Lpc177x_8x_MciReadBlock(buffer, blk, num);

    if (retVal == MCI_FUNC_OK)
    {
        /*
         *  Reading blocks have started, now wait till this job is finished
         *  Please note the driver uses 16 word FIFO in the background to
         *  transfer the data under interrupt from the card
         */
        while (Lpc177x_8x_MciGetDataXferEndState() != 0);

        errorState = Lpc177x_8x_MciGetXferErrState();

        if ((num > 1) || errorState)
        {
            Lpc177x_8x_MciCmd_StopTransmission();
        }

        if (errorState)
        {
            retVal = MCI_FUNC_FAILED;
        }
    }

    /* Release mutex access. */
    NutEventPost(&mutex);

    return(retVal);
}

/*!
 * \brief write n-blocks of data, starting at blocknum. Wait for ending!
 *
 * \param buffer Pointer to the data buffer to write from.
 * \param blk    first blocknumber to write
 * \param num    Maximum number of blocks to write.
 *
 * \return A return value of <0 indicates an error.
 */
static uint32_t Lpc177x_8x_MmcardWriteData(uint8_t* buffer, int blk, int num)
{
    int32_t retVal;
    uint32_t errorState;


    /* Gain mutex access. */
    NutEventWait(&mutex, 0);

    retVal = Lpc177x_8x_MciWriteBlock(buffer, blk, num);

    if (retVal == MCI_FUNC_OK)
    {
        /*
         *  Writing blocks have started, now wait till this job is finished
         *  Please note the driver uses 16 word FIFO in the background to
         *  transfer the data under interrupt to the card
         */
        while (Lpc177x_8x_MciGetDataXferEndState() != 0);

        errorState = Lpc177x_8x_MciGetXferErrState();

        if ((num > 1) || errorState)
        {
            Lpc177x_8x_MciCmd_StopTransmission();
        }

        if (errorState)
        {
            retVal = MCI_FUNC_FAILED;
        }
    }

    /* Release mutex access. */
    NutEventPost(&mutex);

    return(retVal);
}

/*!
 * \brief Initialize the multimedia card.
 *
 *
 * \param ifc Specifies the hardware interface.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc177x_8x_MmcardInit(NUTDEVICE * dev)
{
    int32_t retVal;
    uint8_t error = 0;
    st_Mci_CardId cidval;
    en_Mci_CardType cardType;
    uint32_t rcAddress;
    uint32_t csdVal[4];
//    uint32_t errorState;

    MCIFC *ifc = (MCIFC *) dev->dev_icb;


    /*********************************/
    /*          Init                 */
    /*********************************/
    retVal = Lpc177x_8x_MciInit(BRD_MCI_POWERED_ACTIVE_LEVEL);
    if (retVal != MCI_FUNC_OK)
    {
        return((int)retVal);
    }

    /*********************************/
    /*          CardType             */
    /*********************************/
    cardType = Lpc177x_8x_MciGetCardType();

#ifdef NUTDEBUG
    switch (cardType)
    {
        case MCI_SDHC_SDXC_CARD:
            printf("\nCurrently the SDXC/SDHC CARD ver2.0 is being used");
            break;
        case MCI_SDSC_V2_CARD:
            printf("\nCurrently the SD CARD ver2.0 is being used");
            break;
        case MCI_SDSC_V1_CARD:
            printf("\nCurrently the SD CARD ver1.0 is being used");
            break;

        case MCI_MMC_CARD:
            printf("\nCurrently the MMC CARD is being used");
            break;

        case MCI_CARD_UNKNOWN:
            printf("\nNo CARD is being plugged, Please check!!!");
            error = 1;
            break;
    }
#endif
    if (error)
    {
        return(MCI_FUNC_FAILED);
    }

    /*********************************/
    /*              CID              */
    /*********************************/
    retVal = Lpc177x_8x_MciGetCID(&cidval);
    if (retVal != MCI_FUNC_OK)
    {
        return((int)retVal);
    }
#ifdef NUTDEBUG
    else
    {
        printf("\nManufacture ID: 0x%08X", cidval.MID);
        printf("\nOEM/Application ID: 0x%08X", cidval.OID);
        printf("\nProduct Name: %d", cidval.PNM_H);
        printf("\nProduct Revision: 0x%08X", cidval.PRV);
        printf("\nProduct Serial Number: 0x%08X", cidval.PSN);
        printf("\nManufacturing Date: 0x%08X",cidval.MDT);
    }
#endif
    /*********************************/
    /*          Card Address         */
    /*********************************/
    retVal = Lpc177x_8x_MciSetCardAddress();
    if (retVal != MCI_FUNC_OK)
    {
        return((int)retVal);
    }
    else
    {
        rcAddress = Lpc177x_8x_MciGetCardAddress();
        ifc->ifc_reladdr = rcAddress;
    }


    /*********************************/
    /*              CDS             */
    /*********************************/
    retVal = Lpc177x_8x_MciGetCSD(csdVal);
    if (retVal != MCI_FUNC_OK)
    {
        return((int)retVal);
    }
#ifdef NUTDEBUG
    else
    {
        printf("\nGet Card Specific Data (CSD) Ok:");
        printf("\n\t[0] = 0x%08X", csdVal[0]);
        printf("\n\t[1] = 0x%08X", csdVal[1]);
        printf("\n\t[2] = 0x%08X", csdVal[2]);
        printf("\n\t[3] = 0x%08X", csdVal[3]);
    }
#endif
    /*********************************/
    /*          Card Select          */
    /*********************************/
    retVal = Lpc177x_8x_MciCmd_SelectCard();

    if (retVal != MCI_FUNC_OK)
    {
        return((int)retVal);
    }

    /*********************************/
    /*          Bandwidth            */
    /*********************************/
    if (cardType == MCI_SDSC_V1_CARD || cardType == MCI_SDSC_V2_CARD || cardType == MCI_SDHC_SDXC_CARD)
    {
        Lpc177x_8x_MciSet_MCIClock( MCI_NORMAL_RATE );

        if (Lpc177x_8x_MciSetBusWidth( SD_4_BIT ) != MCI_FUNC_OK)
        {
            return((int)retVal);
        }
    }

    /*********************************/
    /*          BlockLength          */
    /*********************************/
    retVal = Lpc177x_8x_MciSetBlockLen(BLOCK_LENGTH);
    if (retVal != MCI_FUNC_OK)
    {
        return((int)retVal);
    }

    return(MCI_FUNC_OK);
}

/*!
 * \brief Read data blocks from a mounted partition.
 *
 * Applications should not call this function directly, but use the
 * stdio interface.
 *
 * \param nfp    Pointer to a ::NUTFILE structure, obtained by a previous
 *               call to MmCardMount().
 * \param buffer Pointer to the data buffer to fill.
 * \param num    Maximum number of blocks to read. However, reading
 *               multiple blocks is not yet supported by this driver.
 *
 * \return The number of blocks actually read. A return value of -1
 *         indicates an error.
 */
static int Lpc177x_8x_MmcardBlockRead(NUTFILE * nfp, void *buffer, int num)
{
    MMCFCB *fcb = (MMCFCB *) nfp->nf_fcb;
    uint32_t blk = fcb->fcb_blknum;
//    NUTDEVICE *dev = (NUTDEVICE *) nfp->nf_dev;

    if (buffer == 0)
    {
        buffer = fcb->fcb_blkbuf;
    }
    /*
     *  when using the filesystem, the sectornumbering is different then when
     *  directly accesing the card. For example, the MBR can be found at the
     *  sector 0 of the card, but the filesystem's first sector is the sector
     *  where the start is of the FAT VolumeID (also called the BOOT SECTOR).
     *  This position (or offset) is indicated by reading the partion-table,
     *  more specific: by reading the LBA begin info.
     *  This offset we need to add here to the sector# we get in as
     *  parameter. This way we acces the real sector on the card.
     *
     */
    blk += fcb->fcb_part.part_sect_offs;

    if (Lpc177x_8x_MmcardReadData(fcb->fcb_blkbuf, blk, 1) == MCI_FUNC_OK)
    {
        // return the number of blocks that were succesfully read
        return(num);
    }

    return(-1);
}

/*!
 * \brief Write data blocks to a mounted partition.
 *
 * Applications should not call this function directly, but use the
 * stdio interface.
 *
 * \param nfp    Pointer to a \ref NUTFILE structure, obtained by a previous
 *               call to MmCardMount().
 * \param buffer Pointer to the data to be written.
 * \param num    Maximum number of blocks to write. However, writing
 *               multiple blocks is not yet supported by this driver.
 *
 * \return The number of blocks written. A return value of -1 indicates an
 *         error.
 */
static int Lpc177x_8x_MmcardBlockWrite(NUTFILE * nfp, const void *buffer, int num)
{
    MMCFCB *fcb = (MMCFCB *) nfp->nf_fcb;
    uint32_t blk = fcb->fcb_blknum;
//    NUTDEVICE *dev = (NUTDEVICE *) nfp->nf_dev;


    if (buffer == 0)
    {
        buffer = fcb->fcb_blkbuf;
    }

    blk += fcb->fcb_part.part_sect_offs;

    if (Lpc177x_8x_MmcardWriteData(fcb->fcb_blkbuf, blk, 1) == MCI_FUNC_OK)
    {
        return(num);
    }

    return(-1);
}

/*!
 * \brief Mount a partition.
 *
 * Nut/OS doesn't provide specific routines for mounting. Instead routines
 * for opening files are used.
 *
 * Applications should not directly call this function, but use the high
 * level stdio routines for opening a file.
 *
 * \param dev  Pointer to the MMC device.
 * \param name Partition number followed by a slash followed by a name
 *             of the file system device. Both items are optional. If no
 *             file system driver name is given, the first file system
 *             driver found in the list of registered devices will be
 *             used. If no partition number is specified or if partition
 *             zero is given, the first active primary partition will be
 *             used.
 * \param mode Opening mode. Currently ignored, but
 *             \code _O_RDWR | _O_BINARY \endcode should be used for
 *             compatibility with future enhancements.
 * \param acc  File attributes, ignored.
 *
 * \return Pointer to a newly created file pointer to the mounted
 *         partition or NUTFILE_EOF in case of any error.
 */
static NUTFILE *Lpc177x_8x_MmcardMount(NUTDEVICE * dev, const char *name, int mode, int acc)
{
    int partno = 0;
    u_int i;
    NUTDEVICE *fsdev;
    NUTFILE *nfp;
    MMCFCB *fcb;
    DOSPART *part;
//    MCIFC *ifc = (MCIFC *) dev->dev_icb;
    FSCP_VOL_MOUNT mparm;


    /* Set the card in SDIO mode and check for SD-HC cards. */
    if (Lpc177x_8x_MmcardInit(dev))
    {
        errno = ENODEV;
        return(NUTFILE_EOF);
    }

    /* Parse the name for a partition number and a file system driver. */
    if (*name)
    {
        partno = atoi(name);
        do
        {
            name++;
        } while (*name && *name != '/');
        if (*name == '/')
        {
            name++;
        }
    }


    /*
     * Check the list of registered devices for the given name of the
     * files system driver. If none has been specified, get the first
     * file system driver in the list. Hopefully the application
     * registered one only.
     */
    for (fsdev = nutDeviceList; fsdev; fsdev = fsdev->dev_next)
    {
        if (*name == 0)
        {
            if (fsdev->dev_type == IFTYP_FS)
            {
                break;
            }
        }
        else if (strcmp(fsdev->dev_name, name) == 0)
        {
            break;
        }
    }

    if (fsdev == 0)
    {
#ifdef NUTDEBUG
        printf("[No FSDriver]");
#endif
        errno = ENODEV;
        return(NUTFILE_EOF);
    }

    if ((fcb = NutHeapAllocClear(sizeof(MMCFCB))) == 0)
    {
        errno = ENOMEM;
        return(NUTFILE_EOF);
    }

    // link this block-device driver with the file system device driver
    fcb->fcb_fsdev = fsdev;

    // Initialize MMC access mutex semaphore. */
    NutEventPost(&mutex);

    /* Read MBR. */
    if (Lpc177x_8x_MmcardReadData(fcb->fcb_blkbuf, 0, 1) != MCI_FUNC_OK)
    {
        NutHeapFree(fcb);
        return(NUTFILE_EOF);
    }


    // check sanity of MBR
    if ((fcb->fcb_blkbuf[510]!=0x55) || (fcb->fcb_blkbuf[511]!=0xAA))
    {
#ifdef NUTDEBUG
        printf("\nMBR ");
#endif
//        HexDump(LogGetStream(), fcb->fcb_blkbuf, 512);
//        NutHeapFree(fcb);
        return(NUTFILE_EOF);
    }

    /* Read partition table. */
    part = (DOSPART *) & fcb->fcb_blkbuf[DOSPART_SECTORPOS];
    for (i = 1; i <= 4; i++)
    {
        if (partno)
        {
            if (i == partno)
            {
                /* Found specified partition number. */
                fcb->fcb_part = *part;
                break;
            }
        }
        else if (part->part_state & 0x80)
        {
            /* Located first active partition. */
            fcb->fcb_part = *part;
            break;
        }
        part++;
    }

    if (fcb->fcb_part.part_type == PTYPE_EMPTY)
    {
        NutHeapFree(fcb);
        return(NUTFILE_EOF);
    }

    // show MBR
//    HexDump(LogGetStream(), fcb->fcb_blkbuf, 512);

    // show active entry in partion table
//    HexDump(LogGetStream(), &fcb->fcb_blkbuf[DOSPART_SECTORPOS], 16);


#ifdef NUTDEBUG
    printf("\nnumber of sectors %lu", fcb->fcb_part.part_sects);
    printf("\nstarting LBA %lu", fcb->fcb_part.part_sect_offs);
#endif

    if ((nfp = NutHeapAllocClear(sizeof(NUTFILE))) == 0)
    {
        NutHeapFree(fcb);
        errno = ENOMEM;
        return(NUTFILE_EOF);
    }
    nfp->nf_next = 0;
    nfp->nf_dev = dev;
    nfp->nf_fcb = fcb;


    /*
     * Mount the file system volume.
     */
    mparm.fscp_bmnt = nfp;
    mparm.fscp_part_type = fcb->fcb_part.part_type;


    if (fsdev->dev_ioctl(fsdev, FS_VOL_MOUNT, &mparm))
    {

//        Lpc177x_8x_MmcardUnmount(nfp);       // freezes the system!
        return(NUTFILE_EOF);
    }

    return(nfp);
}

/*!
 * \brief Unmount a previously mounted partition.
 *
 * Applications should not directly call this function, but use the high
 * level stdio routines for closing a previously opened file.
 *
 * \return 0 on success, -1 otherwise.
 */
static int Lpc177x_8x_MmcardUnmount(NUTFILE * nfp)
{
    int rc = -1;

    if (nfp)
    {
        MMCFCB *fcb = (MMCFCB *) nfp->nf_fcb;

        if (fcb)
        {
  //          NUTDEVICE *dev = (NUTDEVICE *) nfp->nf_dev;

            rc = fcb->fcb_fsdev->dev_ioctl(fcb->fcb_fsdev, FS_VOL_UNMOUNT, NULL);

            NutHeapFree(fcb);
        }
        NutHeapFree(nfp);
    }
    return(rc);
}


/*!
 * \brief Perform MMC control functions.
 *
 * This function is called by the ioctl() function of the C runtime
 * library. Applications should not directly call this function.
 *
 * \todo Card change detection should verify the serial card number.
 *
 * \param dev  Identifies the device that receives the device-control
 *             function.
 * \param req  Requested control function. May be set to one of the
 *             following constants:
 *             - \ref NUTBLKDEV_MEDIACHANGE
 *             - \ref NUTBLKDEV_INFO
 *             - \ref NUTBLKDEV_SEEK
 *             - \ref MMCARD_GETCID
 *             - \ref MMCARD_GETCSD
 *
 * \param conf Points to a buffer that contains any data required for
 *             the given control function or receives data from that
 *             function.
 * \return 0 on success, -1 otherwise.
 */
static int Lpc177x_8x_MmcardIOCtl(NUTDEVICE * dev, int req, void *conf)
{
    int rc = 0;

    switch (req)
    {
        case NUTBLKDEV_MEDIAAVAIL:
            {
                *((int *) conf) = 1;
            }
            break;
        case NUTBLKDEV_MEDIACHANGE:
            {
                *((int *) conf) = 0;
            }
            break;
        case NUTBLKDEV_INFO:
            {
                BLKPAR_INFO *par = (BLKPAR_INFO *) conf;
                MMCFCB *fcb = (MMCFCB *) par->par_nfp->nf_fcb;

                /*
                 *  note that we don't read the card's CSD-register for this info,
                 *  in stead, we use the info that we found in the partition table of
                 *  the formatted card.
                 */
                //
                par->par_nblks = fcb->fcb_part.part_sects;
                par->par_blksz = MMC_BLOCK_SIZE;
                par->par_blkbp = fcb->fcb_blkbuf;
            }
            break;
        case NUTBLKDEV_SEEK:
            {
                BLKPAR_SEEK *par = (BLKPAR_SEEK *) conf;
                MMCFCB *fcb = (MMCFCB *) par->par_nfp->nf_fcb;

                fcb->fcb_blknum = par->par_blknum;
            }
            break;
        case MMCARD_GETSTATUS:
            rc = Lpc177x_8x_MciGetCardStatus((int32_t*) conf);
            break;
        case MMCARD_GETCID:
            rc = Lpc177x_8x_MciGetCID((st_Mci_CardId *) conf);
            break;
        case MMCARD_GETCSD:
            rc = Lpc177x_8x_MciGetCSD((uint32_t *) conf);
            break;
        default:
            rc = -1;
            break;
    }
    return(rc);
}


/*!
 * \brief Multimedia card device information structure.
 *
 * A pointer to this structure must be passed to NutRegisterDevice()
 * to bind this driver to the Nut/OS kernel. An application may then
 * call
 * /verbatim
 * _open("MMC0:", _O_RDWR | _O_BINARY);
 * /endverbatim
 * to mount the first active primary partition with any previously
 * registered file system driver (typically devPhat0).
 */
NUTDEVICE devLpcMci0 = {
    0,                          /*!< Pointer to next device, dev_next. */
    {'M', 'M', 'C', '0', 0, 0, 0, 0, 0}
    ,                           /*!< Unique device name, dev_name. */
    0,                          /*!< Type of device, dev_type. Obsolete. */
    0,                          /*!< Base address, dev_base. Unused. */
    0,                          /*!< First interrupt number, dev_irq. Unused. */
    &mci0_ifc,                  /*!< Interface control block, dev_icb. */
    0,                          /*!< Driver control block used by the low level part, dev_dcb. */
    Lpc177x_8x_MmcardInit,                 /*!< Driver initialization routine, dev_init. */
    Lpc177x_8x_MmcardIOCtl,                /*!< Driver specific control function, dev_ioctl. */
    Lpc177x_8x_MmcardBlockRead,            /*!< Read data from a file, dev_read. */
    Lpc177x_8x_MmcardBlockWrite,           /*!< Write data to a file, dev_write. */
    Lpc177x_8x_MmcardMount,                /*!< Mount a file system, dev_open. */
    Lpc177x_8x_MmcardUnmount,              /*!< Unmount a file system, dev_close. */
    0                           /*!< Return file size, dev_size. */
};


/*@}*/
