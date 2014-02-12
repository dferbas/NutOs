#ifndef _ARCH_CM3_NXP_MACH_LPC_EMAC_H_
#define _ARCH_CM3_NXP_MACH_LPC_EMAC_H_

/*
 * Copyright 2011 by egnite GmbH
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
 * \file arch/cm3/nxp/mach/lpc_emac.h
 * \brief LPC Ethernet MAC definitions.
 *
 * \verbatim
 * $Id$
 * \endverbatim
 */

/*!
 * \addtogroup xgNutArchArmLpcEmac
 */
/*@{*/


/*! \name MAC Register */
/*@{*/
#define EMAC_MAC1_OFF           0x00000000

#define EMAC_MAC1_REC_EN        0x00000001
#define EMAC_MAC1_PASS_ALL      0x00000002
#define EMAC_MAC1_RX_FLOWC      0x00000004
#define EMAC_MAC1_TX_FLOWC      0x00000008
#define EMAC_MAC1_LOOPB         0x00000010
#define EMAC_MAC1_RES_TX        0x00000100
#define EMAC_MAC1_RES_MCS_TX    0x00000200
#define EMAC_MAC1_RES_RX        0x00000400
#define EMAC_MAC1_RES_MCS_RX    0x00000800
#define EMAC_MAC1_SIM_RES       0x00004000
#define EMAC_MAC1_SOFT_RES      0x00008000
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MAC2_OFF           0x00000004

#define EMAC_MAC2_FULL_DUP      0x00000001
#define EMAC_MAC2_FRM_LEN_CHK   0x00000002
#define EMAC_MAC2_HUGE_FRM_EN   0x00000004
#define EMAC_MAC2_DLY_CRC       0x00000008
#define EMAC_MAC2_CRC_EN        0x00000010
#define EMAC_MAC2_PAD_EN        0x00000020
#define EMAC_MAC2_VLAN_PAD_EN   0x00000040
#define EMAC_MAC2_ADET_PAD_EN   0x00000080
#define EMAC_MAC2_PPREAM_ENF    0x00000100
#define EMAC_MAC2_LPREAM_ENF    0x00000200
#define EMAC_MAC2_NO_BACKOFF    0x00001000
#define EMAC_MAC2_BACK_PRESSURE 0x00002000
#define EMAC_MAC2_EXCESS_DEF    0x00004000
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_IPGT_OFF   0x00000008
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_IPGR_OFF   0x0000000C
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_CLRT_OFF   0x00000010
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MAXF_OFF   0x00000014
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_SUPP_OFF       0x00000018

#define EMAC_SUPP_SPEED     0x00000100
#define EMAC_SUPP_RES_RMII  0x00000800
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_TEST_OFF           0x0000001C

#define EMAC_TEST_SHCUT_PQUANTA 0x00000001
#define EMAC_TEST_TST_PAUSE     0x00000002
#define EMAC_TEST_TST_BACKP     0x00000004
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MCFG_OFF           0x00000020

#define EMAC_MCFG_SCAN_INC      0x00000001
#define EMAC_MCFG_SUPP_PREAM    0x00000002
#define EMAC_MCFG_RES_MII       0x00008000
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MCMD_OFF       0x00000024

#define EMAC_MCMD_READ      0x00000001
#define EMAC_MCMD_SCAN      0x00000002
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MADR_OFF   0x00000028
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MWTD_OFF   0x0000002C
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MRDD_OFF   0x00000030
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_MIND_OFF           0x00000034

#define EMAC_MIND_BUSY          0x00000001
#define EMAC_MIND_SCAN          0x00000002
#define EMAC_MIND_NOT_VAL       0x00000004
#define EMAC_MIND_MII_LINK_FAIL 0x00000008
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_SA0_OFF    0x00000040
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_SA1_OFF    0x00000044
/*@}*/

/*! \name MAC Register */
/*@{*/
#define EMAC_SA2_OFF    0x00000048
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_CR_OFF             0x00000100

#define EMAC_CR_RX_EN           0x00000001
#define EMAC_CR_TX_EN           0x00000002
#define EMAC_CR_REG_RES         0x00000008
#define EMAC_CR_TX_RES          0x00000010
#define EMAC_CR_RX_RES          0x00000020
#define EMAC_CR_PASS_RUNT_FRM   0x00000040
#define EMAC_CR_PASS_RX_FILT    0x00000080
#define EMAC_CR_TX_FLOW_CTRL    0x00000100
#define EMAC_CR_RMII            0x00000200
#define EMAC_CR_FULL_DUP        0x00000400
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_SR_OFF             0x00000104
#define EMAC_SR_RX_EN           0x00000001
#define EMAC_SR_TX_EN           0x00000002
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_RXDESCR_OFF        0x00000108
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_RXSTAT_OFF         0x0000010C
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_RXDESCR_NUM_OFF    0x00000110
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_RXPROD_IDX_OFF     0x00000114
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_RXCONS_IDX_OFF     0x00000118
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TXDESCR_OFF        0x0000011C
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TXSTAT_OFF         0x00000120
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TXDESCR_NUM_OFF    0x00000124
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TXPROD_IDX_OFF     0x00000128
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TXCONS_IDX_OFF     0x0000012C
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TSV0_OFF           0x00000158

#define EMAC_TSV0_CRC_ERR       0x00000001
#define EMAC_TSV0_LEN_CHKERR    0x00000002
#define EMAC_TSV0_LEN_OUTRNG    0x00000004
#define EMAC_TSV0_DONE          0x00000008
#define EMAC_TSV0_MCAST         0x00000010
#define EMAC_TSV0_BCAST         0x00000020
#define EMAC_TSV0_PKT_DEFER     0x00000040
#define EMAC_TSV0_EXC_DEFER     0x00000080
#define EMAC_TSV0_EXC_COLL      0x00000100
#define EMAC_TSV0_LATE_COLL     0x00000200
#define EMAC_TSV0_GIANT         0x00000400
#define EMAC_TSV0_UNDERRUN      0x00000800
#define EMAC_TSV0_BYTES         0x0FFFF000
#define EMAC_TSV0_CTRL_FRAME    0x10000000
#define EMAC_TSV0_PAUSE         0x20000000
#define EMAC_TSV0_BACK_PRESS    0x40000000
#define EMAC_TSV0_VLAN          0x80000000
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_TSV1_OFF           0x0000015C

#define EMAC_TSV1_BYTE_CNT      0x0000FFFF
#define EMAC_TSV1_COLL_CNT      0x000F0000
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_RSV_OFF            0x00000160

#define EMAC_RSV_BYTE_CNT       0x0000FFFF
#define EMAC_RSV_PKT_IGNORED    0x00010000
#define EMAC_RSV_RXDV_SEEN      0x00020000
#define EMAC_RSV_CARR_SEEN      0x00040000
#define EMAC_RSV_REC_CODEV      0x00080000
#define EMAC_RSV_CRC_ERR        0x00100000
#define EMAC_RSV_LEN_CHKERR     0x00200000
#define EMAC_RSV_LEN_OUTRNG     0x00400000
#define EMAC_RSV_REC_OK         0x00800000
#define EMAC_RSV_MCAST          0x01000000
#define EMAC_RSV_BCAST          0x02000000
#define EMAC_RSV_DRIB_NIBB      0x04000000
#define EMAC_RSV_CTRL_FRAME     0x08000000
#define EMAC_RSV_PAUSE          0x10000000
#define EMAC_RSV_UNSUPP_OPC     0x20000000
#define EMAC_RSV_VLAN           0x40000000
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_FCC_OFF            0x00000170
/*@}*/

/*! \name EMAC Control Register */
/*@{*/
#define EMAC_FCS_OFF            0x00000174
/*@}*/

/*! \name EMAC Receiver Filter Register */
/*@{*/
#define EMAC_RFC_OFF            0x00000200

#define EMAC_RFC_UCAST_EN       0x00000001
#define EMAC_RFC_BCAST_EN       0x00000002
#define EMAC_RFC_MCAST_EN       0x00000004
#define EMAC_RFC_UCAST_HASH_EN  0x00000008
#define EMAC_RFC_MCAST_HASH_EN  0x00000010
#define EMAC_RFC_PERFECT_EN     0x00000020
#define EMAC_RFC_MAGP_WOL_EN    0x00001000
#define EMAC_RFC_PFILT_WOL_EN   0x00002000
/*@}*/

/*! \name EMAC Receiver Filter Register */
/*@{*/
#define EMAC_WOLSR_OFF          0x00000204

#define EMAC_WOL_UCAST          0x00000001
#define EMAC_WOL_BCAST          0x00000002
#define EMAC_WOL_MCAST          0x00000004
#define EMAC_WOL_UCAST_HASH     0x00000008
#define EMAC_WOL_MCAST_HASH     0x00000010
#define EMAC_WOL_PERFECT        0x00000020
#define EMAC_WOL_RX_FILTER      0x00000080
#define EMAC_WOL_MAG_PACKET     0x00000100
/*@}*/

/*! \name EMAC Receiver Filter Register */
/*@{*/
#define EMAC_WOLCR_OFF          0x00000208
/*@}*/

/*! \name EMAC Receiver Filter Register */
/*@{*/
#define EMAC_HASHFILTERL_OFF    0x00000210
/*@}*/

/*! \name EMAC Receiver Filter Register */
/*@{*/
#define EMAC_HASHFILTERH_OFF    0x00000214
/*@}*/

/*! \name EMAC Module Control Register */
/*@{*/
#define EMAC_INT_STAT_OFF       0x00000FE0

#define EMAC_INT_RX_OVERRUN     0x00000001
#define EMAC_INT_RX_ERR         0x00000002
#define EMAC_INT_RX_FIN         0x00000004
#define EMAC_INT_RX_DONE        0x00000008
#define EMAC_INT_TX_UNDERRUN    0x00000010
#define EMAC_INT_TX_ERR         0x00000020
#define EMAC_INT_TX_FIN         0x00000040
#define EMAC_INT_TX_DONE        0x00000080
#define EMAC_INT_SOFT_INT       0x00001000
#define EMAC_INT_WAKEUP         0x00002000
/*@}*/

/*! \name EMAC Module Control Register */
/*@{*/
#define EMAC_INT_ENA_OFF        0x00000FE4
/*@}*/

/*! \name EMAC Module Control Register */
/*@{*/
#define EMAC_INT_CLR_OFF        0x00000FE8
/*@}*/

/*! \name EMAC Module Control Register */
/*@{*/
#define EMAC_INT_SET_OFF        0x00000FEC
/*@}*/

/*! \name EMAC Module Control Register */
/*@{*/
#define EMAC_PD_OFF             0x00000FF4

#define EMAC_PD_POWER_DOWN      0x80000000
/*@}*/

/*! \name EMAC Module Control Register */
/*@{*/
#define EMAC_MODULE_ID_OFF      0x00000FFC
/*@}*/

//#define EMAC_RCTRL_
//#define EMAC_RHASH_
//#define EMAC_RINFO_
//#define EMAC_TCTRL_
//#define EMAC_TINFO_




#define EMAC_MAC1     0x50000000
#define EMAC_MAC2     0x50000004
#define EMAC_IPGT     0x50000008
#define EMAC_IPGR     0x5000000C
#define EMAC_CLRT     0x50000010
#define EMAC_MAXF     0x50000014
#define EMAC_SUPP     0x50000018
#define EMAC_TEST     0x5000001C
#define EMAC_MCFG     0x50000020
#define EMAC_MCMD     0x50000024
#define EMAC_MADR     0x50000028
#define EMAC_MWTD     0x5000002C
#define EMAC_MRDD     0x50000030
#define EMAC_MIND     0x50000034
#define EMAC_SA0      0x50000040
#define EMAC_SA1      0x50000044
#define EMAC_SA2      0x50000048
#define EMAC_Command  0x50000100

#define EMAC_Status Status       0x50000104
#define EMAC_RxDescriptor        0x50000108
#define EMAC_RxStatus            0x5000010C
#define EMAC_RxDescriptorNumber  0x50000110
#define EMAC_RxProduceIndex      0x50000114
#define EMAC_RxConsumeIndex      0x50000118
#define EMAC_TxDescriptor        0x5000011C
#define EMAC_TxStatus            0x50000120
#define EMAC_TxDescriptorNumber  0x50000124
#define EMAC_TxProduceIndex      0x50000128
#define EMAC_TxConsumeIndex      0x5000012C
#define EMAC_TSV0                0x50000158
#define EMAC_TSV1                0x5000015C
#define EMAC_RSV                 0x50000160
#define EMAC_FlowControlCounter  0x50000170
#define EMAC_FlowControlStatus   0x50000174

#define EMAC_RxFilterWoLStatus   0x50000204
#define EMAC_RxFilterWoLClear    0x50000208
#define EMAC_HashFilterL         0x50000210
#define EMAC_HashFilterH         0x50000214
#define EMAC_IntStatus           0x50000FE0
#define EMAC_IntEnable           0x50000FE4
#define EMAC_IntClear            0x50000FE8
#define EMAC_IntSet              0x50000FEC
#define EMAC_PowerDown           0x50000FF4

/*@}*/

#endif
