/*
 * Copyright 2012 by Embedded Technologies s.r.o
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

#ifndef MCF51CN_CONTROL_H_
#define MCF51CN_CONTROL_H_

/* System Power Management Status and Control 1 Register */
#define MCF_SPMSC1 						 (*(volatile uint8_t *)(0xFFFF8120))
#define MCF_SPMSC1_BGBE                     0x01
#define MCF_SPMSC1_LVDE                     0x04
#define MCF_SPMSC1_LVDSE                    0x08
#define MCF_SPMSC1_LVDRE                    0x10
#define MCF_SPMSC1_LVDIE                    0x20
#define MCF_SPMSC1_LVDACK                   0x40
#define MCF_SPMSC1_LVDF                     0x80


/* System Power Management Status and Control 2 Register */
#define MCF_SPMSC2 						 (*(volatile uint8_t *)(0xFFFF8121))
#define MCF_SPMSC2_PPDC                     0x01
#define MCF_SPMSC2_PPDE                     0x02
#define MCF_SPMSC2_PPDACK                   0x04
#define MCF_SPMSC2_PPDF                     0x08
#define MCF_SPMSC2_LPWUI                    0x20
#define MCF_SPMSC2_LPRS                     0x40
#define MCF_SPMSC2_LPR                      0x80


/* System Power Management Status and Control 3 Register */
#define MCF_SPMSC3 						 (*(volatile uint8_t *)(0xFFFF8123))
#define MCF_SPMSC3_LVWIE                    0x08
#define MCF_SPMSC3_LVWV                     0x10
#define MCF_SPMSC3_LVDV                     0x20
#define MCF_SPMSC3_LVWACK                   0x40
#define MCF_SPMSC3_LVWF                     0x80


/* System Clock Gating Control 1 Register */
#define MCF_SCGC1 							 (*(volatile uint8_t *)(0xFFFF8108))
#define MCF_SCGC1_SCI1                      0x01
#define MCF_SCGC1_SCI2                      0x02
#define MCF_SCGC1_IIC1                      0x04
#define MCF_SCGC1_IIC2                      0x08
#define MCF_SCGC1_ADC                       0x10
#define MCF_SCGC1_TPM1                      0x20
#define MCF_SCGC1_TPM2                      0x40
#define MCF_SCGC1_MTIM1                     0x80


/* System Clock Gating Control 2 Register */
#define MCF_SCGC2 							 (*(volatile uint8_t *)(0xFFFF8109))
#define MCF_SCGC2_SPI1                      0x01
#define MCF_SCGC2_SPI2                      0x02
#define MCF_SCGC2_RTC                       0x04
#define MCF_SCGC2_KBI1                      0x08
#define MCF_SCGC2_KBI2                      0x10
#define MCF_SCGC2_IRQ                       0x20
#define MCF_SCGC2_FTSR                      0x40
#define MCF_SCGC2_SCI3                      0x80


/* System Clock Gating Control 3 Register */
#define MCF_SCGC3 							 (*(volatile uint8_t *)(0xFFFF810A))
#define MCF_SCGC3_PTA                       0x01
#define MCF_SCGC3_PTB                       0x02
#define MCF_SCGC3_PTC                       0x04
#define MCF_SCGC3_PTD                       0x08
#define MCF_SCGC3_PTE                       0x10
#define MCF_SCGC3_PTF                       0x20
#define MCF_SCGC3_PTG                       0x40
#define MCF_SCGC3_PTH                       0x80


/* System Clock Gating Control 4 Register */
#define MCF_SCGC4 							 (*(volatile uint8_t *)(0xFFFF810B))
#define MCF_SCGC4_PTJ                       0x01
#define MCF_SCGC4_FEC                       0x02
#define MCF_SCGC4_MB                        0x04
#define MCF_SCGC4_MC                        0x08
#define MCF_SCGC4_MTIM2                     0x10


/* System Options Register 1 */
#define MCF_SOPT1 							(*(volatile uint8_t *)(0xFFFF8101))
#define MCF_SOPT1_COPW                      0x01
#define MCF_SOPT1_COPCLKS                   0x02
#define MCF_SOPT1_COPT0                     0x04
#define MCF_SOPT1_COPT1                     0x08
#define MCF_SOPT1_WAITE                     0x10
#define MCF_SOPT1_STOPE                     0x20
#define MCF_SOPT1_SL                        0x40
#define MCF_SOPT1_COPT                      0x0C
#define MCF_SOPT1_COPT_BITNUM               0x02


/* SIM Options Register 3 */
#define MCF_SOPT3 							(*(volatile uint8_t *)(0xFFFF8103))
#define MCF_SOPT3_PCS_DISABLED              0x00
#define MCF_SOPT3_PCS_OSCOUT                0x01
#define MCF_SOPT3_PCS_MCGOUT                0x02
#define MCF_SOPT3_PCS_BUSCLK                0x03
#define MCF_SOPT3_PCS_BITNUM                0x00
#define MCF_SOPT3_CS                        0x1C
#define MCF_SOPT3_CS_BITNUM                 0x02



/* System Device Identification Register */
#define MCF_SDID 							(*(volatile uint16_t *)(0xFFFF8106))
#define MCF_SDID_ID0                        0x01
#define MCF_SDID_ID1                        0x02
#define MCF_SDID_ID2                        0x04
#define MCF_SDID_ID3                        0x08
#define MCF_SDID_ID4                        0x10
#define MCF_SDID_ID5                        0x20
#define MCF_SDID_ID6                        0x40
#define MCF_SDID_ID7                        0x80
#define MCF_SDID_ID8                        0x0100
#define MCF_SDID_ID9                        0x0200
#define MCF_SDID_ID10                       0x0400
#define MCF_SDID_ID11                       0x0800
#define MCF_SDID_REV0                       0x1000
#define MCF_SDID_REV1                       0x2000
#define MCF_SDID_REV2                       0x4000
#define MCF_SDID_REV3                       0x8000
#define MCF_SDID_ID                         0x0FFF
#define MCF_SDID_ID_BITNUM                  0x00
#define MCF_SDID_REV                        0xF000
#define MCF_SDID_REV_BITNUM                 0x0C



/* SIM Internal Peripheral Select Register */
#define MCF_SIMIPS 							(*(volatile uint8_t *)(0xFFFF810C))
#define MCF_SIMIPS_MTIM1                    0x01
#define MCF_SIMIPS_MTIM2                    0x02
#define MCF_SIMIPS_TPM1                     0x04
#define MCF_SIMIPS_TPM2                     0x08
#define MCF_SIMIPS_MTIM_1                   0x03
#define MCF_SIMIPS_MTIM_1_BITNUM            0x00
#define MCF_SIMIPS_TPM_1                    0x0C
#define MCF_SIMIPS_TPM_1_BITNUM             0x02

/*** FCDIV - FLASH Clock Divider Register; 0xFFFF82E0 ***/
#define MCF_FCDIV 								(*(volatile uint8_t *)(0xFFFF82E0))
#define MCF_FCDIV_PRDIV8                    	0x40U
#define MCF_FCDIV_FDIVLD                    	0x80U
#define MCF_FCDIV_FDIV                      	0x3FU
#define MCF_FCDIV_FDIV_BITNUM               	0x00U


/*** FOPT - Flash Options Register; 0xFFFF82E1 ***/
#define MCF_FOPT 								(*(volatile uint8_t *)(0xFFFF82E1))
#define MCF_FOPT_SEC                        	0x03U
#define MCF_FOPT_SEC_BITNUM                 	0x00U
#define MCF_FOPT_KEYEN                      	0xC0U
#define MCF_FOPT_KEYEN_BITNUM               	0x06U


/*** FCNFG - Flash Configuration Register; 0xFFFF82E3 ***/
#define MCF_FCNFG 								(*(volatile uint8_t *)(0xFFFF82E3))
#define MCF_FCNFG_KEYACC                    	0x20U


/*** FPROT - Flash Protection Register; 0xFFFF82E4 ***/
#define MCF_FPROT 								(*(volatile uint8_t *)(0xFFFF82E4))
#define MCF_FPROT_FPOPEN                    	0x01U
#define MCF_FPROT_FPS                       	0xFEU
#define MCF_FPROT_FPS_BITNUM                	0x01U


/*** FSTAT - Flash Status Register; 0xFFFF82E5 ***/
#define MCF_FSTAT 								(*(volatile uint8_t *)(0xFFFF82E5))
#define MCF_FSTAT_FBLANK                    	0x04U
#define MCF_FSTAT_FACCERR                   	0x10U
#define MCF_FSTAT_FPVIOL                    	0x20U
#define MCF_FSTAT_FCCF                      	0x40U
#define MCF_FSTAT_FCBEF                     	0x80U


/*** FCMD - Flash Command Register; 0xFFFF82E6 ***/
#define MCF_FCMD 								(*(volatile uint8_t *)(0xFFFF82E6))
#define MCF_FCMD_FCMD                       	0x7FU
#define MCF_FCMD_FCMD_BITNUM                	0x00U

/*** SPMSC1 - System Power Management Status and Control 1 Register; 0xFFFF8120 ***/
#define MCF_SPMSC1 								(*(volatile uint8_t *)(0xFFFF8120))

#define MCF_SPMSC1_BGBE_MASK                0x01U
#define MCF_SPMSC1_LVDE_MASK                0x04U
#define MCF_SPMSC1_LVDSE_MASK               0x08U
#define MCF_SPMSC1_LVDRE_MASK               0x10U
#define MCF_SPMSC1_LVDIE_MASK               0x20U
#define MCF_SPMSC1_LVDACK_MASK              0x40U
#define MCF_SPMSC1_LVDF_MASK                0x80U


/*** SPMSC2 - System Power Management Status and Control 2 Register; 0xFFFF8121 ***/
#define MCF_SPMSC2 								(*(volatile uint8_t *)(0xFFFF8121))

#define MCF_SPMSC2_PPDC_MASK                0x01U
#define MCF_SPMSC2_PPDE_MASK                0x02U
#define MCF_SPMSC2_PPDACK_MASK              0x04U
#define MCF_SPMSC2_PPDF_MASK                0x08U
#define MCF_SPMSC2_LPWUI_MASK               0x20U
#define MCF_SPMSC2_LPRS_MASK                0x40U
#define MCF_SPMSC2_LPR_MASK                 0x80U


/*** SPMSC3 - System Power Management Status and Control 3 Register; 0xFFFF8123 ***/
#define MCF_SPMSC3 								(*(volatile uint8_t *)(0xFFFF8123))

#define MCF_SPMSC3_LVWIE_MASK               0x08U
#define MCF_SPMSC3_LVWV_MASK                0x10U
#define MCF_SPMSC3_LVDV_MASK                0x20U
#define MCF_SPMSC3_LVWACK_MASK              0x40U
#define MCF_SPMSC3_LVWF_MASK                0x80U

/*** PTDPF1 - Port D Routing Register 1; 0xFFFF80C6 ***/
#define MCF_PTDPF1 (*(volatile uint8_t*)(0xFFFF80C6))
#define MCF_PTDPF1_D40                      0x01U
#define MCF_PTDPF1_D41                      0x02U
#define MCF_PTDPF1_D50                      0x04U
#define MCF_PTDPF1_D51                      0x08U
#define MCF_PTDPF1_D60                      0x10U
#define MCF_PTDPF1_D61                      0x20U
#define MCF_PTDPF1_D70                      0x40U
#define MCF_PTDPF1_D71                      0x80U
#define MCF_PTDPF1_D4                       0x03U
#define MCF_PTDPF1_D4_BITNUM                0x00U
#define MCF_PTDPF1_D5                       0x0CU
#define MCF_PTDPF1_D5_BITNUM                0x02U
#define MCF_PTDPF1_D6                       0x30U
#define MCF_PTDPF1_D6_BITNUM                0x04U
#define MCF_PTDPF1_D7                       0xC0U
#define MCF_PTDPF1_D7_BITNUM                0x06U


/*** PTDPF2 - Port D Routing Register 2; 0xFFFF80C7 ***/
#define MCF_PTDPF2 (*(volatile uint8_t*)(0xFFFF80C7))
#define MCF_PTDPF2_D00                      0x01U
#define MCF_PTDPF2_D01                      0x02U
#define MCF_PTDPF2_D10                      0x04U
#define MCF_PTDPF2_D11                      0x08U
#define MCF_PTDPF2_D20                      0x10U
#define MCF_PTDPF2_D21                      0x20U
#define MCF_PTDPF2_D30                      0x40U
#define MCF_PTDPF2_D31                      0x80U
#define MCF_PTDPF2_D0                       0x03U
#define MCF_PTDPF2_D0_BITNUM                0x00U
#define MCF_PTDPF2_D1                       0x0CU
#define MCF_PTDPF2_D1_BITNUM                0x02U
#define MCF_PTDPF2_D2                       0x30U
#define MCF_PTDPF2_D2_BITNUM                0x04U
#define MCF_PTDPF2_D3                       0xC0U
#define MCF_PTDPF2_D3_BITNUM                0x06U

/*** MCF_SRS - System Reset Status Register; 0xFFFF8100 ***/
#define MCF_SRS (*(volatile uint8_t*)(0xFFFF8100))
#define MCF_SRS_LVD                         0x02U
#define MCF_SRS_LOC                         0x04U
#define MCF_SRS_ILAD                        0x08U
#define MCF_SRS_ILOP                        0x10U
#define MCF_SRS_COP                         0x20U
#define MCF_SRS_PIN                         0x40U
#define MCF_SRS_POR                         0x80U

#endif /* MCF51CN_CONTROL_H_ */
