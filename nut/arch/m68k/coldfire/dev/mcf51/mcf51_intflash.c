/*
 * Copyright 2012-2016 by Embedded Technologies s.r.o. All rights reserved.
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

#include <arch/m68k.h>
#include <sys/atom.h>

/*!
 * \addtogroup xgMcf51
 */
/*@{*/

#define BM_FLASH_ERR_MASK 0x30


__attribute__ ((section (".data.flash_ram"))) int Mcf51IntFlashRamCMD(uint8_t cmd)
{
	MCF_FCMD = cmd;
	MCF_FSTAT = 0x80U;
	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) == 0U)
	{
		while ((MCF_FSTAT & MCF_FSTAT_FCCF) == 0U)
		{
		}
	}
	else
	{
		return -1;
	}
	return 0;
}

/*!
 * \brief Return flash protect register
 *
 * \return MCF_FPROT
 */
uint8_t Mcf51IntFlashProtectRegister(void)
{
	return MCF_FPROT;
}

int Mcf51IntFlashInit(void)
{

	MCF_FSTAT = 0x30; /* Clear FPVIOL & FACERR flag */
	MCF_FCDIV = 0x50; /* Initialize FCDIV register */
	return 0;
}

/*!
 * \brief Read data from flash memory
 *
 * \param dst		Destination in flash memory
 * \param *data		Pointer to data that you want read
 * \param size		Size of data
 *
 * \return 0 = SUCCESS
 */
int Mcf51IntFlashRead(uint32_t dst, uint32_t *data, uint32_t size)
{
	uint32_t *p_dst = (uint32_t *) dst;
	uint32_t *p_data = data;
	while ((uint32_t) p_dst < dst + size * sizeof(uint32_t))
	{
		*p_data = *p_dst;
		p_data++;
		p_dst++;
	}
	return 0;
}

/*!
 * \brief Write data to flash memory
 *
 * \param dst		Destination in flash memory
 * \param *data		Pointer to data that you want write
 * \param size		Size of data
 *
 * \return 0 = SUCCESS, -1 = ERROR
 */
int Mcf51IntFlashWrite(uint32_t dst, uint32_t *data, uint32_t size)
{
	int i;
	uint32_t *flash = (uint32_t *) dst;

	if ((dst > 0x0001FFFFUL) || (((dst + size) - 1U) > 0x0001FFFFUL))
	{
		return -1; /* Address is out of FLASH memory */
	}
	if ((MCF_FSTAT & MCF_FSTAT_FCCF) == 0U)
	{ /* Is previous command completed ? */
		return -1;
	}

	NutUseCritical();
	NutEnterCritical();

	MCF_FSTAT = 0x00U; /* Init. flash engine */
	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) != 0U)
	{
		MCF_FSTAT = BM_FLASH_ERR_MASK;
	}

	size = (size / sizeof(uint32_t)) + ((size % sizeof(uint32_t)) ? 1 : 0); // prepocet na velikost v int

	for (i = 0; i < size; ++i)
	{

		if (*flash == 0xFFFFFFFF)
		{
			*flash = data[i];
		}
		else
		{
			NutJumpOutCritical();
			return -1;
		}
		flash++;
		if (Mcf51IntFlashRamCMD(0x25) != 0)
			break;
	}

	NutExitCritical();

	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) != 0U)
	{
		if ((MCF_FSTAT & MCF_FSTAT_FPVIOL) != 0U)
		{
			return -1; //ERR_PROTECT;
		}
		else
		{
			return -1; //ERR_NOTAVAIL;
		}
	}
	return 0;
}

/*!
 * \brief Erase Sector (128 sectors of 1024 bytes each)
 *
 * \param addr	Flash memory adress
 *
 * \return 0 = SUCCESS, -1 = ERROR
 */
int Mcf51IntFlashSectorErase(uint32_t addr)
{
	if (addr > 0x0001FFFFUL)
	{
		return -1; //ERR_RANGE;
	}
	if ((MCF_FSTAT & MCF_FSTAT_FCCF) == 0U)
	{
		return -1; //ERR_BUSY;
	}

	NutUseCritical();
	NutEnterCritical();

	MCF_FSTAT = 0x00U;
	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) != 0U)
	{ /* Protection violation or access error? */
		MCF_FSTAT = BM_FLASH_ERR_MASK; /* Clear FPVIOL & FACERR flag */
	}
	*(volatile uint32_t *) (addr) = 0x0; /* Write data to the flash memory */
	Mcf51IntFlashRamCMD(0x40);

	NutExitCritical();

	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) != 0U)
	{
		if ((MCF_FSTAT & MCF_FSTAT_FPVIOL) != 0U)
		{
			return -1; /* Return error code ERR_PROTECT */
		}
		else
		{
			return -1; /* Return error code ERR_NOTAVAIL */
		}
	}

	return 0;
}

/*!
 * \brief Erase entire flash
 *
 * \return 0 = SUCCESS, -1 = ERROR
 */
int Mcf51IntFlashMassErase(void)
{
	if ((MCF_FSTAT & MCF_FSTAT_FCCF) == 0U)
	{
		return -1; //ERR_BUSY;
	}

	NutUseCritical();
	NutEnterCritical();

	MCF_FSTAT = 0x00U;
	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) != 0U)
	{ /* Protection violation or access error? */
		MCF_FSTAT = BM_FLASH_ERR_MASK; /* Clear FPVIOL & FACERR flag */
	}
	*(volatile uint32_t *) (0x0) = 0x0; /* Write data to the flash memory */
	Mcf51IntFlashRamCMD(0x41);

	NutExitCritical();

	if ((MCF_FSTAT & BM_FLASH_ERR_MASK) != 0U)
	{
		if ((MCF_FSTAT & MCF_FSTAT_FPVIOL) != 0U)
		{
			return -1; /* Return error code ERR_PROTECT */
		}
		else
		{
			return -1; /* Return error code ERR_NOTAVAIL */
		}
	}

	return 0;
}

