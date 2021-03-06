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

#include <dev/gpio.h>

/*!
 * \addtogroup xgMcf51qe
 */
/*@{*/

/*!
 * \brief Get pin configuration.
 *
 * \param bank GPIO bank/port number.
 * \param bit  Bit number of the specified bank/port.
 *
 * \return Attribute flags of the pin.
 */
uint32_t GpioPinConfigGet(int bank, int bit)
{
	uint32_t rc = 0;
	uint8_t mask = _BV(bit);

	/* Verify bank */
	if (bank > PORTJ)
	{
		return 0;
	}

	if (MCF_GPIO_DD(bank) & mask)
	{
		rc = GPIO_CFG_OUTPUT;
	}

	/* Read pin config */
	if (MCF_GPIO_PE(bank) & mask)
	{
		rc |= GPIO_CFG_PULLUP;
	}
	if (MCF_GPIO_SE(bank) & mask)
	{
		rc |= GPIO_CFG_SLEW_RATE;
	}
	if (MCF_GPIO_DS(bank) & mask)
	{
		rc |= GPIO_CFG_DRIVE_STRENGTH;
	}

	return rc;
}

/*!
 * \brief Set pin configuration.
 *
 * Applications may also use this function to make sure, that a specific
 * attribute is available for a specific pin.
 *
 * \note GPIO pins are typically initialized to a safe state after power
 *       up. This routine is not able to determine the consequences of
 *       changing pin configurations. In the worst case you may permanently
 *       damage your hardware by bad pin settings.
 *
 * \param bank  GPIO bank/port number.
 * \param bit   Bit number of the specified bank/port.
 * \param flags Attribute flags.
 *
 * \return 0 if all attributes had been set, -1 otherwise.
 */
int GpioPinConfigSet(int bank, int bit, uint32_t flags)
{
	GpioPortConfigSet(bank, _BV(bit), flags);

	/* Check the result. */
	if (GpioPinConfigGet(bank, bit) != flags)
	{
		return -1;
	}
	return 0;
}

/*!
 * \brief Set port wide pin configuration.
 *
 * \note This function does not check for undefined ports and pins or
 *       invalid attributes. If this is required, use GpioPinConfigSet().
 *
 * \param bank  GPIO bank/port number.
 * \param mask  The given attributes are set for a specific pin, if the
 *              corresponding bit in this mask is 1.
 * \param flags Attribute flags to set.
 *
 * \return Always 0.
 */
int GpioPortConfigSet(int bank, uint32_t mask, uint32_t flags)
{
	uint32_t role = flags & (GPIO_CFG_PERIPHERAL_MASK | GPIO_CFG_OUTPUT);

	/* Verify bank */
	if (bank > PORTJ)
	{
		return 0;
	}

	/* Configure GPIO direction first */
	/* For PTC3(Reset) and PTD6(BKGD) is GPIO function in GPIO_CFG_ALT1 */
	if (role & GPIO_CFG_OUTPUT)
	{
		MCF_GPIO_DD(bank) |= mask;
	}
	else if (role == GPIO_CFG_INPUT)
	{
		MCF_GPIO_DD(bank) &= ~mask;
	}

	/* Configure pin features */
	if (flags & GPIO_CFG_PULLUP)
	{
		MCF_GPIO_PE(bank) |= mask;
	}
	else
	{
		MCF_GPIO_PE(bank) &= ~mask;
	}

	if (flags & GPIO_CFG_SLEW_RATE)
	{
		MCF_GPIO_SE(bank) |= mask;
	}
	else
	{
		MCF_GPIO_SE(bank) &= ~mask;
	}

	if (flags & GPIO_CFG_DRIVE_STRENGTH)
	{
		MCF_GPIO_DS(bank) |= mask;
	}
	else
	{
		MCF_GPIO_DS(bank) &= ~mask;
	}

	return 0;
}
