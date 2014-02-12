#ifndef _STM32_SPI_H_
#define _STM32_SPI_H_

static int Stm32SpiSetup(NUTSPINODE * node);
static void Stm32SpiEnable(SPI_TypeDef* SPIx);
static void Stm32SpiDisable(SPI_TypeDef* SPIx);
static int Stm32SpiBusNodeInit(NUTSPINODE * node);
static int Stm32SpiBusSelect(NUTSPINODE * node, uint32_t tmo);
static int Stm32SpiBusDeselect(NUTSPINODE * node);
static int Stm32SpiBusTransfer(NUTSPINODE * node, const void *txbuf, void *rxbuf, int xlen);

#endif
