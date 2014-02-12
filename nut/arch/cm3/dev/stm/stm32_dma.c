#include <arch/cm3.h>
#include <dev/irqreg.h>
#include <arch/cm3.h>
#include <arch/cm3/stm/stm32f10x.h>
#include <arch/cm3/stm/stm32f10x_dma.h>
#include <arch/cm3/stm/stm32f10x_rcc.h>

//HANDLE pointer on which to generate event in the end of DMA transfer
//They are modified in DMA_Register_Interrupt
HANDLE* dma1_args[7];
HANDLE* dma2_args[5];

//DMA IRQ numbers for channels (for convenience of registering handler)
IRQn_Type DMA_IRQn[12]={
  DMA1_Channel1_IRQn          ,
  DMA1_Channel2_IRQn          ,
  DMA1_Channel3_IRQn          ,
  DMA1_Channel4_IRQn          ,
  DMA1_Channel5_IRQn          ,
  DMA1_Channel6_IRQn          ,
  DMA1_Channel7_IRQn          ,
  DMA2_Channel1_IRQn          ,
  DMA2_Channel2_IRQn          ,
  DMA2_Channel3_IRQn          ,
  DMA2_Channel4_IRQn          ,
  DMA2_Channel5_IRQn
};

//Setup function for DMA transfer
void DMA_Setup(DMA_Channel_TypeDef* channel,void* mem,void* periph,uint16_t length,uint32_t flags){
    uint32_t b;
    RCC->AHBENR|=RCC_AHBPeriph_DMA1|RCC_AHBPeriph_DMA2;
    channel->CCR=0;
    channel->CNDTR=length;
    channel->CPAR=periph;
    channel->CMAR=mem?mem:&b;
    channel->CCR=flags;
    if(mem){
        channel->CCR|=DMA_MemoryInc_Enable;
    };
};

void DMA_Enable(DMA_Channel_TypeDef* chan){
    chan->CCR |= DMA_Enable_Flag;
};

void DMA_Disable(DMA_Channel_TypeDef* chan){
    chan->CCR &= ~DMA_Enable_Flag;
};

//static void DMA_Handler_IRQ(void);
//Interrupt handler for the DMA transfer finished event.
//It is common for all 12 interrupts
void DMA_Handler_IRQ(void){
    uint32_t tmpreg;
    uint32_t i;
    tmpreg=DMA1->ISR;
    for(i=0;i<7;i++){
        if(tmpreg & (1<<(i*4+1))){
            if(dma1_args[i]!=NULL)
                NutEventPostFromIrq(dma1_args[i]);
        }
    };
    DMA1->IFCR=(uint32_t)-1;
    tmpreg=DMA2->ISR;
    for(i=0;i<5;i++){
        if(tmpreg & (1<<(i*4+1))){
            if(dma2_args[i]!=NULL)
                NutEventPostFromIrq(dma2_args[i]);
        }
    };
    DMA2->IFCR=(uint32_t)-1;
};

//Initialise DMA subsystem - zero out handle pointers, register interrupt handlers,
//and enable interrupts.
void DMA_Init(){
    int i;
    for(i=0;i<sizeof(dma1_args)/sizeof(HANDLE*);i++) dma1_args[i]=NULL;
    for(i=0;i<sizeof(dma2_args)/sizeof(HANDLE*);i++) dma2_args[i]=NULL;
    for(i=0;i<sizeof(DMA_IRQn)/sizeof(IRQn_Type);i++){
        Cortex_RegisterInt(DMA_IRQn[i],DMA_Handler_IRQ);
        NVIC_EnableIRQ(DMA_IRQn[i]);
    };
};

void DMA_ClearFlag(uint32_t DMA_FLAG){//FIXME: rewrite
  if ((DMA_FLAG & DMA2_FLAG_Mask) != (uint32_t)0){
    /* Clear the selected DMA flags */
    DMA2->IFCR = DMA_FLAG;
  } else {
    /* Clear the selected DMA flags */
    DMA1->IFCR = DMA_FLAG;
  }
};

//FIXME: should this be simpler?
void DMA_Register_Interrupt(DMA_Channel_TypeDef* channel,HANDLE* handle){
    if(channel==DMA1_Channel1){
        dma1_args[0]=handle;
    }else if(channel==DMA1_Channel2){
        dma1_args[1]=handle;
    }else if(channel==DMA1_Channel3){
        dma1_args[2]=handle;
    }else if(channel==DMA1_Channel4){
        dma1_args[3]=handle;
    }else if(channel==DMA1_Channel5){
        dma1_args[4]=handle;
    }else if(channel==DMA1_Channel6){
        dma1_args[5]=handle;
    }else if(channel==DMA1_Channel7){
        dma1_args[6]=handle;
    }else if(channel==DMA2_Channel1){
        dma2_args[0]=handle;
    }else if(channel==DMA2_Channel2){
        dma2_args[1]=handle;
    }else if(channel==DMA2_Channel3){
        dma2_args[2]=handle;
    }else if(channel==DMA2_Channel4){
        dma2_args[3]=handle;
    }else if(channel==DMA2_Channel5){
        dma2_args[4]=handle;
    }
};

