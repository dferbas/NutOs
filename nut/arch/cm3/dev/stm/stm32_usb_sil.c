/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
* File Name          : usb_sil.c
* Author             : MCD Application Team
* Version            : V3.1.1
* Date               : 04/07/2010
* Description        : Simplified Interface Layer for Global Initialization and
*                      Endpoint Rea/Write operations.
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/*!
 * \verbatim
 * $Id: stm32_usb_sil.c 4608 2012-09-14 13:14:15Z haraldkipp $
 * \endverbatim
 */

/* Includes ------------------------------------------------------------------*/
#include <dev/usb_stm32/usb_lib.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern variables ----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : USB_SIL_Init
* Description    : Initialize the USB Device IP and the Endpoint 0.
* Input          : None.
* Output         : None.
* Return         : Status.
*******************************************************************************/
uint32_t USB_SIL_Init(void)
{

  /* Perform OTG Device initialization procedure (including EP0 init) */
  OTG_DEV_Init();


  return 0;
}

/*******************************************************************************
* Function Name  : USB_SIL_Write
* Description    : Write a buffer of data to a selected endpoint.
* Input          : - bEpAddr: The address of the non control endpoint.
*                  - pBufferPointer: The pointer to the buffer of data to be written
*                    to the endpoint.
*                  - wBufferSize: Number of data to be written (in bytes).
* Output         : None.
* Return         : Status.
*******************************************************************************/
uint32_t USB_SIL_Write(uint8_t bEpAddr, uint8_t* pBufferPointer, uint32_t wBufferSize)
{

   /* Use the PCD interface layer function to write to the selected endpoint */
   OTGD_FS_PCD_EP_Write (bEpAddr, pBufferPointer, wBufferSize);
   //FIXME: почему не возвращается статус?

  return 0;
}

/*******************************************************************************
* Function Name  : USB_SIL_Read
* Description    : Write a buffer of data to a selected endpoint.
* Input          : - bEpAddr: The address of the non control endpoint.
*                  - pBufferPointer: The pointer to which will be saved the
*                     received data buffer.
* Output         : None.
* Return         : Number of received data (in Bytes).
*******************************************************************************/
uint32_t USB_SIL_Read(uint8_t bEpAddr, uint8_t* pBufferPointer)
{
  uint32_t DataLength = 0;

  USB_OTG_EP *ep;

  /* Get the structure pointer of the selected Endpoint */
  ep = OTGD_FS_PCD_GetOutEP(bEpAddr);

  /* Get the number of received data */
  DataLength = ep->xfer_len;

  /* Use the PCD interface layer function to read the selected endpoint */
  OTGD_FS_PCD_EP_Read (bEpAddr, pBufferPointer, DataLength);


  /* Return the number of received data */
  return DataLength;
}

/******************* (C) COPYRIGHT 2010 STMicroelectronics *****END OF FILE****/
