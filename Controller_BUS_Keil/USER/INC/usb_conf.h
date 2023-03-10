/**
  ******************************************************************************
  * @file    usb_conf.h
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    17-January-2014
  * @brief   General low level driver configuration
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_CONF__H__
#define __USB_CONF__H__

/* Includes ------------------------------------------------------------------*/

#include "stm32f0xx.h"
//#include "stm32f072b_discovery.h"

/** @addtogroup USB_DEVICE_DRIVER
  * @{
  */

/** @defgroup USB_CONF
  * @brief USB low level driver configuration file
  * @{
  */

/** @defgroup USB_CONF_Exported_Defines
  * @{
  */



/* Endpoints used by the device */
#define EP_NUM     (2)  /* EP0 + EP1 (IN/OUT) For HID */

/* buffer table base address */
#define BTABLE_ADDRESS      (0x000)

/* EP0, RX/TX buffers base address */
#define ENDP0_RX_ADDRESS   (0x40)
#define ENDP0_TX_ADDRESS   (0x80)

/* EP1 Tx buffer base address */
#define HID_IN_TX_ADDRESS  (0x150)

/* EP1 Rx buffer base address */
#define HID_OUT_RX_ADDRESS (0x160)

/**
  * @}
  */


/** @defgroup USB_CONF_Exported_Types
  * @{
  */
/**
  * @}
  */


/** @defgroup USB_CONF_Exported_Macros
  * @{
  */
/**
  * @}
  */

/** @defgroup USB_CONF_Exported_Variables
  * @{
  */
/**
  * @}
  */

/** @defgroup USB_CONF_Exported_FunctionsPrototype
  * @{
  */
/**
  * @}
  */


#endif /* __USB_CONF__H__ */


/**
* @}
*/

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
