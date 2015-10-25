/**
  ******************************************************************************
  * @file    main.h 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    23-March-2012
  * @brief   Header for Arduino main.c module
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
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
#ifndef __MAIN_H
#define __MAIN_H

/* Includes ------------------------------------------------------------------*/
#include "utilities.h"
#include "setup.h"
//#include "loop.h"
#include "lsm6ds3.h"
#include "lis302dl.h"
//#include "PID_v1.h"
#include <stdio.h>
#include <string.h>
//#include "timeb.h"
//#include "const.h"
#include "command.h"
#include "configuration.h"
#include "uart.h"
#include "Six_Axis_Sensor.h"

#if 0
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "stm32f0xx_conf.h"
#include "utilities.h"
#include "serial.h"
#include "gpio.h"

#include <rt_misc.h>
#pragma import(__use_no_semihosting_swi)
struct __FILE { int handle; /* Add whatever you need here */ };
FILE __stdout;
FILE __stdin;
#endif

/* Exported constants --------------------------------------------------------*/
/* Exported macros -----------------------------------------------------------*/
/* typedef -------------------------------------------------------------------*/
/* function prototypes -------------------------------------------------------*/

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/


