/*
 * Protocol.h
 *
 *  Created on: Feb 17, 2022
 *      Author: thans
 */

#ifndef INC_PROTOCOL_H_
#define INC_PROTOCOL_H_

// include stm32h7 driver
#include "stm32h7xx.h"

typedef enum
{
  Idle_state       		= 0x00,
  Error_Link_length    	= 0x01,
  Singularity     		= 0x02,
} PotocolTypeDef;




#endif /* INC_PROTOCOL_H_ */
