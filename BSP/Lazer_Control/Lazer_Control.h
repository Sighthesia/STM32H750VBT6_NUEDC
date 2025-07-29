/** ***************************************************************************
 * @File Name: Lazer_Control.h
 * @brief
 * @Author : Sighthesia
 * @Version : 1.0.0
 * @Creat Date : 2025-07-28
 * ----------------------------------------------------------------------------
 * @Modification
 * @Author : Sighthesia
 * @Changes :
 * @Modifi Date :
 */
#ifndef __LAZER_CONTROL_H
#define __LAZER_CONTROL_H

#include "Lazer_Control_HAL.h"

#include "lwrb.h"
#include "stdint.h"
#include "stdio.h"

#define FRAME_MAX_LEN 7
#define FRAME_TAIL 0xFA

#define REC_FRAME_HEAD1 0x55
#define REC_FRAME_HEAD2 0xAA

#define LAZ_FRAME_HEAD1 0x56
#define LAZ_FRAME_HEAD2 0xA1

extern uint8_t frame_buffer[FRAME_MAX_LEN];
extern uint8_t frame_index;
extern uint8_t frame_receiving;

#endif /* __LAZER_CONTROL_H */