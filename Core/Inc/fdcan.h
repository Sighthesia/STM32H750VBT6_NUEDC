#ifndef __FDCAN_H__
#define __FDCAN_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

extern FDCAN_HandleTypeDef hfdcan2;

void MX_FDCAN2_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __FDCAN_H__ */
