#ifndef FLASH_LAYOUT_H
#define FLASH_LAYOUT_H

#define W25QXX_FLASH_SECTOR_SIZE             4096U
#define FLASH_ANO_STORAGE_ADDRESS            0U
#define FLASH_ARM_CALIBRATION_STORAGE_A      4096U
#define FLASH_ARM_CALIBRATION_STORAGE_B      8192U
#define FLASH_APP_ROLE_STORAGE_ADDRESS       12288U
#define FLASH_ARM_ANGLE_STORAGE_ADDRESS       16384U

/* Firmware before the calibration slots were added stored the role here. */
#define FLASH_APP_ROLE_LEGACY_ADDRESS        4096U

#endif /* FLASH_LAYOUT_H */
