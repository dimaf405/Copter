/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <hal.h>
#include "SPIDevice.h"
#include "sdcard.h"
#include "bouncebuffer.h"
#include "hwdef/common/spi_hook.h"
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_Filesystem/AP_Filesystem.h>
#ifndef HAL_BOOTLOADER_BUILD
#include <GCS_MAVLink/GCS.h>
#endif
#include "bouncebuffer.h"
#include "stm32_util.h"

extern const AP_HAL::HAL& hal;

#ifdef USE_POSIX
static FATFS SDC_FS; // FATFS object
#ifndef HAL_BOOTLOADER_BUILD
static HAL_Semaphore sem;
#endif
static bool block_device_running;   // 物理块设备（SDC/SPI）已初始化
static bool filesystem_mounted;     // FAT文件系统已成功挂载
static FRESULT last_mount_result = FR_NOT_READY;  // 最近一次挂载结果，用于GCS故障上报
#endif

#if HAL_USE_SDC
static SDCConfig sdcconfig = {
  SDC_MODE_4BIT,
  0
};
#elif HAL_USE_MMC_SPI
MMCDriver MMCD1;
static AP_HAL::OwnPtr<AP_HAL::SPIDevice> device;
static MMCConfig mmcconfig;
static SPIConfig lowspeed;
static SPIConfig highspeed;
static mmc_connect_error_t last_connect_error = MMC_CONNECT_ERROR_NONE;
static uint8_t last_connect_error_r1 = 0xFFU;
#endif

#if defined(USE_POSIX)
static const char *fatfs_result_name(FRESULT result)
{
    switch (result) {
    case FR_OK:                  return "OK";
    case FR_DISK_ERR:            return "DISK_ERR";
    case FR_INT_ERR:             return "INT_ERR";
    case FR_NOT_READY:           return "NOT_READY";
    case FR_NO_FILE:             return "NO_FILE";
    case FR_NO_PATH:             return "NO_PATH";
    case FR_INVALID_NAME:        return "INVALID_NAME";
    case FR_DENIED:              return "DENIED";
    case FR_EXIST:               return "EXIST";
    case FR_INVALID_OBJECT:      return "INVALID_OBJECT";
    case FR_WRITE_PROTECTED:     return "WRITE_PROTECTED";
    case FR_INVALID_DRIVE:       return "INVALID_DRIVE";
    case FR_NOT_ENABLED:         return "NOT_ENABLED";
    case FR_NO_FILESYSTEM:       return "NO_FILESYSTEM";
    case FR_MKFS_ABORTED:        return "MKFS_ABORTED";
    case FR_TIMEOUT:             return "TIMEOUT";
    case FR_LOCKED:              return "LOCKED";
    case FR_NOT_ENOUGH_CORE:     return "NOT_ENOUGH_CORE";
    case FR_TOO_MANY_OPEN_FILES: return "TOO_MANY_OPEN_FILES";
    case FR_INVALID_PARAMETER:   return "INVALID_PARAMETER";
    default:                     return "UNKNOWN";
    }
}
#endif

#if HAL_USE_MMC_SPI
static const char *mmc_connect_error_name(mmc_connect_error_t error)
{
    switch (error) {
    case MMC_CONNECT_ERROR_NONE:      return "none";
    case MMC_CONNECT_ERROR_CMD0:      return "CMD0";
    case MMC_CONNECT_ERROR_CMD8:      return "CMD8";
    case MMC_CONNECT_ERROR_CMD8_ECHO: return "CMD8_R7";
    case MMC_CONNECT_ERROR_ACMD41:    return "ACMD41";
    case MMC_CONNECT_ERROR_CMD1:      return "CMD1";
    case MMC_CONNECT_ERROR_CMD58:     return "CMD58";
    case MMC_CONNECT_ERROR_CMD16:     return "CMD16";
    case MMC_CONNECT_ERROR_CSD:       return "CSD";
    case MMC_CONNECT_ERROR_CAPACITY: return "capacity";
    case MMC_CONNECT_ERROR_CID:       return "CID";
    default:                          return "unknown";
    }
}
#endif

/*
  initialise microSD card if avaialble. This is called during
  AP_BoardConfig initialisation. The parameter BRD_SD_SLOWDOWN
  controls a scaling factor on the microSD clock
 */
static bool sdcard_init_internal(bool mount_filesystem)
{
#ifdef USE_POSIX
#ifndef HAL_BOOTLOADER_BUILD
    WITH_SEMAPHORE(sem);

    uint8_t sd_slowdown = AP_BoardConfig::get_sdcard_slowdown();
#else
    uint8_t sd_slowdown = 0;  // maybe take from a define?
#endif
#if HAL_USE_SDC

#if STM32_SDC_USE_SDMMC2 == TRUE
    auto &sdcd = SDCD2;
#else
    auto &sdcd = SDCD1;
#endif

    if (sdcd.bouncebuffer == nullptr) {
        // allocate 4k bouncebuffer for microSD to match size in
        // AP_Logger
#if defined(STM32H7)
        bouncebuffer_init(&sdcd.bouncebuffer, 4096, true);
#else
        bouncebuffer_init(&sdcd.bouncebuffer, 4096, false);
#endif
    }

    if (block_device_running) {
        sdcard_stop();
    }
    filesystem_mounted = false;
    last_mount_result = FR_NOT_READY;

    const uint8_t tries = 3;
    for (uint8_t i=0; i<tries; i++) {
        sdcconfig.slowdown = sd_slowdown;
        sdcStart(&sdcd, &sdcconfig);
        if(sdcConnect(&sdcd) == HAL_FAILED) {
            sdcStop(&sdcd);
            continue;
        }
        block_device_running = true;
        if (!mount_filesystem) {
            printf("SDCard: block device initialized (filesystem not mounted)\n");
            return true;
        }
        const FRESULT mount_result = f_mount(&SDC_FS, "/", 1);
        last_mount_result = mount_result;
        if (mount_result != FR_OK) {
            printf("SDCard: mount failed (FRESULT=%u %s)\n",
                   (unsigned)mount_result, fatfs_result_name(mount_result));
            sdcDisconnect(&sdcd);
            sdcStop(&sdcd);
            block_device_running = false;
            continue;
        }
        printf("Successfully mounted SDCard (slowdown=%u)\n", (unsigned)sd_slowdown);

        filesystem_mounted = true;
        last_mount_result = FR_OK;
        return true;
    }
#elif HAL_USE_MMC_SPI
    if (MMCD1.buffer == nullptr) {
        // allocate 16 byte non-cacheable buffer for microSD
        MMCD1.buffer = (uint8_t*)malloc_axi_sram(MMC_BUFFER_SIZE);
        if (MMCD1.buffer == nullptr) {
            printf("SDCard: unable to allocate MMC DMA buffer\n");
            return false;
        }
    }

    if (block_device_running) {
        sdcard_stop();
    }

    block_device_running = true;
    filesystem_mounted = false;
    last_mount_result = FR_NOT_READY;
    last_connect_error = MMC_CONNECT_ERROR_NONE;
    last_connect_error_r1 = 0xFFU;

    device = AP_HAL::get_HAL().spi->get_device("sdcard");
    if (!device) {
        printf("No sdcard SPI device found\n");
        block_device_running = false;
        return false;
    }
    device->set_slowdown(sd_slowdown);

    mmcObjectInit(&MMCD1, MMCD1.buffer);

    mmcconfig.spip =
            static_cast<ChibiOS::SPIDevice*>(device.get())->get_driver();
    mmcconfig.hscfg = &highspeed;
    mmcconfig.lscfg = &lowspeed;

    /*
      try up to 3 times to init microSD interface
     */
    const uint8_t tries = 3;
    for (uint8_t i=0; i<tries; i++) {
        mmcStart(&MMCD1, &mmcconfig);

        if (mmcConnect(&MMCD1) == HAL_FAILED) {
            last_connect_error = MMCD1.connect_error;
            last_connect_error_r1 = MMCD1.connect_error_r1;
            last_mount_result = FR_NOT_READY;
            mmcStop(&MMCD1);
            continue;
        }
        last_connect_error = MMC_CONNECT_ERROR_NONE;
        last_connect_error_r1 = 0x00U;
        BlockDeviceInfo info;
        if (mmcGetInfo(&MMCD1, &info) == HAL_SUCCESS) {
            printf("SDCard: connected, blocks=%lu block_size=%lu\n",
                   (unsigned long)info.blk_num,
                   (unsigned long)info.blk_size);
        }
        if (!mount_filesystem) {
            printf("SDCard: block device initialized (filesystem not mounted)\n");
            return true;
        }
        last_mount_result = f_mount(&SDC_FS, "/", 1);
        if (last_mount_result != FR_OK) {
            mmcDisconnect(&MMCD1);
            mmcStop(&MMCD1);
            continue;
        }
        printf("Successfully mounted SDCard (slowdown=%u)\n", (unsigned)sd_slowdown);
        filesystem_mounted = true;
        last_mount_result = FR_OK;
        return true;
    }
    if (last_connect_error != MMC_CONNECT_ERROR_NONE) {
        printf("SDCard: MMC connect failed at %s (R1=0x%02x)\n",
               mmc_connect_error_name(last_connect_error),
               (unsigned)last_connect_error_r1);
    } else if (last_mount_result != FR_NOT_READY) {
        printf("SDCard: mount failed (FRESULT=%u %s)\n",
               (unsigned)last_mount_result,
               fatfs_result_name(last_mount_result));
    }
#endif
    block_device_running = false;
    filesystem_mounted = false;
#endif  // USE_POSIX
    return false;
}

bool sdcard_init()
{
    return sdcard_init_internal(true);
}

bool sdcard_prepare_for_format()
{
    // 仅初始化块设备，不挂载文件系统；用于对出厂未分区的SD NAND执行格式化
    return sdcard_init_internal(false);
}

/*
  stop sdcard interface (for reboot)
 */
void sdcard_stop(void)
{
#ifdef USE_POSIX
    // unmount
    f_mount(nullptr, "/", 1);
    filesystem_mounted = false;
#endif
#if HAL_USE_SDC
#if STM32_SDC_USE_SDMMC2 == TRUE
    auto &sdcd = SDCD2;
#else
    auto &sdcd = SDCD1;
#endif
    if (block_device_running) {
        sdcDisconnect(&sdcd);
        sdcStop(&sdcd);
        block_device_running = false;
    }
#elif HAL_USE_MMC_SPI
    if (block_device_running) {
        mmcDisconnect(&MMCD1);
        mmcStop(&MMCD1);
        block_device_running = false;
    }
#endif
}

bool sdcard_retry(void)
{
#ifdef USE_POSIX
    if (!filesystem_mounted) {
        if (sdcard_init()) {
#if AP_FILESYSTEM_FILE_WRITING_ENABLED
            // create APM directory
            AP::FS().mkdir("/APM");
#endif
        }
    }
#ifndef HAL_BOOTLOADER_BUILD
    static uint32_t last_failure_report_ms;
    if (!filesystem_mounted && hal.scheduler->is_system_initialized()) {
        const uint32_t now = AP_HAL::millis();
        if (last_failure_report_ms == 0 || now - last_failure_report_ms >= 30000U) {
            last_failure_report_ms = now;
#if HAL_USE_MMC_SPI
            if (last_connect_error != MMC_CONNECT_ERROR_NONE) {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "SDCard init: %s R1=0x%02x",
                              mmc_connect_error_name(last_connect_error),
                              (unsigned)last_connect_error_r1);
            } else
#endif
            {
                GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                              "SDCard mount: %s (%u)",
                              fatfs_result_name(last_mount_result),
                              (unsigned)last_mount_result);
            }
        }
    } else {
        last_failure_report_ms = 0;
    }
#endif
    return filesystem_mounted;
#endif
    return false;
}

#if HAL_USE_MMC_SPI

/*
  hooks to allow hal_mmc_spi.c to work with HAL_ChibiOS SPI
  layer. This provides bounce buffers for DMA, DMA channel sharing and
  bus locking
 */

void spiStartHook(SPIDriver *spip, const SPIConfig *config)
{
    device->set_speed(config == &lowspeed ?
        AP_HAL::Device::SPEED_LOW : AP_HAL::Device::SPEED_HIGH);
}

void spiStopHook(SPIDriver *spip)
{
}

__RAMFUNC__ void spiAcquireBusHook(SPIDriver *spip)
{
    if (block_device_running) {
        ChibiOS::SPIDevice *devptr = static_cast<ChibiOS::SPIDevice*>(device.get());
        devptr->acquire_bus(true, true);
    }
}

__RAMFUNC__ void spiReleaseBusHook(SPIDriver *spip)
{
    if (block_device_running) {
        ChibiOS::SPIDevice *devptr = static_cast<ChibiOS::SPIDevice*>(device.get());
        devptr->acquire_bus(false, true);
    }
}

__RAMFUNC__ void spiSelectHook(SPIDriver *spip)
{
    if (block_device_running) {
        device->get_semaphore()->take_blocking();
        device->set_chip_select(true);
    }
}

__RAMFUNC__ void spiUnselectHook(SPIDriver *spip)
{
    if (block_device_running) {
        device->set_chip_select(false);
        device->get_semaphore()->give();
    }
}

bool spiIgnoreHook(SPIDriver *spip, size_t n)
{
    if (!block_device_running) {
        return false;
    }
    return device->clock_pulse(n);
}

__RAMFUNC__ bool spiSendHook(SPIDriver *spip, size_t n, const void *txbuf)
{
    if (!block_device_running) {
        return false;
    }
    return device->transfer((const uint8_t *)txbuf, n, nullptr, 0);
}

__RAMFUNC__ bool spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf)
{
    if (!block_device_running) {
        memset(rxbuf, 0, n);
        return false;
    }
    if (!device->transfer(nullptr, 0, (uint8_t *)rxbuf, n)) {
        // Never leave stale 0xFF data that could be mistaken for card idle.
        memset(rxbuf, 0, n);
        return false;
    }
    return true;
}

#endif
