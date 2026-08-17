/* 
   AP_Logger logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */
#pragma once

#include <AP_Filesystem/AP_Filesystem.h>

#include <AP_Common/Bitmask.h>
#include <AP_HAL/utility/RingBuffer.h>
#include "AP_Logger_Backend.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#ifndef HAL_LOGGER_WRITE_CHUNK_SIZE
#define HAL_LOGGER_WRITE_CHUNK_SIZE 4096
#endif

// LOG_MAX_FILES is constrained to 500 by the AP_Logger frontend.
static constexpr uint16_t LOGGER_FILE_MAX_LOGS = 500;

class AP_Logger_File : public AP_Logger_Backend
{
public:
    // constructor
    AP_Logger_File(AP_Logger &front,
                   LoggerMessageWriter_DFLogStart *);

    static AP_Logger_Backend  *probe(AP_Logger &front,
                                     LoggerMessageWriter_DFLogStart *ls) {
        return NEW_NOTHROW AP_Logger_File(front, ls);
    }

    // initialisation
    void Init() override;
    bool CardInserted(void) const override;

    // erase handling
    void EraseAll() override;

    /* Write a block of data at current offset */
    bool _WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical) override;
    uint32_t bufferspace_available() override;

    // high level interface
    uint16_t find_last_log() override;
    void get_log_boundaries(uint16_t log_num, uint32_t & start_page, uint32_t & end_page) override;
    void get_log_info(uint16_t log_num, uint32_t &size, uint32_t &time_utc) override;
    int16_t get_log_data(uint16_t log_num, uint16_t page, uint32_t offset, uint16_t len, uint8_t *data) override;
    void end_log_transfer() override;
    uint16_t get_num_logs() override;
    void start_new_log(void) override;
    uint16_t find_oldest_log() override;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
    void flush(void) override;
#endif
    void periodic_1Hz() override;
    void periodic_fullrate() override;

    // this method is used when reporting system status over mavlink
    bool logging_failed() const override;

    bool logging_started(void) const override { return _write_fd != -1; }
    void io_timer(void) override;

protected:

    bool WritesOK() const override;
    bool StartNewLogOK() const override;
    void PrepForArming_start_logging() override;

private:
    int _write_fd = -1;
    char *_write_filename;
    bool last_log_is_marked_discard;
    uint32_t _last_write_ms;
#if AP_RTC_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    bool _need_rtc_update;
#endif
    
    int _read_fd = -1;
    uint16_t _read_fd_log_num;
    uint32_t _read_offset;
    uint32_t _write_offset;
    volatile uint32_t _open_error_ms;
    const char *_log_directory;
    bool _last_write_failed;

    uint32_t _io_timer_heartbeat;
    bool io_thread_alive() const;
    uint8_t io_thread_warning_decimation_counter;

    // do we have a recent open error?
    bool recent_open_error(void) const;

    // possibly time-consuming preparations handling
    void Prep_MinSpace();
    int64_t disk_space_avail();
    int64_t disk_space();

    void ensure_log_directory_exists();

    bool file_exists(const char *filename) const;
    bool log_exists(const uint16_t lognum) const;

    bool dirent_to_log_num(const dirent *de, uint16_t &log_num) const;
    // 目录扫描结果缓存：用位掩码记录实际存在的日志文件编号，
    // 支持环绕编号和掉电恢复场景下的最新/最旧日志推断
    struct LogDirectoryState {
        Bitmask<LOGGER_FILE_MAX_LOGS> present;  // 各编号对应BIN文件是否存在
        uint16_t count;         // 当前实际存在的日志文件总数
        uint16_t newest;        // 最新日志编号（由LASTLOG.TXT标记锚定）
        uint16_t oldest;        // 最旧日志编号（newest之后的第一个有效编号）
        bool marker_found;      // LASTLOG.TXT中的编号是否与目录匹配
    };
    bool read_lastlog_marker(uint16_t &log_num, bool &marked_discard) const;
    bool scan_log_directory(uint16_t marker_log_num,
                            bool marker_valid,
                            LogDirectoryState &state) const;
    bool refresh_log_directory_state() const;
    void invalidate_log_directory_state() { log_directory_state_valid = false; }
    uint16_t log_num_from_list_entry(const uint16_t list_entry) const;
    bool write_lastlog_file(uint16_t log_num);

    mutable LogDirectoryState log_directory_state;
    mutable bool log_directory_state_valid = false;

    // write buffer
    ByteBuffer _writebuf{0};
    const uint16_t _writebuf_chunk = HAL_LOGGER_WRITE_CHUNK_SIZE;
    uint32_t _last_write_time;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(const uint16_t log_num) const;
    char *_lastlog_file_name() const;
    uint32_t _get_log_size(const uint16_t log_num);
    uint32_t _get_log_time(const uint16_t log_num);

    void stop_logging(void) override;
    void stop_logging_async(void) override;
    bool stop_logging_pending(void) const override { return stop_logging_requested; }
    bool stop_logging_succeeded(void) const override { return stop_logging_success; }
    bool close_write_file();

    uint32_t last_messagewrite_message_sent;
    volatile bool stop_logging_requested = false;
    volatile bool stop_logging_success = true;

    // free-space checks; filling up SD cards under NuttX leads to
    // corrupt filesystems which cause loss of data, failure to gather
    // data and failures-to-boot.
    uint32_t _free_space_last_check_time; // milliseconds
    const uint32_t _free_space_check_interval = 1000UL; // milliseconds
    const uint32_t _free_space_min_avail = 8388608; // bytes

    // semaphore mediates access to the ringbuffer
    HAL_Semaphore semaphore;
    // write_fd_semaphore mediates access to write_fd so the frontend
    // can open/close files without causing the backend to write to a
    // bad fd
    HAL_Semaphore write_fd_semaphore;

    // async erase state
    struct {
        bool was_logging;
        uint16_t log_num;
    } erase;
    void erase_next(void);

    const char *last_io_operation = "";

    bool start_new_log_pending;
};

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
