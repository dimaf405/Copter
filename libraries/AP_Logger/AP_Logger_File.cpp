/* 
   AP_Logger logging - file oriented variant

   This uses posix file IO to create log files called logs/NN.bin in the
   given directory

   SD Card Rates on PixHawk:
    - deletion rate seems to be ~50 files/second.
    - stat seems to be ~150/second
    - readdir loop of 511 entry directory ~62,000 microseconds
 */

#include "AP_Logger_config.h"

#if HAL_LOGGING_FILESYSTEM_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include "AP_Logger.h"
#include "AP_Logger_File.h"

#include <AP_Common/AP_Common.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_RTC/AP_RTC.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_AHRS/AP_AHRS.h>

#include <AP_Math/AP_Math.h>
#include <GCS_MAVLink/GCS.h>
#include <stdio.h>


extern const AP_HAL::HAL& hal;

#define LOGGER_PAGE_SIZE 1024UL

#define MB_to_B 1000000
#define B_to_MB 0.000001

// time between tries to open log
#define LOGGER_FILE_REOPEN_MS 5000

/*
  constructor
 */
AP_Logger_File::AP_Logger_File(AP_Logger &front,
                               LoggerMessageWriter_DFLogStart *writer) :
    AP_Logger_Backend(front, writer),
    _log_directory(HAL_BOARD_LOG_DIRECTORY)
{
    df_stats_clear();
}


void AP_Logger_File::ensure_log_directory_exists()
{
    int ret;
    struct stat st;

    EXPECT_DELAY_MS(3000);
    ret = AP::FS().stat(_log_directory, &st);
    if (ret == -1) {
        ret = AP::FS().mkdir(_log_directory);
    }
    if (ret == -1 && errno != EEXIST) {
        printf("Failed to create log directory %s : %s\n", _log_directory, strerror(errno));
    }
}

void AP_Logger_File::Init()
{
    // determine and limit file backend buffersize
    uint32_t bufsize = _front._params.file_bufsize;
    bufsize *= 1024;

    const uint32_t desired_bufsize = bufsize;

    // If we can't allocate the full size, try to reduce it until we can allocate it
    while (!_writebuf.set_size(bufsize) && bufsize >= _writebuf_chunk) {
        bufsize *= 0.9;
    }
    if (bufsize >= _writebuf_chunk && bufsize != desired_bufsize) {
        DEV_PRINTF("AP_Logger: reduced buffer %u/%u\n", (unsigned)bufsize, (unsigned)desired_bufsize);
    }

    if (!_writebuf.get_size()) {
        DEV_PRINTF("Out of memory for logging\n");
        return;
    }

    DEV_PRINTF("AP_Logger_File: buffer size=%u\n", (unsigned)bufsize);

    _initialised = true;

    const char* custom_dir = hal.util->get_custom_log_directory();
    if (custom_dir != nullptr){
        _log_directory = custom_dir;
    }

    uint16_t last_log_num = find_last_log();
    if (last_log_is_marked_discard) {
        // delete the last log leftover from LOG_DISARMED=3
        char *filename = _log_file_name(last_log_num);
        if (filename != nullptr) {
            AP::FS().unlink(filename);
            free(filename);
            invalidate_log_directory_state();
        }
    }

    Prep_MinSpace();
}

bool AP_Logger_File::file_exists(const char *filename) const
{
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(filename, &st) == -1) {
        // Only ENOENT proves the slot is free.  On an I/O error, treating it
        // as occupied avoids truncating a surviving log after a power fault.
        return errno != ENOENT;
    }
    return true;
}

bool AP_Logger_File::log_exists(const uint16_t lognum) const
{
    char *filename = _log_file_name(lognum);
    if (filename == nullptr) {
        return false; // ?!
    }
    bool ret = file_exists(filename);
    free(filename);
    return ret;
}

void AP_Logger_File::periodic_1Hz()
{
    AP_Logger_Backend::periodic_1Hz();

    if (_initialised &&
        !stop_logging_requested &&
        _write_fd == -1 && _read_fd == -1 &&
        erase.log_num == 0 &&
        erase.was_logging) {
        // restart logging after an erase if needed
        erase.was_logging = false;
        // setup to open the log in the backend thread
        start_new_log_pending = true;
    }
    
    if (_initialised &&
        !stop_logging_requested &&
        !start_new_log_pending &&
        _write_fd == -1 && _read_fd == -1 &&
        logging_enabled() &&
        !recent_open_error()) {
        // setup to open the log in the backend thread
        start_new_log_pending = true;
    }

    if (!io_thread_alive()) {
        if (io_thread_warning_decimation_counter == 0 && _initialised) {
            // we don't print this error unless we did initialise. When _initialised is set to true
            // we register the IO timer callback
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "AP_Logger: stuck thread (%s)", last_io_operation);
        }
        if (io_thread_warning_decimation_counter++ > 30) {
            io_thread_warning_decimation_counter = 0;
        }
    }

    if (rate_limiter == nullptr &&
        (_front._params.file_ratemax > 0 ||
         _front._params.disarm_ratemax > 0 ||
         _front._log_pause)) {
        // setup rate limiting if log rate max > 0Hz or log pause of streaming entries is requested
        rate_limiter = NEW_NOTHROW AP_Logger_RateLimiter(_front, _front._params.file_ratemax, _front._params.disarm_ratemax);
    }
}

void AP_Logger_File::periodic_fullrate()
{
    AP_Logger_Backend::push_log_blocks();
}

uint32_t AP_Logger_File::bufferspace_available()
{
    const uint32_t space = _writebuf.space();
    const uint32_t crit = critical_message_reserved_space(_writebuf.get_size());

    return (space > crit) ? space - crit : 0;
}

bool AP_Logger_File::recent_open_error(void) const
{
    if (_open_error_ms == 0) {
        return false;
    }
    return AP_HAL::millis() - _open_error_ms < LOGGER_FILE_REOPEN_MS;
}

// return true for CardInserted() if we successfully initialized
bool AP_Logger_File::CardInserted(void) const
{
    return _initialised && !recent_open_error();
}

// returns the amount of disk space available in _log_directory (in bytes)
// returns -1 on error
int64_t AP_Logger_File::disk_space_avail()
{
    return AP::FS().disk_free(_log_directory);
}

// returns the total amount of disk space (in use + available) in
// _log_directory (in bytes).
// returns -1 on error
int64_t AP_Logger_File::disk_space()
{
    return AP::FS().disk_space(_log_directory);
}

/*
  convert a dirent to a log number
 */
bool AP_Logger_File::dirent_to_log_num(const dirent *de, uint16_t &log_num) const
{
    const size_t length = strlen(de->d_name);
    if (length < 5) {
        return false;
    }
    if (strncmp(&de->d_name[length-4], ".BIN", 4) != 0) {
        // doesn't end in .BIN
        return false;
    }

    for (size_t i = 0; i < length - 4; i++) {
        if (de->d_name[i] < '0' || de->d_name[i] > '9') {
            return false;
        }
    }

    const unsigned long thisnum = strtoul(de->d_name, nullptr, 10);
    if (thisnum == 0 || thisnum > _front.get_max_num_logs()) {
        return false;
    }
    log_num = uint16_t(thisnum);
    return true;
}

bool AP_Logger_File::read_lastlog_marker(uint16_t &log_num, bool &marked_discard) const
{
    log_num = 0;
    marked_discard = false;

    char *fname = _lastlog_file_name();
    if (fname == nullptr) {
        return false;
    }
    EXPECT_DELAY_MS(3000);
    FileData *fd = AP::FS().load_file(fname);
    free(fname);
    if (fd == nullptr || fd->length == 0) {
        delete fd;
        return false;
    }

    char *endptr = nullptr;
    const unsigned long parsed = strtoul((const char *)fd->data, &endptr, 10);
    if (endptr == (const char *)fd->data ||
        parsed == 0 || parsed > _front.get_max_num_logs()) {
        delete fd;
        return false;
    }

    marked_discard = *endptr == 'D';
    if (marked_discard) {
        endptr++;
    }
    const bool valid = *endptr == '\0' || *endptr == '\r' || *endptr == '\n';
    if (valid) {
        log_num = uint16_t(parsed);
    }
    delete fd;
    return valid;
}

/*
  Scan the actual BIN files once and derive their chronological order.  The
  marker anchors wrapped numbering, but directory contents remain the source
  of truth after an abrupt power cut.
 */
bool AP_Logger_File::scan_log_directory(uint16_t marker_log_num,
                                        bool marker_valid,
                                        LogDirectoryState &state) const
{
    state.count = 0;
    state.newest = 0;
    state.oldest = 0;
    state.marker_found = false;
    state.present.clearall();

    const uint16_t max_logs = _front.get_max_num_logs();

    EXPECT_DELAY_MS(3000);
    auto *d = AP::FS().opendir(_log_directory);
    if (d == nullptr) {
        return false;
    }

    errno = 0;
    while (struct dirent *de = AP::FS().readdir(d)) {
        uint16_t log_num;
        if (!dirent_to_log_num(de, log_num)) {
            continue;
        }
        if (!state.present.get(log_num - 1U)) {
            state.present.set(log_num - 1U);
            state.count++;
        }
    }
    const int scan_errno = errno;
    const bool close_ok = AP::FS().closedir(d) == 0;
    if (scan_errno != 0 || !close_ok) {
        state.count = 0;
        state.present.clearall();
        return false;
    }

    if (state.count == 0) {
        return true;
    }

    if (marker_valid && state.present.get(marker_log_num - 1U)) {
        state.marker_found = true;
        state.newest = marker_log_num;

        // LASTLOG.TXT can lag behind one or more BIN directory entries.  Move
        // forward through the contiguous run that follows the marker.
        if (state.count < max_logs) {
            while (true) {
                uint16_t next = state.newest + 1U;
                if (next > max_logs) {
                    next = 1;
                }
                if (next == marker_log_num || !state.present.get(next - 1U)) {
                    break;
                }
                state.newest = next;
            }
        }
    } else {
        uint16_t recovered = 0;
        uint16_t run_end_count = 0;
        for (uint16_t log_num = 1; log_num <= max_logs; log_num++) {
            uint16_t next = log_num + 1U;
            if (next > max_logs) {
                next = 1;
            }
            if (state.present.get(log_num - 1U) && !state.present.get(next - 1U)) {
                recovered = log_num;
                run_end_count++;
            }
        }
        if (run_end_count == 1) {
            state.newest = recovered;
        } else {
            // Multiple gaps without a marker are ambiguous.  Prefer the
            // highest filename; subsequent starts will rebuild a clear anchor.
            for (uint16_t log_num = max_logs; log_num > 0; log_num--) {
                if (state.present.get(log_num - 1U)) {
                    state.newest = log_num;
                    break;
                }
            }
        }
    }

    uint16_t candidate = state.newest;
    for (uint16_t i = 0; i < max_logs; i++) {
        candidate = candidate == max_logs ? 1 : candidate + 1U;
        if (state.present.get(candidate - 1U)) {
            state.oldest = candidate;
            break;
        }
    }
    return state.oldest != 0;
}

bool AP_Logger_File::refresh_log_directory_state() const
{
    uint16_t marker_log_num;
    bool marker_discard;
    const bool marker_valid = read_lastlog_marker(marker_log_num, marker_discard);
    if (!scan_log_directory(marker_log_num, marker_valid, log_directory_state)) {
        log_directory_state_valid = false;
        return false;
    }
    log_directory_state_valid = true;
    return true;
}


// find_oldest_log - find oldest log in _log_directory
// returns 0 if no log was found
uint16_t AP_Logger_File::find_oldest_log()
{
    uint16_t marker_log_num;
    bool marker_discard;
    const bool marker_valid = read_lastlog_marker(marker_log_num, marker_discard);
    LogDirectoryState state;
    if (!scan_log_directory(marker_log_num, marker_valid, state)) {
        return 0;
    }
    return state.oldest;
}

void AP_Logger_File::Prep_MinSpace()
{
    if (hal.util->was_watchdog_reset()) {
        // don't clear space if watchdog reset, it takes too long
        return;
    }

    if (!CardInserted()) {
        return;
    }

    uint16_t marker_log_num;
    bool marker_discard;
    const bool marker_valid = read_lastlog_marker(marker_log_num, marker_discard);
    LogDirectoryState state;
    if (!scan_log_directory(marker_log_num, marker_valid, state) || state.count == 0) {
        // Never delete files based on a partial or failed directory scan.
        return;
    }

    const uint16_t last_log_to_keep = state.newest;
    const uint16_t first_log_to_remove = state.oldest;
    if (first_log_to_remove == 0) {
        // no files to remove
        return;
    }

    int64_t target_free = (int64_t)_front._params.min_MB_free * MB_to_B;
    const int64_t total_space = disk_space();
    if (total_space > int64_t(_free_space_min_avail) &&
        target_free > total_space - _free_space_min_avail) {
        // A saved parameter from a larger card can be impossible to satisfy
        // after changing to a smaller device.  Limit it at runtime so startup
        // does not erase every log while chasing unattainable free space.
        const int64_t fallback_target = MAX(int64_t(10) * MB_to_B,
                                            total_space / 10);
        target_free = MIN(target_free, fallback_target);
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING,
                      "LOG_FILE_MB_FREE too high, using %uMB",
                      unsigned(target_free / MB_to_B));
    }

    uint16_t log_to_remove = first_log_to_remove;

    uint16_t count = 0;
    do {
        const int64_t avail = disk_space_avail();
        if (avail < 0) {
            break;
        }
        if (avail >= target_free) {
            break;
        }
        if (log_to_remove == last_log_to_keep) {
            // Space settings must never remove the newest surviving log.
            break;
        }
        if (count++ > _front.get_max_num_logs() + 10) {
            // *way* too many deletions going on here.  Possible internal error.
            INTERNAL_ERROR(AP_InternalError::error_t::logger_too_many_deletions);
            break;
        }
        char *filename_to_remove = _log_file_name(log_to_remove);
        if (filename_to_remove == nullptr) {
            INTERNAL_ERROR(AP_InternalError::error_t::logger_bad_getfilename);
            break;
        }
        if (file_exists(filename_to_remove)) {
            DEV_PRINTF("Removing (%s) for minimum-space requirements (%.0fMB < %.0fMB)\n",
                                filename_to_remove, (double)avail*B_to_MB, (double)target_free*B_to_MB);
            EXPECT_DELAY_MS(2000);
            if (AP::FS().unlink(filename_to_remove) == -1) {
                _cached_oldest_log = 0;
                DEV_PRINTF("Failed to remove %s: %s\n", filename_to_remove, strerror(errno));
                free(filename_to_remove);
                if (errno == ENOENT) {
                    // corruption - should always have a continuous
                    // sequence of files...  however, there may be still
                    // files out there, so keep going.
                } else {
                    break;
                }
            } else {
                free(filename_to_remove);
                state.present.clear(log_to_remove - 1U);
            }
        }
        bool found_next = false;
        for (uint16_t i = 0; i < _front.get_max_num_logs(); i++) {
            log_to_remove++;
            if (log_to_remove > _front.get_max_num_logs()) {
                log_to_remove = 1;
            }
            if (state.present.get(log_to_remove - 1U)) {
                found_next = true;
                break;
            }
        }
        if (!found_next) {
            break;
        }
    } while (log_to_remove != first_log_to_remove);
}

/*
  construct a log file name given a log number.
  The number in the log filename will be zero-padded.
  Note: Caller must free.
 */
char *AP_Logger_File::_log_file_name(const uint16_t log_num) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/%08u.BIN", _log_directory, (unsigned)log_num) == -1) {
        return nullptr;
    }
    return buf;
}

/*
  return path name of the lastlog.txt marker file
  Note: Caller must free.
 */
char *AP_Logger_File::_lastlog_file_name(void) const
{
    char *buf = nullptr;
    if (asprintf(&buf, "%s/LASTLOG.TXT", _log_directory) == -1) {
        return nullptr;
    }
    return buf;
}


// remove all log files
void AP_Logger_File::EraseAll()
{
    if (hal.util->get_soft_armed()) {
        // do not want to do any filesystem operations while we are e.g. flying
        return;
    }
    if (!_initialised) {
        return;
    }

    erase.was_logging = (_write_fd != -1);
    stop_logging();

    erase.log_num = 1;
}

bool AP_Logger_File::WritesOK() const
{
    if (_write_fd == -1) {
        return false;
    }
    if (recent_open_error()) {
        return false;
    }
    return true;
}


bool AP_Logger_File::StartNewLogOK() const
{
    if (recent_open_error()) {
        return false;
    }
#if !APM_BUILD_TYPE(APM_BUILD_Replay) && !APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
    if (hal.scheduler->in_main_thread()) {
        return false;
    }
#endif
    return AP_Logger_Backend::StartNewLogOK();
}

/* Write a block of data at current offset */
bool AP_Logger_File::_WritePrioritisedBlock(const void *pBuffer, uint16_t size, bool is_critical)
{
    WITH_SEMAPHORE(semaphore);

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    if (AP::FS().write(_write_fd, pBuffer, size) != size) {
        AP_HAL::panic("Short write");
    }
    return true;
#endif


    uint32_t space = _writebuf.space();

    if (_writing_startup_messages &&
        _startup_messagewriter->fmt_done()) {
        // the state machine has called us, and it has finished
        // writing format messages out.  It can always get back to us
        // with more messages later, so let's leave room for other
        // things:
        const uint32_t now = AP_HAL::millis();
        const bool must_dribble = (now - last_messagewrite_message_sent) > 100;
        if (!must_dribble &&
            space < non_messagewriter_message_reserved_space(_writebuf.get_size())) {
            // this message isn't dropped, it will be sent again...
            return false;
        }
        last_messagewrite_message_sent = now;
    } else {
        // we reserve some amount of space for critical messages:
        if (!is_critical && space < critical_message_reserved_space(_writebuf.get_size())) {
            _dropped++;
            return false;
        }
    }

    // if no room for entire message - drop it:
    if (space < size) {
        _dropped++;
        return false;
    }

    _writebuf.write((uint8_t*)pBuffer, size);
    df_stats_gather(size, _writebuf.space());
    return true;
}

/*
  find the highest log number
 */
uint16_t AP_Logger_File::find_last_log()
{
    uint16_t marker_log_num;
    bool marker_discard;
    const bool marker_valid = read_lastlog_marker(marker_log_num, marker_discard);
    LogDirectoryState state;
    if (!scan_log_directory(marker_log_num, marker_valid, state)) {
        last_log_is_marked_discard = false;
        return 0;
    }
    last_log_is_marked_discard = marker_valid && state.marker_found &&
                                 state.newest == marker_log_num && marker_discard;
    return state.newest;
}

uint16_t AP_Logger_File::log_num_from_list_entry(const uint16_t list_entry) const
{
    if (list_entry == 0) {
        return 0;
    }

    if ((!log_directory_state_valid && !refresh_log_directory_state()) ||
        list_entry > log_directory_state.count) {
        return 0;
    }

    const uint16_t max_logs = _front.get_max_num_logs();
    uint16_t candidate = log_directory_state.oldest;
    uint16_t entry = 0;
    for (uint16_t i = 0; i < max_logs; i++) {
        if (log_directory_state.present.get(candidate - 1U) && ++entry == list_entry) {
            return candidate;
        }
        candidate = candidate == max_logs ? 1 : candidate + 1U;
    }
    return 0;
}

uint32_t AP_Logger_File::_get_log_size(const uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }
    if (_write_fd != -1 && write_fd_semaphore.take_nonblocking()) {
        if (_write_filename != nullptr && strcmp(_write_filename, fname) == 0) {
            // it is the file we are currently writing
            free(fname);
            write_fd_semaphore.give();
            return _write_offset;
        }
        write_fd_semaphore.give();
    }
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_size;
}

uint32_t AP_Logger_File::_get_log_time(const uint16_t log_num)
{
    char *fname = _log_file_name(log_num);
    if (fname == nullptr) {
        return 0;
    }
    if (_write_fd != -1 && write_fd_semaphore.take_nonblocking()) {
        if (_write_filename != nullptr && strcmp(_write_filename, fname) == 0) {
            // it is the file we are currently writing
            free(fname);
            write_fd_semaphore.give();
#if AP_RTC_ENABLED
            uint64_t utc_usec;
            if (!AP::rtc().get_utc_usec(utc_usec)) {
                return 0;
            }
            return utc_usec / 1000000U;
#else
            return 0;
#endif
        }
        write_fd_semaphore.give();
    }
    struct stat st;
    EXPECT_DELAY_MS(3000);
    if (AP::FS().stat(fname, &st) != 0) {
        free(fname);
        return 0;
    }
    free(fname);
    return st.st_mtime;
}

/*
  find the number of pages in a log
 */
void AP_Logger_File::get_log_boundaries(const uint16_t list_entry, uint32_t & start_page, uint32_t & end_page)
{
    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        start_page = 0;
        end_page = 0;
        return;
    }

    start_page = 0;
    end_page = _get_log_size(log_num) / LOGGER_PAGE_SIZE;
}

/*
  retrieve data from a log file
 */
int16_t AP_Logger_File::get_log_data(const uint16_t list_entry, const uint16_t page, const uint32_t offset, const uint16_t len, uint8_t *data)
{
    if (!_initialised || recent_open_error()) {
        return -1;
    }

    const uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        return -1;
    }

    if (_read_fd != -1 && log_num != _read_fd_log_num) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }
    if (_read_fd == -1) {
        char *fname = _log_file_name(log_num);
        if (fname == nullptr) {
            return -1;
        }
        stop_logging_async();
        const uint32_t stop_start_ms = AP_HAL::millis();
        while (stop_logging_requested &&
               AP_HAL::millis() - stop_start_ms < 3000U) {
            hal.scheduler->delay_microseconds(1000);
        }
        if (stop_logging_requested || !stop_logging_success || logging_started()) {
            free(fname);
            return -1;
        }
        EXPECT_DELAY_MS(3000);
        _read_fd = AP::FS().open(fname, O_RDONLY);
        if (_read_fd == -1) {
            _open_error_ms = AP_HAL::millis();
            int saved_errno = errno;
            ::printf("Log read open fail for %s - %s\n",
                     fname, strerror(saved_errno));
            DEV_PRINTF("Log read open fail for %s - %s\n",
                                fname, strerror(saved_errno));
            free(fname);
            return -1;            
        }
        free(fname);
        _read_offset = 0;
        _read_fd_log_num = log_num;
    }
    uint32_t ofs = page * (uint32_t)LOGGER_PAGE_SIZE + offset;

    if (ofs != _read_offset) {
        if (AP::FS().lseek(_read_fd, ofs, SEEK_SET) == (off_t)-1) {
            AP::FS().close(_read_fd);
            _read_fd = -1;
            return -1;
        }
        _read_offset = ofs;
    }
    int16_t ret = (int16_t)AP::FS().read(_read_fd, data, len);
    if (ret > 0) {
        _read_offset += ret;
    }
    return ret;
}

void AP_Logger_File::end_log_transfer()
{
    if (_read_fd != -1) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }
}

/*
  find size and date of a log
 */
void AP_Logger_File::get_log_info(const uint16_t list_entry, uint32_t &size, uint32_t &time_utc)
{
    uint16_t log_num = log_num_from_list_entry(list_entry);
    if (log_num == 0) {
        // that failed - probably no logs
        size = 0;
        time_utc = 0;
        return;
    }

    size = _get_log_size(log_num);
    time_utc = _get_log_time(log_num);
}


// get the number of actual BIN files; numbering may contain power-loss gaps
uint16_t AP_Logger_File::get_num_logs()
{
    if (!refresh_log_directory_state()) {
        return 0;
    }
    return log_directory_state.count;
}

/*
  stop logging
 */
void AP_Logger_File::stop_logging(void)
{
    if (!_initialised) {
        start_new_log_pending = false;
        stop_logging_requested = false;
        stop_logging_success = _writebuf.available() == 0;
        return;
    }
    if (!write_fd_semaphore.take(hal.util->get_soft_armed() ? 1 : 20)) {
        start_new_log_pending = false;
        stop_logging_requested = true;
        return;
    }
    stop_logging_requested = false;
    stop_logging_success = true;
    start_new_log_pending = false;
    if (_write_fd != -1) {
        const int fd = _write_fd;
        const bool synced = AP::FS().fsync(fd) == 0;
        const bool closed = AP::FS().close(fd) == 0;
        _write_fd = -1;
        if (!synced || !closed) {
            _last_write_failed = true;
            stop_logging_success = false;
        }
    }
    write_fd_semaphore.give();
}

/*
  异步请求停止日志记录，由IO线程在缓冲区清空后负责关闭文件；
  相比同步stop_logging()，不会阻塞主线程等待文件关闭完成
 */
void AP_Logger_File::stop_logging_async(void)
{
    start_new_log_pending = false;
    stop_logging_success = true;
    stop_logging_requested = true;
}

bool AP_Logger_File::close_write_file()
{
    if (!write_fd_semaphore.take_nonblocking()) {
        return false;
    }

    bool success = true;
    if (_write_fd != -1) {
        const int fd = _write_fd;
        const bool synced = AP::FS().fsync(fd) == 0;
        const bool closed = AP::FS().close(fd) == 0;
        if (!synced || !closed) {
            _last_write_failed = true;
            success = false;
        }
        _write_fd = -1;
    }
    write_fd_semaphore.give();
    stop_logging_success = success;
    stop_logging_requested = false;
    return success;
}

/*
  does start_new_log in the logger thread
 */
void AP_Logger_File::PrepForArming_start_logging()
{
    if (logging_started()) {
        return;
    }

    uint32_t start_ms = AP_HAL::millis();
    const uint32_t open_limit_ms = 1000;

    /*
      log open happens in the io_timer thread. We allow for a maximum
      of 1s to complete the open
     */
    start_new_log_pending = true;
    EXPECT_DELAY_MS(1000);
    while (AP_HAL::millis() - start_ms < open_limit_ms) {
        if (logging_started()) {
            break;
        }
#if !APM_BUILD_TYPE(APM_BUILD_Replay) && AP_AHRS_ENABLED
        // keep the EKF ticking over
        AP::ahrs().update();
#endif
        hal.scheduler->delay(1);
    }
}

/*
  start writing to a new log file
 */
void AP_Logger_File::start_new_log(void)
{
    if (recent_open_error()) {
        // we have previously failed to open a file - don't try again
        // to prevent us trying to open files while in flight
        return;
    }

    if (erase.log_num != 0) {
        // don't start a new log while erasing, but record that we
        // want to start logging when erase finished
        erase.was_logging = true;
        return;
    }

    const bool open_error_ms_was_zero = (_open_error_ms == 0);

    // set _open_error here to avoid infinite recursion.  Simply
    // writing a prioritised block may try to open a log - which means
    // if anything in the start_new_log path does a GCS_SEND_TEXT()
    // (for example), you will end up recursing if we don't take
    // precautions.  We will reset _open_error if we actually manage
    // to open the log...
    _open_error_ms = AP_HAL::millis();

    stop_logging();

    start_new_log_reset_variables();

    if (_read_fd != -1) {
        AP::FS().close(_read_fd);
        _read_fd = -1;
    }

    const int64_t space_avail = disk_space_avail();
    if (space_avail >= 0 && space_avail < _free_space_min_avail && disk_space() > 0) {
        DEV_PRINTF("Out of space for logging\n");
        return;
    }

    uint16_t log_num = find_last_log();
    // re-use empty logs if possible
    if (_get_log_size(log_num) > 0 || log_num == 0) {
        log_num++;
    }
    if (log_num > _front.get_max_num_logs()) {
        log_num = 1;
    }

    // A power cut can leave the BIN directory entry newer than LASTLOG.TXT.
    // Do not truncate such a recovered log unless every log slot is occupied.
    const uint16_t first_candidate = log_num;
    bool free_slot_found;
    while (!(free_slot_found = !log_exists(log_num))) {
        log_num++;
        if (log_num > _front.get_max_num_logs()) {
            log_num = 1;
        }
        if (log_num == first_candidate) {
            break;
        }
    }
    if (!free_slot_found) {
        // All slots are occupied, so overwrite the oldest actual log.
        const uint16_t oldest_log = find_oldest_log();
        if (oldest_log == 0) {
            return;
        }
        log_num = oldest_log;
    }
    if (!write_fd_semaphore.take(1)) {
        return;
    }
    if (_write_filename) {
        free(_write_filename);
        _write_filename = nullptr;        
    }
    _write_filename = _log_file_name(log_num);
    if (_write_filename == nullptr) {
        write_fd_semaphore.give();
        return;
    }

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // remember if we had utc time when we opened the file
#if AP_RTC_ENABLED
    uint64_t utc_usec;
    _need_rtc_update = !AP::rtc().get_utc_usec(utc_usec);
#endif
#endif

    // create the log directory if need be
    ensure_log_directory_exists();

    EXPECT_DELAY_MS(3000);
    _write_fd = AP::FS().open(_write_filename, O_WRONLY|O_CREAT|O_TRUNC);
    _cached_oldest_log = 0;
    invalidate_log_directory_state();

    if (_write_fd == -1) {
        write_fd_semaphore.give();
        int saved_errno = errno;
        if (open_error_ms_was_zero) {
            ::printf("Log open fail for %s - %s\n",
                     _write_filename, strerror(saved_errno));
            DEV_PRINTF("Log open fail for %s - %s\n",
                                _write_filename, strerror(saved_errno));
        }
        return;
    }

    // Persist the new directory entry before LASTLOG.TXT references it.  If
    // power fails between these operations, startup recovery can still find
    // the BIN file without ever pointing at a missing one.
    if (AP::FS().fsync(_write_fd) != 0) {
        AP::FS().close(_write_fd);
        _write_fd = -1;
        _last_write_failed = true;
        write_fd_semaphore.give();
        return;
    }
    _last_write_ms = AP_HAL::millis();
    _open_error_ms = 0;
    _write_offset = 0;
    _writebuf.clear();
    write_fd_semaphore.give();

    // now update lastlog.txt with the new log number
    last_log_is_marked_discard = _front._params.log_disarmed == AP_Logger::LogDisarmed::LOG_WHILE_DISARMED_DISCARD;
    if (!write_lastlog_file(log_num)) {
        _open_error_ms = AP_HAL::millis();
    }
}

/*
  write LASTLOG.TXT, possibly with a discard marker
 */
bool AP_Logger_File::write_lastlog_file(uint16_t log_num)
{
    // now update lastlog.txt with the new log number
    char *fname = _lastlog_file_name();

    EXPECT_DELAY_MS(3000);
    int fd = AP::FS().open(fname, O_WRONLY|O_CREAT|O_TRUNC);
    free(fname);
    if (fd == -1) {
        return false;
    }

    char buf[30];
    snprintf(buf, sizeof(buf), "%u%s\r\n", (unsigned)log_num, last_log_is_marked_discard?"D":"");
    const ssize_t to_write = strlen(buf);
    const ssize_t written = AP::FS().write(fd, buf, to_write);
    const bool synced = written == to_write && AP::FS().fsync(fd) == 0;
    const bool closed = AP::FS().close(fd) == 0;
    return synced && closed;
}

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX
void AP_Logger_File::flush(void)
#if APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
{
    uint32_t tnow = AP_HAL::millis();
    while (_write_fd != -1 && _initialised && !recent_open_error() && _writebuf.available()) {
        // convince the IO timer that it really is OK to write out
        // less than _writebuf_chunk bytes:
        if (tnow > 2001) { // avoid resetting _last_write_time to 0
            _last_write_time = tnow - 2001;
        }
        io_timer();
    }
    if (write_fd_semaphore.take(1)) {
        if (_write_fd != -1) {
            ::fsync(_write_fd);
        }
        write_fd_semaphore.give();
    } else {
        INTERNAL_ERROR(AP_InternalError::error_t::logger_flushing_without_sem);
    }
}
#else
{
    // flush is for replay and examples only
}
#endif // APM_BUILD_TYPE(APM_BUILD_Replay) || APM_BUILD_TYPE(APM_BUILD_UNKNOWN)
#endif

void AP_Logger_File::io_timer(void)
{
    uint32_t tnow = AP_HAL::millis();
    _io_timer_heartbeat = tnow;

    if (start_new_log_pending && !stop_logging_requested) {
        start_new_log();
        start_new_log_pending = false;
    }

    if (erase.log_num != 0) {
        // continue erase
        erase_next();
        return;
    }

    if (_write_fd == -1 || !_initialised) {
        if (stop_logging_requested) {
            stop_logging_success = _write_fd == -1 && _writebuf.available() == 0;
            stop_logging_requested = false;
        }
        return;
    }

    if (recent_open_error() && !stop_logging_requested) {
        return;
    }

    if (last_log_is_marked_discard && hal.util->get_soft_armed()) {
        // time to make the log permanent
        const auto log_num = find_last_log();
        last_log_is_marked_discard = false;
        write_lastlog_file(log_num);
    }

    uint32_t nbytes = _writebuf.available();
    if (nbytes == 0) {
        if (stop_logging_requested) {
            close_write_file();
        }
        return;
    }
    if (nbytes < _writebuf_chunk && 
        tnow - _last_write_time < 2000UL &&
        !stop_logging_requested) {
        // write in _writebuf_chunk-sized chunks, but always write at
        // least once per 2 seconds if data is available
        return;
    }
    if (tnow - _free_space_last_check_time > _free_space_check_interval) {
        _free_space_last_check_time = tnow;
        last_io_operation = "disk_space_avail";
        const int64_t space_avail = disk_space_avail();
        if (space_avail >= 0 && space_avail < _free_space_min_avail && disk_space() > 0) {
            DEV_PRINTF("Out of space for logging\n");
            stop_logging();
            _open_error_ms = AP_HAL::millis(); // prevent logging starting again for 5s
            last_io_operation = "";
            return;
        }
        last_io_operation = "";
    }

    _last_write_time = tnow;
    if (nbytes > _writebuf_chunk) {
        // be kind to the filesystem layer
        nbytes = _writebuf_chunk;
    }

    uint32_t size;
    const uint8_t *head = _writebuf.readptr(size);
    nbytes = MIN(nbytes, size);

    // try to align writes on a 512 byte boundary to avoid filesystem reads
    if ((nbytes + _write_offset) % 512 != 0) {
        uint32_t ofs = (nbytes + _write_offset) % 512;
        if (ofs < nbytes) {
            nbytes -= ofs;
        }
    }

    last_io_operation = "write";
    if (!write_fd_semaphore.take(1)) {
        return;
    }
    if (_write_fd == -1) {
        write_fd_semaphore.give();
        return;
    }
    ssize_t nwritten = AP::FS().write(_write_fd, head, nbytes);
    last_io_operation = "";
    if (nwritten <= 0) {
        if ((tnow - _last_write_ms)/1000U > unsigned(_front._params.file_timeout)) {
            // if we can't write for LOG_FILE_TIMEOUT seconds we give up and close
            // the file. This allows us to cope with temporary write
            // failures caused by directory listing
            last_io_operation = "close";
            AP::FS().close(_write_fd);
            last_io_operation = "";
            _write_fd = -1;
            printf("Failed to write to File: %s\n", strerror(errno));
            if (stop_logging_requested) {
                stop_logging_success = false;
                stop_logging_requested = false;
            }
        }
        _last_write_failed = true;
    } else {
        _last_write_ms = tnow;
        _write_offset += nwritten;
        _writebuf.advance(nwritten);
        /*
          the best strategy for minimizing corruption on microSD cards
          seems to be to write in 4k chunks and fsync the file on each
          chunk, ensuring the directory entry is updated after each
          write.
         */
#if CONFIG_HAL_BOARD != HAL_BOARD_SITL && CONFIG_HAL_BOARD_SUBTYPE != HAL_BOARD_SUBTYPE_LINUX_NONE
        last_io_operation = "fsync";
        if (AP::FS().fsync(_write_fd) != 0) {
            _last_write_failed = true;
            last_io_operation = "";
            write_fd_semaphore.give();
            return;
        }
        last_io_operation = "";
#endif
        _last_write_failed = false;

#if AP_RTC_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
        // ChibiOS does not update mtime on writes, so if we opened
        // without knowing the time we should update it later
        if (_need_rtc_update) {
            uint64_t utc_usec;
            if (AP::rtc().get_utc_usec(utc_usec)) {
                AP::FS().set_mtime(_write_filename, utc_usec/(1000U*1000U));
                _need_rtc_update = false;
            }
        }
#endif
    }

    write_fd_semaphore.give();
}

bool AP_Logger_File::io_thread_alive() const
{
    if (!hal.scheduler->is_system_initialized()) {
        // the system has long pauses during initialisation, assume still OK
        return true;
    }
    // if the io thread hasn't had a heartbeat in a while then it is
#if CONFIG_HAL_BOARD == HAL_BOARD_ESP32
    uint32_t timeout_ms = 10000;
#else
    uint32_t timeout_ms = 5000;
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL && !defined(HAL_BUILD_AP_PERIPH)
    // the IO thread is working with hardware - writing to a physical
    // disk.  Unfortunately these hardware devices do not obey our
    // SITL speedup options, so we allow for it here.
    SITL::SIM *sitl = AP::sitl();
    if (sitl != nullptr) {
        timeout_ms *= sitl->speedup;
    }
#endif
    return (AP_HAL::millis() - _io_timer_heartbeat) < timeout_ms;
}

bool AP_Logger_File::logging_failed() const
{
    if (!_initialised) {
        return true;
    }
    if (recent_open_error()) {
        return true;
    }
    if (!io_thread_alive()) {
        // No heartbeat in a second.  IO thread is dead?! Very Not
        // Good.
        return true;
    }
    if (_last_write_failed) {
        return true;
    }

    return false;
}

/*
  erase another file in async erase operation
 */
void AP_Logger_File::erase_next(void)
{
    char *fname = _log_file_name(erase.log_num);
    if (fname == nullptr) {
        erase.log_num = 0;
        return;
    }

    AP::FS().unlink(fname);
    free(fname);

    erase.log_num++;
    if (erase.log_num <= _front.get_max_num_logs()) {
        return;
    }
    
    fname = _lastlog_file_name();
    if (fname != nullptr) {
        AP::FS().unlink(fname);
        free(fname);
    }

    _cached_oldest_log = 0;
    invalidate_log_directory_state();

    erase.log_num = 0;
}

#endif // HAL_LOGGING_FILESYSTEM_ENABLED
