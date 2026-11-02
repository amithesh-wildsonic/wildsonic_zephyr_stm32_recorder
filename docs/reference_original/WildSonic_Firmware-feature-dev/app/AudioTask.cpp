#if __CORE__ == AUDIO_CORE

#include "AudioTask.hpp"
#include "assert.hpp"
#include "device_config.hpp"
#include "shared.hpp"
#include "solar_calculator.hpp"
#include "trace.hpp"

#include "w_hal_i2s.hpp"
#include "w_hal_rtc.hpp"

#include <algorithm>

/**
 * @brief
 * Set up and initialization for Audio Task
 *
 * @param queues_info queue information
 */
void AudioTask::initialize(queues_info_t &queues_info)
{
    Task::initialize();

    // Populate buffer pool and set up buffer management
    m_buffer_pool.initialize();
    for (buffer_id_t buffer_id = 0; buffer_id < NUM_BUFFERS; buffer_id++)
    {
        m_buffer_active_count[buffer_id] = 0;
    }
    m_num_active_buffers        = 0;

    // Save pointer to message arrays
    m_p_detect_msgs             = queues_info.detect_msgs;
    m_p_storage_msgs            = queues_info.storage_msgs;
    m_p_comm_msgs               = queues_info.comm_msgs;
    m_p_ocm_msgs                = queues_info.ocm_msgs;
    m_p_comm_hp_msgs            = queues_info.comm_hp_msgs;

    // Save queue pointers
    m_p_detect_request_queue    = &queues_info.detect_request_queue;
    m_p_storage_request_queue   = &queues_info.storage_request_queue;
    m_p_comm_request_queue      = &queues_info.comm_request_queue;
    m_p_ocm_read_request_queue  = &queues_info.ocm_read_request_queue;
    m_p_ocm_write_request_queue = &queues_info.ocm_write_request_queue;
    m_p_comm_hp_request_queue   = &queues_info.comm_hp_request_queue;

    m_p_detect_response_queue   = &queues_info.detect_response_queue;
    m_p_storage_response_queue  = &queues_info.storage_response_queue;
    m_p_comm_response_queue     = &queues_info.comm_response_queue;
    m_p_ocm_response_queue      = &queues_info.ocm_response_queue;
    m_p_comm_hp_response_queue  = &queues_info.comm_hp_response_queue;

    // Populate detect message pool
    m_detect_request_pool.initialize();
    for (detect_request_id_t id = 0; id < NUM_DETECT_REQUEST_MSGS; id++)
    {
        m_detect_request_pool.free(id);
    }

    // Populate storage message pools
    m_storage_request_pool.initialize();
    for (storage_request_id_t id = 0; id < NUM_STORAGE_REQUEST_MSGS; id++)
    {
        m_storage_request_pool.free(id);
    }

    // Populate comm message pool
    m_comm_request_pool.initialize();
    for (comm_request_id_t id = 0; id < NUM_COMM_REQUEST_MSGS; id++)
    {
        m_comm_request_pool.free(id);
    }

    // Populate ocm message pool
    m_ocm_request_pool.initialize();
    for (ocm_request_id_t id = 0; id < NUM_OCM_REQUEST_MSGS; id++)
    {
        m_ocm_request_pool.free(id);
    }

    // Populate high priority comm message pool
    m_comm_hp_request_pool.initialize();
    for (comm_hp_request_id_t id = 0; id < NUM_COMM_HP_REQUEST_MSGS; id++)
    {
        m_comm_hp_request_pool.free(id);
    }

    m_device_params.time_zone_offset = DeviceConfig::p_page->time_zone_offset;
    m_device_params.sample_rate      = DeviceConfig::p_page->sampling_rate;
    m_device_params.sample_width = DeviceConfig::p_page->audio_sampling_width;
    switch (m_device_params.sample_width)
    {
        case BITS_PER_AUDIO_SAMPLE_8:
        {
            m_samples_per_buffer = BUFFER_SIZE;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_16:
        {
            m_samples_per_buffer = BUFFER_SIZE / 2;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_24:
        {
            m_samples_per_buffer = BUFFER_SIZE / 3;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_32:
        {
            m_samples_per_buffer = BUFFER_SIZE / 4;
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    // Divide by number of channels
    m_device_params.num_channels = DeviceConfig::p_page->num_audio_channels;
    m_samples_per_buffer /= m_device_params.num_channels;

    const uint64_t nanoseconds = NSEC_PER_SEC * BUFFER_SIZE /
                                 (DeviceConfig::get_data_bit_rate() / CHAR_BIT);
    m_buffer_time_delta.seconds =
        static_cast<int64_t>(nanoseconds / NSEC_PER_SEC);
    m_buffer_time_delta.nanoseconds = nanoseconds % NSEC_PER_SEC;

    m_nanoseconds_per_sample =
        (NSEC_PER_SEC << NS_PRECISION_BITS) / m_device_params.sample_rate;

    // Max comm requests to make that are related to each other.
    // Equal to Recording Length * Sample Rate * Sample Size / BUFFER_SIZE.
    uint32_t total_wav_file_sample_data_size =
        DeviceConfig::get_wav_file_sample_data_size();
    m_max_connected_comm_requests =
        total_wav_file_sample_data_size / BUFFER_SIZE +
        ((total_wav_file_sample_data_size % BUFFER_SIZE) != 0);

    if ((DeviceConfig::p_page->start_date.year == DeviceConfig::INVALID_YEAR) ||
        (DeviceConfig::p_page->start_date.month ==
         DeviceConfig::INVALID_MONTH) ||
        (DeviceConfig::p_page->start_date.day ==
         DeviceConfig::INVALID_DAY_IN_MONTH))
    {
        m_device_params.start_date = 0;
    }
    else
    {
        tm tm_struct;
        memset(&tm_struct, 0, sizeof(tm_struct));
        tm_struct.tm_year          = DeviceConfig::p_page->start_date.year;
        tm_struct.tm_mon           = DeviceConfig::p_page->start_date.month;
        tm_struct.tm_mday          = DeviceConfig::p_page->start_date.day;
        m_device_params.start_date = w_hal_rtc_timegm(&tm_struct);
    }

    if ((DeviceConfig::p_page->end_date.year == DeviceConfig::INVALID_YEAR) ||
        (DeviceConfig::p_page->end_date.month == DeviceConfig::INVALID_MONTH) ||
        (DeviceConfig::p_page->end_date.day ==
         DeviceConfig::INVALID_DAY_IN_MONTH))
    {
        m_device_params.end_date = INT64_MAX;
    }
    else
    {
        tm tm_struct;
        memset(&tm_struct, 0, sizeof(tm_struct));
        tm_struct.tm_year        = DeviceConfig::p_page->end_date.year;
        tm_struct.tm_mon         = DeviceConfig::p_page->end_date.month;
        tm_struct.tm_mday        = DeviceConfig::p_page->end_date.day;
        m_device_params.end_date = w_hal_rtc_timegm(&tm_struct);

        // End date is inclusive, so add a day to make it exclusive
        m_device_params.end_date += SECS_PER_DAY;
    }

    m_device_params.time_zone_offset = DeviceConfig::p_page->time_zone_offset;
    m_device_params.record_length    = DeviceConfig::p_page->recording_length;
    m_device_params.sleep_interval = DeviceConfig::p_page->sleep_pause_interval;
    memcpy(m_device_params.time_of_day_range,
           DeviceConfig::p_page->time_of_day_range,
           sizeof(m_device_params.time_of_day_range));
    m_device_params.day_of_week_mask = DeviceConfig::p_page->day_of_week_mask;
    // Sun Mode Parameters
    m_device_params.sun_recording_mode =
        DeviceConfig::p_page->sun_recording_mode;
    m_device_params.sunrise_interval_before_min =
        DeviceConfig::p_page->sunrise_interval_before;
    m_device_params.sunrise_interval_after_min =
        DeviceConfig::p_page->sunrise_interval_after;
    m_device_params.sunset_interval_before_min =
        DeviceConfig::p_page->sunset_interval_before;
    m_device_params.sunset_interval_after_min =
        DeviceConfig::p_page->sunset_interval_after;
    m_device_params.sunrise_sunset_rounding_min =
        DeviceConfig::p_page->sunrise_sunset_rounding;
    m_last_sun_calc_day    = 0;
    m_pending_sun_calc_day = 0;
    m_sun_range_update_pending =
        (m_device_params.sun_recording_mode != DeviceConfig::SUN_MODE_DISABLED);

    for (uint32_t index = 0; index < MAX_NUM_RECORD_INTERVALS; index++)
    {
        m_record_interval[index].start_time.clear();
        m_record_interval[index].num_samples = 0;
    }
    m_add_record_interval_tracker.samples                  = 0;
    m_add_record_interval_tracker.record_interval_index    = 0;

    m_filled_record_interval_tracker.samples               = 0;
    m_filled_record_interval_tracker.record_interval_index = 0;

    m_store_record_interval_tracker.samples                = 0;
    m_store_record_interval_tracker.record_interval_index  = 0;

    m_no_more_record_intervals                             = false;
    m_last_calculated_end_time_local.clear();

    m_audio_fill_state = audio_fill_state_t::FIRST;
    m_add_buffer_start_time.invalidate();
    m_fill_start_time.clear();
    m_store_time.invalidate();
    m_led_sample_counter  = 0;
    m_capture_rtc_on_fill = true; // For first buffer

    m_ocm_buffers_queue.initialize();
    m_detect_buffers_queue.initialize();
    m_ocm_read_response_buffers_queue.initialize();

    m_ocm_store_pointer                    = 0;
    m_ocm_read_request_pointer             = 0;
    m_ocm_write_ocm_response_pointer       = 0;
    m_ocm_read_ocm_response_pointer        = 0;
    m_ocm_detect_transfer_response_pointer = 0;
    m_ocm_detect_done_pointer              = 0;
    m_ocm_start_pointer                    = 0;

    m_num_buffers_audio_fill_to_ocm        = 0;
    m_num_buffers_ocm_to_comm_storage      = 0;

    m_num_buffers_in_hal                   = 0;
    m_num_active_ocm_read_buffers          = 0;
    m_num_active_ocm_write_buffers         = 0;
    m_num_active_comm_requests             = 0;
    m_num_active_storage_requests          = 0;

#if OFF_CHIP_MEMORY_CORE == INVALID_CORE_NUM
    m_ocm_size = 0;
#else
    m_ocm_size = shared_glbl.external_memory_size;
    // Round down to nearest multiple of buffer size
    m_ocm_size = (m_ocm_size >> BUFFER_SIZE_LOG2) << BUFFER_SIZE_LOG2;
    SYSTEM_ASSERT_LTEQ(m_ocm_size, OCM_POINTER_WRAP_BIT_MASK);
    SYSTEM_ASSERT_GT(m_ocm_size, 0);
#endif

    m_suspend_comm_requests = false;

    m_last_comm_status_query_time.clear();
}

/**
 * @brief   Function to handle the calculation of sun-based time of day ranges
 * for recording
 *
 * @param event_second The sunrise or sunset time in seconds since local
 * midnight
 * @param before_sec Number of seconds before the event to include in the range
 * @param after_sec Number of seconds after the event to include in the range
 * @param out_ranges Output array to populate with the resulting time of day
 * ranges.
 *
 * @return uint32_t Number of valid time of day ranges populated in out_ranges.
 */
uint32_t
AudioTask::compute_sun_ranges(uint32_t                            event_second,
                              uint32_t                            before_sec,
                              uint32_t                            after_sec,
                              DeviceConfig::time_of_day_range_st *out_ranges)
{
    int64_t  raw_start = static_cast<int64_t>(event_second) - before_sec;
    int64_t  raw_end   = static_cast<int64_t>(event_second) + after_sec;
    uint32_t count     = 0;

    if (raw_start < 0 && raw_end <= SECONDS_PER_DAY)
    {
        // End spills into previous day — split into two ranges:
        // Range 0: spillover from previous day (end of today)
        // Range 1: normal portion from 00:00
        out_ranges[count].start_second =
            static_cast<uint32_t>(SECONDS_PER_DAY + raw_start);
        out_ranges[count].end_second = SECONDS_PER_DAY;
        count++;
        out_ranges[count].start_second = 0;
        out_ranges[count].end_second   = static_cast<uint32_t>(raw_end);
        count++;
    }
    else if (raw_start >= 0 && raw_end > SECONDS_PER_DAY)
    {
        // End spills into next day — split into two ranges:
        // Range 0: normal portion until midnight
        // Range 1: spillover into next day from 00:00
        out_ranges[count].start_second = static_cast<uint32_t>(raw_start);
        out_ranges[count].end_second   = SECONDS_PER_DAY;
        count++;
        out_ranges[count].start_second = 0;
        out_ranges[count].end_second =
            static_cast<uint32_t>(raw_end - SECONDS_PER_DAY);
        count++;
    }
    else
    {
        // No split — clamp to [0, SECONDS_PER_DAY]
        out_ranges[count].start_second =
            static_cast<uint32_t>(raw_start < 0 ? 0 : raw_start);
        out_ranges[count].end_second = static_cast<uint32_t>(
            raw_end > SECONDS_PER_DAY ? SECONDS_PER_DAY : raw_end);
        count++;
    }

    return count;
}

/**
 * @brief Function to compute sun time of day ranges
 *
 * @param local_midnight local midnight of the day to compute
 */
void AudioTask::compute_sun_time_of_day_ranges(time_t local_midnight)
{
    // if location is not set, sun calculation is not possible.
    // Clear all time_of_day ranges and return — the scheduler will treat
    // this as no valid ranges and record all day.
    if ((DeviceConfig::p_page->latitude_e7 == INVALID_LOCATION_COORDINATE) ||
        (DeviceConfig::p_page->longitude_e7 == INVALID_LOCATION_COORDINATE))
    {
        memset(m_device_params.time_of_day_range,
               0,
               sizeof(m_device_params.time_of_day_range));
        memcpy(DeviceConfig::p_page->time_of_day_range,
               m_device_params.time_of_day_range,
               sizeof(m_device_params.time_of_day_range));
        printf("Sun mode active but location not set — skipping sun "
               "calculation\n");
        return;
    }

    // location and timezone from DeviceConfig
    // location and timezone from DeviceConfig
    double lat = static_cast<double>(DeviceConfig::p_page->latitude_e7) /
                 LOCATION_COORDINATE_SCALE;
    double lon = static_cast<double>(DeviceConfig::p_page->longitude_e7) /
                 LOCATION_COORDINATE_SCALE;
    int              tz_offset_sec    = DeviceConfig::p_page->time_zone_offset;
    // constants for sun calculation
    const int        TIME_SINCE_EPOCH = 1900;
    const int        ZERO_INDEX_MONTH = 1;
    const int        SECONDS_PER_MINUTE           = 60;
    constexpr double MIDDAY_FRACTIONAL_UT         = 0.0;
    const int        MAXIMUM_SUN_RANGES_PER_EVENT = 2;

    // Extract the Local midnight (year, month, day) so the NOAA
    // algorithm computes sunrise/sunset for the correct local day.
    struct tm        tm_struct;
    gmtime_r(&local_midnight, &tm_struct);

    // Compute Julian Date for that local calendar day
    double julian_date =
        SunCalculator::compute_julian_date(tm_struct.tm_year + TIME_SINCE_EPOCH,
                                           tm_struct.tm_mon + ZERO_INDEX_MONTH,
                                           tm_struct.tm_mday,
                                           MIDDAY_FRACTIONAL_UT);
    uint32_t sunrise_second = 0;
    uint32_t sunset_second  = 0;
    if (!SunCalculator::compute_sunrise_sunset_seconds(julian_date,
                                                       lat,
                                                       lon,
                                                       tz_offset_sec,
                                                       sunrise_second,
                                                       sunset_second))
    {
        // Solar calculation failed (e.g. polar day or night){
        printf(
            "Sun calculation failed for the date and location. This can happen "
            "in polar regions during periods of polar day or night. Clearing "
            "sun time of day ranges so no sun-based recording will occur.\n");
        memset(m_device_params.time_of_day_range,
               0,
               sizeof(m_device_params.time_of_day_range));
        memcpy(DeviceConfig::p_page->time_of_day_range,
               m_device_params.time_of_day_range,
               sizeof(m_device_params.time_of_day_range));
        return;
    }

    // Apply rounding (e.g. 05:28 -> 05:30) if enabled
    if (m_device_params.sunrise_sunset_rounding_min > 0)
    {
        uint32_t round_sec =
            m_device_params.sunrise_sunset_rounding_min * SECONDS_PER_MINUTE;
        sunrise_second =
            ((sunrise_second + (round_sec / 2)) / round_sec) * round_sec;
        sunset_second =
            ((sunset_second + (round_sec / 2)) / round_sec) * round_sec;
    }

    // Clear ALL regular time_of_day slots first
    memset(m_device_params.time_of_day_range,
           0,
           sizeof(m_device_params.time_of_day_range));

    uint32_t                           slot = 0;
    DeviceConfig::time_of_day_range_st tmp[MAXIMUM_SUN_RANGES_PER_EVENT];

    // Write computed windows directly into the regular time_of_day_range
    // so the existing scheduler and >TIMEOFDAY console command see them.
    switch (m_device_params.sun_recording_mode)
    {
        case DeviceConfig::SUN_MODE_SUNRISE:
        {
            uint32_t n = compute_sun_ranges(
                sunrise_second,
                static_cast<uint32_t>(
                    m_device_params.sunrise_interval_before_min *
                    SECONDS_PER_MINUTE),
                static_cast<uint32_t>(
                    m_device_params.sunrise_interval_after_min *
                    SECONDS_PER_MINUTE),
                tmp);
            for (uint32_t i = 0; i < n; i++)
                m_device_params.time_of_day_range[slot++] = tmp[i];
            break;
        }

        case DeviceConfig::SUN_MODE_SUNSET:
        {
            uint32_t n = compute_sun_ranges(
                sunset_second,
                static_cast<uint32_t>(
                    m_device_params.sunset_interval_before_min *
                    SECONDS_PER_MINUTE),
                static_cast<uint32_t>(
                    m_device_params.sunset_interval_after_min *
                    SECONDS_PER_MINUTE),
                tmp);
            for (uint32_t i = 0; i < n; i++)
                m_device_params.time_of_day_range[slot++] = tmp[i];
            break;
        }

        case DeviceConfig::SUN_MODE_SUNRISE_AND_SUNSET:
        {
            uint32_t n = compute_sun_ranges(
                sunrise_second,
                static_cast<uint32_t>(
                    m_device_params.sunrise_interval_before_min *
                    SECONDS_PER_MINUTE),
                static_cast<uint32_t>(
                    m_device_params.sunrise_interval_after_min *
                    SECONDS_PER_MINUTE),
                tmp);
            for (uint32_t i = 0; i < n; i++)
                m_device_params.time_of_day_range[slot++] = tmp[i];

            n = compute_sun_ranges(
                sunset_second,
                static_cast<uint32_t>(
                    m_device_params.sunset_interval_before_min *
                    SECONDS_PER_MINUTE),
                static_cast<uint32_t>(
                    m_device_params.sunset_interval_after_min *
                    SECONDS_PER_MINUTE),
                tmp);
            for (uint32_t i = 0; i < n; i++)
                m_device_params.time_of_day_range[slot++] = tmp[i];
            break;
        }

        case DeviceConfig::SUN_MODE_SUNRISE_TO_SUNSET:
        {
            // Single range: sunrise-before -> sunset+after, clamped
            int64_t raw_start = static_cast<int64_t>(sunrise_second) -
                                m_device_params.sunrise_interval_before_min *
                                    SECONDS_PER_MINUTE;
            int64_t raw_end =
                static_cast<int64_t>(sunset_second) +
                m_device_params.sunset_interval_after_min * SECONDS_PER_MINUTE;
            m_device_params.time_of_day_range[0].start_second =
                static_cast<uint32_t>(raw_start < 0 ? 0 : raw_start);
            m_device_params.time_of_day_range[0].end_second =
                static_cast<uint32_t>(
                    raw_end > SECONDS_PER_DAY ? SECONDS_PER_DAY : raw_end);
            break;
        }

        case DeviceConfig::SUN_MODE_SUNSET_TO_SUNRISE:
        {
            // Range 0: 00:00:00 -> sunrise+after (clamped to SECONDS_PER_DAY)
            int64_t raw_end =
                static_cast<int64_t>(sunrise_second) +
                m_device_params.sunrise_interval_after_min * SECONDS_PER_MINUTE;
            m_device_params.time_of_day_range[0].start_second = 0;
            m_device_params.time_of_day_range[0].end_second =
                static_cast<uint32_t>(
                    raw_end > SECONDS_PER_DAY ? SECONDS_PER_DAY : raw_end);

            // Range 1: sunset-before (clamped to 0) -> SECONDS_PER_DAY
            int64_t raw_start =
                static_cast<int64_t>(sunset_second) -
                m_device_params.sunset_interval_before_min * SECONDS_PER_MINUTE;
            m_device_params.time_of_day_range[1].start_second =
                static_cast<uint32_t>(raw_start < 0 ? 0 : raw_start);
            m_device_params.time_of_day_range[1].end_second = SECONDS_PER_DAY;
            break;
        }

        default:
            break;
    }

    // Sort and merge the sun-based time of day ranges, and push any invalid
    // entries to the back.  This simplifies the scheduler logic later since it
    // can rely on the invariant that valid time of day ranges are sorted,
    // non-overlapping, and packed at the front of the array.
    DeviceConfig::sort_time_of_day_ranges(m_device_params.time_of_day_range,
                                          MAX_SCHEDULE_TIME_OF_DAY_RANGES);

    // Sync computed ranges back to the flash Page so the >TIMEOFDAY console
    // command prints them
    memcpy(DeviceConfig::p_page->time_of_day_range,
           m_device_params.time_of_day_range,
           sizeof(m_device_params.time_of_day_range));
}

/**
 * @brief Function to get local midnight time for a given local time
 *
 * @param local_time the local time to get the midnight of current local day
 * @return time_t representing the local midnight time
 */
time_t AudioTask::get_local_midnight(time_t local_time) const
{
    struct tm local_tm;
    gmtime_r(&local_time, &local_tm);

    return local_time - (local_tm.tm_hour * SECS_PER_HR +
                         local_tm.tm_min * SECS_PER_MIN + local_tm.tm_sec);
}

/**
 * @brief Function to set sun range update pending flag based on current local
 * time
 *
 * @param current_time_local
 */
void AudioTask::request_sun_range_update(
    const w_hal_rtc_time_st &current_time_local)
{
    // If sun mode is disabled, make sure pending update is cleared and return
    if (m_device_params.sun_recording_mode == DeviceConfig::SUN_MODE_DISABLED)
    {
        m_sun_range_update_pending = false;
        m_pending_sun_calc_day     = 0;
        m_last_sun_calc_day        = 0;
        return;
    }

    // if location is not set, sun calculation is not possible.
    if ((DeviceConfig::p_page->latitude_e7 == INVALID_LOCATION_COORDINATE) ||
        (DeviceConfig::p_page->longitude_e7 == INVALID_LOCATION_COORDINATE))
    {
        memset(m_device_params.time_of_day_range,
               0,
               sizeof(m_device_params.time_of_day_range));
        m_sun_range_update_pending = false;
        return;
    }

    // Calculate local midnight for current local time and compare with last sun
    // calculated day to determine if sun range update is needed.
    const time_t local_midnight =
        get_local_midnight(current_time_local.time_seconds);

    if ((m_last_sun_calc_day != local_midnight) &&
        (!m_sun_range_update_pending ||
         (m_pending_sun_calc_day != local_midnight)))
    {
        m_pending_sun_calc_day     = local_midnight;
        m_sun_range_update_pending = true;
    }
}

/**
 * @brief Apply the pending sun range update if it exists
 *
 * @return true if the update was applied, false if no update was pending
 */
bool AudioTask::apply_pending_sun_range_update()
{
    if (!m_sun_range_update_pending)
    {
        return false;
    }
    // Compute and apply sun time of day ranges for the pending sun calc day
    compute_sun_time_of_day_ranges(m_pending_sun_calc_day);
    m_last_sun_calc_day                   = m_pending_sun_calc_day;
    m_sun_range_update_pending            = false;
    shared_glbl.update_config_status_flag = true;

    return true;
}

/**
 * @brief Function to update sun based time ranges
 *
 */
void AudioTask::service_sun_range_update()
{
    w_hal_rtc_time_st current_time;
    w_hal_rtc_get_time(current_time);

    // convert UTC -> local time
    w_hal_rtc_time_st current_time_local = current_time;
    current_time_local.time_seconds += m_device_params.time_zone_offset;

    // This Function call updates flags and variables related to pending sun
    // range update based on current local time.
    request_sun_range_update(current_time_local);
    // Compute and apply sun range update if pending.
    if (!apply_pending_sun_range_update())
    {
        return;
    }

    // Clear the audio scheduler record intervals, so
    // calculate_next_record_length() will recalculate the record intervals
    // based on the new sun time of day ranges.
    record_interval_tracker_st &record_interval_tracker =
        m_add_record_interval_tracker;
    record_interval_st &record_interval =
        m_record_interval[record_interval_tracker.record_interval_index];

    if ((record_interval_tracker.samples == 0) &&
        (record_interval.num_samples > 0) && (m_num_buffers_in_hal == 0) &&
        (m_num_buffers_audio_fill_to_ocm == 0) &&
        (current_time < record_interval.start_time))
    {
        record_interval.num_samples = 0;
        record_interval.start_time.invalidate();
        m_add_buffer_start_time = current_time;
    }
}

/**
 * @brief Function to determine if sleep pause should be skipped based on sun
 * mode and last calculated sun-based recording end time
 *
 * @param active_start_time_local the local start time of the active recording
 * interval that is being evaluated for sleep pause skipping
 *
 * @return true if sleep pause should be skipped, false otherwise
 */
bool AudioTask::skip_sleep_pause(
    const w_hal_rtc_time_st &active_start_time_local) const
{
    if (m_device_params.sun_recording_mode == DeviceConfig::SUN_MODE_DISABLED ||
        m_device_params.sun_recording_mode ==
            DeviceConfig::SUN_MODE_SUNRISE_TO_SUNSET)
    {
        return false; // only block modes that structurally cannot cross
                      // midnight
    }

    struct tm last_end_tm;
    gmtime_r(&m_last_calculated_end_time_local.time_seconds, &last_end_tm);

    if ((last_end_tm.tm_hour != 0) || (last_end_tm.tm_min != 0) ||
        (last_end_tm.tm_sec != 0))
    {
        return false;
    }

    return get_local_midnight(active_start_time_local.time_seconds) ==
           m_last_calculated_end_time_local.time_seconds;
}

/**
 * @brief Find the earliest record interval that is active
 * and set the active start and end times.
 *
 * @param start_time_boundary the start time boundary for the search
 * @param end_time_boundary the end time boundary for the search
 * @param record_start_time [out] the start time of the earliest active record
 * interval within the time boundary
 * @param record_end_time [out] the end time of the earliest active record
 * interval within the time boundary
 *
 * @return true if record start and end times are valid
 */
bool AudioTask::find_record_interval_from_day_of_week_mask(
    const w_hal_rtc_time_st &start_time_boundary,
    const w_hal_rtc_time_st &end_time_boundary,
    w_hal_rtc_time_st &      record_start_time,
    w_hal_rtc_time_st &      record_end_time)
{
    // See if active range can be shortened due to day of week mask
    if (m_device_params.day_of_week_mask == 0)
    {
        // Shorten active time to 0
        record_start_time = end_time_boundary;
        record_end_time   = end_time_boundary;

        return true;
    }
    else if (m_device_params.day_of_week_mask == DeviceConfig::FULL_WEEK_MASK)
    {
        record_start_time = start_time_boundary;
        record_end_time   = end_time_boundary;

        return true;
    }
    else
    {
        tm tm_struct;
        gmtime_r(&start_time_boundary.time_seconds, &tm_struct);

        // Lowest bit of mask in device config is Monday
        // Lowest value of weekday in time struct corresponds to
        // Sunday (0 value)
        uint32_t current_weekday =
            (tm_struct.tm_wday == 0) ?
                (DeviceConfig::NUM_DAYS_IN_WEEK - 1) :
                static_cast<uint32_t>(tm_struct.tm_wday - 1);
        SYSTEM_ASSERT_LT(current_weekday, DeviceConfig::NUM_DAYS_IN_WEEK);

        // Find first active day in week mask
        uint32_t index;
        for (index = 0; index < DeviceConfig::NUM_DAYS_IN_WEEK; index++)
        {
            if (m_device_params.day_of_week_mask & (1 << current_weekday))
            {
                break;
            }

            if (++current_weekday >= DeviceConfig::NUM_DAYS_IN_WEEK)
            {
                current_weekday = 0;
            }
        }

        SYSTEM_ASSERT_LT(index, DeviceConfig::NUM_DAYS_IN_WEEK);

        // Based on the first active day in week mask, calculate the
        // start time of the first active day.
        w_hal_rtc_time_st day_mask_begin_time = start_time_boundary;
        day_mask_begin_time.time_seconds -= tm_struct.tm_hour * SECS_PER_HR +
                                            tm_struct.tm_min * SECS_PER_MIN +
                                            tm_struct.tm_sec;
        day_mask_begin_time.time_nanoseconds = 0;
        day_mask_begin_time.time_seconds += index * SECS_PER_DAY;

        // Count number of consecutive active days in week mask from the first
        // discovered active day.
        uint32_t num_consecutive_days = 0;
        for (index = index; index < DeviceConfig::NUM_DAYS_IN_WEEK; index++)
        {
            if ((m_device_params.day_of_week_mask & (1 << current_weekday)) ==
                0)
            {
                break;
            }

            ++num_consecutive_days;

            if (++current_weekday >= DeviceConfig::NUM_DAYS_IN_WEEK)
            {
                current_weekday = 0;
            }
        }

        // End time is at least the next day
        SYSTEM_ASSERT_GT(num_consecutive_days, 0);
        w_hal_rtc_time_st day_mask_end_time = day_mask_begin_time;
        day_mask_end_time.time_seconds += num_consecutive_days * SECS_PER_DAY;

        if (day_mask_begin_time > start_time_boundary)
        {
            // Shorten active time to the start of the active day
            record_start_time = day_mask_begin_time;
        }
        else
        {
            record_start_time = start_time_boundary;
        }

        if (day_mask_end_time < end_time_boundary)
        {
            // Shorten active time to the end of the active day
            record_end_time = day_mask_end_time;
        }
        else
        {
            record_end_time = end_time_boundary;
        }

        return (record_start_time < record_end_time);
    }
}

/**
 * @brief Find the earliest record interval that is active
 * and set the active start and end times.
 *
 * @param start_time_boundary the start time boundary for the search
 * @param end_time_boundary the end time boundary for the search
 * @param record_start_time [out] the start time of the earliest active record
 * interval within the time boundary
 * @param record_end_time [out] the end time of the earliest active record
 * interval within the time boundary
 *
 * @return true if record start and end times are valid
 */
bool AudioTask::find_record_interval_from_time_of_day_ranges(
    const w_hal_rtc_time_st &start_time_boundary,
    const w_hal_rtc_time_st &end_time_boundary,
    w_hal_rtc_time_st &      record_start_time,
    w_hal_rtc_time_st &      record_end_time)
{
    tm tm_struct;
    gmtime_r(&start_time_boundary.time_seconds, &tm_struct);

    // For the current time interval, calculate what the time would
    // be if it started at the beginning of the day.
    w_hal_rtc_time_st active_start_time_day_begin = start_time_boundary;

    active_start_time_day_begin.time_seconds -=
        tm_struct.tm_hour * SECS_PER_HR + tm_struct.tm_min * SECS_PER_MIN +
        tm_struct.tm_sec;
    active_start_time_day_begin.time_nanoseconds = 0;

    uint32_t num_valid_time_of_day_ranges;
    for (num_valid_time_of_day_ranges = 0;
         num_valid_time_of_day_ranges < MAX_SCHEDULE_TIME_OF_DAY_RANGES;
         num_valid_time_of_day_ranges++)
    {
        if (!DeviceConfig::is_valid_time_of_day_range(
                m_device_params
                    .time_of_day_range[num_valid_time_of_day_ranges]))
        {
            break;
        }
    }

    if (num_valid_time_of_day_ranges == 0)
    {
        // If there were no valid time of day ranges, then
        // the active time is the entire day.
        record_start_time = start_time_boundary;
        record_end_time   = end_time_boundary;

        return true;
    }

    // Default result is no active time
    record_start_time = record_end_time = end_time_boundary;

    // It is assumed that the time of day ranges are sorted in ascending
    // order by start time of day and non-overlapping.
    // Check overlap over a 2-day period just in case the start time boundary
    // is past any configured end time of day.
    for (uint32_t day_index = 0; day_index < 2; day_index++)
    {
        for (uint32_t index = 0; index < num_valid_time_of_day_ranges; index++)
        {
            // Then add the start time of day to the beginning of the day.
            w_hal_rtc_time_st active_time_interval_start =
                active_start_time_day_begin;
            active_time_interval_start.time_seconds += day_index * SECS_PER_DAY;
            active_time_interval_start.time_seconds +=
                m_device_params.time_of_day_range[index].start_second;

            // Then add the end time of day to the beginning of the day.
            w_hal_rtc_time_st active_time_interval_end =
                active_start_time_day_begin;
            active_time_interval_end.time_seconds += day_index * SECS_PER_DAY;
            active_time_interval_end.time_seconds +=
                m_device_params.time_of_day_range[index].end_second;

            // Check for non-zero overlap between active time-of-day interval
            // and (active start time, active end time).
            if (active_time_interval_start < end_time_boundary &&
                active_time_interval_end > start_time_boundary)
            {
                // Shorten active time to the overlapping time period
                if (active_time_interval_start > start_time_boundary)
                {
                    // clip to start of active time interval
                    record_start_time = active_time_interval_start;
                }
                else
                {
                    record_start_time = start_time_boundary;
                }

                if (active_time_interval_end < end_time_boundary)
                {
                    // clip to end of active time interval
                    record_end_time = active_time_interval_end;
                }
                else
                {
                    record_end_time = end_time_boundary;
                }

                return true;
            }
        }
    }

    return false;
}

/**
 * @brief Calculate the number of samples to record and the number of seconds
 * to sleep before recording.
 *
 * @param current_time the reference time for the calculation
 * @param num_samples[out]: number of samples to record
 * @param start_time[out]: start time of the recording
 */
void AudioTask::calculate_next_record_length(
    const w_hal_rtc_time_st &current_time,
    uint64_t &               num_samples,
    w_hal_rtc_time_st &      start_time)
{
    w_hal_rtc_time_st active_start_time_local;
    w_hal_rtc_time_st active_end_time_local;
    w_hal_rtc_time_st start_time_local;

    w_hal_rtc_time_st current_time_local = current_time;
    current_time_local.time_seconds += m_device_params.time_zone_offset;

    active_start_time_local.clear();
    active_start_time_local.time_seconds = m_device_params.start_date;

    active_end_time_local.clear();
    active_end_time_local.time_seconds = m_device_params.end_date;

    if (current_time_local > active_start_time_local)
    {
        active_start_time_local = current_time_local;
    }

    if ((m_device_params.sleep_interval > 0) &&
        !skip_sleep_pause(active_start_time_local))
    {
        // Make sure recording has slept for the sleep interval
        // before starting again.
        w_hal_rtc_time_st last_time_after_sleep =
            m_last_calculated_end_time_local;
        last_time_after_sleep.time_seconds += m_device_params.sleep_interval;
        if (last_time_after_sleep > active_start_time_local)
        {
            // Shorten active time to the time after sleep
            active_start_time_local = last_time_after_sleep;
        }
    }

    if (active_end_time_local < active_start_time_local)
    {
        active_end_time_local = active_start_time_local;
    }

    w_hal_rtc_time_st start_time_boundary = active_start_time_local;
    w_hal_rtc_time_st end_time_boundary   = active_end_time_local;
    bool              found_time_range    = false;
    // For each contiguous day range, find out if there is a time range
    // to record. Should only need to search for max 2 separate day ranges.
    for (uint32_t day_range_index = 0; day_range_index < 2; day_range_index++)
    {
        w_hal_rtc_time_st found_day_range_start, found_day_range_end;

        bool              found_day_range =
            find_record_interval_from_day_of_week_mask(start_time_boundary,
                                                       end_time_boundary,
                                                       found_day_range_start,
                                                       found_day_range_end);

        if (found_day_range)
        {
            w_hal_rtc_time_st found_time_range_start;
            w_hal_rtc_time_st found_time_range_end;

            found_time_range = find_record_interval_from_time_of_day_ranges(
                found_day_range_start,
                found_day_range_end,
                found_time_range_start,
                found_time_range_end);

            if (found_time_range)
            {
                active_start_time_local = found_time_range_start;
                active_end_time_local   = found_time_range_end;
                break;
            }
        }

        // Search for next contiguous day range from days of week mask in the
        // following loop iteration
        start_time_boundary = found_day_range_end;
    }

    if (!found_time_range)
    {
        active_end_time_local = active_start_time_local;
    }

    // If there is no sleep interval, then the active time does
    // not need to be broken up into multiple intervals.
    if (m_device_params.sleep_interval > 0)
    {
        if (active_end_time_local > active_start_time_local)
        {
            // Shorten active time to record length
            w_hal_rtc_time_delta_st time_delta;
            active_end_time_local.subtract(active_start_time_local, time_delta);

            if (time_delta.seconds >= m_device_params.record_length)
            {
                active_end_time_local.time_seconds =
                    active_start_time_local.time_seconds +
                    m_device_params.record_length;
                active_end_time_local.time_nanoseconds =
                    active_start_time_local.time_nanoseconds;
            }
        }
    }

    SYSTEM_ASSERT_LTEQ(current_time_local.time_seconds,
                       active_start_time_local.time_seconds);

    if (active_end_time_local <= active_start_time_local)
    {
        // No active time, sleep forever
        num_samples = 0;
        start_time_local.invalidate();
    }
    else if ((active_end_time_local.time_seconds == INT64_MAX) ||
             (active_end_time_local.time_seconds ==
              w_hal_rtc_time_st::INVALID_SECONDS))
    {
        // Active time is forever
        num_samples                      = UINT64_MAX;
        start_time_local                 = active_start_time_local;

        m_last_calculated_end_time_local = active_end_time_local;
    }
    else
    {
        start_time_local = active_start_time_local;

        w_hal_rtc_time_delta_st active_delta;
        active_end_time_local.subtract(active_start_time_local, active_delta);

        // Calculate number of samples to sleep before record
        num_samples = static_cast<uint64_t>(active_delta.seconds) *
                          m_device_params.sample_rate +
                      static_cast<uint64_t>(active_delta.nanoseconds) *
                          m_device_params.sample_rate / NSEC_PER_SEC;

        m_last_calculated_end_time_local = active_end_time_local;
    }

    if (start_time_local.is_valid())
    {
        start_time.time_seconds = start_time_local.time_seconds;
        start_time.time_seconds -= m_device_params.time_zone_offset;
        start_time.time_nanoseconds = start_time_local.time_nanoseconds;
    }
    else
    {
        start_time.invalidate();
    }
}

/**
 * @brief
 * Decrement active count for buffer. If active count reaches 0,
 * release it.
 *
 * @param buffer_id id of the buffer
 */
void AudioTask::decrement_active_count(buffer_id_t buffer_id)
{
    SYSTEM_ASSERT_NOT_EQUAL(buffer_id, INVALID_BUFFER_ID);
    SYSTEM_ASSERT_NOT_EQUAL(m_buffer_active_count[buffer_id], 0);
    if (--m_buffer_active_count[buffer_id] == 0)
    {
        bool result = m_buffer_pool.free(buffer_id);
        SYSTEM_ASSERT_TRUE(result);

        SYSTEM_ASSERT_NOT_EQUAL(m_num_active_buffers, 0);
        m_num_active_buffers--;
    }
}

/**
 * @brief
 * Handle response from Detect Task
 *
 * @return true if a response was received
 * @return false otherwise
 */
bool AudioTask::handle_detect_response()
{
    bool                hadResponse            = false;
    bool                release_detect_request = true;
    detect_request_id_t detect_response_id     = INVALID_DETECT_MSG_ID;

    if (m_p_detect_response_queue->pop(detect_response_id))
    {
        hadResponse = true;

        SYSTEM_ASSERT_NOT_EQUAL(detect_response_id, INVALID_DETECT_MSG_ID);
        SYSTEM_ASSERT_LT(detect_response_id, NUM_DETECT_REQUEST_MSGS);
        DetectRequestMsg &detect_msg = m_p_detect_msgs[detect_response_id];
        const buffer_id_t buffer_id  = detect_msg.get_buffer_id();
        const MsgStatus   status     = detect_msg.get_status();

        switch (status)
        {
            case MsgStatus::DETECT_SOUND_EVENT_DETECTED:
            {
                // Detect done pointer may go past ocm write
                // response pointer for a short time, but ocm
                // write response pointer will catch up
                SYSTEM_ASSERT_EQUAL(buffer_id, INVALID_BUFFER_ID);
                m_ocm_detect_done_pointer =
                    m_ocm_detect_transfer_response_pointer;
                break;
            }
            case MsgStatus::DETECT_SOUND_EVENT_NOT_DETECTED:
            {
                SYSTEM_ASSERT_EQUAL(buffer_id, INVALID_BUFFER_ID);

                // Discard this data
                // Move store time counter
                uint32_t diff = ocm_pointer_difference(
                    m_ocm_store_pointer,
                    m_ocm_detect_transfer_response_pointer);

                // Increment store time
                for (uint32_t index = 0; index < diff; index++)
                {
                    m_store_time += m_buffer_time_delta;
                    m_led_sample_counter += m_samples_per_buffer;
                }

                // Detect done pointer may go past ocm write
                // response pointer for a short time, but ocm
                // write response pointer will catch up
                m_ocm_store_pointer = m_ocm_read_ocm_response_pointer =
                    m_ocm_read_request_pointer = m_ocm_detect_done_pointer =
                        m_ocm_detect_transfer_response_pointer;
                break;
            }
            case MsgStatus::DETECT_BUFFER_FULLY_LOADED:
            {
                release_detect_request = false;

                // Use same msg to request detect result
                w_hal_rtc_time_st time;
                w_hal_rtc_get_time(time);
                detect_msg.initialize(time, true, INVALID_BUFFER_ID, 0, true);
                bool result;
#if DETECT_CORE == INVALID_CORE_NUM
                detect_msg.set_status(MsgStatus::DETECT_SOUND_EVENT_DETECTED);
                result = m_p_detect_response_queue->push(detect_response_id);
#else
                result = m_p_detect_request_queue->push(detect_response_id);
#endif
                SYSTEM_ASSERT_TRUE(result);
            }
            // FALL-THROUGH
            case MsgStatus::GOOD:
            default:
            {
                SYSTEM_ASSERT_NOT_EQUAL(status, MsgStatus::FAIL);
                decrement_active_count(buffer_id);
                m_ocm_detect_transfer_response_pointer = increment_ocm_pointer(
                    m_ocm_detect_transfer_response_pointer);
                break;
            }
        }

        if (release_detect_request)
        {
            m_detect_request_pool.free(detect_response_id);
        }
    }

    return hadResponse;
}

/**
 * @brief
 * Handle response from Comm Task
 * Just retire the message and handle buffer accounting.
 *
 * @return true if a response was received
 * @return false otherwise
 */
bool AudioTask::handle_comm_response()
{
    bool              hadResponse      = false;
    comm_request_id_t comm_response_id = INVALID_COMM_MSG_ID;

    if (m_p_comm_response_queue->pop(comm_response_id))
    {
        SYSTEM_ASSERT_NOT_EQUAL(comm_response_id, INVALID_COMM_MSG_ID);
        CommRequestMsg &  comm_msg  = m_p_comm_msgs[comm_response_id];

        const buffer_id_t buffer_id = comm_msg.get_buffer_id();

        decrement_active_count(buffer_id);
        if (m_buffer_active_count[buffer_id] == 0)
        {
            SYSTEM_ASSERT_GT(m_num_buffers_ocm_to_comm_storage, 0);
            m_num_buffers_ocm_to_comm_storage--;
        }

        m_comm_request_pool.free(comm_response_id);

        m_num_active_comm_requests--;

        hadResponse = true;
    }

    return hadResponse;
}

/**
 * @brief
 * Handle response from Comm Task
 * Just admin actions as a result of the response.
 *
 * @return true if a response was received
 * @return false otherwise
 */
bool AudioTask::handle_comm_hp_response()
{
    bool                 hadResponse         = false;
    comm_hp_request_id_t comm_hp_response_id = INVALID_COMM_HP_MSG_ID;

    if (m_p_comm_hp_response_queue->pop(comm_hp_response_id))
    {
        SYSTEM_ASSERT_NOT_EQUAL(comm_hp_response_id, INVALID_COMM_HP_MSG_ID);
        CommHpRequestMsg &comm_hp_msg = m_p_comm_hp_msgs[comm_hp_response_id];

        switch (comm_hp_msg.get_status())
        {
            case MsgStatus::COMM_OK:
            case MsgStatus::COMM_RESTORED:
            {
                if (m_suspend_comm_requests)
                {
                    LOG_MSG(TraceFile::AUDIO_TASK_CPP, "Comm resumed\n");
                    m_suspend_comm_requests = false;
                }
                break;
            }
            case MsgStatus::COMM_SUSPENDED:
            {
                if (!m_suspend_comm_requests)
                {
                    LOG_MSG(TraceFile::AUDIO_TASK_CPP, "Comm paused\n");
                    m_suspend_comm_requests = true;
                }
                break;
            }
            default:
            {
                SYSTEM_ASSERT_TRUE(false);
                break;
            }
        }

        m_comm_hp_request_pool.free(comm_hp_response_id);

        hadResponse = true;
    }

    return hadResponse;
}

/**
 * @brief
 * Handle response from Storage Task
 * Just retire the message and handle buffer accounting.
 *
 * @return true if a response was received
 * @return false otherwise
 */
bool AudioTask::handle_storage_response()
{
    bool                 hadResponse         = false;
    storage_request_id_t storage_response_id = INVALID_STORAGE_MSG_ID;

    if (m_p_storage_response_queue->pop(storage_response_id))
    {
        SYSTEM_ASSERT_NOT_EQUAL(storage_response_id, INVALID_STORAGE_MSG_ID);
        StorageRequestMsg &storage_msg = m_p_storage_msgs[storage_response_id];

        const buffer_id_t  buffer_id   = storage_msg.get_buffer_id();

        decrement_active_count(buffer_id);
        if (m_buffer_active_count[buffer_id] == 0)
        {
            SYSTEM_ASSERT_GT(m_num_buffers_ocm_to_comm_storage, 0);
            m_num_buffers_ocm_to_comm_storage--;
        }

        m_storage_request_pool.free(storage_response_id);
        m_num_active_storage_requests--;

        hadResponse = true;
    }

    return hadResponse;
}

/**
 * @brief
 * Handle response from OCM Task
 * Just retire the message and handle buffer accounting.
 *
 * @return true if a response was received
 * @return false otherwise
 */
bool AudioTask::handle_ocm_response()
{
    bool             hadResponse     = false;
    ocm_request_id_t ocm_response_id = INVALID_OCM_MSG_ID;

    if (m_p_ocm_response_queue->pop(ocm_response_id))
    {
        hadResponse = true;

        SYSTEM_ASSERT_NOT_EQUAL(ocm_response_id, INVALID_OCM_MSG_ID);
        OCMRequestMsg &   ocm_msg   = m_p_ocm_msgs[ocm_response_id];
        const buffer_id_t buffer_id = ocm_msg.get_buffer_id();

        if (ocm_msg.is_write())
        {
            SYSTEM_ASSERT_GT(m_num_buffers_audio_fill_to_ocm, 0);
            m_num_buffers_audio_fill_to_ocm--;

            SYSTEM_ASSERT_GT(m_num_active_ocm_write_buffers, 0);
            m_num_active_ocm_write_buffers--;

            m_ocm_write_ocm_response_pointer =
                increment_ocm_pointer(m_ocm_write_ocm_response_pointer);
        }
        else
        {
            // Don't decrement active count for read buffers here
            // because we are not done with the buffer yet.

            m_ocm_read_ocm_response_pointer =
                increment_ocm_pointer(m_ocm_read_ocm_response_pointer);

            m_ocm_read_response_buffers_queue.push(buffer_id);

            // This should be the first response received for this buffer
            SYSTEM_ASSERT_EQUAL(m_buffer_active_count[buffer_id],
                                NUM_EXPECTED_OCM_COMM_STORAGE_RESPONSES);
        }

        decrement_active_count(buffer_id);

        m_ocm_request_pool.free(ocm_response_id);
    }

    return hadResponse;
}

/**
 * @brief
 * Handle all incoming responses from other tasks
 *
 * @param hadResponse [out] : indicates if any response was received from
 * the producing tasks
 * @param waitingForResponse [out] : indicates if a response is still
 * expected from the other tasks
 */
void AudioTask::handle_responses(bool &hadResponse, bool &waitingForResponse)
{
    // Check for responses from Detect Task
    hadResponse        = handle_detect_response();

    // Check for responses from Comm Task
    hadResponse        = handle_comm_response() || hadResponse;

    // Check for high priority responses from Comm Task
    hadResponse        = handle_comm_hp_response() || hadResponse;

    // Check for responses from Storage Task
    hadResponse        = handle_storage_response() || hadResponse;

    // Check for responses from OCM Task
    hadResponse        = handle_ocm_response() || hadResponse;

    waitingForResponse = (m_num_active_buffers != 0);
}

/**
 * @brief Convert number of samples to number of bytes
 *
 * @param num_samples number of samples.
 * @return uint32_t number of bytes.
 */
uint32_t AudioTask::samples_to_bytes(uint64_t num_samples)
{
    uint32_t size = num_samples;
    switch (m_device_params.sample_width)
    {
        case BITS_PER_AUDIO_SAMPLE_16:
        {
            size = num_samples * 2;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_24:
        {
            size = num_samples * 3;
            break;
        }
        case BITS_PER_AUDIO_SAMPLE_32:
        {
            size = num_samples * 4;
            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }

    size *= m_device_params.num_channels;

    return size;
}

/**
 * @brief Allocate buffers for HAL
 */
void AudioTask::allocate_buffers_to_hal()
{
    if (shared_glbl.reboot_flag)
    {
        return;
    }

    while ((m_num_buffers_in_hal < MAX_BUFFERS_IN_HAL) &&
           (m_num_buffers_audio_fill_to_ocm < MAX_BUFFERS_AUDIO_FILL_TO_OCM) &&
           !m_no_more_record_intervals)
    {
        record_interval_tracker_st &record_interval_tracker =
            m_add_record_interval_tracker;

        record_interval_st &record_interval =
            m_record_interval[record_interval_tracker.record_interval_index];

        if ((record_interval.num_samples == 0) && (m_num_buffers_in_hal == 0))
        {
            SYSTEM_ASSERT_EQUAL(record_interval_tracker.samples, 0);

            // Wait to populate buffers until the start time
            // of the current record interval
            w_hal_rtc_time_st current_time;
            w_hal_rtc_get_time(current_time);

            if (!m_add_buffer_start_time.is_valid() ||
                (m_add_buffer_start_time < current_time))
            {
                m_add_buffer_start_time = current_time;
            }

            calculate_next_record_length(m_add_buffer_start_time,
                                         record_interval.num_samples,
                                         record_interval.start_time);
#if RTC_WAKE != INVALID_CORE_NUM

            //  Check If woken by BLE before the scheduled alarm time
            if (!shared_glbl.custom_flags.rtc_mfp_wake_flag &&
                shared_glbl.next_wakeup_time_utc.is_valid() &&
                (current_time < shared_glbl.next_wakeup_time_utc))
            {
                record_interval.start_time = shared_glbl.next_wakeup_time_utc;
            }
#endif // RTC_WAKE != INVALID_CORE_NUM

            if (record_interval.num_samples > 0)
            {
                m_add_buffer_start_time = record_interval.start_time;
            }
        }

        // No more record intervals to add
        if (record_interval.num_samples == 0)
        {
            break;
        }

        // Wait to populate buffers until the start time
        // of the current record interval
        w_hal_rtc_time_st current_time;
        w_hal_rtc_get_time(current_time);
        if (current_time < record_interval.start_time)
        {
#if RTC_WAKE != INVALID_CORE_NUM
            int64_t sleep_time_seconds =
                record_interval.start_time.time_seconds -
                current_time.time_seconds;
            if (!shared_glbl.custom_flags.ble_connection_flag &&
                sleep_time_seconds > MIN_SLEEP_GAP_SECONDS)
            {
                shared_glbl.power_down_flag = true;
                shared_glbl.next_wakeup_time_utc =
                    record_interval.start_time -
                    EARLY_WAKEUP_SECONDS;       // Copied as struct
                shared_glbl.reboot_flag = true; // Triggers shutdown stages
                shared_glbl.shutdown_stage =
                    shutdown_stage_t::Stage_1; // Initiates graceful task  //
                                               // shutdown
            }
#endif // RTC_WAKE != INVALID_CORE_NUM
            break;
        }

        buffer_id_t buffer_id;
        if (!m_buffer_pool.allocate(buffer_id))
        {
            // No buffers available
            break;
        }

        uint32_t samples = m_samples_per_buffer;

        SYSTEM_ASSERT_GT(record_interval.num_samples, 0);
        if (record_interval_tracker.samples + m_samples_per_buffer >=
            record_interval.num_samples)
        {
            // Truncate the last buffer to the number of samples remaining
            samples =
                record_interval.num_samples - record_interval_tracker.samples;
        }

        const uint32_t size = samples_to_bytes(samples);

        wHalErrorCode  error =
            w_hal_i2s_add_rx_buffer(get_buffer_address(buffer_id), size);
        if (error == wHalErrorCode::BUSY)
        {
            // HAL has enough buffers
            bool result = m_buffer_pool.free(buffer_id);
            SYSTEM_ASSERT_TRUE(result);
            break;
        }
        else
        {
            SYSTEM_ASSERT_EQUAL(error, wHalErrorCode::NO_ERROR);
            ++m_num_buffers_in_hal;
            ++m_num_buffers_audio_fill_to_ocm;

            if (size == BUFFER_SIZE)
            {
                m_add_buffer_start_time += m_buffer_time_delta;
            }
            else
            {
                // Need to calculate the time for the last buffer
                const uint64_t buffer_time_nanoseconds =
                    (samples * m_nanoseconds_per_sample) >> NS_PRECISION_BITS;

                m_add_buffer_start_time.time_nanoseconds +=
                    buffer_time_nanoseconds;
                while (m_add_buffer_start_time.time_nanoseconds >= NSEC_PER_SEC)
                {
                    m_add_buffer_start_time.time_seconds++;
                    m_add_buffer_start_time.time_nanoseconds -= NSEC_PER_SEC;
                }
            }

            SYSTEM_ASSERT_GT(samples, 0);
            record_interval_tracker.samples += samples;
            SYSTEM_ASSERT_LTEQ(record_interval_tracker.samples,
                               record_interval.num_samples);
            if (record_interval_tracker.samples == record_interval.num_samples)
            {
                record_interval_tracker.samples = 0;

                // Increment the record interval index for adding buffers
                if (++record_interval_tracker.record_interval_index ==
                    MAX_NUM_RECORD_INTERVALS)
                {
                    record_interval_tracker.record_interval_index = 0;
                }

                if (record_interval_tracker.record_interval_index ==
                    m_store_record_interval_tracker.record_interval_index)
                {
                    // No more record intervals to add
                    m_no_more_record_intervals = true;
                }
            }
        }
    }
}

/**
 * @brief Receive filled buffers from HAL
 */
void AudioTask::receive_filled_buffers_from_hal()
{
    if (w_hal_i2s_samples_dropped() > 0)
    {
        // If samples were dropped, the buffer fill start
        // time is unknown, so reacquire the current time.
        m_capture_rtc_on_fill = true;
    }

    switch (m_audio_fill_state)
    {
        case audio_fill_state_t::ACQUIRE:
        {
            m_audio_fill_state = audio_fill_state_t::FILL;
            // If samples were dropped or this is the first buffer being
            // filled since power-on, reacquire the current time.
            if (m_capture_rtc_on_fill)
            {
                w_hal_rtc_get_time(m_fill_start_time);

                // After the first buffer is filled using the rtc time,
                // each subsequent buffer is calculated using the
                // the time delta for generating a full buffer.
                m_capture_rtc_on_fill = false;
            }
            else
            {
                // Otherwise, increment the time by the number of samples
                // in the last buffer.
                m_fill_start_time += m_buffer_time_delta;
            }
        }
        // FALL-THROUGH
        case audio_fill_state_t::FILL:
        {
            // read samples from buffer
            void *        p_buffer;
            uint32_t      size;
            wHalErrorCode error =
                w_hal_i2s_get_filled_rx_buffer(p_buffer, size);

            if (error != wHalErrorCode::NO_ERROR)
            {
                break;
            }

            buffer_id_t buffer_id =
                get_buffer_id(static_cast<uint8_t *>(p_buffer));
            SYSTEM_ASSERT_LT(buffer_id, NUM_BUFFERS);

            record_interval_tracker_st &record_interval_tracker =
                m_filled_record_interval_tracker;
            const record_interval_st &record_interval =
                m_record_interval[record_interval_tracker
                                      .record_interval_index];

            uint32_t samples = m_samples_per_buffer;

            SYSTEM_ASSERT_GT(record_interval.num_samples, 0);
            if (record_interval_tracker.samples + m_samples_per_buffer >=
                record_interval.num_samples)
            {
                // Truncate the last buffer to the number of samples remaining
                samples = record_interval.num_samples -
                          record_interval_tracker.samples;
            }

            if (m_capture_rtc_on_fill)
            {
                w_hal_rtc_get_time(m_fill_start_time);
            }
            else if (record_interval_tracker.samples == 0)
            {
                m_fill_start_time = record_interval.start_time;
            }
            else
            {
                m_fill_start_time = m_fill_start_time;
            }

            SYSTEM_ASSERT_GT(samples, 0);
            record_interval_tracker.samples += samples;
            SYSTEM_ASSERT_LTEQ(record_interval_tracker.samples,
                               record_interval.num_samples);
            if (record_interval_tracker.samples == record_interval.num_samples)
            {
                record_interval_tracker.samples = 0;

                // Increment the record interval index for adding buffers
                if (++record_interval_tracker.record_interval_index ==
                    MAX_NUM_RECORD_INTERVALS)
                {
                    record_interval_tracker.record_interval_index = 0;
                }
            }

            // Add buffer id and timestamp to queue
            uint32_t          buffer_size = samples_to_bytes(samples);
            buffer_tracker_st buffer_tracker;
            buffer_tracker.buffer_id = buffer_id;
            buffer_tracker.size      = buffer_size;
            buffer_tracker.time      = m_fill_start_time;

            // Send buffers for detection and saving to OCM
            SYSTEM_ASSERT_LT(buffer_id, NUM_BUFFERS);
            SYSTEM_ASSERT_EQUAL(m_buffer_active_count[buffer_id], 0);
            m_buffer_active_count[buffer_id] =
                NUM_EXPECTED_DETECT_OCM_RESPONSES;

            bool success = m_ocm_buffers_queue.push(buffer_tracker);
            SYSTEM_ASSERT_TRUE(success);

            success = m_detect_buffers_queue.push(buffer_tracker);
            SYSTEM_ASSERT_TRUE(success);

            --m_num_buffers_in_hal;
            ++m_num_active_buffers;

            m_audio_fill_state = audio_fill_state_t::FIRST;

            break;
        }
        default:
        {
            SYSTEM_ASSERT_TRUE(false);
            break;
        }
    }
}

/**
 * @brief Process write requests to OCM
 */
void AudioTask::process_ocm_write_requests()
{
    if (m_ocm_request_pool.isEmpty())
    {
        return;
    }

    // If OCM is full, then stop filling buffers
    if (ocm_pointers_check_full_condition(m_ocm_store_pointer,
                                          m_ocm_start_pointer))
    {
        return;
    }

    buffer_tracker_st buffer_tracker;
    if (m_ocm_buffers_queue.pop(buffer_tracker))
    {
        // Get a ocm message
        ocm_request_id_t ocm_request_id = INVALID_OCM_MSG_ID;
        bool             result = m_ocm_request_pool.allocate(ocm_request_id);
        SYSTEM_ASSERT_TRUE(result);
        SYSTEM_ASSERT_NOT_EQUAL(ocm_request_id, INVALID_OCM_MSG_ID);
        SYSTEM_ASSERT_LT(ocm_request_id, NUM_OCM_REQUEST_MSGS);
        OCMRequestMsg &ocm_msg = m_p_ocm_msgs[ocm_request_id];

        // Setup and send ocm message
        SYSTEM_ASSERT_LTEQ((m_ocm_start_pointer & ~OCM_POINTER_WRAP_BIT_MASK) +
                               BUFFER_SIZE,
                           m_ocm_size);
        ocm_msg.initialize(buffer_tracker.time,
                           buffer_tracker.buffer_id,
                           BUFFER_SIZE,
                           true,
                           m_ocm_start_pointer & ~OCM_POINTER_WRAP_BIT_MASK,
                           true);
#if OFF_CHIP_MEMORY_CORE == INVALID_CORE_NUM
        // Fake response
        result = m_p_ocm_response_queue->push(ocm_request_id);
#else
        result = m_p_ocm_write_request_queue->push(ocm_request_id);
#endif
        SYSTEM_ASSERT_TRUE(result);

        m_num_active_ocm_write_buffers++;

        // If this is the first buffer to be saved to OCM, store timestamp
        if (ocm_pointers_check_empty_condition(m_ocm_store_pointer,
                                               m_ocm_start_pointer))
        {
            m_store_time = buffer_tracker.time;
        }

        m_ocm_start_pointer = increment_ocm_pointer(m_ocm_start_pointer);
    }
}

/**
 * @brief Process write requests to OCM
 */
void AudioTask::process_detect_requests()
{
    // Only allow one detect request at a time
    if (!m_detect_request_pool.isFull())
    {
        return;
    }

    buffer_tracker_st buffer_tracker;
    if (m_detect_buffers_queue.pop(buffer_tracker))
    {
        // Get a detect message
        detect_request_id_t detect_request_id = INVALID_DETECT_MSG_ID;
        bool result = m_detect_request_pool.allocate(detect_request_id);
        SYSTEM_ASSERT_TRUE(result);
        SYSTEM_ASSERT_NOT_EQUAL(detect_request_id, INVALID_DETECT_MSG_ID);
        SYSTEM_ASSERT_LT(detect_request_id, NUM_DETECT_REQUEST_MSGS);
        DetectRequestMsg &detect_msg = m_p_detect_msgs[detect_request_id];

        // Setup and send detect message
        detect_msg.initialize(buffer_tracker.time,
                              false,
                              buffer_tracker.buffer_id,
                              buffer_tracker.size,
                              false);

#if DETECT_CORE == INVALID_CORE_NUM
        detect_msg.set_status(MsgStatus::DETECT_BUFFER_FULLY_LOADED);
        result = m_p_detect_response_queue->push(detect_request_id);
#else
        result = m_p_detect_request_queue->push(detect_request_id);
#endif
        SYSTEM_ASSERT_TRUE(result);
    }
}

/**
 * @brief Send reads to OCM task for buffers that triggered positive detect
 * result
 *
 * @return true if work has been done or there is work to do
 * @return false otherwise
 */
bool AudioTask::process_ocm_reads()
{
    bool hasWork = false;

    if (!ocm_pointers_check_empty_condition(m_ocm_read_request_pointer,
                                            m_ocm_detect_done_pointer))
    {
        hasWork = true;

        if (m_num_buffers_ocm_to_comm_storage >=
            MAX_BUFFERS_OCM_TO_COMM_STORAGE)
        {
            return hasWork;
        }

        if (!m_ocm_request_pool.isEmpty() && !m_buffer_pool.isEmpty())
        {
            // Get a ocm message
            ocm_request_id_t ocm_request_id = INVALID_OCM_MSG_ID;
            bool result = m_ocm_request_pool.allocate(ocm_request_id);
            SYSTEM_ASSERT_TRUE(result);
            SYSTEM_ASSERT_NOT_EQUAL(ocm_request_id, INVALID_OCM_MSG_ID);
            OCMRequestMsg &ocm_msg = m_p_ocm_msgs[ocm_request_id];

            // Get a buffer
            buffer_id_t    buffer_id;
            result = m_buffer_pool.allocate(buffer_id);
            SYSTEM_ASSERT_TRUE(result);
            SYSTEM_ASSERT_NOT_EQUAL(buffer_id, INVALID_BUFFER_ID);

            // Setup and send ocm message
            SYSTEM_ASSERT_LTEQ(
                (m_ocm_start_pointer & ~OCM_POINTER_WRAP_BIT_MASK) +
                    BUFFER_SIZE,
                m_ocm_size);
            w_hal_rtc_time_st time;
            w_hal_rtc_get_time(time);
            ocm_msg.initialize(time,
                               buffer_id,
                               BUFFER_SIZE,
                               false,
                               m_ocm_read_request_pointer &
                                   ~OCM_POINTER_WRAP_BIT_MASK,
                               false);
#if OFF_CHIP_MEMORY_CORE == INVALID_CORE_NUM
            // Fake response
            result = m_p_ocm_response_queue->push(ocm_request_id);
#else
            result = m_p_ocm_read_request_queue->push(ocm_request_id);
#endif
            SYSTEM_ASSERT_TRUE(result);

            m_num_active_ocm_read_buffers++;
            m_num_buffers_ocm_to_comm_storage++;

            SYSTEM_ASSERT_EQUAL(m_buffer_active_count[buffer_id], 0);
            m_buffer_active_count[buffer_id] =
                NUM_EXPECTED_OCM_COMM_STORAGE_RESPONSES;
            ++m_num_active_buffers;

            m_ocm_read_request_pointer =
                increment_ocm_pointer(m_ocm_read_request_pointer);
        }
    }

    return hasWork;
}

/**
 * @brief Send storage request to Storage Task
 *
 * @param time time
 * @param buffer_id id of the buffer
 * @param size size of the buffer
 * @param flush flush the storage file after this buffer
 */
void AudioTask::send_storage_request(const w_hal_rtc_time_st &time,
                                     buffer_id_t              buffer_id,
                                     uint32_t                 size,
                                     bool                     flush)
{
    // Get a storage message
    storage_request_id_t storage_request_id = INVALID_STORAGE_MSG_ID;
    bool result = m_storage_request_pool.allocate(storage_request_id);
    SYSTEM_ASSERT_TRUE(result);
    SYSTEM_ASSERT_NOT_EQUAL(storage_request_id, INVALID_STORAGE_MSG_ID);
    SYSTEM_ASSERT_LT(storage_request_id, NUM_STORAGE_REQUEST_MSGS);
    StorageRequestMsg &storage_msg = m_p_storage_msgs[storage_request_id];

    SYSTEM_ASSERT_NOT_EQUAL(m_buffer_active_count[buffer_id], 0);
    // Setup and send storage message
    storage_msg.initialize(time, buffer_id, size, flush);
#if STORAGE_CORE == INVALID_CORE_NUM
    // Fake response
    m_p_storage_response_queue->push(storage_request_id);
#else
    m_p_storage_request_queue->push(storage_request_id);
#endif
    m_num_active_storage_requests++;
}

/**
 * @brief Send comm request to Comm Task
 *
 * @param time time
 * @param buffer_id id of the buffer
 * @param size size of the buffer
 * @param remaining_buffers remaining buffers after this one in comm request
 * series
 */
void AudioTask::send_comm_request(const w_hal_rtc_time_st &time,
                                  buffer_id_t              buffer_id,
                                  uint32_t                 size,
                                  uint32_t                 remaining_buffers)
{
    comm_request_id_t comm_request_id = INVALID_COMM_MSG_ID;
    bool              result = m_comm_request_pool.allocate(comm_request_id);
    SYSTEM_ASSERT_TRUE(result);
    SYSTEM_ASSERT_NOT_EQUAL(comm_request_id, INVALID_COMM_MSG_ID);
    SYSTEM_ASSERT_LT(comm_request_id, NUM_COMM_REQUEST_MSGS);
    CommRequestMsg &comm_msg = m_p_comm_msgs[comm_request_id];

    SYSTEM_ASSERT_NOT_EQUAL(m_buffer_active_count[buffer_id], 0);
    comm_msg.initialize(time, buffer_id, size, remaining_buffers);
#if COMM_CORE == INVALID_CORE_NUM
    // Fake response
    m_p_comm_response_queue->push(comm_request_id);
#else
    m_p_comm_request_queue->push(comm_request_id);
#endif
    m_num_active_comm_requests++;
}

/**
 * @brief Send high priority comm request to Comm Task
 *
 * @param wHalErrorCode error describing the
 */
void AudioTask::send_hp_comm_request(MsgStatus message)
{
    comm_hp_request_id_t request_id = INVALID_COMM_HP_MSG_ID;
    bool                 result = m_comm_hp_request_pool.allocate(request_id);

    // If there are no more high priority requests available, then we can
    // just wait until the next time we need to send a high priority
    // request.
    if (!result)
    {
        return;
    }

    SYSTEM_ASSERT_NOT_EQUAL(request_id, INVALID_COMM_HP_MSG_ID);
    SYSTEM_ASSERT_LT(request_id, NUM_COMM_HP_REQUEST_MSGS);
    CommHpRequestMsg &comm_msg = m_p_comm_hp_msgs[request_id];

    w_hal_rtc_time_st time;
    w_hal_rtc_get_time(time);
    comm_msg.initialize(time, INVALID_BUFFER_ID, 0, false);
#if COMM_CORE == INVALID_CORE_NUM
    // Fake response
    comm_msg.set_status(MsgStatus::COMM_OK);
    m_p_comm_hp_response_queue->push(request_id);
#else
    comm_msg.set_status(message);
    m_p_comm_hp_request_queue->push(request_id);
#endif
}

/**
 * @brief Process buffers that need to be sent to storage and comm tasks
 *
 * @return true if work has been done or there is work to do
 * @return false otherwise
 */
bool AudioTask::process_store_buffers()
{
    buffer_id_t buffer_id;

    if (m_ocm_read_response_buffers_queue.isEmpty())
    {
        return false;
    }

    if (m_storage_request_pool.isEmpty() || m_comm_request_pool.isEmpty())
    {
        return true;
    }

    if (m_num_active_comm_requests >= MAX_ACTIVE_COMM_REQUESTS)
    {
        return true;
    }

    if (m_num_active_storage_requests >= MAX_ACTIVE_STORAGE_REQUESTS)
    {
        return true;
    }

    if (m_ocm_read_response_buffers_queue.pop(buffer_id))
    {
        m_num_active_ocm_read_buffers--;

        SYSTEM_ASSERT_NOT_EQUAL(buffer_id, INVALID_BUFFER_ID);

        m_ocm_store_pointer = increment_ocm_pointer(m_ocm_store_pointer);

        m_led_sample_counter += m_samples_per_buffer;

        const uint32_t half_second_samples = m_device_params.sample_rate / 2;
        const uint32_t coarse_time_seconds = 4;

        // The purpose of this is to speed up the time update
        const uint32_t coarse_time_samples =
            m_device_params.sample_rate * coarse_time_seconds;

        if (m_led_sample_counter >= half_second_samples)
        {
            // Update time coarsely
            while (m_led_sample_counter >= coarse_time_samples)
            {
                m_led_sample_counter -= coarse_time_samples;
            }

            // Update time finely
            const w_hal_rtc_time_delta_st half_second_time_delta(
                0,
                (NSEC_PER_SEC / 2));
            while (m_led_sample_counter >= half_second_samples)
            {
                m_led_sample_counter -= half_second_samples;
            }

            if (shared_glbl.power_mgmt_info.low_battery_flag)
            {
                // If the battery is low, then the activity LED should
                // be off since the system is in a low power state and
                // recording is not occurring.
                w_hal_system_set_activity_led(false);
            }
            else
            {
                // Toggle the activity LED every half second
                w_hal_system_toggle_activity_led();
            }
        }

        record_interval_tracker_st &record_interval_tracker =
            m_store_record_interval_tracker;
        record_interval_st &record_interval =
            m_record_interval[record_interval_tracker.record_interval_index];

        uint32_t samples = m_samples_per_buffer;

        SYSTEM_ASSERT_GT(record_interval.num_samples, 0);
        if (record_interval_tracker.samples + m_samples_per_buffer >=
            record_interval.num_samples)
        {
            // Truncate the last buffer to the number of samples remaining
            samples =
                record_interval.num_samples - record_interval_tracker.samples;
        }

        bool flush = false;
        SYSTEM_ASSERT_GT(samples, 0);
        record_interval_tracker.samples += samples;
        if (record_interval_tracker.samples == record_interval.num_samples)
        {
            record_interval_tracker.samples = 0;

            // This is the last stage for the record interval, so
            // clear the record interval.
            record_interval.num_samples     = 0;

            // Increment the record interval index for adding buffers
            if (++record_interval_tracker.record_interval_index ==
                MAX_NUM_RECORD_INTERVALS)
            {
                record_interval_tracker.record_interval_index = 0;
            }

            // No more record intervals to add
            m_no_more_record_intervals = false;

            // If this is the end of the record interval, then flush the file.
            flush                      = true;
        }

        uint32_t size = samples_to_bytes(samples);
        send_storage_request(m_store_time, buffer_id, size, flush);

        if (m_suspend_comm_requests)
        {
            // Adjust buffer active count
            decrement_active_count(buffer_id);
        }
        else
        {
            // TODO: Ideally a countdown value should be used for
            // remaining_buffers, but this is not available yet.
            send_comm_request(m_store_time,
                              buffer_id,
                              size,
                              flush ? 0 : (m_max_connected_comm_requests - 1));
        }

        // Increment store time
        m_store_time += m_buffer_time_delta;
    }

    return true;
}

/**
 * @brief Check if buffers are becoming less available and if so, send a
 * high priority comm request to Comm Task to check if it is ready to
 * receive more data or if it needs to be suspended.
 */
void AudioTask::check_buffer_slowdown()
{
    if (m_suspend_comm_requests)
    {
        // If Comm Requests are already bypassed, then check
        // if the system is ready to start sending Comm
        // Requests again.
        w_hal_rtc_time_st time;
        w_hal_rtc_get_time(time);

        w_hal_rtc_time_delta_st delta;
        time.subtract(m_last_comm_status_query_time, delta);

        // Only do this query every 0.5 seconds
        if ((delta.seconds > 0) || (delta.nanoseconds >= (NSEC_PER_SEC / 2)))
        {
            send_hp_comm_request(MsgStatus::COMM_STATUS_QUERY);

            m_last_comm_status_query_time = time;
        }
    }
    // OCM is almost full or there are audio sample drop alerts from the I2S
    // HAL.
    else if (((m_num_buffers_in_hal == 0) &&
              (w_hal_i2s_check_drop_alert() != wHalErrorCode::NO_ERROR)) ||
             ocm_pointers_check_almost_full_condition(m_ocm_store_pointer,
                                                      m_ocm_start_pointer,
                                                      OCM_LOW_SPACE_THRESHOLD))
    {
        // Flag the system to bypass Comm Requests
        send_hp_comm_request(MsgStatus::BUFFER_SLOWDOWN);
    }
}

/**
 * @brief
 * Acquire audio samples from I2S. Once buffer is full, push buffer
 * for save to SD and transmission
 *
 * @return true if an audio sample was acquired
 * @return false otherwise
 */
bool AudioTask::process_audio_samples()
{
    bool hasWork = true;

    w_hal_i2s_handler();
    receive_filled_buffers_from_hal();
    allocate_buffers_to_hal();
    process_ocm_write_requests();
    process_detect_requests();
    check_buffer_slowdown();

    return hasWork;
}

/**
 * @brief
 * Main routine for audio task
 *
 * @return TaskErrorCode Sleep if idle and not waiting on response,
 * Yield if idle, NoError otherwise
 */
TaskErrorCode AudioTask::run()
{
    TaskErrorCode errorCode;
    bool          hadResponse;
    bool          waitingForResponse;
    bool          queued;

    // Update sun based time ranges before processing responses and audio
    // samples
    service_sun_range_update();

    handle_responses(hadResponse, waitingForResponse);
    queued = process_audio_samples();
    queued = process_store_buffers() || queued;
    queued = process_ocm_reads() || queued;

    if (!hadResponse && !queued)
    {
        // no activity but still waiting for a response
        errorCode = TaskErrorCode::Yield;
    }
    else
    {
        // some processing occurred. Keep task running
        errorCode = TaskErrorCode::NoError;
    }

    // Check for shutdown flag For tasks shutdown
    if (shared_glbl.reboot_flag)
    {
        switch (shared_glbl.shutdown_stage)
        {
            case shutdown_stage_t::Stage_1:
            {
                // Stop allocating anymore buffers to hal and send filled
                // buffers to OCM
                if ((m_num_buffers_in_hal == 0) &&
                    (m_num_buffers_audio_fill_to_ocm == 0))
                {
                    SYSTEM_ASSERT_TRUE(m_p_ocm_write_request_queue->isEmpty());
                    shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                        static_cast<int>(shared_glbl.shutdown_stage) + 1);
                }
                else
                {
                    break;
                }
            }
            // FALL-THROUGH
            case shutdown_stage_t::Stage_2:
            {
                // Check for OCM empty and transfer all the buffers from OCM to
                // Storage_Task and Comm_Task
                if (ocm_pointers_check_empty_condition(m_ocm_store_pointer,
                                                       m_ocm_start_pointer) &&
                    (m_num_buffers_ocm_to_comm_storage == 0))
                {
                    SYSTEM_ASSERT_TRUE(m_p_ocm_read_request_queue->isEmpty());
                    SYSTEM_ASSERT_TRUE(m_p_ocm_response_queue->isEmpty());
                    SYSTEM_ASSERT_TRUE(m_p_detect_request_queue->isEmpty());
                    SYSTEM_ASSERT_TRUE(m_p_detect_response_queue->isEmpty());

                    shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                        static_cast<int>(shared_glbl.shutdown_stage) + 1);
                }
                break;
            }

            case shutdown_stage_t::Stage_7:
            {
                // Final Stage of Audio task and go to sleep
                if (m_p_comm_response_queue->isEmpty() &&
                    m_p_storage_response_queue->isEmpty())
                {
                    SYSTEM_ASSERT_TRUE(m_buffer_pool.isFull());

                    errorCode                  = TaskErrorCode::Sleep;
                    shared_glbl.shutdown_stage = static_cast<shutdown_stage_t>(
                        static_cast<int>(shared_glbl.shutdown_stage) + 1);
                    break;
                }
            }
            default:
            {
                break;
            }
        }
    }

    return errorCode;
}

#endif // __CORE__ == AUDIO_CORE