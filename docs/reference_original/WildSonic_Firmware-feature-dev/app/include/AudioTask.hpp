#ifndef _AUDIO_TASK_HPP
#define _AUDIO_TASK_HPP

#include "BitwisePool.hpp"
#include "Messages.hpp"
#include "Task.hpp"
#include "constants.hpp"
#include "device_config.hpp"

/**
 * @brief
 * Task for acquiring and processing audio samples from I2S.
 * Once samples are acquired, audio task will facilitate
 * moving of data to other modules.
 */
class AudioTask :
    public Task
{
  public:
    // Default constructor
    AudioTask() = default;

    // Set up and initialization for Audio Task
    void          initialize(queues_info_t &queues_info);

    // Main routine for audio task
    TaskErrorCode run() override;

  private:
    using date_st = DeviceConfig::date_st;

    // Number of expected responses from other modules per buffer
    static const uint32_t NUM_EXPECTED_DETECT_OCM_RESPONSES =
        2; // One OCM Write and Detect transfer response
    static const uint32_t NUM_EXPECTED_COMM_STORAGE_RESPONSES =
        2; // One Comm and Storage Response
    static const uint32_t NUM_EXPECTED_OCM_COMM_STORAGE_RESPONSES =
        NUM_EXPECTED_COMM_STORAGE_RESPONSES +
        1; // One OCM Read, Comm, and Storage Response

    // OCM Pointer management. Highest bit in pointer indicates wrap condition
    // to distinguish between full and empty.
    using ocm_pointer_t = uint32_t;
    static const uint32_t OCM_POINTER_WRAP_BIT =
        sizeof(uint32_t) * CHAR_BIT - 1;
    static const ocm_pointer_t OCM_POINTER_WRAP_BIT_MASK =
        1ull << OCM_POINTER_WRAP_BIT;

    static const uint32_t MAX_BUFFERS_IN_HAL              = 5;

    static const uint32_t MAX_BUFFERS_AUDIO_FILL_TO_OCM   = 6;
    static const uint32_t MAX_BUFFERS_OCM_TO_COMM_STORAGE = 4;
    static const uint32_t MAX_ACTIVE_COMM_REQUESTS        = 4;
    static const uint32_t MAX_ACTIVE_STORAGE_REQUESTS     = 4;

    static const uint32_t OCM_LOW_SPACE_THRESHOLD         = 4 * BUFFER_SIZE;

    static const uint32_t MAX_NUM_RECORD_INTERVALS        = 6;

    static const uint32_t NS_PRECISION_BITS               = 20;

    static const uint32_t MIN_SLEEP_GAP_SECONDS           = 5;

    static const int8_t   EARLY_WAKEUP_SECONDS            = 2;

    static_assert(MAX_BUFFERS_AUDIO_FILL_TO_OCM +
                          MAX_BUFFERS_OCM_TO_COMM_STORAGE ==
                      NUM_BUFFERS,
                  "Number of buffers must equal sum of buffers in each stage");

    /**
     * @brief Audio fill routine state machine state
     */
    enum class audio_fill_state_t : uint8_t
    {
        FIRST,
        ACQUIRE = FIRST, // Acquire buffer
        FILL,            // Fill buffer with audio samples
    };

    /**
     * @brief Device parameters pertaining to audio recording.
     * Extracted from DeviceConfig.
     */
    struct device_params_st
    {
        int      time_zone_offset; // Time zone offset
        uint32_t sample_rate;      // Sample rate of
                                   // audio samples
        uint8_t  num_channels;     // Number of audio
                                   // channels
        uint8_t  sample_width;     // Number of bits per
                                   // audio sample
        uint32_t record_length;    // Length of time to
                                   // record in seconds
        uint32_t sleep_interval;   // Length of time to
                                   // sleep in seconds
        time_t   start_date;       // Start date for
                                   // recording
        time_t   end_date;         // End date for
                                   // recording (not inclusive)
        // Time of day ranges for recording
        DeviceConfig::time_of_day_range_st
                time_of_day_range[MAX_SCHEDULE_TIME_OF_DAY_RANGES];

        // for recording
        uint8_t day_of_week_mask; // Day of week mask
                                  // for recording
        // Sun Mode Parameters
        DeviceConfig::sun_recording_mode_t
                sun_recording_mode;          // Sun recording mode
        uint8_t sunrise_interval_before_min; // Sunrise interval before
        uint8_t sunrise_interval_after_min;  // Sunrise interval after
        uint8_t sunset_interval_before_min;  // Sunset interval before
        uint8_t sunset_interval_after_min;   // Sunset interval after
        uint8_t sunrise_sunset_rounding_min; // Sunrise sunset rounding
    };

    /**
     * @brief Record interval information
     */
    struct record_interval_st
    {
        w_hal_rtc_time_st start_time;  // Start time of
                                       // recording
        uint64_t          num_samples; // Number of
                                       // samples to
                                       // record
    };

    /**
     * @brief Tracker information for
     * managing audio recording
     * intervals.
     */
    struct record_interval_tracker_st
    {
        uint64_t samples;               // Number of samples accumulated
        uint8_t  record_interval_index; // Record interval index
    };

    /**
     * @brief
     * Tracker information for managing
     * filling of audio buffer
     */
    struct buffer_tracker_st
    {
        uint8_t           buffer_id; // buffer id number
        uint16_t          size;      // size of buffer
        w_hal_rtc_time_st time;      // time when buffer was filled
    };

    // Sunrise/Sunset dynamic scheduling
    time_t   m_last_sun_calc_day;    // Local midnight of last day sun windows
                                     // that were computed
    time_t   m_pending_sun_calc_day; // Local midnight of current local day for
                                     // pending sun window update
    bool     m_sun_range_update_pending; // Flag indicating sun range update is
                                         // pending

    // Increment ocm pointer
    uint32_t increment_ocm_pointer(uint32_t pointer);

    // Compare pointers to check empty condition
    bool     ocm_pointers_check_empty_condition(ocm_pointer_t pointer1,
                                                ocm_pointer_t pointer2);

    // Compare pointers to check full condition
    bool     ocm_pointers_check_full_condition(ocm_pointer_t pointer1,
                                               ocm_pointer_t pointer2);

    // Compare pointers to check almost full condition
    bool     ocm_pointers_check_almost_full_condition(ocm_pointer_t pointer1,
                                                      ocm_pointer_t pointer2,
                                                      uint32_t low_space_threshold);

    // Compute difference to see how much data is between two pointers
    uint32_t ocm_pointer_difference(ocm_pointer_t pointer1,
                                    ocm_pointer_t pointer2);

    // Convert number of samples to number of bytes
    uint32_t samples_to_bytes(uint64_t num_samples);

    // Find the earliest record interval that is active
    // and set the active start and end times.
    bool     find_record_interval_from_day_of_week_mask(
            const w_hal_rtc_time_st &start_time_boundary,
            const w_hal_rtc_time_st &end_time_boundary,
            w_hal_rtc_time_st &      record_start_time,
            w_hal_rtc_time_st &      record_end_time);

    // Find the earliest record interval that is active
    // and set the active start and end times.
    bool find_record_interval_from_time_of_day_ranges(
        const w_hal_rtc_time_st &start_time_boundary,
        const w_hal_rtc_time_st &end_time_boundary,
        w_hal_rtc_time_st &      record_start_time,
        w_hal_rtc_time_st &      record_end_time);

    // Calculate the number of samples to record and the number of seconds to
    // sleep before recording.
    void     calculate_next_record_length(const w_hal_rtc_time_st &current_time,
                                          uint64_t &               num_samples,
                                          w_hal_rtc_time_st &      start_time);

    // Decrement active count for buffer
    void     decrement_active_count(buffer_id_t buffer_id);

    // Send storage request to Storage Task
    void     send_storage_request(const w_hal_rtc_time_st &time,
                                  buffer_id_t              buffer_id,
                                  uint32_t                 size,
                                  bool                     flush);

    // Send comm request to Comm Task
    void     send_comm_request(const w_hal_rtc_time_st &time,
                               buffer_id_t              buffer_id,
                               uint32_t                 size,
                               uint32_t                 remaining_buffers);

    void     check_buffer_slowdown();

    // Send high priority comm request to Comm Task
    void     send_hp_comm_request(MsgStatus error);

    // Handle response from Comm Task
    bool     handle_comm_hp_response();

    // Handle response from Detect Task
    bool     handle_detect_response();

    // Handle response from Storage Task
    bool     handle_storage_response();

    // Handle response from Comm Task
    bool     handle_comm_response();

    // Handle response from OCM Task
    bool     handle_ocm_response();

    // Handle all incoming responses from other tasks
    void     handle_responses(bool &hadResponse, bool &waitingForResponse);

    // Allocate buffers for HAL
    void     allocate_buffers_to_hal();

    // Receive filled buffers from HAL
    void     receive_filled_buffers_from_hal();

    // Process write requests to OCM
    void     process_ocm_write_requests();

    // Process write requests to OCM
    void     process_detect_requests();

    // Calculate the next record interval based on sun time of day ranges
    uint32_t compute_sun_ranges(uint32_t event_second,
                                uint32_t before_sec,
                                uint32_t after_sec,
                                DeviceConfig::time_of_day_range_st *out_ranges);
    // Compute sun time of day ranges
    void     compute_sun_time_of_day_ranges(time_t local_midnight);

    // Get local midnight for a local timestamp.
    time_t   get_local_midnight(time_t local_time) const;

    // Queue sun range update if the local day changed.
    void request_sun_range_update(const w_hal_rtc_time_st &current_time_local);

    // Apply pending sun range update.
    bool apply_pending_sun_range_update();

    // Poll once per run for local-day sun range updates.
    void service_sun_range_update();

    // Determine if sleep pause should be skipped across a cross-day sun split.
    bool
    skip_sleep_pause(const w_hal_rtc_time_st &active_start_time_local) const;

    // Acquire audio samples from I2S
    bool process_audio_samples();

    // Send reads to OCM task for buffers that triggered positive detect result
    bool process_ocm_reads();

    // Process buffers that need to be sent to storage and comm tasks
    bool process_store_buffers();

    using BufferPool = BitwisePool<buffer_id_t, NUM_BUFFERS>;
    using DetectRequestPool =
        BitwisePool<detect_request_id_t, NUM_DETECT_REQUEST_MSGS>;
    using StorageRequestPool =
        BitwisePool<storage_request_id_t, NUM_STORAGE_REQUEST_MSGS>;
    using CommRequestPool =
        BitwisePool<comm_request_id_t, NUM_COMM_REQUEST_MSGS>;
    using OCMRequestPool = BitwisePool<ocm_request_id_t, NUM_OCM_REQUEST_MSGS>;
    using CommHpRequestPool =
        BitwisePool<comm_request_id_t, NUM_COMM_HP_REQUEST_MSGS>;

    BufferPool         m_buffer_pool;          // Pool of buffer id's
    DetectRequestPool  m_detect_request_pool;  // Pool of detect request id's
    StorageRequestPool m_storage_request_pool; // Pool of storage request id's
    CommRequestPool    m_comm_request_pool;    // Pool of comm request id's
    OCMRequestPool     m_ocm_request_pool;     // Pool of ocm request id's
    CommHpRequestPool  m_comm_hp_request_pool; // Pool of comm high priority
                                               // request id's

    audio_fill_state_t m_audio_fill_state; // Audio fill routine state

    w_hal_rtc_time_st  m_last_calculated_end_time_local; // The last end
                                                         // time in
                                                         // local timezone
                                                         // that was
                                                         // computed
                                                         // for recording

    w_hal_rtc_time_st  m_add_buffer_start_time;  // Time when buffer was added
    w_hal_rtc_time_st  m_fill_start_time;        // Buffer fill start time
    w_hal_rtc_time_st  m_store_time;             // Time to use when sending
                                                 // next storage or comm request
    uint32_t           m_led_sample_counter;     // LED sample counter for
                                                 // changing LED state
    w_hal_rtc_time_delta_st m_buffer_time_delta; // Expected time elapsed since
                                                 // buffer fill start time
    uint64_t                m_nanoseconds_per_sample; // Nanoseconds per
                                                      // audio sample
                                                      // shifted up
                                                      // to preserve
                                                      // precision

    device_params_st        m_device_params; // Device parameters

    uint32_t                m_samples_per_buffer; // Number of audio
                                                  // samples per buffer

    record_interval_st      m_record_interval[MAX_NUM_RECORD_INTERVALS]; // Next
                                                                    // start
                                                                    // time for
                                                                    // recording

    record_interval_tracker_st m_add_record_interval_tracker;
    record_interval_tracker_st m_filled_record_interval_tracker;
    record_interval_tracker_st m_store_record_interval_tracker;

    bool                       m_no_more_record_intervals; // Flag to indicate
                                                           // no more record
                                                           // intervals

    FifoQueue<buffer_tracker_st, NUM_BUFFERS> m_ocm_buffers_queue;    // Send to
                                                                      // OCM
                                                                      // Task
    FifoQueue<buffer_tracker_st, NUM_BUFFERS> m_detect_buffers_queue; // Send to
                                                                      // detect
                                                                      // task

    FifoQueue<buffer_id_t, NUM_BUFFERS>       m_ocm_read_response_buffers_queue;

    uint32_t m_num_active_buffers; // Number of buffers that are being used by
                                   // other modules
    uint8_t  m_buffer_active_count[NUM_BUFFERS]; // Number of modules using each
                                                 // buffer

    bool     m_capture_rtc_on_fill; // Flag to indicate first buffer is filled

    DetectFifoQueue *  m_p_detect_request_queue;    // request queue to
                                                    // detect task
    StorageFifoQueue * m_p_storage_request_queue;   // request queue to
                                                    // storage task
    CommFifoQueue *    m_p_comm_request_queue;      // request queue to
                                                    // comm task
    OCMFifoQueue *     m_p_ocm_read_request_queue;  // read request queue
                                                    // to OCM task
    OCMFifoQueue *     m_p_ocm_write_request_queue; // write request queue
                                                    // to OCM task
    CommHpFifoQueue *  m_p_comm_hp_request_queue;   // high priority
                                                    // request queue to
                                                    // comm task

    DetectFifoQueue *  m_p_detect_response_queue;  // response queue from
                                                   // detect task
    StorageFifoQueue * m_p_storage_response_queue; // response queue from
                                                   // storage task
    CommFifoQueue *    m_p_comm_response_queue;    // response queue from
                                                   // comm task
    OCMFifoQueue *     m_p_ocm_response_queue;     // response queue from
                                                   // OCM task
    CommHpFifoQueue *  m_p_comm_hp_response_queue; // high priority
                                                   // response queue from
                                                   // comm task

    DetectRequestMsg * m_p_detect_msgs;  // Array of detect request messages
    StorageRequestMsg *m_p_storage_msgs; // Array of storage request messages
    CommRequestMsg *   m_p_comm_msgs;    // Array of comm request messages
    OCMRequestMsg *    m_p_ocm_msgs;     // Array of OCM request messages
    CommHpRequestMsg * m_p_comm_hp_msgs; // Array of comm high priority
                                         // request messages

    ocm_pointer_t      m_ocm_start_pointer;               // Write pointer
                                                          // for OCM
    ocm_pointer_t      m_ocm_write_ocm_response_pointer;  // Write OCM
                                                          // completion pointer
    ocm_pointer_t m_ocm_detect_transfer_response_pointer; // Indicates how much
                                                          // data has been
                                                          // transferred to
                                                          // detect module
    ocm_pointer_t m_ocm_detect_done_pointer;              // Indicates what past
                                                          // data detection has
                                                          // been performed or
                                                          // skipped on
    ocm_pointer_t m_ocm_read_request_pointer;             // Read request
                                                          // pointer to OCM
    ocm_pointer_t m_ocm_read_ocm_response_pointer;        // Read response
                                                          // pointer from OCM
    ocm_pointer_t m_ocm_store_pointer;                    // Pointer to write
                                                          // to Comm and Storage
                                                          // Tasks

    uint32_t      m_num_buffers_in_hal;            // Number of buffers in HAL
    uint32_t      m_num_buffers_audio_fill_to_ocm; // Number of buffers filled
                                                   // with audio samples to OCM
    uint32_t      m_num_buffers_ocm_to_comm_storage; // Number of buffers
                                                     // transferred from OCM to
                                                     // Comm and Storage Tasks

    uint32_t      m_num_active_ocm_read_buffers;  // Number of active OCM read
                                                  // buffers
    uint32_t      m_num_active_ocm_write_buffers; // Number of active OCM write
                                                  // buffers
    uint32_t      m_num_active_comm_requests;     // Number of active
                                                  // comm requests
    uint32_t      m_num_active_storage_requests;  // Number of active
                                                  // storage requests

    uint32_t      m_ocm_size; // Size of Off-chip memory

    uint32_t m_max_connected_comm_requests; // Max comm requests to make that
                                            // are related to each other

    bool     m_suspend_comm_requests; // Don't send Comm requests if
                                      // CommTask is stalled
    w_hal_rtc_time_st m_last_comm_status_query_time; // Last time
                                                     // CommTask
                                                     // status
                                                     // was
                                                     // queried
};

/**
 * @brief Increment ocm pointer
 *
 * @param pointer previous pointer value
 * @return uint32_t next pointer value
 */
inline AudioTask::ocm_pointer_t
AudioTask::increment_ocm_pointer(ocm_pointer_t pointer)
{
    SYSTEM_ASSERT_NOT_EQUAL(m_ocm_size, 0);

    if ((pointer & ~OCM_POINTER_WRAP_BIT_MASK) + BUFFER_SIZE >= m_ocm_size)
    {
        if (pointer & OCM_POINTER_WRAP_BIT_MASK)
        {
            pointer = 0;
        }
        else
        {
            pointer = OCM_POINTER_WRAP_BIT_MASK;
        }
    }
    else
    {
        pointer += BUFFER_SIZE;
    }

    return pointer;
}

/**
 * @brief Compare pointers to check empty condition
 *
 * @param pointer1 trailing pointer
 * @param pointer2 head pointer
 * @return true if empty
 * @return false otherwise
 */
inline bool
AudioTask::ocm_pointers_check_empty_condition(ocm_pointer_t pointer1,
                                              ocm_pointer_t pointer2)
{
    return (pointer1 == pointer2);
}

/**
 * @brief Compare pointers to check if full condition has been reached
 *
 * @param pointer1 trailing pointer
 * @param pointer2 head pointer
 * @return true if full
 * @return false otherwise
 */
inline bool AudioTask::ocm_pointers_check_full_condition(ocm_pointer_t pointer1,
                                                         ocm_pointer_t pointer2)
{
    return ((pointer1 != pointer2) &&
            ((pointer1 & ~OCM_POINTER_WRAP_BIT_MASK) ==
             (pointer2 & ~OCM_POINTER_WRAP_BIT_MASK)));
}

/**
 * @brief Compute difference to see how much data is between two pointers
 *
 * @param pointer1 trailing pointer
 * @param pointer2 head pointer
 * @return uint32_t amount of data between two pointers
 */
inline uint32_t AudioTask::ocm_pointer_difference(ocm_pointer_t pointer1,
                                                  ocm_pointer_t pointer2)
{
    SYSTEM_ASSERT_NOT_EQUAL(m_ocm_size, 0);

    uint32_t result;

    uint32_t pointer1_not_wrapped = pointer1 & ~OCM_POINTER_WRAP_BIT_MASK;
    uint32_t pointer2_not_wrapped = pointer2 & ~OCM_POINTER_WRAP_BIT_MASK;

    if ((pointer1 & OCM_POINTER_WRAP_BIT_MASK) !=
        (pointer2 & OCM_POINTER_WRAP_BIT_MASK))
    {
        // If pointer2 has passed pointer1, then overflow has occurred
        SYSTEM_ASSERT_GTEQ(pointer1_not_wrapped, pointer2_not_wrapped);

        result = pointer2_not_wrapped + (m_ocm_size - pointer1_not_wrapped);
        SYSTEM_ASSERT_LTEQ(result, m_ocm_size);
    }
    else
    {
        SYSTEM_ASSERT_LTEQ(pointer1_not_wrapped, pointer2_not_wrapped);
        result = pointer2_not_wrapped - pointer1_not_wrapped;
    }

    return result;
}

/**
 * @brief Calculate if almost full condition has been reached
 *
 * @param pointer1 trailing pointer
 * @param pointer2 head pointer
 * @param low_space_threshold threshold in bytes to determine if almost full
 * @return bool true if almost full, false otherwise
 */
inline bool AudioTask::ocm_pointers_check_almost_full_condition(
    ocm_pointer_t pointer1,
    ocm_pointer_t pointer2,
    uint32_t      low_space_threshold)
{
    uint32_t empty_space =
        m_ocm_size - ocm_pointer_difference(pointer1, pointer2);

    return (empty_space <= low_space_threshold);
}

#endif // _AUDIO_TASK_HPP