#ifndef _MESSAGES_HPP
#define _MESSAGES_HPP

#include "Buffer.hpp"
#include "FifoQueue.hpp"
#include "assert.hpp"

#include <stdint.h>
#include <time.h>

using detect_request_id_t                              = uint8_t;
using storage_request_id_t                             = uint8_t;
using comm_request_id_t                                = uint8_t;
using ocm_request_id_t                                 = uint8_t;
using comm_hp_request_id_t                             = uint8_t;

constexpr uint32_t             NUM_DETECT_REQUEST_MSGS = NUM_BUFFERS;
constexpr detect_request_id_t  INVALID_DETECT_MSG_ID = NUM_DETECT_REQUEST_MSGS;

constexpr uint32_t             NUM_STORAGE_REQUEST_MSGS = NUM_BUFFERS;
constexpr storage_request_id_t INVALID_STORAGE_MSG_ID = NUM_DETECT_REQUEST_MSGS;

constexpr uint32_t             NUM_COMM_REQUEST_MSGS  = NUM_BUFFERS;
constexpr comm_request_id_t    INVALID_COMM_MSG_ID    = NUM_DETECT_REQUEST_MSGS;

constexpr uint32_t             NUM_OCM_REQUEST_MSGS   = NUM_BUFFERS;
constexpr ocm_request_id_t     INVALID_OCM_MSG_ID     = NUM_OCM_REQUEST_MSGS;

constexpr uint32_t             NUM_COMM_HP_REQUEST_MSGS = 1;
constexpr comm_hp_request_id_t INVALID_COMM_HP_MSG_ID =
    NUM_COMM_HP_REQUEST_MSGS;

using DetectFifoQueue = FifoQueue<detect_request_id_t, NUM_DETECT_REQUEST_MSGS>;
using StorageFifoQueue =
    FifoQueue<storage_request_id_t, NUM_STORAGE_REQUEST_MSGS>;
using CommFifoQueue   = FifoQueue<comm_request_id_t, NUM_COMM_REQUEST_MSGS>;
using OCMFifoQueue    = FifoQueue<ocm_request_id_t, NUM_OCM_REQUEST_MSGS>;
using CommHpFifoQueue = FifoQueue<comm_request_id_t, NUM_COMM_HP_REQUEST_MSGS>;

/**
 * @brief
 * Response status for request
 */
enum class MsgStatus : uint8_t
{
    GOOD,
    FAIL,
    ABORT,

    BUFFER_SLOWDOWN,

    COMM_STATUS_QUERY,
    COMM_OK,
    COMM_SUSPENDED,
    COMM_RESTORED,

    DETECT_BUFFER_FULLY_LOADED,
    DETECT_SOUND_EVENT_DETECTED,
    DETECT_SOUND_EVENT_NOT_DETECTED,
};

/**
 * @brief
 * Request message for communication between Tasks
 */
class RequestMsg
{
  public:
    // Default constructor
    RequestMsg() = default;

    // Initialize the request message
    void        initialize(const w_hal_rtc_time_st &time,
                           buffer_id_t              buffer_id,
                           uint16_t                 buffer_length,
                           bool                     flush);

    // Is this message indicates the last in series of requests
    bool        is_flush() const;

    // Get time
    void        get_time(w_hal_rtc_time_st &time) const;

    // Get buffer id
    buffer_id_t get_buffer_id() const;

    // Get buffer length
    uint16_t    get_buffer_length() const;

    // Get buffer information
    void      get_buffer(buffer_id_t &buffer_id, uint16_t &buffer_length) const;

    // Get status of the processed request
    MsgStatus get_status() const;

    // Set response status for processed request
    void      set_status(MsgStatus status);

  private:
    w_hal_rtc_time_st m_time;               // Timestamp
    uint16_t          m_buffer_length : 15; // length of data in buffer
    uint16_t          m_flush         : 1;  // Indicates if the data should be
                                            // flushed after this request
    buffer_id_t       m_buffer_id;          // Buffer id
    MsgStatus         m_resp_status;        // status on response
};

/**
 * @brief
 * Initialize the request message
 *
 * @param time time
 * @param buffer_id id of the data buffer
 * @param buffer_length length of data in buffer
 * @param flush indicates if the buffer should be flushed
 */
inline void RequestMsg::initialize(const w_hal_rtc_time_st &time,
                                   buffer_id_t              buffer_id,
                                   uint16_t                 buffer_length,
                                   bool                     flush)
{
    m_time          = time;
    m_buffer_id     = buffer_id;
    m_buffer_length = buffer_length;
    m_flush         = flush;
    m_resp_status   = MsgStatus::GOOD;
}

/**
 * @brief Is this message indicates the last in series of requests?
 *
 * @return true if message is last
 * @return false otherwise
 */
inline bool RequestMsg::is_flush() const
{
    return (m_flush > 0);
}

/**
 * @brief
 * Get time
 *
 * @param time[out] : time
 */
inline void RequestMsg::get_time(w_hal_rtc_time_st &time) const
{
    time = m_time;
}

/**
 * @brief
 * Get buffer id
 *
 * @return buffer_id_t id of the data buffer
 */
inline buffer_id_t RequestMsg::get_buffer_id() const
{
    return m_buffer_id;
}

/**
 * @brief
 * Get buffer length
 *
 * @return uint16_t length of data in buffer
 */
inline uint16_t RequestMsg::get_buffer_length() const
{
    return m_buffer_length;
}

/**
 * @brief
 * Get buffer information
 *
 * @param buffer_id[out] : id of the data buffer
 * @param buffer_offset[out] : offset in buffer
 * @param buffer_length[out] : length of data in buffer
 */
inline void RequestMsg::get_buffer(buffer_id_t &buffer_id,
                                   uint16_t &   buffer_length) const
{
    buffer_id     = m_buffer_id;
    buffer_length = m_buffer_length;
}

/**
 * @brief
 * Get status of the processed request
 *
 * @return MsgStatus status of response
 */
inline MsgStatus RequestMsg::get_status() const
{
    return m_resp_status;
}

/**
 * @brief
 * Set response status for processed request
 *
 * @param status status of response
 */
inline void RequestMsg::set_status(MsgStatus status)
{
    m_resp_status = status;
}

/**
 * @brief Request message for communication between Audio Task and Comm Task
 */
class CommRequestMsg :
    public RequestMsg
{
  public:
    // Default constructor
    CommRequestMsg() = default;

    // Initialize the request message
    void     initialize(const w_hal_rtc_time_st &time,
                        buffer_id_t              buffer_id,
                        uint16_t                 buffer_length,
                        uint16_t                 remaining_buffers);

    // Is this message indicates the last in series of comm request
    bool     is_last() const;

    // Get remaining buffers after this one in comm request series
    uint16_t get_remaining_buffers() const;

  private:
    uint16_t m_remaining_buffers; // Remaining buffers after this
                                  // one in comm request series.
};

/**
 * @brief
 * Initialize the request message
 *
 * @param time time
 * @param buffer_id id of the data buffer
 * @param buffer_length length of data in buffer
 * @param remaining_buffers remaining buffers after this one in comm request
 * series
 */
inline void CommRequestMsg::initialize(const w_hal_rtc_time_st &time,
                                       buffer_id_t              buffer_id,
                                       uint16_t                 buffer_length,
                                       uint16_t remaining_buffers)
{
    RequestMsg::initialize(time,
                           buffer_id,
                           buffer_length,
                           m_remaining_buffers == 0);
    m_remaining_buffers = remaining_buffers;
}

/**
 * @brief Is this message the last in series of comm requests
 *
 * @return true if message is last
 * @return false otherwise
 */
inline bool CommRequestMsg::is_last() const
{
    return (m_remaining_buffers == 0);
}

/**
 * @brief Get remaining buffers after this one in comm request series
 *
 * @return uint16_t remaining buffers after this one in comm request series
 */
inline uint16_t CommRequestMsg::get_remaining_buffers() const
{
    return m_remaining_buffers;
}

/**
 * @brief Request message for communication between Audio Task and OCM Task
 */
class OCMRequestMsg :
    public RequestMsg
{
  public:
    // Default constructor
    OCMRequestMsg() = default;

    // Initialize the request message
    void     initialize(const w_hal_rtc_time_st &time,
                        buffer_id_t              buffer_id,
                        uint16_t                 buffer_length,
                        bool                     flush,
                        uint32_t                 external_address,
                        bool                     is_write);

    // Get the external memory address to write or read
    uint32_t get_external_address() const;

    // Is this message for write
    bool     is_write() const;

  private:
    uint32_t m_external_address; // External memory address
    bool     m_write;            // Indicates if request is for write
};

/**
 * @brief
 * Initialize the request message
 *
 * @param time time
 * @param buffer_id id of the data buffer
 * @param buffer_length length of data in buffer
 * @param flush indicates if the buffer should be flushed
 * @param external_address external memory address
 * @param is_write indicates if request is for write
 */
inline void OCMRequestMsg::initialize(const w_hal_rtc_time_st &time,
                                      buffer_id_t              buffer_id,
                                      uint16_t                 buffer_length,
                                      bool                     flush,
                                      uint32_t                 external_address,
                                      bool                     is_write)
{
    RequestMsg::initialize(time, buffer_id, buffer_length, flush);
    m_external_address = external_address;
    m_write            = is_write;
}

/**
 * @brief Get the external memory address to write or read
 *
 * @return uint32_t external memory address
 */
inline uint32_t OCMRequestMsg::get_external_address() const
{
    return m_external_address;
}

/**
 * @brief Is this message for write
 *
 * @return true if message is for write
 * @return false otherwise
 */
inline bool OCMRequestMsg::is_write() const
{
    return m_write;
}

/**
 * @brief Request message for communication between Audio Task and Detect Task
 */
class DetectRequestMsg :
    public RequestMsg
{
  public:
    // Default constructor
    DetectRequestMsg() = default;

    // Initialize the request message
    void initialize(const w_hal_rtc_time_st &time,
                    bool                     is_detect,
                    buffer_id_t              buffer_id,
                    uint16_t                 buffer_length,
                    bool                     flush);

    // Is this message for getting detection result?
    bool is_detect_msg() const;

  private:
    bool m_is_detect; // Indicates if request is for write
};

/**
 * @brief
 * Initialize the request message
 *
 * @param time time
 * @param is_detect is the message to retrieve detect result. If so, buffer id
 * should not be provided.
 * @param buffer_id id of the data buffer
 * @param buffer_length length of data in buffer
 * @param flush indicates if the buffer should be flushed
 */
inline void DetectRequestMsg::initialize(const w_hal_rtc_time_st &time,
                                         bool                     is_detect,
                                         buffer_id_t              buffer_id,
                                         uint16_t                 buffer_length,
                                         bool                     flush)
{
    if (is_detect)
    {
        SYSTEM_ASSERT_EQUAL(buffer_id, INVALID_BUFFER_ID);
    }
    else
    {
        SYSTEM_ASSERT_NOT_EQUAL(buffer_id, INVALID_BUFFER_ID);
    }

    RequestMsg::initialize(time, buffer_id, buffer_length, flush);
    m_is_detect = is_detect;
}

/**
 * @brief Is this message for getting detection result?
 *
 * @return true if message is for getting the detection result
 * @return false otherwise
 */
inline bool DetectRequestMsg::is_detect_msg() const
{
    return m_is_detect;
}

// Expand these to be derived classes if specific implementations are needed
using StorageRequestMsg = RequestMsg;
using CommHpRequestMsg  = RequestMsg;

/**
 * @brief
 * Structure containing all queues and messages
 */
struct queues_info_t
{
    DetectFifoQueue   detect_request_queue;    // From audio to detect task
    StorageFifoQueue  storage_request_queue;   // From audio to storage task
    CommFifoQueue     comm_request_queue;      // From audio to comm task
    OCMFifoQueue      ocm_read_request_queue;  // From audio to ocm task
    OCMFifoQueue      ocm_write_request_queue; // From audio to ocm task

    DetectFifoQueue   detect_response_queue;  // From detect to audio task
    StorageFifoQueue  storage_response_queue; // From storage to audio task
    CommFifoQueue     comm_response_queue;    // From comm to audio task
    OCMFifoQueue      ocm_response_queue;     // From ocm to audio task

    CommHpFifoQueue   comm_hp_request_queue;  // From audio to comm task
    CommHpFifoQueue   comm_hp_response_queue; // From comm to audio task

    // Array of Detect messages
    DetectRequestMsg  detect_msgs[NUM_DETECT_REQUEST_MSGS];
    // Array of Storage messages
    StorageRequestMsg storage_msgs[NUM_STORAGE_REQUEST_MSGS];
    // Array of Comm messages
    CommRequestMsg    comm_msgs[NUM_COMM_REQUEST_MSGS];
    // Array of OCM messages
    OCMRequestMsg     ocm_msgs[NUM_OCM_REQUEST_MSGS];
    // Array of HP Comm messages
    CommRequestMsg    comm_hp_msgs[NUM_COMM_HP_REQUEST_MSGS];
};

#endif // _MESSAGES_HPP
