#ifndef _WAV_FILE_HPP
#define _WAV_FILE_HPP

#include <stdint.h>

constexpr char     WAV_RIFF_CHUNK_ID[]        = "RIFF";
constexpr uint32_t WAV_RIFF_CHUNK_ID_LEN      = 4;

constexpr char     WAV_RIFF_CHUNK_FORMAT[]    = "WAVE";
constexpr uint32_t WAV_RIFF_CHUNK_FORMAT_LEN  = 4;

constexpr char     WAV_ENV_SUBCHUNK_ID[]      = "env ";
constexpr uint32_t WAV_ENV_SUBCHUNK_ID_LEN    = 4;

constexpr char     WAV_BATT_SUBCHUNK_ID[]     = "batt";
constexpr uint32_t WAV_BATT_SUBCHUNK_ID_LEN   = 4;

constexpr char     WAV_LOC_SUBCHUNK_ID[]      = "loc ";
constexpr uint32_t WAV_LOC_SUBCHUNK_ID_LEN    = 4;

constexpr char     WAV_FORMAT_SUBCHUNK_ID[]   = "fmt ";
constexpr uint32_t WAV_FORMAT_SUBCHUNK_ID_LEN = 4;

constexpr char     WAV_DATA_SUBCHUNK_ID[]     = "data";
constexpr uint32_t WAV_DATA_SUBCHUNK_ID_LEN   = 4;

constexpr uint32_t AUDIO_FORMAT_PCM           = 1;
constexpr uint32_t WAV_FORMAT_SUBCHUNK_SIZE   = 16;

/**
 * @brief
 * RIFF chunk descriptor
 * This indicates that the file is a WAV file
 */
struct wav_riff_chunk_descriptor_t
{
    char     id[4];      // Should be "RIFF"
    uint32_t chunk_size; // Equals 4 + (8 + format sub chunk size) + (8 + data
                         // sub chunk size)
    char     format[4];  // Should be "WAVE"
};

/**
 * @brief
 * Format subchunk
 * Provides information on how to interpret the sound data
 */
struct wav_format_subchunk_t
{
    char     id[4];        // Should be "fmt "
    uint32_t size;         // Size of rest of subchunk after this field
    uint16_t audio_format; // PCM = 1, Greater than 1 indicates compression
    uint16_t num_channels; // 1 for Mono, 2 for Stereo, etc.
    uint32_t sample_rate;  // Sample rate
    uint32_t byte_rate;    // = sample_rate * num_channels * bits_per_sample/8
    uint16_t block_align;  // = num_channels * bits_per_sample/8
    uint16_t bits_per_sample; // Bits per sample
};

/**
 * @brief
 * Env subchunk
 * Provides information on how to interpret the environment data
 */
struct wav_env_subchunk_t
{
    char     id[4];       // Should be "env "
    uint32_t size;        // Size of rest of subchunk after this field
    uint16_t temperature; // Temperature in Celsius
    uint16_t humidity;    // Relative humidity in %
};

/*
 * Battery subchunk
 * Contains battery information
 */
struct wav_batt_subchunk_t
{
    char     id[4];              // Should be "batt"
    uint32_t size;               // Size of rest of subchunk after this field
    int32_t  voltage;            // in Millivolts
    int32_t  current;            // in milliampere
    int32_t  remaining_capacity; // in milliampere-hours
    int32_t  average_current;    // in milliampere
    int32_t  time_to_empty;      // in seconds
};

/**
 * @brief
 * Location subchunk
 * Contains device location information
 */
struct wav_loc_subchunk_t
{
    char     id[4];        // Should be location id
    uint32_t size;         // Size of rest of subchunk after this field
    int32_t  latitude_e7;  // Latitude in degrees scaled by 1e7
    int32_t  longitude_e7; // Longitude in degrees scaled by 1e7
};

/**
 * @brief
 * Data subchunk header
 * Actual sound data not included here.
 * Only the prefix to the sound data is in this
 * structure.
 */
struct wav_data_subchunk_header_t
{
    char     id[4]; // Should be "data"
    uint32_t size;  // = num_samples * num_channels * bits_per_sample/8
};

/**
 * @brief
 * Header that precedes RAW data in WAV file
 */
struct wav_header_t
{
    wav_riff_chunk_descriptor_t
        riff_chunk_descriptor;             // Indicates that this is a WAV file
    wav_format_subchunk_t format_subchunk; // format of sound data
    wav_env_subchunk_t    env_subchunk;    // environment data
    wav_batt_subchunk_t   batt_subchunk;   // battery info subchunk
    wav_loc_subchunk_t    loc_subchunk;    // location info subchunk
    wav_data_subchunk_header_t
        data_subchunk_header; // had size of sound data and sound data
};

/**
 * @brief
 * Initialize the header structure for the wav file
 *
 * @param wav_header [out] - header to populate
 * @param num_channels number of input channels (1 for mono, 2 for stereo)
 * @param sample_rate sample rate in Hz
 * @param bits_per_sample sample size in bits
 * @param temperature temperature in Celsius
 * @param humidity relative humidity in %
 * @param voltage battery voltage in Millivolts
 * @param current battery current in milliampere
 * @param remaining_capacity battery remaining capacity in milliampere-hours
 * @param average_current battery average current in milliampere
 * @param time_to_empty battery time to empty in seconds
 * @param latitude_e7 latitude in degrees scaled by 1e7
 * @param longitude_e7 longitude in degrees scaled by 1e7
 */
void wav_file_init_header(wav_header_t &wav_header,
                          uint16_t      num_channels,
                          uint32_t      sample_rate,
                          uint32_t      bits_per_sample,
                          uint16_t      temperature,
                          uint16_t      humidity,
                          int32_t       voltage,
                          int32_t       current,
                          int32_t       remaining_capacity,
                          int32_t       average_current,
                          int32_t       time_to_empty,
                          int32_t       latitude_e7,
                          int32_t       longitude_e7);

/**
 * @brief
 * Verify the contents of header structure for the wav file
 *
 * @param wav_header [out] - header to populate
 * @param num_channels number of input channels (1 for mono, 2 for stereo)
 * @param sample_rate sample rate in Hz
 * @param bits_per_sample sample size in bits
 * @param temperature temperature in Celsius
 * @param humidity relative humidity in %
 * @param voltage battery voltage in Millivolts
 * @param current battery current in milliampere
 * @param remaining_capacity battery remaining capacity in milliampere-hours
 * @param average_current battery average current in milliampere
 * @param time_to_empty battery time to empty in seconds
 * @return true if no error found, false otherwise
 * @param latitude_e7 latitude in degrees scaled by 1e7
 * @param longitude_e7 longitude in degrees scaled by 1e7
 */
bool wav_file_verify_header(const wav_header_t &wav_header,
                            uint16_t            num_channels,
                            uint32_t            sample_rate,
                            uint32_t            bits_per_sample,
                            uint16_t            temperature,
                            uint16_t            humidity,
                            int32_t             voltage,
                            int32_t             current,
                            int32_t             remaining_capacity,
                            int32_t             average_current,
                            int32_t             time_to_empty,
                            int32_t             latitude_e7,
                            int32_t             longitude_e7);

/**
 * @brief
 * Set the total size of the sample data in the wav header
 *
 * @param wav_header [out] - header to populate
 * @param data_size total size of the sample data
 */
void wav_file_set_data_size(wav_header_t &wav_header, uint32_t data_size);

/**
 * @brief
 * Verify the total size of the sample data in the wav header
 *
 * @param wav_header - header to verify
 * @param data_size total size of the sample data
 * @return true if no error found, false otherwise
 */
bool wav_file_verify_data_size(const wav_header_t &wav_header,
                               uint32_t            data_size);

#endif // _WAV_FILE_HPP
