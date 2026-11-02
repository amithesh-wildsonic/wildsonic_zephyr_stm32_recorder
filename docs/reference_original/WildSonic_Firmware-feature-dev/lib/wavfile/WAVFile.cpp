#include "WAVFile.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

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
                          int32_t       longitude_e7)
{
    memcpy(wav_header.riff_chunk_descriptor.id,
           WAV_RIFF_CHUNK_ID,
           WAV_RIFF_CHUNK_ID_LEN);
    memcpy(wav_header.riff_chunk_descriptor.format,
           WAV_RIFF_CHUNK_FORMAT,
           WAV_RIFF_CHUNK_FORMAT_LEN);
    memcpy(wav_header.env_subchunk.id,
           WAV_ENV_SUBCHUNK_ID,
           WAV_ENV_SUBCHUNK_ID_LEN);
    memcpy(wav_header.batt_subchunk.id,
           WAV_BATT_SUBCHUNK_ID,
           WAV_BATT_SUBCHUNK_ID_LEN);
    memcpy(wav_header.loc_subchunk.id,
           WAV_LOC_SUBCHUNK_ID,
           WAV_LOC_SUBCHUNK_ID_LEN);

    memcpy(wav_header.format_subchunk.id,
           WAV_FORMAT_SUBCHUNK_ID,
           WAV_FORMAT_SUBCHUNK_ID_LEN);
    wav_header.format_subchunk.size         = WAV_FORMAT_SUBCHUNK_SIZE;

    wav_header.format_subchunk.audio_format = AUDIO_FORMAT_PCM;
    wav_header.format_subchunk.num_channels = num_channels;
    wav_header.format_subchunk.sample_rate  = sample_rate;
    wav_header.format_subchunk.byte_rate =
        sample_rate * num_channels * (bits_per_sample >> 3);
    wav_header.format_subchunk.block_align =
        num_channels * (bits_per_sample >> 3);
    wav_header.format_subchunk.bits_per_sample = bits_per_sample;
    wav_header.env_subchunk.size = sizeof(wav_header.env_subchunk.temperature) +
                                   sizeof(wav_header.env_subchunk.humidity);
    wav_header.env_subchunk.temperature = temperature;
    wav_header.env_subchunk.humidity    = humidity;

    wav_header.batt_subchunk.size =
        sizeof(wav_header.batt_subchunk.voltage) +
        sizeof(wav_header.batt_subchunk.current) +
        sizeof(wav_header.batt_subchunk.remaining_capacity) +
        sizeof(wav_header.batt_subchunk.average_current) +
        sizeof(wav_header.batt_subchunk.time_to_empty);

    wav_header.batt_subchunk.voltage            = voltage;
    wav_header.batt_subchunk.current            = current;
    wav_header.batt_subchunk.remaining_capacity = remaining_capacity;
    wav_header.batt_subchunk.average_current    = average_current;
    wav_header.batt_subchunk.time_to_empty      = time_to_empty;

    wav_header.loc_subchunk.size = sizeof(wav_header.loc_subchunk.latitude_e7) +
                                   sizeof(wav_header.loc_subchunk.longitude_e7);
    wav_header.loc_subchunk.latitude_e7  = latitude_e7;
    wav_header.loc_subchunk.longitude_e7 = longitude_e7;

    memcpy(wav_header.data_subchunk_header.id,
           WAV_DATA_SUBCHUNK_ID,
           WAV_DATA_SUBCHUNK_ID_LEN);
}

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
                            int32_t             longitude_e7)
{
    if (memcmp(wav_header.riff_chunk_descriptor.id,
               WAV_RIFF_CHUNK_ID,
               WAV_RIFF_CHUNK_ID_LEN) != 0)
    {
        return false;
    }

    if (memcmp(wav_header.riff_chunk_descriptor.format,
               WAV_RIFF_CHUNK_FORMAT,
               WAV_RIFF_CHUNK_FORMAT_LEN) != 0)
    {
        return false;
    }

    if (memcmp(wav_header.env_subchunk.id,
               WAV_ENV_SUBCHUNK_ID,
               WAV_ENV_SUBCHUNK_ID_LEN) != 0)
    {
        return false;
    }

    if (wav_header.env_subchunk.size !=
        (sizeof(wav_header.env_subchunk.temperature) +
         sizeof(wav_header.env_subchunk.humidity)))
    {
        return false;
    }

    if (memcmp(wav_header.batt_subchunk.id,
               WAV_BATT_SUBCHUNK_ID,
               WAV_BATT_SUBCHUNK_ID_LEN) != 0)
    {
        return false;
    }

    if (wav_header.batt_subchunk.size !=
        sizeof(wav_header.batt_subchunk.voltage) +
            sizeof(wav_header.batt_subchunk.current) +
            sizeof(wav_header.batt_subchunk.remaining_capacity) +
            sizeof(wav_header.batt_subchunk.average_current) +
            sizeof(wav_header.batt_subchunk.time_to_empty))
    {
        return false;
    }

    if (memcmp(wav_header.loc_subchunk.id,
               WAV_LOC_SUBCHUNK_ID,
               WAV_LOC_SUBCHUNK_ID_LEN) != 0)
    {
        return false;
    }

    if (wav_header.loc_subchunk.size !=
        (sizeof(wav_header.loc_subchunk.latitude_e7) +
         sizeof(wav_header.loc_subchunk.longitude_e7)))
    {
        return false;
    }

    if (memcmp(wav_header.format_subchunk.id,
               WAV_FORMAT_SUBCHUNK_ID,
               WAV_FORMAT_SUBCHUNK_ID_LEN) != 0)
    {
        return false;
    }

    if (wav_header.format_subchunk.size != WAV_FORMAT_SUBCHUNK_SIZE)
    {
        return false;
    }

    if (wav_header.format_subchunk.audio_format != AUDIO_FORMAT_PCM)
    {
        return false;
    }

    if (wav_header.format_subchunk.num_channels != num_channels)
    {
        return false;
    }

    if (wav_header.format_subchunk.sample_rate != sample_rate)
    {
        return false;
    }

    if (wav_header.format_subchunk.byte_rate !=
        sample_rate * num_channels * (bits_per_sample >> 3))
    {
        return false;
    }

    if (wav_header.format_subchunk.block_align !=
        num_channels * (bits_per_sample >> 3))
    {
        return false;
    }

    if (wav_header.format_subchunk.bits_per_sample != bits_per_sample)
    {
        return false;
    }

    if (wav_header.env_subchunk.temperature != temperature)
    {
        return false;
    }

    if (wav_header.env_subchunk.humidity != humidity)
    {
        return false;
    }

    if (wav_header.batt_subchunk.voltage != voltage)
    {
        return false;
    }

    if (wav_header.batt_subchunk.current != current)
    {
        return false;
    }

    if (wav_header.batt_subchunk.remaining_capacity != remaining_capacity)
    {
        return false;
    }

    if (wav_header.batt_subchunk.average_current != average_current)
    {
        return false;
    }

    if (wav_header.batt_subchunk.time_to_empty != time_to_empty)
    {
        return false;
    }

    if (wav_header.loc_subchunk.latitude_e7 != latitude_e7)
    {
        return false;
    }

    if (wav_header.loc_subchunk.longitude_e7 != longitude_e7)
    {
        return false;
    }

    if (memcmp(wav_header.data_subchunk_header.id,
               WAV_DATA_SUBCHUNK_ID,
               WAV_DATA_SUBCHUNK_ID_LEN) != 0)
    {
        return false;
    }

    return true;
}

/**
 * @brief
 * Set the total size of the sample data in the wav header
 *
 * @param wav_header [out] - header to populate
 * @param data_size total size of the sample data
 */
void wav_file_set_data_size(wav_header_t &wav_header, uint32_t data_size)
{
    wav_header.riff_chunk_descriptor.chunk_size =
        sizeof(wav_header_t) - sizeof(wav_header.riff_chunk_descriptor.id) -
        sizeof(wav_header.riff_chunk_descriptor.chunk_size) + data_size;
    wav_header.data_subchunk_header.size = data_size;
}

/**
 * @brief
 * Verify the total size of the sample data in the wav header
 *
 * @param wav_header - header to verify
 * @param data_size total size of the sample data
 * @return true if no error found, false otherwise
 */
bool wav_file_verify_data_size(const wav_header_t &wav_header,
                               uint32_t            data_size)
{
    if (wav_header.riff_chunk_descriptor.chunk_size !=
        sizeof(wav_header_t) - sizeof(wav_header.riff_chunk_descriptor.id) -
            sizeof(wav_header.riff_chunk_descriptor.chunk_size) + data_size)
    {
        return false;
    }

    if (wav_header.data_subchunk_header.size != data_size)
    {
        return false;
    }

    return true;
}
