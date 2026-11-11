/* SPDX-License-Identifier: Apache-2.0 */
#ifndef WAV_H_
#define WAV_H_

#include <stdint.h>

/*
 * The WAV header is padded to cluster alignment (32768 bytes) so the audio data starts
 * on a cluster boundary (multiple of 32 KB). FatFS can then write whole 32 KB clusters
 * straight from our buffers without splitting across cluster boundaries.
 * Extra space is a standard 'JUNK' chunk that every WAV reader skips.
 */
#define WAV_HEADER_BYTES 32768

/* Build the header for 16-bit PCM. data_bytes may be 0 while still recording. */
void wav_fill_header(uint8_t hdr[WAV_HEADER_BYTES], uint32_t sample_rate,
		     uint16_t channels, uint16_t bits_per_sample, uint32_t data_bytes);

/* Validate header fields against expected format and size. Returns 0 on success, negative on error. */
int wav_verify_header(const uint8_t hdr[WAV_HEADER_BYTES], uint32_t expected_rate,
		      uint16_t expected_channels, uint16_t expected_bits, uint32_t expected_data_bytes);

#endif
