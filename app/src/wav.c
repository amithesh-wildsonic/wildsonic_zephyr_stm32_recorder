/* SPDX-License-Identifier: Apache-2.0 */
#include <string.h>
#include "wav.h"

_Static_assert((WAV_HEADER_BYTES % 512U) == 0U, "WAV header must be 512-byte sector aligned");
_Static_assert(WAV_HEADER_BYTES >= 512U, "WAV header too small");

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
	p[2] = (v >> 16) & 0xff;
	p[3] = (v >> 24) & 0xff;
}

static void put16(uint8_t *p, uint16_t v)
{
	p[0] = v & 0xff;
	p[1] = (v >> 8) & 0xff;
}

void wav_fill_header(uint8_t hdr[WAV_HEADER_BYTES], uint32_t sample_rate,
		     uint16_t channels, uint16_t bits_per_sample, uint32_t data_bytes)
{
	const uint16_t block_align = channels * (bits_per_sample / 8);
	/* 12 (RIFF) + 24 (fmt) + 8 (JUNK hdr) + junk + 8 (data hdr) = 512 */
	const uint32_t junk_bytes = WAV_HEADER_BYTES - 12 - 24 - 8 - 8;

	memset(hdr, 0, WAV_HEADER_BYTES);

	memcpy(hdr + 0, "RIFF", 4);
	put32(hdr + 4, (WAV_HEADER_BYTES - 8) + data_bytes);
	memcpy(hdr + 8, "WAVE", 4);

	memcpy(hdr + 12, "fmt ", 4);
	put32(hdr + 16, 16);
	put16(hdr + 20, 1);			/* PCM */
	put16(hdr + 22, channels);
	put32(hdr + 24, sample_rate);
	put32(hdr + 28, sample_rate * block_align);
	put16(hdr + 32, block_align);
	put16(hdr + 34, bits_per_sample);

	memcpy(hdr + 36, "JUNK", 4);
	put32(hdr + 40, junk_bytes);
	/* junk payload is already zero */

	memcpy(hdr + 36 + 8 + junk_bytes, "data", 4);
	put32(hdr + 36 + 8 + junk_bytes + 4, data_bytes);
}

static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t get16(const uint8_t *p)
{
	return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

int wav_verify_header(const uint8_t hdr[WAV_HEADER_BYTES], uint32_t expected_rate,
		      uint16_t expected_channels, uint16_t expected_bits, uint32_t expected_data_bytes)
{
	if (expected_rate == 0U || (expected_channels != 1U && expected_channels != 2U) ||
	    (expected_bits != 16U && expected_bits != 32U)) {
		return -16;
	}
	const uint16_t block_align = expected_channels * (expected_bits / 8U);
	if ((expected_data_bytes % block_align) != 0U) {
		return -17;
	}
	if (expected_data_bytes > (UINT32_MAX - (WAV_HEADER_BYTES - 8U))) {
		return -18;
	}
	const uint32_t junk_bytes = WAV_HEADER_BYTES - 12 - 24 - 8 - 8;

	if (memcmp(hdr + 0, "RIFF", 4) != 0) {
		return -1;
	}
	uint32_t riff_size = get32(hdr + 4);
	if (riff_size != (WAV_HEADER_BYTES - 8) + expected_data_bytes) {
		return -2;
	}
	if (memcmp(hdr + 8, "WAVE", 4) != 0) {
		return -3;
	}
	if (memcmp(hdr + 12, "fmt ", 4) != 0) {
		return -4;
	}
	if (get32(hdr + 16) != 16) {
		return -5;
	}
	if (get16(hdr + 20) != 1) { /* PCM */
		return -6;
	}
	if (get16(hdr + 22) != expected_channels) {
		return -7;
	}
	if (get32(hdr + 24) != expected_rate) {
		return -8;
	}
	if (get32(hdr + 28) != expected_rate * block_align) {
		return -9;
	}
	if (get16(hdr + 32) != block_align) {
		return -10;
	}
	if (get16(hdr + 34) != expected_bits) {
		return -11;
	}
	if (memcmp(hdr + 36, "JUNK", 4) != 0) {
		return -12;
	}
	if (get32(hdr + 40) != junk_bytes) {
		return -13;
	}
	if (memcmp(hdr + 36 + 8 + junk_bytes, "data", 4) != 0) {
		return -14;
	}
	if (get32(hdr + 36 + 8 + junk_bytes + 4) != expected_data_bytes) {
		return -15;
	}

	return 0;
}

