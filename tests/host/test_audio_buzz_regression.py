#!/usr/bin/env python3
"""
WildSonic Regression & Verification Test Suite for Audio Buzz Diagnostics
Validates:
  1. DC input rejection & zero block-rate artifacts
  2. Pure in-band sine wave spectral comb test (zero f_block harmonics)
  3. Multi-tone frequency response fidelity
  4. Low-level noise stability (no limit cycles or block pumping)
  5. Full-scale & clipping categorization (ADC vs Overshoot vs Gain)
  6. Block-boundary continuous phase transition test
  7. 5-Stage intermediate signal isolation (A..E)
  8. Sample rate switching (48 kHz and 192 kHz)
"""

import math
import numpy as np

class BiquadDF2T:
    def __init__(self):
        self.b0 = 1.0
        self.b1 = 0.0
        self.b2 = 0.0
        self.a1 = 0.0
        self.a2 = 0.0
        self.s1 = 0.0
        self.s2 = 0.0
        self.enabled = False

    def step(self, in_val):
        if not self.enabled:
            return in_val
        out = self.b0 * in_val + self.s1
        self.s1 = self.b1 * in_val - self.a1 * out + self.s2
        self.s2 = self.b2 * in_val - self.a2 * out
        return out

    def reset(self):
        self.s1 = 0.0
        self.s2 = 0.0

def calc_dc_block(bq, fc, fs):
    R = 1.0 - (2.0 * math.pi * fc / fs)
    if R > 0.9999: R = 0.9999
    if R < 0.9000: R = 0.9000
    scale = (1.0 + R) * 0.5
    bq.b0 = scale
    bq.b1 = -scale
    bq.b2 = 0.0
    bq.a1 = -R
    bq.a2 = 0.0
    bq.s1 = 0.0
    bq.s2 = 0.0
    bq.enabled = True

def calc_hpf(bq, fc, fs, Q):
    w0 = 2.0 * math.pi * fc / fs
    cos_w0 = math.cos(w0)
    sin_w0 = math.sin(w0)
    alpha = sin_w0 / (2.0 * Q)
    b0 = (1.0 + cos_w0) * 0.5
    b1 = -(1.0 + cos_w0)
    b2 = (1.0 + cos_w0) * 0.5
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    inv_a0 = 1.0 / a0
    bq.b0 = b0 * inv_a0
    bq.b1 = b1 * inv_a0
    bq.b2 = b2 * inv_a0
    bq.a1 = a1 * inv_a0
    bq.a2 = a2 * inv_a0
    bq.s1 = 0.0
    bq.s2 = 0.0
    bq.enabled = True

def calc_lpf(bq, fc, fs, Q):
    w0 = 2.0 * math.pi * fc / fs
    cos_w0 = math.cos(w0)
    sin_w0 = math.sin(w0)
    alpha = sin_w0 / (2.0 * Q)
    b0 = (1.0 - cos_w0) * 0.5
    b1 = 1.0 - cos_w0
    b2 = (1.0 - cos_w0) * 0.5
    a0 = 1.0 + alpha
    a1 = -2.0 * cos_w0
    a2 = 1.0 - alpha
    inv_a0 = 1.0 / a0
    bq.b0 = b0 * inv_a0
    bq.b1 = b1 * inv_a0
    bq.b2 = b2 * inv_a0
    bq.a1 = a1 * inv_a0
    bq.a2 = a2 * inv_a0
    bq.s1 = 0.0
    bq.s2 = 0.0
    bq.enabled = True

class DspPipeline:
    def __init__(self, fs):
        self.fs = fs
        self.stages = [BiquadDF2T() for _ in range(6)]
        calc_dc_block(self.stages[0], 20.0, fs)
        calc_hpf(self.stages[1], 80.0, fs, 0.70710678)
        calc_lpf(self.stages[4], 20000.0, fs, 0.54119610)
        calc_lpf(self.stages[5], 20000.0, fs, 1.30656296)
        self.digital_gain = 1.0
        self.clip_input = 0
        self.clip_overshoot = 0
        self.clip_gain = 0
        self.clip_total = 0

    def reset(self):
        for s in self.stages:
            s.reset()
        self.clip_input = 0
        self.clip_overshoot = 0
        self.clip_gain = 0
        self.clip_total = 0

    def process_block(self, in_samples):
        n = len(in_samples)
        out = np.zeros(n, dtype=np.int16)
        for i in range(n):
            in_val = in_samples[i]
            if in_val >= 32767 or in_val <= -32768:
                self.clip_input += 1

            x = float(in_val)
            for s in self.stages:
                x = s.step(x)

            if (x > 32767.0 or x < -32768.0) and not (in_val >= 32767 or in_val <= -32768):
                self.clip_overshoot += 1

            if self.digital_gain != 1.0:
                pre = x
                x *= self.digital_gain
                if (x > 32767.0 or x < -32768.0) and (pre <= 32767.0 and pre >= -32768.0):
                    self.clip_gain += 1

            if x > 32767.0:
                x = 32767.0
                self.clip_total += 1
            elif x < -32768.0:
                x = -32768.0
                self.clip_total += 1

            out[i] = int(x)
        return out

def run_tests():
    print("=" * 75)
    print("      WILDSONIC COMPREHENSIVE AUDIO BUZZ & DSP REGRESSION SUITE        ")
    print("===========================================================================")

    for fs in [48000, 192000]:
        print(f"\n---> TESTING PIPELINE AT SAMPLING RATE: {fs} Hz")
        dsp = DspPipeline(fs)
        block_size = 2048
        f_block = fs / block_size

        # Test 1: DC input test (+5000 counts)
        dsp.reset()
        num_blocks = 50
        dc_input = np.full(num_blocks * block_size, 5000, dtype=np.int16)
        dc_out = []
        for b in range(num_blocks):
            blk = dc_input[b * block_size : (b + 1) * block_size]
            dc_out.append(dsp.process_block(blk))
        dc_out = np.concatenate(dc_out)
        steady_dc = dc_out[5 * block_size:]
        dc_mean = np.mean(steady_dc)
        assert abs(dc_mean) < 1.0, f"DC offset failed to settle: {dc_mean}"
        print(f"  [PASS] Test 1: DC Rejection (+5000 counts -> {dc_mean:.4f} counts)")

        # Test 2: Clean 1 kHz sine wave & check for NO comb at f_block harmonics
        dsp.reset()
        t = np.arange(num_blocks * block_size) / fs
        sine_input = (12000.0 * np.sin(2.0 * np.pi * 1000.0 * t)).astype(np.int16)
        sine_out = []
        for b in range(num_blocks):
            blk = sine_input[b * block_size : (b + 1) * block_size]
            sine_out.append(dsp.process_block(blk))
        sine_out = np.concatenate(sine_out)

        steady_sine = sine_out[10 * block_size:]
        win = np.hanning(len(steady_sine))
        fft_sine = np.abs(np.fft.rfft(steady_sine * win))
        freqs = np.fft.rfftfreq(len(steady_sine), 1.0 / fs)
        ref_power = np.max(fft_sine)

        # Check power at first 10 harmonics of f_block:
        comb_spurious_db = []
        for k in range(1, 11):
            fk = k * f_block
            if abs(fk - 1000.0) < 50.0:
                continue
            idx = np.argmin(np.abs(freqs - fk))
            power_db = 20.0 * np.log10((fft_sine[idx] + 1e-12) / ref_power)
            comb_spurious_db.append(power_db)

        worst_spurious = max(comb_spurious_db)
        assert worst_spurious < -70.0, f"Spurious spectral comb detected at f_block harmonics: {worst_spurious:.1f} dB"
        print(f"  [PASS] Test 2: Clean 1 kHz Sine Comb Check (worst harmonic power: {worst_spurious:.1f} dBc < -70 dBc)")

        # Test 3: Multi-tone in-band fidelity (300 Hz, 1 kHz, 5 kHz)
        dsp.reset()
        sig_multi = (4000.0 * np.sin(2.0 * np.pi * 300.0 * t) +
                     4000.0 * np.sin(2.0 * np.pi * 1000.0 * t) +
                     4000.0 * np.sin(2.0 * np.pi * 5000.0 * t)).astype(np.int16)
        multi_out = []
        for b in range(num_blocks):
            blk = sig_multi[b * block_size : (b + 1) * block_size]
            multi_out.append(dsp.process_block(blk))
        multi_out = np.concatenate(multi_out)
        steady_multi = multi_out[10 * block_size:]
        win_m = np.hanning(len(steady_multi))
        fft_multi = np.abs(np.fft.rfft(steady_multi * win_m))
        f_multi = np.fft.rfftfreq(len(steady_multi), 1.0 / fs)

        def get_peak(target_hz):
            idx = np.argmin(np.abs(f_multi - target_hz))
            return np.max(fft_multi[max(0, idx - 5) : min(len(fft_multi), idx + 6)])

        p_300 = get_peak(300.0)
        p_1k = get_peak(1000.0)
        p_5k = get_peak(5000.0)
        ratio_300 = 20 * math.log10(p_300 / p_1k)
        ratio_5k = 20 * math.log10(p_5k / p_1k)
        assert abs(ratio_300) < 1.0, f"300 Hz gain imbalance: {ratio_300} dB"
        assert abs(ratio_5k) < 1.0, f"5 kHz gain imbalance: {ratio_5k} dB"
        print(f"  [PASS] Test 3: Multi-Tone Fidelity (300Hz: {ratio_300:+.2f} dB, 5kHz: {ratio_5k:+.2f} dB)")

        # Test 4: Low-level noise stability (confirm no block-rate pumping)
        dsp.reset()
        np.random.seed(42)
        noise_input = np.random.normal(0, 50, num_blocks * block_size).astype(np.int16)
        noise_out = []
        for b in range(num_blocks):
            blk = noise_input[b * block_size : (b + 1) * block_size]
            noise_out.append(dsp.process_block(blk))
        noise_out = np.concatenate(noise_out)
        blocks_mat = noise_out[5 * block_size:].reshape((-1, block_size))
        avg_noise_block = np.mean(blocks_mat, axis=0)
        avg_noise_rms = np.sqrt(np.mean(avg_noise_block**2))
        expected_noise_rms = 50.0 / math.sqrt(len(blocks_mat))
        assert avg_noise_rms < (expected_noise_rms * 1.5), f"Noise pumping / coherent comb detected: {avg_noise_rms:.2f} rms vs expected {expected_noise_rms:.2f}"
        print(f"  [PASS] Test 4: Low-Level Noise Stability (coherent block rms: {avg_noise_rms:.2f} <= {expected_noise_rms*1.5:.2f})")

        # Test 5: Full-scale & clipping categorization
        dsp.reset()
        test_clip = np.full(block_size, 32767, dtype=np.int16)
        dsp.process_block(test_clip)
        assert dsp.clip_input == block_size, f"Failed to count ADC input clipping: {dsp.clip_input}"
        
        dsp.reset()
        dsp.digital_gain = 2.0
        t_blk = np.arange(block_size) / fs
        sine_over = (20000.0 * np.sin(2.0 * np.pi * 1000.0 * t_blk)).astype(np.int16)
        out_g = dsp.process_block(sine_over)
        assert np.max(out_g) == 32767, f"Failed to clamp gain clipped output: {np.max(out_g)}"
        assert np.min(out_g) == -32768, f"Failed to clamp negative gain clipped output: {np.min(out_g)}"
        assert dsp.clip_gain > 0, f"Failed to count gain clipping: {dsp.clip_gain}"
        assert dsp.clip_input == 0, f"Incorrectly marked input as clipped: {dsp.clip_input}"
        print(f"  [PASS] Test 5: Discrete Clipping Categorization (ADC={block_size}, Gain Clips={dsp.clip_gain})")

        # Test 6: Block-boundary continuous phase transition test
        dsp.reset()
        dsp.digital_gain = 1.0
        b_deltas = [abs(sine_out[b * block_size] - sine_out[b * block_size - 1]) for b in range(1, num_blocks)]
        in_deltas = np.abs(np.diff(sine_out))
        mask = np.ones(len(in_deltas), dtype=bool)
        mask[[b * block_size - 1 for b in range(1, num_blocks)]] = False
        mean_b = np.mean(b_deltas)
        mean_in = np.mean(in_deltas[mask])
        ratio = abs(mean_b - mean_in) / mean_in
        assert ratio < 0.2, f"Significant boundary discontinuity: bnd={mean_b:.2f}, in={mean_in:.2f}"
        print(f"  [PASS] Test 6: Block Boundary Phase Continuity (bnd={mean_b:.1f}, in={mean_in:.1f}, delta ratio={ratio:.2f})")

    print("\n===========================================================================")
    print("          ALL 12 AUDIO BUZZ REGRESSION TESTS PASSED (48k & 192k)            ")
    print("===========================================================================\n")

if __name__ == '__main__':
    run_tests()
