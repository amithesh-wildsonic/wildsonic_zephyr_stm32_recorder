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

    def step(self, x):
        if not self.enabled:
            return x
        out = self.b0 * x + self.s1
        self.s1 = self.b1 * x - self.a1 * out + self.s2
        self.s2 = self.b2 * x - self.a2 * out
        return out

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

def test_dsp(fs=48000):
    stages = [BiquadDF2T() for _ in range(6)]
    calc_dc_block(stages[0], 20.0, fs)
    calc_hpf(stages[1], 80.0, fs, 0.70710678)
    calc_lpf(stages[4], 20000.0, fs, 0.54119610)
    calc_lpf(stages[5], 20000.0, fs, 1.30656296)

    # Generate 100 blocks of pure sine wave at 1000 Hz:
    block_size = 2048
    num_blocks = 100
    t = np.arange(num_blocks * block_size) / fs
    sig = 10000.0 * np.sin(2.0 * np.pi * 1000.0 * t)

    out = np.zeros_like(sig)
    for b in range(num_blocks):
        blk = sig[b * block_size : (b + 1) * block_size]
        out_blk = np.zeros(block_size)
        for i in range(block_size):
            x = blk[i]
            for s in stages:
                x = s.step(x)
            out_blk[i] = x
        out[b * block_size : (b + 1) * block_size] = out_blk

    # Check coherent block average of the output:
    blocks = out[10 * block_size:].reshape((-1, block_size))
    avg_block = np.mean(blocks, axis=0)
    print(f"DSP simulation at fs={fs}:")
    print(f"  Coherent avg block ptp = {np.ptp(avg_block):.4f}, rms = {np.sqrt(np.mean(avg_block**2)):.4f}")

    # Check FFT
    fft_out = np.abs(np.fft.rfft(out[10 * block_size:]))
    freqs = np.fft.rfftfreq(len(out[10 * block_size:]), 1.0 / fs)
    f_block = fs / block_size
    print(f"  Power at 1 kHz: {np.max(fft_out):.1f}")
    # Power at f_block harmonics
    harm_powers = []
    for k in range(1, 10):
        idx = np.argmin(np.abs(freqs - k * f_block))
        harm_powers.append(fft_out[idx])
    print(f"  Powers at first 9 harmonics of f_block: {harm_powers}")

if __name__ == '__main__':
    test_dsp(48000)
    test_dsp(192000)

