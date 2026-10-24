# Hardware Integration & In-The-Loop Testing

This directory documents the hardware verification procedures for the WildSonic STM32L476RG recorder.

## Test Target Builds
1. **Benchmark Suite (`build/bench`):**
   - Built with `configs/test_benchmark.conf`.
   - Exercises internal SRAM, QUADSPI IS66WVS4M8 PSRAM, and microSD card write throughput.
   - Run console commands: `bench ram`, `bench psram`, `bench sd`, `bench pipeline`.
2. **Deterministic PCM Integrity Replay (`build/integrity`):**
   - Built with `configs/test_integrity_replay.conf`.
   - Injects deterministic pseudo-random PCM into the DMA streaming pipeline.
   - Run console command: `integrity replay 10` or `integrity adc 10`.
   - Reads back file from microSD and validates 100% bit-exact CRC32 match against golden polynomial.
3. **Stage Isolation Signal Diagnostics (`build/diag`):**
   - Built with `configs/test_diagnostics.conf`.
   - Taps raw ADC, DC blocker output, HPF output, and LPF output simultaneously in real time.
   - Run console command: `diagnose 5`.

