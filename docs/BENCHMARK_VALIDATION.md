# STM32L476RG firmware audit and validation

## Reproducible local evidence

- Target: `nucleo_l476rg/stm32l476xx`; Zephyr `4.5.0-rc2` (`a915b2f`); Zephyr SDK `1.0.1`, GNU Arm 14.3.0.
- Clean command: `west build --pristine always --board nucleo_l476rg/stm32l476xx --build-dir build/audit_final --source-dir .` (PowerShell environment selected Zephyr base/toolchain and Ninja).
- Result: linked `build/audit_final/zephyr/zephyr.elf`; FLASH 221,628 B / 1 MiB (21.14%); configured RAM about 96,226 B / 96 KiB (97.88%).
- Host suite: `python -m pytest -q tools` — 25 passed (latest run repeated twice). The staging interleaving checks are deterministic state-machine tests plus source-order assertions; they do not execute Zephyr C or emulate QSPI electrical faults.
- The app directory has no Git metadata, so there is no app branch/status or commit to report. The shared Zephyr tree has a pre-existing local edit to `drivers/disk/sdmmc_stm32.c`; it was not changed by this audit. The audit changes are retained in the workspace and patch artifact.
- No board was attached. No UART run, repeated cold boot, SD benchmark, QSPI waveform, physical pin/voltage check, or measured performance result is claimed.

## Datasheet-level configuration review

| Requirement | Firmware configuration | Source-level verification | Result / uncertainty |
|---|---|---|---|
| STM32L476RG QUADSPI pins | AF10: PB10 CLK, PB11 NCS, PB1 IO0, PB0 IO1, PA7 IO2, PA6 IO3 | PSRAM GPIO setup and ST STM32L476 datasheet AF table | Pin mux mapping agrees. PCB-to-package wiring and signal quality remain physically unverified. |
| PSRAM identity | MFID 0x9D, KGD 0x5D, Density 0x4D | ID parser checks all three bytes | Correct for the observed log; fitted full suffix/voltage/temperature grade remains unknown. |
| SPI Quad I/O protocol | SPI instruction phase; `0x38` write and `0xEB` read; 24-bit address; quad address/data; read dummy=6 | HAL command construction and emitted diagnostic configuration | Uses 1-4-4 SPI Quad I/O, not QPI. No `0x35`/QPI transition is sent. Instruction remains single-line by design. |
| QPI protocol | Not used | Command constants/call paths | No mixed SPI/QPI phases. |
| 1024-byte page/wrap behavior | Each transfer is split at 1024-byte boundaries and capped by the active 4-byte SPI / 16-byte Quad burst | `psram_read()`/`psram_write()` chunk calculation | Prevents wrap aliasing in a single QSPI command; physical CE# intervals still need capture. |
| QUADSPI clock | HCLK read at runtime; divider rounded up to target no faster than 18 MHz | `HAL_RCC_GetHCLKFreq()` and QUADSPI prescaler | Current overlay gives 72 MHz HCLK, divider 4, 18 MHz QSPI. Not a measured clock. |
| CE# timing | Burst size is bounded in software | Estimated from phase clocks only | Not strictly enforced or measured. Confirm exact device grade and temperature limit, then capture CE#/CLK with a logic analyzer. |
| HAL timeout | QSPI polling runs with interrupts enabled; HAL tick is Zephyr uptime | Zephyr HAL `HAL_GetTick()` and driver transfer paths | Removes stalled-tick risk caused by IRQ masking. Fault recovery still requires target fault injection or a controlled stuck-busy test. |
| SDMMC | Qualified-board CMake now selects the board-level SDMMC overlay | Clean build generated native SD disk and linked `sdmmc_stm32.c` | Build configuration is valid. Card I/O and recovery need target execution. |

Primary references: [ISSI IS66WVS4M8 datasheet](https://www.issi.com/WW/pdf/66-67WVS4M8ALL-BLL.pdf), [ST STM32L476 datasheet](https://www.st.com/resource/en/datasheet/stm32l476vg.pdf), [ST RM0351 reference manual](https://www.st.com/resource/en/reference_manual/rm0351-stm32l47xxx-stm32l48xxx-stm32l49xxx-and-stm32l4axxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf), and [ST ES0250 errata](https://www.st.com/resource/en/errata_sheet/es0250-stm32l471xx475xx476xx486xx-device-errata-stmicroelectronics.pdf).

## Correctness changes

- PSRAM staging reserves ring capacity before physical writes and publishes head/occupancy only after every wrapped write segment succeeds. Free-space includes reservations. The documented contract is one producer and one consumer; the metadata lock is not held over QSPI transfers.
- Quad enable follows three data read-back passes across 0x38/SPI-read, SPI-write/0xEB, and Quad loopback checks. A data mismatch or controller error prevents Quad activation. The SPI fallback is accepted only after its own ID and read-back verification. The test is a limited probe, not a full-memory sweep.
- QSPI HAL calls no longer run with global interrupts masked. Controller errors abort/reset/reinitialize state; initialization and ID errors are checked.
- RTC UART transmission-complete polling is bounded.
- Qualified Zephyr board names now fall back to the shared SD overlay name; the previous clean build had silently missed that overlay and FatFs failed with `Wrong FF_VOLUMES setting`.
- SD quick is 512 KiB, 4/16/64 KiB blocks, one warm-up plus three measured runs. Medium is 4 MiB with the same blocks and repetitions. Full remains opt-in at 8 MiB, 512 B through 64 KiB, one warm-up plus five measured runs. Workload and estimated total I/O are printed before execution. File size, full payload, close, and unlink are checked; write throughput includes a separate effective rate with sync. Stored rates preserve milli-units.
- The pipeline command remains a single-threaded synthetic PCM16 storage pacing simulation. Its output no longer describes an SRAM queue or claims queue reliability. It reports late block counts and maximum lateness and accepts 256/384/512 kHz as synthetic storage rates only; those are not real ADC modes.

## Remaining target work and limits

- RAM is at 97.79% of the configured 96 KiB. Stack high-water instrumentation is disabled, so the linker number does not establish runtime stack margin. Enabling stack instrumentation needs review against this limited headroom.
- Run `psram test` repeatedly after cold boots; capture expected/actual bytes, confirm fallback behavior, and verify CLK/CE# and the physical IO mapping with test equipment. Do not enable Quad for deployment until the board read-back probe passes repeatedly.
- Run quick, medium, and full SD tests on the actual card, repeat failure/recovery paths, and record raw latency and sync stalls. Host/build results do not establish card reliability or sustained recording capacity.
- Confirm the fitted PSRAM suffix, supply voltage, and temperature grade. Source review cannot establish any of those board facts or electrical timing margins.
- The synthetic pipeline does not model concurrent ADC/DSP production and storage consumption. Live acquisition deadline, queue occupancy/drop rate, audio continuity, and recording/WAV validation remain unverified.




