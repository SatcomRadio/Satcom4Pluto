# satcom4pluto v2: test results

Date: 2026-10-11. Each section says whether it is an **offline simulation**, a **host test** (PC/x86),
or a **hardware test** (Pluto clone, XC7Z010 + AD9363, 192.168.5.5). Nothing here is extrapolated.
Scripts: `src/reference/` (run_suite.py, pd_fa.py, audio_test.py, cmp_c.py, real_capture.py).

## 1. Usable instantaneous bandwidth (hardware, measured)

These were measured with a LO sweep of a fixed signal and the AD9363 BIST tones. (The raw captures are not included in the repository: too large.)

| fs | rf_bw | FIR | response at ±15 MHz | aliases |
|---|---|---|---|---|
| 30.72 MS/s | 30 MHz | off | −8.9 dB | – |
| 30.72 MS/s | 56 MHz | off | −6.2 dB | – |
| 36.0 MS/s | 40 MHz | off | −2.2 dB | from ≥21 MHz offset, ≤ −18 dB |

The default is now **36.864 MS/s** (rf_bw 40 MHz): 240–270 MHz from one LO at 255 MHz, with 4.5 kHz bins (N = 8192).
At that rate the FPGA core keeps up on hardware (drops 0, inovf 0, measured fs 36.864 MS/s).
An interface-delay sweep at 36.864 MS/s gave a working window of clock 0x00–0x10 and data 0x01–0x0b, so the existing 0x06 is near the centre.
The band edges (±15 MHz) are still ~2 dB down, and strong signals 21+ MHz away can alias in at ≤ −18 dB.
Continuous reception is kept; there is no scanning.

## 2. Detector on the synthetic suite (offline, Python reference, 2 seeds per scenario)

Sub-band simulation: 4.608 MS/s, N = 1024, the same 4.5 kHz bins, and the FPGA bit-exact analysis model.
Classes: N5 (5 kHz, dev 1.5 k), N7 (7 kHz, dev 3 k), W25 (dev 12 k), W40 (~40 kHz, dev 22 k).
SNR is in-band SNR. "Above floor" is the level over the noise in one bin's band.

Pd vs SNR (`pd_final.log`):

| class | in-band SNR | above floor | v1.5 Pd | v2 Pd | v2 median latency |
|---|---|---|---|---|---|
| N5 | −8 dB | 0.6 dB | 0.00 | 0.75 | 0.84 s |
| N5 | −6 | 1.0 | 0.00 | 0.99 | 0.24 s |
| N5 | −4 | 1.5 | 0.00 | 1.00 | 0.15 s |
| N5 | 0 | 3.0 | 0.00 | 1.00 | 0.07 s |
| N5 | +2 | 4.1 | 0.11 | 1.00 | 0.04 s |
| N7 | −8 | 0.6 | 0.00 | 0.99 | 0.30 s |
| N7 | −4 | 1.5 | 0.00 | 1.00 | 0.10 s |
| N7 | 0 | 3.0 | 0.01 | 1.00 | 0.07 s |
| N7 | +2 | 4.1 | 0.91 | 1.00 | 0.04 s |
| W40 | −8 | 0.6 | 0.01 | 1.00 | 0.15 s |
| W40 | −6 | 1.0 | 0.25 | 1.00 | 0.13 s |
| W40 | −2 | 2.1 | 0.83 | 1.00 | 0.13 s |
| W40 | 0 | 3.0 | 1.00 | 1.00 | 0.10 s |

False alarms with noise only, 300 s of sub-band (≈ 37.5 s of the full band, `fa300_sin3.log`):

* Flat noise: v1.5 0 runs, v2 0 confirmed tracks.
* Rippled noise floor: v1.5 had 151 162 runs and opened 48 % of the band, while v2 had 0 confirmed tracks.

Scenario suite (`report_final.txt`, both seeds):

* **v2 had no false alarms in any scenario except one.** In `variable_floor` seed 2 it reported 14 FA/s: a track at the extreme sub-band edge (+2.23 MHz), which is an artifact of the simulation edge.
  v1.5: 207 FA/s on variable floor, 494–516 FA/s on rippled noise, 70 FA/s on DC/spurs.
* **Centre-frequency error:** v2 typically 50–320 Hz, v1.5 0.5–8 kHz.
  **Bandwidth error:** v2 typically 0.5–5 kHz, v1.5 6–31 kHz.
* **Multi-signal:** Pd 1.00 for v2 vs 0.77 for v1.5. Strong-adjacent: 1.00 vs 0.74. DC/spurs: 1.00 vs 0.57.
* **Narrow-in-wide:** Pd 1.00 for both. v2 fc error is 1.6–2.2 kHz (v1.5: 4–4.75 kHz) and v2 bw error is 1.2–2.8 kHz (v1.5: 17–19 kHz).
* **Known limit:** two narrow channels 10 kHz apart (`adjacent_narrow`) are merged by both detectors. This is the resolution limit of 4.5 kHz bins with the detection kernel.
  `adjacent_wide` and `partial_overlap` give Pd 0.90–0.98 with some merging.

## 3. Real 240–270 MHz capture + injected weak FM (offline, real data)

This is a 1.5 s capture at 36 MS/s, LO 255 MHz, gain 30 (not included in the repository), giving 51 detector updates.
The band is very busy: v1.5 finds 182 windows ≥ 3 dB, and v2 holds 255 streaming tracks.
Fifteen synthetic FM emitters were injected into free gaps (`real_inject.log`):

| class | above floor | v1.5 (last update) | v2 |
|---|---|---|---|
| N5 | 1.0 dB | yes* | no |
| N7 | 1.0 | no | no |
| W40 | 1.0 | no | yes |
| N5 | 1.5 | no | no |
| N7 | 1.5 | no | yes |
| W40 | 1.5 | yes | yes (bw over-estimated 63 kHz) |
| N5, N7, W40 | 2.1 | yes, yes, no | yes, yes, yes |
| N5, N7, W40 | 3.0 | no, yes, yes | yes, yes, yes |
| N5, N7, W40 | 4.1 | yes, yes, yes | yes, yes, yes |

\* v1.5 "hits" at ≤ 1.5 dB are not reliable detections: in this band v1.5 opens windows on noise and ripple too (see section 2).

In the real band, v2 reliably detects narrow FM from about 2 dB above the floor within 1.5 s; the synthetic suite reaches 1.0 dB.
Some wide signals placed next to real emitters get an inflated bandwidth (merging), which is bounded by `max_bw` = 150 kHz.

## 4. Audio through the chain (offline)

The test runs the FPGA model, the detector, the per-track slot (the same decimated synthesis that `sgch_rx` serves), and an FM demodulator.
It reports audio SNR against the known modulating audio (`audio_final.log`). "Ideal" is a DDC of the unprocessed input.

| scenario | class | ideal | v1.5 rebuild | v2 slot |
|---|---|---|---|---|
| clean | N5 / N7 / W25 / W40 | 10.7 / 18.0 / 29.9 / 41.3 dB | 10.7 / 18.0 / 30.0 / 38.4 | 10.7 / 17.9 / 29.5 / 37.7 |
| moderate | N5 / N7 / W25 / W40 | 0.1 / 7.2 / 25.5 / 34.4 | 0.1 / 7.2 / 25.5 / 29.9 | 0.0 / 7.1 / 25.3 / 33.3 |
| adjacent | 6 signals | 5.2 – 40.1 | ≈ ideal | within 0.1–1.9 dB of ideal |
| bursty | N5 / W40 | −9.1 / 19.9 | −9.2 / 20.6 | −10.3 / 19.9 |

Narrowband slots are equal to the ideal DDC. Wideband slots are within 1–4 dB of it, and only where audio SNR is above 30 dB.
Each slot runs at 36–144 kS/s instead of the wideband rebuild.

The C++ path (`chantest`, and the real TCP chain fake_pluto → sgchd → sgch_rx → `sdra_bench`) gave the same numbers as the ideal DDC (e.g. N5 9.8/9.8, N7 17.3/17.2, W25 29.1/30.4).

## 5. C port vs Python reference (host test)

The test (`cmp_final.log`) covered all 11 scenarios × 70 updates. Track lists matched in every update, as did the streamed runs.
The maximum fc/bw difference was 5e-5 bins (float rounding). The ARM binary's output was identical to the x86 build on the same inputs.

## 6. ARM timing (hardware)

On the busiest real spectrum, a detector update took 173 ms before optimisation and 41 ms after.
Live at 36.864 MS/s, `det_ms` is 50–70 ms with ~250 tracks. Adaptive averaging therefore raised the update to 10 frames-avg (114 ms), and no frames were dropped.
**Limit:** in a band this busy the ARM is the bottleneck. Update rate degrades gracefully instead of losing data.

## 7. Hardware end-to-end (hardware test)

* The v1.1 bitstream (first build, rx_clk 125 MHz constraint) was loaded **into RAM** with fpga_manager; flash was not touched.
  It reports version 0x00010001 with kernel sin3.
* `sgchd` v2 at 36.864 MS/s ran with drops 0 and inovf 0.
* `sgch_rx.exe --channels 8` served 8 SDRA slots at 72 kS/s for 30 s with 0 bad zlib messages, checked with `sdra_bench`.
* SDRangel (7.24.0) acceptance is in section 9.

## 8. FPGA build (Vivado 2026.1)

* **Simulation:** the core simulated in xsim and in iverilog was bit-identical to the numpy model for sin5, sin3 and sin1.
* **First v1.1 build (rx_clk constrained to 125 MHz):** routed with WNS +0.624 ns and WHS +0.016 ns.
  Utilisation: LUT 11 125 (63.2 %), FF 13 367, BRAM 42.5/60, DSP 73/80. The sgc core alone uses 6 184 LUT, 38 RAMB36 + 5 RAMB18 and 39 DSP.
* **Rebuild with rx_clk constrained to 160 MHz** (DATA_CLK = 4×36.864 = 147.5 MHz): the first attempt failed timing at −0.071 ns (gram → b_pr).
  An extra pipeline stage (A2) was added (verified in simulation); the timing-closed rebuild has not been completed yet.

## 9. SDRangel acceptance (hardware)

The wideband stream (`sgch_rx --server`, SDRA protocol) was received by SDRangel 7.24 on Windows. It showed the full
240–270 MHz span at 36.864 MS/s with no lost frames, NFM demodulation worked, and the per-downlink slot streams
(`--channels`) showed the expected 72 kS/s channels. SDRangel's RTL0 (rtl_tcp) mode is limited to 3.2 MS/s and
cannot be used.

## 10. OpenWebRX (host test, simulated Pluto)

The SoapySDR module and OpenWebRX device type were tested with the current OpenWebRX+ code, luarvique csdr, pycsdr
and owrx_connector, and SoapySDR 0.8.1, against `src/reference/fake_pluto.py`:

* The satcom4pluto device type is offered; OpenWebRX starts `soapy_connector` with rate, frequency, gain and detector options.
* The OpenWebRX spectrum shows all test emitters at the right frequencies (+54 … +74 dB over the fill), and an NFM
  client receives audio.
* A live profile switch to 36.864 MS/s set the radio to 36.864 MS/s, 40 MHz analog bandwidth, LO 255 MHz, gain 30.
* With the Pluto removed, zeros were streamed in real time and OpenWebRX did not fail the device. On return with
  reset radio settings, they were restored automatically, along with the detector options.
* The installer was run twice against a stand-in container; it is idempotent.

Not yet tested: a real Pluto with the Linux package, a real Docker container, and the OpenWebRX CPU load at 36.864 MS/s.
