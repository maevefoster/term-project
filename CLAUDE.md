# posture-coach — Team Rules (CS 370)

**Team:** Maeve Foster and Tiffany Chen  
**Target:** Raspberry Pi 4 (2 GB), wired MPU-6050 on upper back, side-mounted USB webcam with two colored markers, and GPIO-driven buzzer/speaker.  
**Graded mechanisms:** **E — multi-process architecture and supervisor**; **C — SCHED_FIFO IMU sampling**.  
**Source of truth:** `DESIGN.md` / the submitted **TP-D2 M2 Design Document (10/28/2026)**. If this file conflicts with the design, flag the conflict, cite the relevant design section, and ask before changing behavior. Do not silently invent or relax a requirement.

## 1. Nonnegotiable boundaries

- **Offline product:** No runtime network calls, cloud services, LLM APIs, downloaded inference weights, or pretrained posture/pose models (including MediaPipe, MoveNet, or OpenCV DNN). The product must work with the network disconnected. `make boundarycheck` is a repository safeguard, not a substitute for manual review.
- **Camera privacy:** Process webcam frames in RAM only. Never save frames, thumbnails, or pixel dumps to disk, logs, crash reports, or test artifacts from live runs. Recorded/replayed sensor inputs must obey the design's privacy boundary; do not assume raw frames may be recorded.
- **Student-written graded logic:** Maeve and Tiffany write the complementary filter, HSV marker detection, marker-centroid/CVA computation, and fusion state machine themselves. Claude may explain concepts, review student-written code, identify issues, and help design tests, but **must not generate or replace implementation of these algorithms**. Ask before proceeding with work that may cross this boundary.
- **Process fault isolation:** An absent/unplugged device is a reported **ABSENT** state, not a fatal process error. Keep the relevant process alive and retry without a restart storm. Preserve degraded coaching through the other available sensor.
- **Storage:** Only `hub` persists application logs/summaries. Logs must be bounded/rotated; never allow SD exhaustion during the 48-hour soak. Treat the systemd journal as separate service diagnostics, not an excuse for other processes to write application data files.
- **Testing integrity:** Never skip, delete, weaken, or change an assertion merely to make a failing test pass. Mark missing hardware, unavailable privileges, and unexecuted tests clearly.

## 2. Architecture and process contracts (mechanism E)

- Processes: `supervisor`, `imu_proc`, `cam_proc`, and `hub`. `supervisor` launches/reaps children with `fork`/`exec` and `waitpid`; **systemd** runs the supervisor with `Restart=always`.
- Communicate using **Unix domain sockets**. Keep message definitions in `src/common/msg.h`; include sequence number, `CLOCK_MONOTONIC` timestamp, sensor status, and enough data for hub freshness checks. Validate message lengths, version/format, status, and errors at IPC boundaries.
- The architecture diagram specifies **1 Hz heartbeats** and a supervisor watchdog triggered after **3 seconds of silence**. Implement/watch for blocked `cam_proc` V4L2 dequeue as well as exited processes; use a bounded termination/restart policy. The current 5-second initial-heartbeat expectation is a **team rule**, not an M2 evaluation target; verify it is compatible with the watchdog.
- On `imu_proc`, `cam_proc`, or `hub` crash/hang, restart the failed process, reconnect its sockets, and restore calibration where required. Hub must enter an appropriate degraded state while sensor packets are missing (design: no IMU packets for 2 seconds). Do not treat a live process reporting `ABSENT` as crashed.
- Recovery targets: after injected `kill -9` faults to `imu_proc`, `cam_proc`, and `hub`, **detect within 3 s** and **recover within 5 s in at least 9/10 trials**. Report detection and recovery times separately.
- After physical unplug: **DEGRADED within 5 s**, no repeated process restarts, **full recovery within 10 s** of reconnection. A sensor should be unplugged for 10 minutes during the test.
- Do not leave orphaned children, sockets, file descriptors, or GPIO resources after restart. Use `O_CLOEXEC`/`FD_CLOEXEC` as applicable; close only owned descriptors and log meaningful failures.

## 3. Sensors, timing, and posture behavior

### IMU (`imu_proc`, mechanism C)

- MPU-6050 on `/dev/i2c-1`, expected address `0x68`, **100 kHz I2C** and approximately 1 m twisted-pair cable; verify device path, address, wiring, and bus speed on the actual Pi. The INT pin is not used.
- Run the IMU sampling thread at **100 Hz (10 ms period)** under `SCHED_FIFO` for the mechanism-C configuration; output pitch to hub at **10 Hz**. Timestamp actual sample times with `CLOCK_MONOTONIC`; integrate using **measured elapsed time**, not an assumed 10 ms. Do not let blocking I/O or logging hold the real-time sampler indefinitely. Design and document an appropriate finite RT priority, privilege/fallback behavior, and thread lifecycle; don't invent measurements.
- After **5 consecutive I2C read errors**, report `ABSENT`, keep trying with bounded/backed-off retries, and let hub use camera-only DEGRADED mode. Do not fork/restart solely because I2C is unplugged.
- If the camera consistently indicates good posture while IMU pitch remains offset for **60 s**, the design calls for IMU baseline re-zeroing and an offset log. Guard this against unreliable camera markers; do not silently substitute a different recalibration rule.

### Camera (`cam_proc`)

- `/dev/video0` via **V4L2**, **320 × 240 YUYV**, **5 fps**, mmap capture buffers (hardware/configuration to verify).
- Two colored stickers are located from **student-written HSV detection in C**; compute centroids, confidence/validity, and CVA from the markers. No pretrained inference model; no storage of live frames.
- On V4L2 `ENODEV`/unplug, report `ABSENT` and retry about **every 5 s**, without crashing. If no markers are detected for **10 s while the IMU still indicates the user is seated**, report a no-markers state and let hub use stricter **IMU-only DEGRADED** behavior. Distinguish this from **AWAY** (no markers **and** large IMU motion).

### Hub and buzzer

- Hub runs the student-written fusion state machine at **1 Hz** as shown in the architecture diagram, producing **GOOD**, **SLOUCH**, **AWAY**, and **DEGRADED** modes. Use actual validity/freshness of each sensor; never pass stale data as fresh.
- Architecture diagram shows slouch gating by **pitch AND CVA**, **5 s hysteresis**, and buzzer alerts with **60 s cooldown**. Preserve those design choices; if implementation details such as numeric angle thresholds or behavior in single-sensor mode are unspecified, flag them for team decision and measurement rather than inventing them.
- Suppress alerts in **AWAY**. In DEGRADED mode, continue coaching from the valid sensor with the documented stricter thresholds (numerical values are not specified in the M2 design).
- Target alert latency: buzzer/speaker goes off **within 5 s of the qualifying bad-posture check**, subject to documented hysteresis/cooldown. Make timestamp definitions explicit so the metric is reproducible.
- Buzzer GPIO is **GPIO18** through a transistor using `libgpiod` (`/dev/gpiochip0` expected). Verify the chip/line mapping on the Pi before driving it. Log commanded buzzer events; the design explicitly does **not** claim physical speaker/buzzer-failure detection.

## 4. Logging, failure handling, and soak behavior

- Hub appends posture state changes, duration summaries (upright/slouched), and **hourly** liveness/health records including RSS and event counts. Keep verbose per-sample benchmark records separate from indefinite production logging, and control their size.
- At the **hourly** disk check, free space **below 10%** triggers removal/rotation of oldest retained logs, a warning, and suspension of optional summaries while retaining essential heartbeat records. Set practical size caps and an explicit retention policy; do not pretend the design specifies byte limits.
- For interrupted writes/power loss, validate final log record length/checksum, discard a torn final line, and continue appending. Document the chosen record framing/checksum before coding.
- At the **hourly** thermal check, report SoC temperature **above 80 °C**; the M2 design prescribes **logging only**, not automatic throttling or shutdown.
- Expected fault log events include IMU absent, process exit/restart and watchdog timeout, IMU baseline re-zero, camera absent/no-markers, AWAY/DEGRADED transitions, disk warning, thermal warning, power-loss recovery, and **commanded** buzzer output.
- All resource acquisition/allocation and important syscalls need checked results and cleanup/error paths. Distinguish recoverable hardware failures from genuine unrecoverable software errors. Avoid unbounded logs and restart loops.

## 5. Mandatory evaluation matrix (report measured results, including misses)

| Evaluation | Required method | Target |
|---|---|---|
| Mechanism C jitter | Record 100 Hz sample timestamps for **30 minutes per configuration**; compare default scheduler vs `SCHED_FIFO`, each **idle** and under **camera + `stress-ng` load**. Report interval **p50, p99, max**, jitter and deadline misses. | FIFO under load: **p99 jitter ≤ 1 ms**, **no gap > 20 ms**, **deadline misses < 0.01%**. Report even if FIFO has no benefit. |
| Mechanism E kill recovery | Inject `kill -9` into `imu_proc`, `cam_proc`, and `hub` at random times; **10 trials**, with exact timestamps. | Detect **≤ 3 s**; recover **≤ 5 s** in **at least 9/10 trials**. |
| Physical sensor unplug | Unplug each sensor during soak, reconnect after **10 minutes**. | DEGRADED **≤ 5 s**, no restart storm, full recovery **≤ 10 s** after replug. |
| Posture accuracy | Scripted, timestamp-labeled sessions; give **IMU-only, camera-only, and fused confusion matrices**. | Fused false-positive rate **< 5%**, detection **≥ 85%**, better than either sensor alone. State exact definitions/denominators. |
| Alert latency | Timestamp posture-check qualification and buzzer command/event. | Alert **≤ 5 s after qualifying posture check**. |
| Resources | CPU% and RSS **per process each minute**, plus temperature throughout soak. | **No RSS growth > 5% after 2 hours**; temperature **< 80 °C**. |
| Soak | **48 consecutive hours on live sensors**, hourly liveness/RSS/event count, at least **one injected fault** recorded. | No **unhandled** process death; preserve logs and recovery evidence. |

Replay fixtures are supplemental; **label every replay run explicitly**. The **48-hour soak and demo must use live sensors**, not replay input. `make soakcheck` is only a shorter rehearsal, never a substitute for the 48-hour evidence.

## 6. Ownership and permissible Claude work

- **Maeve:** `imu_proc` (I2C, complementary filter); fusion state machine and calibration; `SCHED_FIFO` setup and jitter/deadline-miss experiments.
- **Tiffany:** `cam_proc` (V4L2, HSV marker finding, CVA); supervisor; IPC socket plumbing; logging and recovery; kill/unplug fault-injection experiments.
- **Both:** `CLAUDE.md`, Makefiles/build, replay harness, classification experiment, and evaluation report; coordinate `src/common`, hub integration, and CLI changes.
- **Claude may:** plan and critique supervisor/restart policy; explain V4L2, I2C, `epoll`, RT scheduling, and IPC; help build the **replay harness** and **labeled test fixtures**; propose test matrices; review human-written algorithm code without replacing it.
- **Claude may not:** author the team's graded complementary filter, HSV detector, CVA calculation, or fusion decision algorithm, or claim a result without measurement. Before cross-owner edits, ask which teammate owns the current session and obtain explicit approval.

## 7. Coding, commands, and collaboration workflow

- Systems code: **C17**, `-Wall -Wextra -Werror`, no VLAs. Favor narrow functions and `goto cleanup` for multi-resource teardown. Python is restricted to `tools/` for plotting/replay/test support, not graded algorithm implementations.
- Keep IPC formats versioned and stable in `src/common/msg.h`. Avoid gratuitous refactors; prefer the smallest reviewable change.
- **First:** read `DESIGN.md`, relevant code, tests, and ownership map. For multi-file, timing-sensitive, or algorithm-related changes, present a concise plan and **wait for approval before editing**. Explain whether work is student-authored or permissible support.
- **Hardware failures:** request actual bench evidence (`dmesg`, `i2cdetect -y 1`, `v4l2-ctl`, `journalctl`, GPIO mapping, timestamps). Don't assert a hardware diagnosis from a verbal report alone. Cite datasheet/register definitions for sensor configuration; don't guess pin numbers or thresholds.
- **Build:** `make`  | **Tests:** `make test`  | **Sanitizers:** `make asan`  | **Valgrind:** `make memcheck`  
  **Short soak rehearsal:** `make soakcheck`  | **Boundary guard:** `make boundarycheck`  
  **Jitter experiments:** `make jitterbench`  | **Deploy:** `make deploy PI=pi@<host>`
- **On Pi:** `sudo systemctl restart posture-supervisor`; inspect `journalctl -u posture-supervisor` and `/var/lib/posture/`.
- A code change is **done** only after `make`, `make test`, and `make asan` actually pass. Include commands, exit status, meaningful output, and any skipped/unrun Pi-only checks. Do **not** describe an unexecuted command as passing. Additional relevant checks (`boundarycheck`, benchmarks, hardware tests, full soak) must be reported separately.
- Commit only from a verified green state. Commit message format: `M<n>: <what>`.

## 8. Open decisions — do not silently resolve

The M2 design does **not** provide numeric posture-angle thresholds, marker color/HSV ranges, an IMU `SCHED_FIFO` priority, exact retry backoff beyond named cases, log retention byte caps, or the precise rule for excluding cooldown/hysteresis time from alert-latency measurement. Propose explicit options and tests, then let the team choose and document them in `DESIGN.md` before implementation.
