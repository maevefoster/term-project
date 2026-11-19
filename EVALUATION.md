# Evaluation

## Memory and undefined-behavior checking: where each check runs

CLAUDE.md section 7 lists `make asan` as part of "done". AddressSanitizer
cannot start on our Raspberry Pi, so the checks are split across two machines.
Each result below is labeled with where it was run.

| Check | Machine | Result |
|---|---|---|
| `make`, `make test` | Raspberry Pi 4 (`goober-pi`) | `test_applog`, `test_frame_stats` and `test_posture_detect` all print "all checks passed" |
| `make asan` (ASan + UBSan) | PC, WSL | Built and all 3 tests passed, no sanitizer reports. Run when posture_detect was integrated (commit 71c1225). |
| `make asan` | Raspberry Pi 4 | **Could not run** (see below). Not a pass. |
| `make memcheck` (valgrind 3.24.0) | Raspberry Pi 4 | Ran all 3 tests; the output showed "ERROR SUMMARY: 0 errors" and no leaks for the tests visible in the captured output. |

### Why ASan does not run on the Pi

The ASan runtime aborts at startup, before any test code runs (it fails in
`test_applog`, which is unchanged by the posture_detect work):

    AddressSanitizer: CHECK failed: sanitizer_allocator_primary64.h:131 ...
    (0x500000000000, ...)

The Pi kernel (`6.18.50+rpt-rpi-v8`, aarch64, GCC 14.2.0 Debian 14.2.0-19) is
built with `CONFIG_ARM64_VA_BITS_39=y` and 4 KB pages (confirmed from
`/proc/config.gz` and `/boot/config-$(uname -r)`). A 39-bit address space tops
out at about `0x8000000000`; ASan tried to reserve memory at `0x500000000000`.
This is an environment limit, not a defect in our code.

### Policy

- Run `make asan` on the PC (WSL or any Linux machine with a 48-bit address
  space) before committing. Report it as run there.
- Run `make test` and `make memcheck` on the Pi. Valgrind covers memory errors
  and leaks there but is not a substitute for UBSan; UBSan results come from
  the PC run only.
- Never report `make asan` as passing on the Pi. A bare "`make asan` passed"
  without a machine name is not acceptable in the report.
- Still not run on the Pi: `make memcheck-cam` (live camera under valgrind).

## Live color-detection check with tools/cam_debug (2026-11-05)

**Live run on the Pi, not replay.** Raw output: `docs/logs/camdebug_2026-11-05_pi.txt`
(numbers only; no frames or images saved). Tool: `tools/cam_debug.c` on branch
`camdebug-wip`, commit b09bbf4. Default detector ranges, not changed.

**Question:** do the detector's HSV ranges catch the sticker colors on the live camera picture?

**Setup:** `/dev/video0`, 320x240 YUYV (driver reported tpf=1/30). Hand-held Wii
remotes (green, blue) stood in for stickers. A blue poster was also in view.
Lighting not measured. Clicks were only on pixels of the target colors.

**Result: the color ranges catch the target colors; the marker is not accepted yet.**

| Item | Observed | Detector limit |
|---|---|---|
| Green remote clicks: H / S / V | 154-160 / 0.94-1.00 / 0.25-0.27 | 90-165 / >= 0.50 / >= 0.25 |
| Blue remote clicks accepted: H / S / V | 204-210 / 0.50-1.00 / 0.30-0.57 | 200-255 / >= 0.50 / >= 0.20 |
| Blue remote clicks rejected: H / S / V | 215-229 / 0.38-0.47 / 0.38-0.46 | rejected for S < 0.50 |
| EAR `raw` pixels per second | 34-159 | min_pixels = 12 |
| C7 `raw` pixels per second | 95-612 | min_pixels = 12 |
| EAR blob spread `sig` (px) | 57.0-172.4 | max_sigma_px = 12 |
| C7 blob spread `sig` (px) | 47.0-111.8 | max_sigma_px = 12 |
| `found` | 0 on every status line, both colors | |
| Status | NO_USER, CVA=-1.0 throughout | |

The green clicks sit on the V >= 0.25 limit, so accept/reject flips at V=0.25.
The same blue spot read S=0.78 and later S=0.40, so saturation is lighting-sensitive.

**Not achieved:** a valid CVA. Every blob had a spread far above the limit of 12,
with centroids jumping across the frame, which means matching pixels were
scattered over the frame (the blue poster, and possibly noise). The outlier
step removed almost nothing (`kept` close to `raw`). The source of the scattered
pixels is **not confirmed**: only target pixels were clicked.

**This is not a posture-accuracy result.** No sticky notes, no controlled background,
no labeled sessions.

**Retest plan:**
1. Plain background with nothing else in the HSV ranges (check the tint picture shows no specks).
2. Real neon/fluorescent sticky notes, small in frame (a square blob of side s px has
   spread of about s / 2.45, so max_sigma_px = 12 means roughly 29 px across).
3. Record the `found`, `sig`, `at` fields and click lines again.
4. If scattered pixels persist on a clean background, the team decides how the
   detector should handle them (CLAUDE.md section 8). Ranges are not changed from this one run.

**Frame rate (resolved 2026-11-12):** the driver reported tpf=1/30 and cam_proc
measured about 10 fps earlier, but the design says 5 fps. The team decided to run
at the rate the driver grants, so other webcams work unchanged (DESIGN.md, Camera
capture). Why the measured rate was below the granted 30 fps was not investigated.

## Marker color tuning with tools/cam_debug (2026-11-19)

**Live runs on the Pi, not replay.** Numbers only; no frames or images saved.
Question: which HSV limits catch our green and purple sticky notes at the angles
they take, without picking up the background?

| Try | Limits | Result |
|---|---|---|
| Purple S >= 0.35, V >= 0.20 | C7 hue 260-310 | Purple accepted on only about 8 of about 110 seconds; extra scattered pixels inflated the blob spread (20-80 vs 4-9). |
| "Lenient": green hue to 175, S >= 0.40, V >= 0.18; purple hue 258-312, S >= 0.38, V >= 0.15; spread limit 18 | | Worse. Green raw 100-3,700 with spread 23-149, found=0 on every line; purple boxes appeared at random places (noise). Lowering brightness minimums let dark noisy pixels in. |
| Defaults (green hue 90-165, S 0.50, V 0.25; purple hue 260-310, S 0.50, V 0.20) | | Green note works (clicks H 156-162, S 0.86-1.00, V 0.56-0.65); only hue edge is tight (162 vs 165). Purple clicks H 253-265, S 0.33-0.44, V 0.69-0.88: below the hue floor and the saturation minimum. |
| Tuned: green hue 90-172; purple hue 250-310, S >= 0.38, V >= 0.40 | | Best so far. Purple found on nearly every second in view (raw about 100-270, spread 5-8); no stray pixels when the notes were out of view (raw 0-12). Many `OK CVA` seconds with both notes in view. |

**Observations:**
- Green stays steady at about 200-300 matched pixels (spread 6-8); at about 450-650 the spread reaches 10-13.5 and
  the blob is sometimes rejected (limit 12). A limit of 14 is untested.
- CVA reads about 89 deg with the green note directly above the purple one and 40-66 deg when the notes were moved
  sideways by hand. These were hand-placed notes, not labeled postures: this shows the number follows note
  geometry, not posture detection.
- A pair is accepted only when the green note is above the purple one and the notes are at least 15 pixels apart.

**Decision:** the tuned limits became the defaults (see DESIGN.md, known-issues item 6).

**Not done:** posture accuracy, distance range, and a second lighting condition. The earlier 4-posture CVA run
(pasted in chat, not saved in the repo) predates this tuning.
