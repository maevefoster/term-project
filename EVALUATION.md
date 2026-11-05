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
