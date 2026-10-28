# posture-coach design

> The full M2 design (TP-D2, 10/28/2026) still needs to be copied in here. Until then, CLAUDE.md summarizes it.

## M3 interim decisions (camera only)

The MPU-6050 boards we have are defective, so M3 builds and tests only `cam_proc`. These are deliberate, temporary deviations from the M2 design:

| # | Deviation | M2 design | Reverts when |
|---|---|---|---|
| 1 | `cam_proc` writes the application log itself (`src/common/applog.c`) | Only `hub` persists application logs | hub exists; applog moves into hub |
| 2 | No supervisor, hub or IPC. `cam_proc` is started by hand | supervisor + Unix domain sockets | mechanism E work starts |
| 3 | Log is `./posture.log` in the working directory (`--log PATH` overrides) | `/var/lib/posture/` | systemd deployment |
| 4 | No marker detection. `markers_on_frame()` is an empty, student-owned seam | HSV markers, centroids, CVA | Tiffany implements it |

### Camera capture
- `/dev/video0`, V4L2, YUYV 320×240, 5 fps requested, 4 mmap buffers. The format and frame interval the driver actually grants are logged in `CAM_OPEN` (plus `CAM_FORMAT_MISMATCH` if the size differs).
- Frames stay in the mmap buffers and are requeued right after the callback. Nothing derived from pixels is logged.
- Each dequeue waits at most 1 s (`poll`). A timeout is a stall. 5 consecutive stalls close the device, report `CAM_ABSENT op=stall`, and reopen it immediately.
- Unplug (`ENODEV`/`EIO`/`ENOENT`/`ENXIO`/`EBUSY`, or `POLLERR`) closes the device, logs `CAM_ABSENT` once, and retries every 5 s. Failed retries are counted in `CAM_SUMMARY`, not logged one by one. Reconnecting logs `CAM_RECONNECTED down_ms=…`.
- A wrong device or unsupported format is `CAM_FATAL`: the process exits 1.

### Log format
One record per line:

```
<CLOCK_MONOTONIC s.ns> <UTC ISO-8601 ms> <proc> <EVENT> key=value ... *<crc32>
```

- The CRC-32 (IEEE, lowercase hex) covers every byte before ` *`. Records are at most 512 bytes; an oversized body is replaced by `truncated=1`. CR/LF in values become `_`.
- Each record goes out in one `write()` on an `O_APPEND` fd. A failed write is truncated back out of the file.
- On open, an unterminated tail is dropped, then the final complete line is dropped if its CRC fails. A `LOG_TORN_REPAIRED bytes=N` record follows.
- Events: `CAM_START`, `CAM_OPEN`, `CAM_FORMAT_MISMATCH`, `CAM_SUMMARY`, `CAM_STALL`, `CAM_ABSENT`, `CAM_RECONNECTED`, `CAM_FATAL`, `CAM_STOP`, `HEALTH` (hourly: uptime, RSS, totals), `LOG_TORN_REPAIRED`, `LOG_ROTATED`.

### Open decisions (placeholders, team to confirm)
| Item | Placeholder | Where |
|---|---|---|
| Log size cap | 5 MiB, plus one rotated copy `posture.log.1` (about 10 MiB worst case) | `APPLOG_CAP` in `src/common/applog.h` |
| Summary interval | 10 s (per-frame lines would be about 430k/day) | `CAM_SUMMARY_S` in `src/cam_proc/main.c` |
| Whether one rotated `.1` copy fits "single log file" | allowed | `applog.c` `rotate()` |
| Disk-space (<10%) and thermal checks | not in cam_proc; belong to hub | — |
