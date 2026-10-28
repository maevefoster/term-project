/* Shared message definitions for posture-coach processes.
 *
 * M3 defines only the sensor status vocabulary. IPC message structs
 * (sequence, CLOCK_MONOTONIC timestamp, status, payload) are added here
 * when hub and the socket plumbing land; bump MSG_VERSION on any change
 * to a wire format.
 */
#ifndef POSTURE_MSG_H
#define POSTURE_MSG_H

#define MSG_VERSION 1

enum sensor_status {
    SENSOR_OK = 0,
    SENSOR_ABSENT = 1,     /* device missing or unreadable; process stays alive */
    SENSOR_NO_MARKERS = 2, /* camera only: frames arrive but no markers found */
};

static inline const char *sensor_status_name(enum sensor_status s)
{
    switch (s) {
    case SENSOR_OK:         return "OK";
    case SENSOR_ABSENT:     return "ABSENT";
    case SENSOR_NO_MARKERS: return "NO_MARKERS";
    }
    return "UNKNOWN";
}

#endif
