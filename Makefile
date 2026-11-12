# posture-coach build (Linux / Raspberry Pi OS).
#
#   make                 build build/hub, build/cam_proc and build/imu_proc
#   make test            build and run host unit tests
#   make asan            rebuild everything with ASan+UBSan and run the tests
#   make memcheck        run the unit tests under valgrind
#   make memcheck-cam    run cam_proc under valgrind for MEMCHECK_SECS (Pi, live camera;
#                        start build/hub first or its events only go to stderr)
#   make camdebug        build build/cam_debug, the live marker debug viewer (Pi desktop, SDL2)
#   make boundarycheck   grep src/ for network, model and image-writing APIs
#   make deploy PI=pi@<host>

CC       ?= cc
CSTD     := -std=c17
WARN     := -Wall -Wextra -Werror -Wvla
CPPFLAGS := -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L -Isrc/common -Isrc/cam_proc -Isrc/imu_proc \
            -Isrc/hub
CFLAGS   ?= -O2 -g
SAN      := -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all

BUILD    ?= build
MEMCHECK_SECS ?= 60
VALGRIND := valgrind --leak-check=full --show-leak-kinds=definite,indirect --track-fds=yes --error-exitcode=1

COMMON_SRC := src/common/applog.c
IPC_SRC    := src/common/msg.c src/common/ipc_client.c
CAM_SRC    := src/cam_proc/main.c src/cam_proc/v4l2_cap.c src/cam_proc/frame_stats.c \
              src/cam_proc/markers.c src/cam_proc/posture_detect.c
IMU_SRC    := src/imu_proc/main.c src/imu_proc/mpu6050.c
HUB_SRC    := src/hub/main.c src/hub/hub_state.c src/common/msg.c
TESTS      := $(BUILD)/test_applog $(BUILD)/test_frame_stats $(BUILD)/test_posture_detect \
              $(BUILD)/test_mpu6050 $(BUILD)/test_msg $(BUILD)/test_hub_state $(BUILD)/test_ipc

obj = $(patsubst %.c,$(BUILD)/%.o,$(1))

.PHONY: all test asan memcheck memcheck-cam camdebug boundarycheck deploy soakcheck jitterbench clean

all: $(BUILD)/hub $(BUILD)/cam_proc $(BUILD)/imu_proc

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/hub: $(call obj,$(HUB_SRC) $(COMMON_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -lm -o $@

$(BUILD)/cam_proc: $(call obj,$(CAM_SRC) $(IPC_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -lm -o $@

$(BUILD)/imu_proc: $(call obj,$(IMU_SRC) $(IPC_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -lm -o $@

$(BUILD)/test_msg: $(call obj,tests/test_msg.c src/common/msg.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -lm -o $@

$(BUILD)/test_hub_state: $(call obj,tests/test_hub_state.c src/hub/hub_state.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/test_ipc: $(call obj,tests/test_ipc.c $(IPC_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -lm -o $@

# Live on-screen marker debug viewer (Pi desktop + libsdl2-dev). Not part of
# `all`, `test` or `asan`. tools/cam_debug.c #includes posture_detect.c.
camdebug:
	@pkg-config --exists sdl2 || { echo "camdebug: SDL2 not found; run: sudo apt install -y pkg-config libsdl2-dev"; exit 1; }
	$(MAKE) $(BUILD)/cam_debug

$(BUILD)/tools/cam_debug.o: CPPFLAGS += $(shell pkg-config --cflags sdl2 2>/dev/null)
$(BUILD)/cam_debug: $(call obj,tools/cam_debug.c src/cam_proc/v4l2_cap.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) $(shell pkg-config --libs sdl2 2>/dev/null) -lm -o $@

$(BUILD)/test_mpu6050: $(call obj,tests/test_mpu6050.c src/imu_proc/mpu6050.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/test_applog: $(call obj,tests/test_applog.c $(COMMON_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/test_frame_stats: $(call obj,tests/test_frame_stats.c src/cam_proc/frame_stats.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/test_posture_detect: $(call obj,tests/test_posture_detect.c src/cam_proc/markers.c src/cam_proc/posture_detect.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -lm -o $@

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "== $$t"; ./$$t; done

asan:
	$(MAKE) BUILD=build-asan EXTRA_CFLAGS="$(SAN)" all test

memcheck: $(TESTS)
	@set -e; for t in $(TESTS); do echo "== valgrind $$t"; $(VALGRIND) ./$$t; done

# Live run: needs the webcam. SIGINT after MEMCHECK_SECS triggers clean shutdown.
memcheck-cam: $(BUILD)/cam_proc
	timeout --preserve-status -s INT $(MEMCHECK_SECS) $(VALGRIND) ./$(BUILD)/cam_proc

# Repository safeguard only; it does not replace manual review.
BOUNDARY_PATTERN := AF_INET|getaddrinfo|gethostbyname|curl|https?://|mediapipe|movenet|tflite|tensorflow|onnx|cv::dnn|imwrite|stbi_write|\.(jpe?g|png|ppm|pgm|bmp|yuv)\b
boundarycheck:
	@if grep -rnEi '$(BOUNDARY_PATTERN)' src/; then \
		echo "boundarycheck: FAILED (matches above)"; exit 1; \
	else \
		echo "boundarycheck: OK (no matches in src/)"; \
	fi

deploy:
ifndef PI
	$(error usage: make deploy PI=pi@<host>)
endif
	rsync -az --delete --exclude .git --exclude 'build*' --exclude 'posture.log*' --exclude 'imu.log*' ./ $(PI):posture-coach/

soakcheck jitterbench:
	@echo "$@: not implemented in M3"; exit 1

clean:
	rm -rf build build-asan

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)
