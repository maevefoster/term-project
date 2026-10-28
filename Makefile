# posture-coach build (Linux / Raspberry Pi OS).
#
#   make                 build build/cam_proc
#   make test            build and run host unit tests
#   make asan            rebuild everything with ASan+UBSan and run the tests
#   make memcheck        run the unit tests under valgrind
#   make memcheck-cam    run cam_proc under valgrind for MEMCHECK_SECS (Pi, live camera)
#   make boundarycheck   grep src/ for network, model and image-writing APIs
#   make deploy PI=pi@<host>

CC       ?= cc
CSTD     := -std=c17
WARN     := -Wall -Wextra -Werror -Wvla
CPPFLAGS := -D_DEFAULT_SOURCE -D_POSIX_C_SOURCE=200809L -Isrc/common -Isrc/cam_proc
CFLAGS   ?= -O2 -g
SAN      := -fsanitize=address,undefined -fno-omit-frame-pointer -fno-sanitize-recover=all

BUILD    ?= build
MEMCHECK_SECS ?= 60
VALGRIND := valgrind --leak-check=full --show-leak-kinds=definite,indirect --track-fds=yes --error-exitcode=1

COMMON_SRC := src/common/applog.c
CAM_SRC    := src/cam_proc/main.c src/cam_proc/v4l2_cap.c src/cam_proc/frame_stats.c src/cam_proc/markers.c
TESTS      := $(BUILD)/test_applog $(BUILD)/test_frame_stats

obj = $(patsubst %.c,$(BUILD)/%.o,$(1))

.PHONY: all test asan memcheck memcheck-cam boundarycheck deploy soakcheck jitterbench clean

all: $(BUILD)/cam_proc

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CSTD) $(WARN) $(CPPFLAGS) $(CFLAGS) $(EXTRA_CFLAGS) -MMD -MP -c $< -o $@

$(BUILD)/cam_proc: $(call obj,$(CAM_SRC) $(COMMON_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/test_applog: $(call obj,tests/test_applog.c $(COMMON_SRC))
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

$(BUILD)/test_frame_stats: $(call obj,tests/test_frame_stats.c src/cam_proc/frame_stats.c)
	$(CC) $(CFLAGS) $(EXTRA_CFLAGS) $^ $(LDFLAGS) -o $@

test: $(TESTS)
	@set -e; for t in $(TESTS); do echo "== $$t"; ./$$t; done

asan:
	$(MAKE) BUILD=build-asan EXTRA_CFLAGS="$(SAN)" all test

memcheck: $(TESTS)
	@set -e; for t in $(TESTS); do echo "== valgrind $$t"; $(VALGRIND) ./$$t; done

# Live run: needs the webcam. SIGINT after MEMCHECK_SECS triggers clean shutdown.
memcheck-cam: $(BUILD)/cam_proc
	timeout -s INT $(MEMCHECK_SECS) $(VALGRIND) ./$(BUILD)/cam_proc --log memcheck.log

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
	rsync -az --delete --exclude .git --exclude 'build*' --exclude 'posture.log*' ./ $(PI):posture-coach/

soakcheck jitterbench:
	@echo "$@: not implemented in M3"; exit 1

clean:
	rm -rf build build-asan

-include $(shell find $(BUILD) -name '*.d' 2>/dev/null)
