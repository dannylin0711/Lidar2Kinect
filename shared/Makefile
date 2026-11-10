# Quick build of the shared library tests and tools on macOS / Linux.  (Windows: use CMakeLists.txt.)
CC ?= cc
CFLAGS ?= -std=c99 -O2 -g -Wall -Wextra -Wpedantic
SRC := $(wildcard src/*.c)
B := ../build/shared
HDR := $(wildcard include/l2k/*.h) tools/l2k_net.h

all: $(B)/test_l2k $(B)/l2k_dump $(B)/l2k_fake_sender

$(B)/test_l2k: $(SRC) tests/test_l2k.c $(HDR)
	@mkdir -p $(B)
	$(CC) $(CFLAGS) -fsanitize=address,undefined -Iinclude $(SRC) tests/test_l2k.c -o $@ -lm

$(B)/%: tools/%.c $(SRC) $(HDR)
	@mkdir -p $(B)
	$(CC) $(CFLAGS) -D_POSIX_C_SOURCE=200809L -D_DARWIN_C_SOURCE -D_DEFAULT_SOURCE -Iinclude -Itools $(SRC) $< -o $@ -lm

test: $(B)/test_l2k
	./$(B)/test_l2k tests/vectors

clean:
	rm -rf $(B)

.PHONY: all test clean
