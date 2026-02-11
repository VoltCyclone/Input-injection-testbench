# KMBox Trace Analyzer - Cross-Platform Build
#
# macOS:   make                  (Cocoa/CoreGraphics, native)
# Linux:   make                  (GTK3/Cairo, requires libgtk-3-dev)
# Windows: make                  (Win32/GDI, MinGW/MSYS2)
# Cross:   make CROSS=win64      (cross-compile Windows from Linux/macOS)
#
# Tests:   make test             (unit tests, any platform, no hardware)
# Clean:   make clean

# ── Platform detection ──────────────────────────────────────────────
UNAME_S := $(shell uname -s 2>/dev/null || echo Windows)

ifdef CROSS
  ifeq ($(CROSS),win64)
    PLATFORM := Windows
    CC       := x86_64-w64-mingw32-gcc
  endif
else
  ifeq ($(UNAME_S),Darwin)
    PLATFORM := Darwin
  else ifeq ($(UNAME_S),Linux)
    PLATFORM := Linux
  else
    PLATFORM := Windows
  endif
endif

TARGET = kmbox_tester
CFLAGS = -O2 -Wall -Wextra -Wno-unused-parameter
LIBS   = -lm

# ── Platform-specific sources & flags ───────────────────────────────
ifeq ($(PLATFORM),Darwin)
  # macOS – Objective-C Cocoa app (self-contained, does not use app_common)
  CC       ?= clang
  OBJCFLAGS = $(CFLAGS) -fobjc-arc
  FRAMEWORKS = -framework Cocoa -framework CoreGraphics \
               -framework ApplicationServices -framework QuartzCore \
               -framework IOKit
  SRCS     = app.m protocols.c
  OBJS     = app.o protocols.o
  LINK     = $(CC) $(OBJCFLAGS) -o $(TARGET) $(OBJS) $(FRAMEWORKS) $(LIBS)

app.o: app.m protocols.h
	$(CC) $(OBJCFLAGS) -c -o $@ $<

protocols.o: protocols.c protocols.h
	$(CC) $(CFLAGS) -c -o $@ $<

else ifeq ($(PLATFORM),Linux)
  # Linux – GTK3/Cairo app
  CC       ?= gcc
  GTK_CF   := $(shell pkg-config --cflags gtk+-3.0)
  GTK_LF   := $(shell pkg-config --libs   gtk+-3.0)
  CFLAGS   += $(GTK_CF)
  SRCS     = app_linux.c app_common.c protocols.c
  OBJS     = app_linux.o app_common.o protocols.o
  LINK     = $(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(GTK_LF) $(LIBS) -lpthread

app_linux.o: app_linux.c app_common.h protocols.h platform.h
	$(CC) $(CFLAGS) -c -o $@ $<

app_common.o: app_common.c app_common.h protocols.h platform.h
	$(CC) $(CFLAGS) -c -o $@ $<

protocols.o: protocols.c protocols.h
	$(CC) $(CFLAGS) -c -o $@ $<

else ifeq ($(PLATFORM),Windows)
  # Windows – Win32/GDI app (MinGW)
  CC       ?= gcc
  ifeq ($(suffix $(TARGET)),)
    TARGET := $(TARGET).exe
  endif
  WIN_LIBS  = -lgdi32 -luser32 -lkernel32 -lcomctl32 -lcomdlg32 -lwinmm
  CFLAGS   += -D_WIN32_WINNT=0x0601
  SRCS     = app_windows.c app_common.c protocols.c
  OBJS     = app_windows.o app_common.o protocols.o
  LINK     = $(CC) $(CFLAGS) -o $(TARGET) $(OBJS) $(WIN_LIBS) $(LIBS) -mwindows

app_windows.o: app_windows.c app_common.h protocols.h platform.h
	$(CC) $(CFLAGS) -c -o $@ $<

app_common.o: app_common.c app_common.h protocols.h platform.h
	$(CC) $(CFLAGS) -c -o $@ $<

protocols.o: protocols.c protocols.h
	$(CC) $(CFLAGS) -c -o $@ $<

endif

# ── Main targets ────────────────────────────────────────────────────
.PHONY: all clean test gui info

all: $(TARGET)

$(TARGET): $(OBJS)
	$(LINK)
	@echo ""
	@echo "  Built: $(TARGET)  [$(PLATFORM)]"
	@echo ""
	@echo "  GUI:  ./$(TARGET)"
	@echo "  CLI:  ./$(TARGET) --port <serial-port> --proto kmbox --test sweep"
	@echo ""

clean:
	rm -f *.o kmbox_tester kmbox_tester.exe test_protocols test_protocols.exe

# ── Unit tests (platform-independent, no hardware) ──────────────────
TEST_CC     ?= $(firstword $(CC))
TEST_TARGET  = test_protocols$(if $(filter Windows,$(PLATFORM)),.exe,)

$(TEST_TARGET): test_protocols.c protocols.c protocols.h
	$(TEST_CC) $(CFLAGS) -o $@ test_protocols.c protocols.c $(LIBS)

test: $(TEST_TARGET)
	./$(TEST_TARGET)

# ── Convenience targets ─────────────────────────────────────────────
gui: $(TARGET)
	./$(TARGET)

info:
	@echo "Platform:  $(PLATFORM)"
	@echo "Compiler:  $(CC)"
	@echo "Target:    $(TARGET)"
	@echo "Sources:   $(SRCS)"
