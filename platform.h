/**
 * platform.h - Cross-Platform Abstraction Layer
 *
 * Provides platform-independent interfaces for:
 *   - High-resolution timing
 *   - Threading (mutex, thread create/join)
 *   - Serial port I/O
 *   - Cursor position reading
 *   - Atomic operations
 *   - Sleep
 *
 * Each platform (macOS, Linux, Windows) implements these in its app file.
 */

#ifndef PLATFORM_H
#define PLATFORM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// ============================================================================
// Platform Detection
// ============================================================================

#if defined(__APPLE__) && defined(__MACH__)
  #define PLATFORM_MACOS 1
#elif defined(_WIN32) || defined(_WIN64)
  #define PLATFORM_WINDOWS 1
#elif defined(__linux__)
  #define PLATFORM_LINUX 1
#else
  #error "Unsupported platform"
#endif

// ============================================================================
// Threading Abstraction
// ============================================================================

#ifdef PLATFORM_WINDOWS
  #include <windows.h>
  typedef HANDLE            plat_thread_t;
  typedef CRITICAL_SECTION  plat_mutex_t;
  typedef DWORD (WINAPI *plat_thread_fn_t)(LPVOID arg);
  #define PLAT_THREAD_RETURN DWORD WINAPI
#else
  #include <pthread.h>
  typedef pthread_t         plat_thread_t;
  typedef pthread_mutex_t   plat_mutex_t;
  typedef void* (*plat_thread_fn_t)(void* arg);
  #define PLAT_THREAD_RETURN void*
#endif

static inline void plat_mutex_init(plat_mutex_t* m) {
#ifdef PLATFORM_WINDOWS
    InitializeCriticalSection(m);
#else
    pthread_mutex_init(m, NULL);
#endif
}

static inline void plat_mutex_destroy(plat_mutex_t* m) {
#ifdef PLATFORM_WINDOWS
    DeleteCriticalSection(m);
#else
    pthread_mutex_destroy(m);
#endif
}

static inline void plat_mutex_lock(plat_mutex_t* m) {
#ifdef PLATFORM_WINDOWS
    EnterCriticalSection(m);
#else
    pthread_mutex_lock(m);
#endif
}

static inline void plat_mutex_unlock(plat_mutex_t* m) {
#ifdef PLATFORM_WINDOWS
    LeaveCriticalSection(m);
#else
    pthread_mutex_unlock(m);
#endif
}

static inline int plat_thread_create(plat_thread_t* t, plat_thread_fn_t fn, void* arg) {
#ifdef PLATFORM_WINDOWS
    *t = CreateThread(NULL, 0, fn, arg, 0, NULL);
    return (*t == NULL) ? -1 : 0;
#else
    return pthread_create(t, NULL, fn, arg);
#endif
}

static inline void plat_thread_join(plat_thread_t t) {
#ifdef PLATFORM_WINDOWS
    WaitForSingleObject(t, INFINITE);
    CloseHandle(t);
#else
    pthread_join(t, NULL);
#endif
}

// ============================================================================
// Timing
// ============================================================================

/** Initialize the platform timing subsystem. Call once at startup. */
void plat_time_init(void);

/** Return microseconds since an arbitrary epoch. */
uint64_t plat_time_us(void);

/** Sleep for the given number of microseconds. */
static inline void plat_usleep(uint32_t us) {
#ifdef PLATFORM_WINDOWS
    /* Windows Sleep is in milliseconds, use a spin for sub-ms */
    if (us >= 1000) {
        Sleep(us / 1000);
        us %= 1000;
    }
    if (us > 0) {
        uint64_t end = plat_time_us() + us;
        while (plat_time_us() < end) { /* spin */ }
    }
#else
    #include <unistd.h>
    usleep(us);
#endif
}

// ============================================================================
// Serial Port
// ============================================================================

/** Opaque serial port handle. -1 / INVALID_HANDLE_VALUE means closed. */
#ifdef PLATFORM_WINDOWS
  typedef HANDLE plat_serial_t;
  #define PLAT_SERIAL_INVALID INVALID_HANDLE_VALUE
#else
  typedef int plat_serial_t;
  #define PLAT_SERIAL_INVALID (-1)
#endif

plat_serial_t plat_serial_open(const char* port, int baud);
void          plat_serial_close(plat_serial_t s);
int           plat_serial_write(plat_serial_t s, const uint8_t* data, int len);
int           plat_serial_read(plat_serial_t s, uint8_t* buf, int buflen);

// ============================================================================
// Cursor Position
// ============================================================================

typedef struct {
    double x, y;
} plat_cursor_pos_t;

/** Get current absolute cursor position. Returns false on failure. */
bool plat_get_cursor_pos(plat_cursor_pos_t* pos);

// ============================================================================
// Atomics
// ============================================================================

static inline int64_t plat_atomic_inc(volatile int64_t* v) {
#ifdef PLATFORM_WINDOWS
    return InterlockedIncrement64(v);
#else
    return __sync_fetch_and_add(v, 1);
#endif
}

static inline void plat_memory_barrier(void) {
#ifdef PLATFORM_WINDOWS
    MemoryBarrier();
#else
    __sync_synchronize();
#endif
}

#endif /* PLATFORM_H */
