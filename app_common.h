/**
 * app_common.h - Shared Application Logic
 *
 * Contains all test definitions, trace data structures, trace analysis,
 * and serial management shared across macOS, Windows, and Linux builds.
 * Each platform's app file includes this and provides the native GUI.
 */

#ifndef APP_COMMON_H
#define APP_COMMON_H

#include "protocols.h"
#include "platform.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Trace Data Structures
// ============================================================================

typedef struct {
    uint64_t time_us;
    int16_t  dx, dy;
    int32_t  cum_x, cum_y;
} trace_cmd_t;

typedef struct {
    uint64_t time_us;
    double   abs_x, abs_y;
    double   rel_x, rel_y;
} trace_obs_t;

#define TRACE_CMD_MAX 200000
#define TRACE_OBS_MAX 500000

typedef struct {
    double dev_avg, dev_max;
    double jit_avg, jit_max;
    double perp_scatter;
    double speed_cv;
    double dir_flip_rate;
    double sub_px_pct;
    double accel_jerk;
    double path_eff;
    double obs_total_dist;
    double int_cv;
    double cmd_rep_pct;
    double h_score;
    const char* h_grade;
    uint32_t obs_moving;
    uint32_t cmd_hist[8];
    double cmd_rate, obs_rate;
    double total_ms;
    int32_t cmd_bbox_w, cmd_bbox_h;
    double  obs_bbox_w, obs_bbox_h;
} trace_analysis_t;

typedef struct {
    trace_cmd_t*      cmds;
    volatile uint32_t cmd_count;
    trace_obs_t*      obs;
    volatile uint32_t obs_count;

    volatile bool     recording;
    uint64_t          start_us;
    double            start_abs_x, start_abs_y;
    char              test_name[64];

    plat_thread_t     poller_thread;
    volatile bool     poller_running;
    plat_mutex_t      cmd_mutex;

    trace_analysis_t  analysis;
    bool              analysis_valid;

    double all_min_x, all_max_x, all_min_y, all_max_y;
} trace_data_t;

// ============================================================================
// Globals (defined in app_common.c)
// ============================================================================

extern trace_data_t    g_trace;
extern plat_serial_t   g_serial_fd;
extern proto_state_t   g_proto;
extern volatile int64_t g_stat_ok, g_stat_err, g_stat_sent;

// Serial reader thread
extern plat_thread_t   g_reader_thread;
extern volatile bool    g_reader_running;

// ============================================================================
// Test Definition
// ============================================================================

typedef struct {
    const char* name;
    const char* category;
    const char* description;
    void (*run)(void);
} test_def_t;

extern const test_def_t all_tests[];
#define NUM_TESTS 15

// ============================================================================
// API (implemented in app_common.c)
// ============================================================================

void trace_alloc(void);
void trace_dealloc(void);
void trace_start(const char* name);
void trace_stop(void);
void trace_record_cmd(int16_t dx, int16_t dy);
void trace_analyze(void);

void send_move_traced(int dx, int dy);

void serial_reader_start(void);
void serial_reader_stop(void);

void cli_print_results(const char* test_name);
void print_usage(const char* prog);

#endif /* APP_COMMON_H */
