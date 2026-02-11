/**
 * KMBox Trace Analyzer - Native macOS Application
 *
 * A native instrumentation dashboard for testing mouse bridge devices.
 * Replaces the HTML trace output with a real-time, GPU-accelerated viewer.
 *
 * Features:
 *   - Protocol support: KMBox B+, Ferrum One, MAKCU
 *   - Real-time trace visualization during tests
 *   - Dual-channel overlay: commanded vs observed cursor path
 *   - Humanization analysis with composite scoring
 *   - Pan/zoom canvas with speed-colored paths
 *   - Collapsible sidebar with stats, histograms, controls
 *   - Dark instrumentation aesthetic
 *
 * Build:
 *   make
 *   # or directly:
 *   clang -O2 -o kmbox_tester app.m protocols.c \
 *         -framework Cocoa -framework CoreGraphics \
 *         -framework ApplicationServices -framework QuartzCore -lm
 *
 * Usage:
 *   ./kmbox_tester                          # Launch GUI
 *   ./kmbox_tester --port /dev/tty.usbmodem2101 --proto kmbox --test sweep
 */

#import <Cocoa/Cocoa.h>
#import <QuartzCore/QuartzCore.h>
#include <CoreGraphics/CoreGraphics.h>
#include <ApplicationServices/ApplicationServices.h>
#include <mach/mach_time.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <IOKit/serial/ioss.h>
#include <signal.h>
#include <math.h>
#include "protocols.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Colors (NSColor helpers)
// ============================================================================

#define CLR(r,g,b,a) [NSColor colorWithCalibratedRed:(r)/255.0 green:(g)/255.0 \
                      blue:(b)/255.0 alpha:(a)]
#define CGCLR(r,g,b,a) ((CGFloat[]){(r)/255.0,(g)/255.0,(b)/255.0,(a)})

static NSColor* col_bg0;
static NSColor* col_bg1;
static NSColor* col_bg2;
static NSColor* col_bg3;
static NSColor* col_border;
static NSColor* col_text0;
static NSColor* col_text1;
static NSColor* col_text2;
static NSColor* col_text3;
static NSColor* col_cmd;
static NSColor* col_obs;
static NSColor* col_dev;
static NSColor* col_green;
static NSColor* col_red;
static NSColor* col_yellow;
static NSColor* col_cyan;

static void init_colors(void) {
    col_bg0    = CLR(6, 6, 10, 1.0);
    col_bg1    = CLR(11, 11, 18, 1.0);
    col_bg2    = CLR(16, 16, 26, 1.0);
    col_bg3    = CLR(22, 22, 34, 1.0);
    col_border = CLR(30, 30, 48, 1.0);
    col_text0  = CLR(232, 232, 240, 1.0);
    col_text1  = CLR(176, 176, 192, 1.0);
    col_text2  = CLR(112, 112, 136, 1.0);
    col_text3  = CLR(72, 72, 96, 1.0);
    col_cmd    = CLR(78, 154, 240, 1.0);
    col_obs    = CLR(240, 128, 64, 1.0);
    col_dev    = CLR(240, 80, 160, 0.4);
    col_green  = CLR(64, 216, 128, 1.0);
    col_red    = CLR(240, 64, 96, 1.0);
    col_yellow = CLR(232, 200, 64, 1.0);
    col_cyan   = CLR(64, 200, 232, 1.0);
}

// ============================================================================
// Mach Timing
// ============================================================================

static mach_timebase_info_data_t g_timebase;

static uint64_t time_us(void) {
    uint64_t t = mach_absolute_time();
    if (g_timebase.numer <= g_timebase.denom)
        return (t / (g_timebase.denom * 1000ULL)) * g_timebase.numer;
    return (t * g_timebase.numer) / (g_timebase.denom * 1000ULL);
}

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

// Computed analysis results
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
    uint32_t cmd_hist[8]; // Interval histogram buckets
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

    pthread_t         poller_thread;
    volatile bool     poller_running;
    pthread_mutex_t   cmd_mutex;

    trace_analysis_t  analysis;
    bool              analysis_valid;

    // Computed bounds for rendering
    double all_min_x, all_max_x, all_min_y, all_max_y;
} trace_data_t;

static trace_data_t g_trace;

static void trace_alloc(void) {
    mach_timebase_info(&g_timebase);
    g_trace.cmds = calloc(TRACE_CMD_MAX, sizeof(trace_cmd_t));
    g_trace.obs  = calloc(TRACE_OBS_MAX, sizeof(trace_obs_t));
    pthread_mutex_init(&g_trace.cmd_mutex, NULL);
}

static void trace_dealloc(void) {
    free(g_trace.cmds);
    free(g_trace.obs);
    pthread_mutex_destroy(&g_trace.cmd_mutex);
}

static void* trace_poller_fn(void* arg) {
    (void)arg;
    while (g_trace.poller_running) {
        if (g_trace.recording && g_trace.obs_count < TRACE_OBS_MAX) {
            CGEventRef ev = CGEventCreate(NULL);
            if (ev) {
                CGPoint loc = CGEventGetLocation(ev);
                CFRelease(ev);
                uint32_t idx = g_trace.obs_count;
                g_trace.obs[idx].time_us = time_us() - g_trace.start_us;
                g_trace.obs[idx].abs_x = loc.x;
                g_trace.obs[idx].abs_y = loc.y;
                g_trace.obs[idx].rel_x = loc.x - g_trace.start_abs_x;
                g_trace.obs[idx].rel_y = loc.y - g_trace.start_abs_y;
                __sync_synchronize();
                g_trace.obs_count = idx + 1;
            }
        }
        usleep(500);
    }
    return NULL;
}

static void trace_start(const char* name) {
    CGEventRef ev = CGEventCreate(NULL);
    CGPoint loc = {0, 0};
    if (ev) { loc = CGEventGetLocation(ev); CFRelease(ev); }

    g_trace.cmd_count = 0;
    g_trace.obs_count = 0;
    g_trace.start_us = time_us();
    g_trace.start_abs_x = loc.x;
    g_trace.start_abs_y = loc.y;
    g_trace.analysis_valid = false;
    strlcpy(g_trace.test_name, name, sizeof(g_trace.test_name));

    g_trace.poller_running = true;
    g_trace.recording = true;
    pthread_create(&g_trace.poller_thread, NULL, trace_poller_fn, NULL);

    int w = 0;
    while (g_trace.obs_count == 0 && w < 200) { usleep(500); w++; }
}

static void trace_stop(void) {
    g_trace.recording = false;
    g_trace.poller_running = false;
    pthread_join(g_trace.poller_thread, NULL);
}

static void trace_record_cmd(int16_t dx, int16_t dy) {
    if (!g_trace.recording) return;
    pthread_mutex_lock(&g_trace.cmd_mutex);
    uint32_t idx = g_trace.cmd_count;
    if (idx >= TRACE_CMD_MAX) { pthread_mutex_unlock(&g_trace.cmd_mutex); return; }
    g_trace.cmds[idx].time_us = time_us() - g_trace.start_us;
    g_trace.cmds[idx].dx = dx;
    g_trace.cmds[idx].dy = dy;
    g_trace.cmds[idx].cum_x = (idx > 0) ? g_trace.cmds[idx-1].cum_x + dx : dx;
    g_trace.cmds[idx].cum_y = (idx > 0) ? g_trace.cmds[idx-1].cum_y + dy : dy;
    g_trace.cmd_count = idx + 1;
    pthread_mutex_unlock(&g_trace.cmd_mutex);
}

// ============================================================================
// Trace Analysis (runs after test completes)
// ============================================================================

static void trace_analyze(void) {
    trace_analysis_t* a = &g_trace.analysis;
    memset(a, 0, sizeof(*a));

    uint32_t ncmd = g_trace.cmd_count;
    uint32_t nobs = g_trace.obs_count;
    if (ncmd < 2 && nobs < 2) return;

    // Bounds
    int32_t cMinX=0,cMaxX=0,cMinY=0,cMaxY=0;
    for (uint32_t i = 0; i < ncmd; i++) {
        int32_t cx = g_trace.cmds[i].cum_x, cy = g_trace.cmds[i].cum_y;
        if (cx < cMinX) cMinX = cx; if (cx > cMaxX) cMaxX = cx;
        if (cy < cMinY) cMinY = cy; if (cy > cMaxY) cMaxY = cy;
    }
    double oMinX=0,oMaxX=0,oMinY=0,oMaxY=0;
    for (uint32_t i = 0; i < nobs; i++) {
        double rx = g_trace.obs[i].rel_x, ry = g_trace.obs[i].rel_y;
        if (rx < oMinX) oMinX = rx; if (rx > oMaxX) oMaxX = rx;
        if (ry < oMinY) oMinY = ry; if (ry > oMaxY) oMaxY = ry;
    }
    g_trace.all_min_x = fmin((double)cMinX, oMinX);
    g_trace.all_max_x = fmax((double)cMaxX, oMaxX);
    g_trace.all_min_y = fmin((double)cMinY, oMinY);
    g_trace.all_max_y = fmax((double)cMaxY, oMaxY);

    a->cmd_bbox_w = cMaxX - cMinX; a->cmd_bbox_h = cMaxY - cMinY;
    a->obs_bbox_w = oMaxX - oMinX; a->obs_bbox_h = oMaxY - oMinY;

    // Duration and rates
    uint64_t totalUs = 0;
    if (ncmd > 0) totalUs = g_trace.cmds[ncmd-1].time_us;
    if (nobs > 0 && g_trace.obs[nobs-1].time_us > totalUs)
        totalUs = g_trace.obs[nobs-1].time_us;
    a->total_ms = (double)totalUs / 1000.0;
    a->cmd_rate = (totalUs > 0 && ncmd > 1) ? (double)(ncmd-1) / ((double)totalUs / 1e6) : 0;
    a->obs_rate = (totalUs > 0 && nobs > 1) ? (double)(nobs-1) / ((double)totalUs / 1e6) : 0;

    // Deviation analysis
    double devSum = 0, devMax = 0; uint32_t devCount = 0, osi = 0;
    for (uint32_t ci = 0; ci < ncmd && nobs >= 2; ci++) {
        uint64_t ct = g_trace.cmds[ci].time_us;
        if (ct < g_trace.obs[0].time_us) continue;
        if (ct > g_trace.obs[nobs-1].time_us) break;
        while (osi+1 < nobs && g_trace.obs[osi+1].time_us <= ct) osi++;
        if (osi+1 >= nobs) break;
        uint64_t t0 = g_trace.obs[osi].time_us, t1 = g_trace.obs[osi+1].time_us;
        if (t1 <= t0 || ct < t0) continue;
        double fr = (double)(ct - t0) / (double)(t1 - t0);
        if (fr < 0 || fr > 1) continue;
        double ox = g_trace.obs[osi].rel_x + fr*(g_trace.obs[osi+1].rel_x - g_trace.obs[osi].rel_x);
        double oy = g_trace.obs[osi].rel_y + fr*(g_trace.obs[osi+1].rel_y - g_trace.obs[osi].rel_y);
        double d = hypot(ox - g_trace.cmds[ci].cum_x, oy - g_trace.cmds[ci].cum_y);
        devSum += d; if (d > devMax) devMax = d; devCount++;
    }
    a->dev_avg = devCount > 0 ? devSum / devCount : 0;
    a->dev_max = devMax;

    // Jitter
    double jitSum = 0, jitMax = 0; uint32_t jitCount = 0;
    for (uint32_t i = 2; i < nobs; i++) {
        double vx = g_trace.obs[i].rel_x - g_trace.obs[i-2].rel_x;
        double vy = g_trace.obs[i].rel_y - g_trace.obs[i-2].rel_y;
        double len = hypot(vx, vy);
        if (len < 0.5) continue;
        double px = g_trace.obs[i-1].rel_x - g_trace.obs[i-2].rel_x;
        double py = g_trace.obs[i-1].rel_y - g_trace.obs[i-2].rel_y;
        double perp = fabs(vx*py - vy*px) / len;
        jitSum += perp; if (perp > jitMax) jitMax = perp; jitCount++;
    }
    a->jit_avg = jitCount > 0 ? jitSum / jitCount : 0;
    a->jit_max = jitMax;

    // Observed path stats
    double obsMagSum = 0, obsMagSqSum = 0;
    uint32_t obsDirFlipX = 0, obsDirFlipY = 0, obsSubPx = 0;
    int prevSX = 0, prevSY = 0;
    for (uint32_t i = 1; i < nobs; i++) {
        double dx = g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x;
        double dy = g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y;
        double mag = hypot(dx, dy);
        if (mag > 0.01) {
            a->obs_total_dist += mag;
            a->obs_moving++;
            obsMagSum += mag; obsMagSqSum += mag*mag;
            int sx = (dx > 0.05)?1:(dx < -0.05)?-1:0;
            int sy = (dy > 0.05)?1:(dy < -0.05)?-1:0;
            if (sx && prevSX && sx != prevSX) obsDirFlipX++;
            if (sy && prevSY && sy != prevSY) obsDirFlipY++;
            if (sx) prevSX = sx; if (sy) prevSY = sy;
            if (mag < 0.5) obsSubPx++;
        }
    }
    double disp = (nobs > 1) ? hypot(g_trace.obs[nobs-1].rel_x - g_trace.obs[0].rel_x,
                                      g_trace.obs[nobs-1].rel_y - g_trace.obs[0].rel_y) : 0;
    a->path_eff = (a->obs_total_dist > 1) ? disp / a->obs_total_dist : 1.0;
    a->dir_flip_rate = (a->obs_moving > 10) ?
        (double)(obsDirFlipX + obsDirFlipY) / a->obs_moving * 100.0 : 0;
    double magMean = (a->obs_moving > 0) ? obsMagSum / a->obs_moving : 0;
    double magVar = (a->obs_moving > 1) ? (obsMagSqSum / a->obs_moving - magMean*magMean) : 0;
    if (magVar < 0) magVar = 0;
    a->speed_cv = (magMean > 0.01) ? sqrt(magVar) / magMean : 0;
    a->sub_px_pct = (nobs > 1) ? (double)obsSubPx / (nobs-1) * 100.0 : 0;

    // Command timing
    double intSum = 0, intSqSum = 0;
    for (uint32_t i = 1; i < ncmd; i++) {
        double iv = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
        intSum += iv; intSqSum += iv*iv;
    }
    double intMean = (ncmd > 2) ? intSum / (ncmd-1) : 0;
    double intVar = (ncmd > 2) ? (intSqSum / (ncmd-1) - intMean*intMean) : 0;
    if (intVar < 0) intVar = 0;
    a->int_cv = (intMean > 0) ? sqrt(intVar) / intMean : 0;

    // Command delta repeats
    uint32_t reps = 0;
    for (uint32_t i = 1; i < ncmd; i++)
        if (g_trace.cmds[i].dx == g_trace.cmds[i-1].dx &&
            g_trace.cmds[i].dy == g_trace.cmds[i-1].dy) reps++;
    a->cmd_rep_pct = (ncmd > 1) ? (double)reps / (ncmd-1) * 100.0 : 0;

    // Perp scatter
    double psSum = 0; uint32_t psN = 0;
    for (uint32_t i = 10; i < nobs; i += 5) {
        double wx = g_trace.obs[i].rel_x - g_trace.obs[i-10].rel_x;
        double wy = g_trace.obs[i].rel_y - g_trace.obs[i-10].rel_y;
        double wl = hypot(wx, wy);
        if (wl < 2.0) continue;
        double px = -wy/wl, py = wx/wl;
        for (uint32_t j = i-9; j < i; j++) {
            double dx = g_trace.obs[j].rel_x - g_trace.obs[i-10].rel_x;
            double dy = g_trace.obs[j].rel_y - g_trace.obs[i-10].rel_y;
            psSum += fabs(dx*px + dy*py); psN++;
        }
    }
    a->perp_scatter = (psN > 0) ? psSum / psN : 0;

    // Accel jerk
    double acSum = 0; uint32_t acN = 0; double prevSpd = 0;
    for (uint32_t i = 1; i < nobs; i++) {
        double dx = g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x;
        double dy = g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y;
        double dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i-1].time_us);
        if (dt < 1) continue;
        double spd = hypot(dx, dy) / dt;
        if (i > 1 && prevSpd > 0.001) {
            acSum += fabs(spd - prevSpd) / (dt/1000.0); acN++;
        }
        prevSpd = spd;
    }
    a->accel_jerk = (acN > 0) ? acSum / acN : 0;

    // Interval histogram
    for (uint32_t i = 1; i < ncmd; i++) {
        double ms = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us) / 1000.0;
        int b = (ms<1)?0:(ms<2)?1:(ms<5)?2:(ms<10)?3:(ms<20)?4:(ms<50)?5:(ms<100)?6:7;
        a->cmd_hist[b]++;
    }

    // ── Humanization Score ──────────────────────────────────────────
    // Bell-curve model: each metric is scored by how close it falls to
    // the empirical "human" range. Too-perfect (robotic) AND too-noisy
    // (synthetic jitter) both reduce the score.
    //
    // Reference ranges from recorded human mouse data:
    //   Perp scatter:  0.08–0.60 px  (ideal ~0.25)
    //   Jitter avg:    0.005–0.08 px (ideal ~0.03)
    //   Speed CV:      0.15–0.80     (ideal ~0.40)
    //   Dir flip rate: 3–20%         (ideal ~10%)
    //   Sub-pixel:     2–15%         (ideal ~6%)
    //   Interval CV:   0.05–0.40     (ideal ~0.20)
    //   Path eff:      0.85–0.995    (ideal ~0.96)

    // Gaussian-ish bell: score = exp(-0.5 * ((log(x/center)/width)^2))
    // Returns 0–1, peaks at center, symmetric on log scale
    #define BELL(val, center, width) \
        (((val) > 1e-9) ? exp(-0.5 * pow(log((val)/(center)) / (width), 2)) : 0.0)

    double ps  = BELL(a->perp_scatter,  0.25,  1.0);  // 25pts
    double jt  = BELL(a->jit_avg,       0.03,  0.9);  // 20pts
    double scv = BELL(a->speed_cv,      0.40,  0.8);  // 20pts
    double dfr = BELL(a->dir_flip_rate, 10.0,  0.8);  // 10pts
    double spx = BELL(a->sub_px_pct,    6.0,   0.8);  // 5pts
    double icv = BELL(a->int_cv,        0.20,  0.8);  // 10pts
    double pef = (a->path_eff > 0.001 && a->path_eff < 1.0)
               ? BELL(1.0 - a->path_eff, 0.04, 0.8) : 0.0; // 10pts

    #undef BELL

    double h = ps * 25.0 + jt * 20.0 + scv * 20.0 + dfr * 10.0
             + spx * 5.0 + icv * 10.0 + pef * 10.0;
    if (h > 100) h = 100;
    a->h_score = h;
    a->h_grade = (h < 15) ? "Robotic" : (h < 35) ? "Synthetic" :
                 (h < 55) ? "Plausible" : (h < 75) ? "Convincing" : "Human";

    g_trace.analysis_valid = true;
}

// ============================================================================
// Serial Port Management
// ============================================================================

static int g_serial_fd = -1;
static proto_state_t g_proto;
static volatile int64_t g_stat_ok = 0, g_stat_err = 0, g_stat_sent = 0;

static int serial_open(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return -1;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return -1; }

    // Configure termios with a placeholder baud — the real rate is set
    // via IOSSIOSPEED below for non-standard rates (e.g. 2M baud).
    speed_t posix_speed;
    bool need_iossiospeed = false;
    switch (baud) {
        case 9600:   posix_speed = B9600;   break;
        case 19200:  posix_speed = B19200;  break;
        case 38400:  posix_speed = B38400;  break;
        case 57600:  posix_speed = B57600;  break;
        case 115200: posix_speed = B115200; break;
        case 230400: posix_speed = B230400; break;
        default:
            // macOS doesn't define B460800+ — use IOSSIOSPEED ioctl.
            posix_speed = B230400;  // Placeholder for termios setup
            need_iossiospeed = true;
            break;
    }
    cfsetospeed(&tty, posix_speed);
    cfsetispeed(&tty, posix_speed);
    tty.c_cflag = (tty.c_cflag & ~(PARENB|CSTOPB|CSIZE|CRTSCTS)) | CS8 | CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ISIG);
    tty.c_iflag &= ~(IXON|IXOFF|IXANY|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~(OPOST|ONLCR);
    tty.c_cc[VTIME] = 1; tty.c_cc[VMIN] = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return -1; }

    // For non-standard baud rates (>230400), use the macOS IOKit ioctl.
    // IOSSIOSPEED accepts any arbitrary integer baud rate and programs
    // the UART directly — this is the only way to reach 2M baud on macOS.
    if (need_iossiospeed) {
        speed_t custom_speed = (speed_t)baud;
        if (ioctl(fd, IOSSIOSPEED, &custom_speed) == -1) {
            close(fd);
            return -1;
        }
    }

    tcflush(fd, TCIOFLUSH);
    return fd;
}

static void send_move_traced(int dx, int dy) {
    trace_record_cmd((int16_t)dx, (int16_t)dy);
    if (g_serial_fd < 0) return;

    uint8_t buf[64];
    int len = proto_fmt_move(&g_proto, buf, sizeof(buf), (int16_t)dx, (int16_t)dy);
    if (len <= 0) return;

    ssize_t w = write(g_serial_fd, buf, len);
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        usleep(200);
        w = write(g_serial_fd, buf, len);
    }
    if (w > 0) __sync_fetch_and_add(&g_stat_sent, 1);
    usleep(100);
}

// Serial reader thread
static pthread_t g_reader_thread;
static volatile bool g_reader_running = false;

static void response_cb(proto_result_t result, const uint8_t* data,
                        uint16_t len, void* ctx) {
    (void)data; (void)len; (void)ctx;
    if (result == PROTO_OK) __sync_fetch_and_add(&g_stat_ok, 1);
    else __sync_fetch_and_add(&g_stat_err, 1);
}

static void* serial_reader_fn(void* arg) {
    (void)arg;
    uint8_t buf[256];
    while (g_reader_running) {
        ssize_t n = read(g_serial_fd, buf, sizeof(buf));
        if (n > 0) {
            proto_parse(&g_proto, buf, n, response_cb, NULL);
        } else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
            break;
        } else {
            usleep(1000);
        }
    }
    return NULL;
}

// ============================================================================
// Test Functions (same patterns as original, using protocol abstraction)
// ============================================================================

static void aimbot_converge(float tx, float ty, float smooth, int frame_us, int maxf) {
    float rx = tx, ry = ty;
    for (int f = 0; f < maxf; f++) {
        float mx = rx / smooth, my = ry / smooth;
        int dx = (int)mx, dy = (int)my;
        if (dx > 127) dx = 127; if (dx < -127) dx = -127;
        if (dy > 127) dy = 127; if (dy < -127) dy = -127;
        if (dx == 0 && dy == 0) break;
        send_move_traced(dx, dy);
        rx -= dx; ry -= dy;
        if (fabsf(rx) < 0.5f && fabsf(ry) < 0.5f) break;
        usleep(frame_us);
    }
}

typedef struct {
    const char* name;
    const char* category;
    const char* description;
    void (*run)(void);
} test_def_t;

// Forward declarations for all test functions
static void test_rapid(void);
static void test_precise(void);
static void test_flicks(void);
static void test_sweep(void);
static void test_mixed(void);
static void test_aim_approach(void);
static void test_aim_flick(void);
static void test_aim_recoil(void);
static void test_aim_track(void);
static void test_aim_full(void);
static void test_diag_tremor(void);
static void test_diag_line(void);
static void test_diag_repeat(void);
static void test_diag_overshoot(void);
static void test_diag_ease(void);

static const test_def_t all_tests[] = {
    {"rapid",          "Basic",  "10k iterations ±10px, max throughput",          test_rapid},
    {"precise",        "Basic",  "1-5px movements @ 500Hz",                      test_precise},
    {"flicks",         "Basic",  "±127px flicks @ 125Hz",                        test_flicks},
    {"sweep",          "Basic",  "Horizontal sweep @ 1kHz",                      test_sweep},
    {"mixed",          "Basic",  "Micro-adjustments + flicks @ 500Hz",           test_mixed},
    {"aim_approach",   "Aimbot", "Proportional controller approach @ 60Hz",      test_aim_approach},
    {"aim_flick",      "Aimbot", "Flick sequences with target switching",        test_aim_flick},
    {"aim_recoil",     "Aimbot", "AK-47 recoil compensation pattern",            test_aim_recoil},
    {"aim_track",      "Aimbot", "Moving target tracking with phase lag",        test_aim_track},
    {"aim_full",       "Aimbot", "Full engagement: flick→track→spray",           test_aim_full},
    {"diag_tremor",    "Diag",   "Send (0,0) for 3s — isolate tremor",           test_diag_tremor},
    {"diag_line",      "Diag",   "Identical (5,0) @ 1kHz — check perp jitter",  test_diag_line},
    {"diag_repeat",    "Diag",   "Constant (3,3) @ 500Hz — delta breaking",     test_diag_repeat},
    {"diag_overshoot", "Diag",   "50px moves w/ gaps — check overshoot",        test_diag_overshoot},
    {"diag_ease",      "Diag",   "Single 100px move — velocity S-curve",        test_diag_ease},
    {NULL, NULL, NULL, NULL}
};

#define NUM_TESTS 15

// Test implementations (condensed from original)
static void test_rapid(void) {
    trace_start("rapid");
    for (int i = 0; i < 10000; i++) {
        send_move_traced(0, 10); send_move_traced(0, -10);
    }
    trace_stop();
}
static void test_precise(void) {
    int mv[][2] = {{1,0},{0,1},{-1,0},{0,-1},{2,0},{0,2},{-2,0},{0,-2},
                   {3,3},{-3,3},{-3,-3},{3,-3},{5,0},{0,5},{-5,0},{0,-5}};
    trace_start("precise"); usleep(200000);
    for (int r = 0; r < 40; r++)
        for (int i = 0; i < 16; i++) { send_move_traced(mv[i][0],mv[i][1]); usleep(2000); }
    trace_stop();
}
static void test_flicks(void) {
    int fl[][2] = {{127,0},{-127,0},{0,127},{0,-127},{90,90},{-90,-90},{90,-90},{-90,90}};
    trace_start("flicks"); usleep(200000);
    for (int r = 0; r < 20; r++)
        for (int i = 0; i < 8; i++) { send_move_traced(fl[i][0],fl[i][1]); usleep(8000); }
    trace_stop();
}
static void test_sweep(void) {
    trace_start("sweep");
    for (int i = 0; i < 500; i++) { send_move_traced(5, 0); usleep(1000); }
    usleep(100000);
    for (int i = 0; i < 500; i++) { send_move_traced(-5, 0); usleep(1000); }
    trace_stop();
}
static void test_mixed(void) {
    trace_start("mixed");
    for (int c = 0; c < 8; c++) {
        for (int i=0;i<100;i++) { send_move_traced(2,1); usleep(2000); }
        send_move_traced(-100,-50); usleep(20000);
        for (int i=0;i<60;i++) { send_move_traced(-1,2); usleep(2000); }
        send_move_traced(80,40); usleep(30000);
        for (int i=0;i<40;i++) { send_move_traced(0,-3); usleep(1000); }
    }
    trace_stop();
}
static void test_aim_approach(void) {
    trace_start("aim_approach"); usleep(200000);
    aimbot_converge(50,30,0.8f,16667,60); usleep(500000);
    aimbot_converge(-50,-30,0.8f,16667,60); usleep(500000);
    aimbot_converge(200,-100,0.8f,16667,120); usleep(500000);
    aimbot_converge(-200,100,0.8f,16667,120); usleep(500000);
    aimbot_converge(500,-300,0.8f,16667,200); usleep(500000);
    aimbot_converge(-500,300,0.8f,16667,200); usleep(300000);
    aimbot_converge(200,100,1.5f,16667,200); usleep(500000);
    aimbot_converge(-200,-100,1.5f,16667,200);
    trace_stop();
}
static void test_aim_flick(void) {
    struct { float x,y; } tgts[] = {{300,-50},{-150,200},{400,100},{-250,-150},{100,50}};
    trace_start("aim_flick"); usleep(200000);
    for (int rep = 0; rep < 3; rep++) {
        float cx=0,cy=0;
        for (int t=0;t<5;t++) {
            aimbot_converge(tgts[t].x-cx,tgts[t].y-cy,0.8f,16667,80);
            cx=tgts[t].x; cy=tgts[t].y;
            usleep(100000 + (rand()%200000));
        }
        aimbot_converge(-cx,-cy,0.8f,16667,120); usleep(500000);
    }
    trace_stop();
}
static void test_aim_recoil(void) {
    int ry[]={-8,-9,-10,-11,-12,-11,-10,-9,-8,-7,-6,-5,-5,-4,-4,-3,-3,-3,-2,-2,-2,-1,-1,-1,-1,0,0,0,0,0};
    int rx[]={ 0, 0, 1, 0,-1, 0, 1, 2, 1, 0,-1,-2,-3,-2,-1, 0, 1, 2, 3, 2, 1, 0,-1,-2,-1,0,1,0,-1,0};
    trace_start("aim_recoil"); usleep(200000);
    for (int burst=0;burst<6;burst++) {
        for (int b=0;b<30;b++) {
            for (int f=0;f<6;f++) {
                int dx = (f==0)?rx[b]:0;
                int dy = ry[b]/6;
                if (f < abs(ry[b])%6) dy += (ry[b]<0)?-1:1;
                send_move_traced(dx,dy); usleep(2800);
            }
        }
        usleep(800000);
    }
    trace_stop();
}
static void test_aim_track(void) {
    trace_start("aim_track"); usleep(200000);
    float cx=0,cy=0;
    for (int f=0;f<600;f++) {
        float t=f/60.0f;
        float tx=150*sinf(2*M_PI*0.5f*t), ty=30*sinf(2*M_PI*1.0f*t);
        float mx=(tx-cx)/0.8f, my=(ty-cy)/0.8f;
        int dx=(int)mx, dy=(int)my;
        if(dx>127)dx=127;if(dx<-127)dx=-127;if(dy>127)dy=127;if(dy<-127)dy=-127;
        if(dx||dy){send_move_traced(dx,dy);cx+=dx;cy+=dy;}
        usleep(16667);
    }
    cx=0;cy=0;
    for (int f=0;f<300;f++) {
        float t=f/60.0f;
        float tx=200*sinf(2*M_PI*2.0f*t), ty=50*cosf(2*M_PI*1.5f*t);
        float mx=(tx-cx)/0.8f, my=(ty-cy)/0.8f;
        int dx=(int)mx, dy=(int)my;
        if(dx>127)dx=127;if(dx<-127)dx=-127;if(dy>127)dy=127;if(dy<-127)dy=-127;
        if(dx||dy){send_move_traced(dx,dy);cx+=dx;cy+=dy;}
        usleep(16667);
    }
    trace_stop();
}
static void test_aim_full(void) {
    trace_start("aim_full"); usleep(200000);
    float cx=0,cy=0;
    for (int e=0;e<4;e++) {
        float tx=(float)((rand()%600)-300), ty=(float)((rand()%300)-150);
        aimbot_converge(tx-cx,ty-cy,0.8f,16667,80);
        cx=tx;cy=ty;
        float vx=(float)((rand()%100)-50)/10.0f;
        int bullets=8+(rand()%15);
        for (int b=0;b<bullets;b++) {
            tx+=vx; ty+=(float)((rand()%10)-5)*0.3f;
            float adx=(tx-cx)/0.8f, ady=(ty-cy)/0.8f;
            int rcy=(b<10)?-(8+b):-(18-b/2);
            for (int f=0;f<6;f++) {
                int dx=(f==0)?(int)adx:0;
                int dy=rcy/6; if(f<abs(rcy)%6)dy+=(rcy<0)?-1:1;
                send_move_traced(dx,dy); usleep(2800);
            }
            cx+=(int)adx; cy+=(int)ady;
        }
        usleep(300000+(rand()%500000));
    }
    aimbot_converge(-cx,-cy,1.5f,16667,200);
    trace_stop();
}
static void test_diag_tremor(void) {
    trace_start("diag_tremor"); usleep(300000);
    for (int i=0;i<3000;i++) { send_move_traced(0,0); usleep(1000); }
    usleep(200000); trace_stop();
}
static void test_diag_line(void) {
    trace_start("diag_line"); usleep(300000);
    for (int i=0;i<2000;i++) { send_move_traced(5,0); usleep(1000); }
    usleep(200000); trace_stop();
}
static void test_diag_repeat(void) {
    trace_start("diag_repeat"); usleep(300000);
    for (int i=0;i<1000;i++) { send_move_traced(3,3); usleep(2000); }
    usleep(200000); trace_stop();
}
static void test_diag_overshoot(void) {
    trace_start("diag_overshoot"); usleep(300000);
    for (int i=0;i<20;i++) {
        send_move_traced(50*((i%2==0)?1:-1),0); usleep(500000);
    }
    usleep(200000); trace_stop();
}
static void test_diag_ease(void) {
    trace_start("diag_ease"); usleep(500000);
    send_move_traced(100,0);usleep(1000000);
    send_move_traced(0,100);usleep(1000000);
    send_move_traced(70,70);usleep(1000000);
    send_move_traced(-100,0);usleep(1000000);
    send_move_traced(0,-100);usleep(1000000);
    send_move_traced(-70,-70);usleep(1000000);
    trace_stop();
}

// ============================================================================
// Forward declarations for Objective-C classes
// ============================================================================

// ============================================================================
// TraceCanvasView - Hardware-accelerated trace renderer
// ============================================================================

@interface TraceCanvasView : NSView
@property (nonatomic) CGFloat viewX, viewY, viewScale;
@property (nonatomic) BOOL showCmd, showObs, showDots, showDev, showGrid, showSpeed;
@property (nonatomic) CGFloat lineWidth, dotSize;
@property (nonatomic) NSPoint dragStart, dragViewStart;
@property (nonatomic) BOOL dragging;
@property (nonatomic) int hoverIdx;
@property (nonatomic) NSPoint mousePos;
@end

@implementation TraceCanvasView

- (instancetype)initWithFrame:(NSRect)frame {
    self = [super initWithFrame:frame];
    if (self) {
        _viewX = 0; _viewY = 0; _viewScale = 1.0;
        _showCmd = YES; _showObs = YES; _showDots = YES;
        _showDev = NO; _showGrid = YES; _showSpeed = YES;
        _lineWidth = 1.5; _dotSize = 1.0;
        _hoverIdx = -1;
        self.wantsLayer = YES;
        self.layer.backgroundColor = [col_bg0 CGColor];
        // Enable tracking for hover
        NSTrackingArea* ta = [[NSTrackingArea alloc]
            initWithRect:NSZeroRect
            options:NSTrackingMouseMoved|NSTrackingActiveInKeyWindow|NSTrackingInVisibleRect
            owner:self userInfo:nil];
        [self addTrackingArea:ta];
    }
    return self;
}

- (BOOL)isFlipped { return YES; }
- (BOOL)acceptsFirstResponder { return YES; }

- (void)fitView {
    double rx = g_trace.all_max_x - g_trace.all_min_x;
    double ry = g_trace.all_max_y - g_trace.all_min_y;
    if (rx < 10) rx = 10; if (ry < 10) ry = 10;
    CGFloat W = self.bounds.size.width, H = self.bounds.size.height;
    CGFloat pad = 60;
    _viewScale = fmin((W - pad*2) / rx, (H - pad*2) / ry) * 0.9;
    double cx = g_trace.all_min_x + rx/2, cy = g_trace.all_min_y + ry/2;
    _viewX = W/2 - cx * _viewScale;
    _viewY = H/2 - cy * _viewScale;
}

- (NSPoint)toScreen:(double)px y:(double)py {
    return NSMakePoint(px * _viewScale + _viewX, py * _viewScale + _viewY);
}

- (NSPoint)toData:(CGFloat)sx y:(CGFloat)sy {
    return NSMakePoint((sx - _viewX) / _viewScale, (sy - _viewY) / _viewScale);
}

- (void)speedColor:(double)s max:(double)mx r:(CGFloat*)r g:(CGFloat*)g b:(CGFloat*)b {
    double t = fmin(s / (mx * 0.5 + 0.001), 1.0);
    if (t < 0.33) {
        *r = (70 + t*3*100)/255.0; *g = (140 + t*3*60)/255.0; *b = 240/255.0;
    } else if (t < 0.66) {
        double u = (t - 0.33) * 3;
        *r = (70 + u*180)/255.0; *g = (200 - u*20)/255.0; *b = (240 - u*200)/255.0;
    } else {
        double u = (t - 0.66) * 3;
        *r = 240/255.0; *g = (180 - u*140)/255.0; *b = (40 - u*20)/255.0;
    }
}

- (void)drawRect:(NSRect)dirtyRect {
    CGContextRef ctx = [[NSGraphicsContext currentContext] CGContext];
    CGFloat W = self.bounds.size.width, H = self.bounds.size.height;

    // Background
    CGContextSetRGBFillColor(ctx, 6/255.0, 6/255.0, 10/255.0, 1);
    CGContextFillRect(ctx, self.bounds);

    uint32_t ncmd = g_trace.cmd_count;
    uint32_t nobs = g_trace.obs_count;

    // Grid
    if (_showGrid) {
        double gs = pow(10, floor(log10(100.0 / _viewScale)));
        NSPoint d0 = [self toData:0 y:0], d1 = [self toData:W y:H];

        CGContextSetRGBStrokeColor(ctx, 22/255.0, 22/255.0, 32/255.0, 1);
        CGContextSetLineWidth(ctx, 0.5);

        NSDictionary* gridFont = @{
            NSFontAttributeName: [NSFont fontWithName:@"JetBrains Mono" size:9]
                ?: [NSFont monospacedSystemFontOfSize:9 weight:NSFontWeightRegular],
            NSForegroundColorAttributeName: CLR(42, 42, 54, 1.0)
        };

        for (double gx = floor(d0.x/gs)*gs; gx <= d1.x; gx += gs) {
            CGFloat sx = gx * _viewScale + _viewX;
            CGContextMoveToPoint(ctx, sx, 0); CGContextAddLineToPoint(ctx, sx, H);
            CGContextStrokePath(ctx);
            if (fabs(gx) < 0.01) {
                CGContextSetRGBStrokeColor(ctx, 37/255.0, 37/255.0, 53/255.0, 1);
                CGContextSetLineWidth(ctx, 1);
                CGContextMoveToPoint(ctx, sx, 0); CGContextAddLineToPoint(ctx, sx, H);
                CGContextStrokePath(ctx);
                CGContextSetRGBStrokeColor(ctx, 22/255.0, 22/255.0, 32/255.0, 1);
                CGContextSetLineWidth(ctx, 0.5);
            }
            NSString* lbl = [NSString stringWithFormat:@"%.0f", gx];
            [lbl drawAtPoint:NSMakePoint(sx+2, 2) withAttributes:gridFont];
        }
        for (double gy = floor(d0.y/gs)*gs; gy <= d1.y; gy += gs) {
            CGFloat sy = gy * _viewScale + _viewY;
            CGContextMoveToPoint(ctx, 0, sy); CGContextAddLineToPoint(ctx, W, sy);
            CGContextStrokePath(ctx);
            if (fabs(gy) < 0.01) {
                CGContextSetRGBStrokeColor(ctx, 37/255.0, 37/255.0, 53/255.0, 1);
                CGContextSetLineWidth(ctx, 1);
                CGContextMoveToPoint(ctx, 0, sy); CGContextAddLineToPoint(ctx, W, sy);
                CGContextStrokePath(ctx);
                CGContextSetRGBStrokeColor(ctx, 22/255.0, 22/255.0, 32/255.0, 1);
                CGContextSetLineWidth(ctx, 0.5);
            }
            NSString* lbl = [NSString stringWithFormat:@"%.0f", gy];
            [lbl drawAtPoint:NSMakePoint(2, sy+2) withAttributes:gridFont];
        }
    }

    // Origin crosshair
    NSPoint org = [self toScreen:0 y:0];
    CGContextSetRGBStrokeColor(ctx, 51/255.0, 51/255.0, 64/255.0, 1);
    CGContextSetLineWidth(ctx, 1);
    CGContextAddArc(ctx, org.x, org.y, 6, 0, 2*M_PI, 0); CGContextStrokePath(ctx);
    CGContextMoveToPoint(ctx, org.x-8, org.y);
    CGContextAddLineToPoint(ctx, org.x+8, org.y); CGContextStrokePath(ctx);
    CGContextMoveToPoint(ctx, org.x, org.y-8);
    CGContextAddLineToPoint(ctx, org.x, org.y+8); CGContextStrokePath(ctx);

    if (ncmd < 2 && nobs < 2) {
        // Empty state message
        NSDictionary* emptyFont = @{
            NSFontAttributeName: [NSFont fontWithName:@"JetBrains Mono" size:14]
                ?: [NSFont monospacedSystemFontOfSize:14 weight:NSFontWeightMedium],
            NSForegroundColorAttributeName: col_text3
        };
        NSString* msg = @"Run a test to see trace data";
        NSSize sz = [msg sizeWithAttributes:emptyFont];
        [msg drawAtPoint:NSMakePoint(W/2-sz.width/2, H/2-sz.height/2) withAttributes:emptyFont];
        return;
    }

    // Compute speed arrays for coloring
    double cmdMaxSpd = 0, obsMaxSpd = 0;
    if (_showSpeed) {
        for (uint32_t i = 1; i < ncmd; i++) {
            double dt = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
            if (dt <= 0) continue;
            double dx = g_trace.cmds[i].cum_x - g_trace.cmds[i-1].cum_x;
            double dy = g_trace.cmds[i].cum_y - g_trace.cmds[i-1].cum_y;
            double s = hypot(dx,dy) / (dt/1000.0);
            if (s > cmdMaxSpd) cmdMaxSpd = s;
        }
        for (uint32_t i = 1; i < nobs; i++) {
            double dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i-1].time_us);
            if (dt <= 0) continue;
            double dx = g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x;
            double dy = g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y;
            double s = hypot(dx,dy) / (dt/1000.0);
            if (s > obsMaxSpd) obsMaxSpd = s;
        }
    }
    double globalMaxSpd = fmax(cmdMaxSpd, obsMaxSpd);

    // Deviation lines
    if (_showDev && _showCmd && _showObs && nobs >= 2) {
        CGContextSetRGBStrokeColor(ctx, 240/255.0, 64/255.0, 128/255.0, 0.3);
        CGContextSetLineWidth(ctx, 0.5);
        uint32_t step = (ncmd > 300) ? ncmd/300 : 1;
        uint32_t oi = 0;
        for (uint32_t ci = 0; ci < ncmd; ci += step) {
            uint64_t ct = g_trace.cmds[ci].time_us;
            if (ct < g_trace.obs[0].time_us) continue;
            if (ct > g_trace.obs[nobs-1].time_us) break;
            while (oi+1 < nobs && g_trace.obs[oi+1].time_us <= ct) oi++;
            if (oi+1 >= nobs) break;
            uint64_t t0=g_trace.obs[oi].time_us, t1=g_trace.obs[oi+1].time_us;
            if (t1<=t0 || ct<t0) continue;
            double fr = (double)(ct-t0)/(double)(t1-t0);
            if (fr<0||fr>1) continue;
            double ix = g_trace.obs[oi].rel_x + fr*(g_trace.obs[oi+1].rel_x - g_trace.obs[oi].rel_x);
            double iy = g_trace.obs[oi].rel_y + fr*(g_trace.obs[oi+1].rel_y - g_trace.obs[oi].rel_y);
            NSPoint p1 = [self toScreen:g_trace.cmds[ci].cum_x y:g_trace.cmds[ci].cum_y];
            NSPoint p2 = [self toScreen:ix y:iy];
            CGContextMoveToPoint(ctx, p1.x, p1.y);
            CGContextAddLineToPoint(ctx, p2.x, p2.y);
            CGContextStrokePath(ctx);
        }
    }

    // Draw a path (observed or commanded)
    void (^drawPath)(BOOL isCmd) = ^(BOOL isCmd) {
        uint32_t n = isCmd ? ncmd : nobs;
        if (n < 2) return;

        CGContextSetLineWidth(ctx, self.lineWidth);
        CGContextSetLineCap(ctx, kCGLineCapRound);
        CGContextSetLineJoin(ctx, kCGLineJoinRound);

        double prevSpd = 0;
        for (uint32_t i = 1; i < n; i++) {
            NSPoint p0, p1;
            if (isCmd) {
                p0 = [self toScreen:g_trace.cmds[i-1].cum_x y:g_trace.cmds[i-1].cum_y];
                p1 = [self toScreen:g_trace.cmds[i].cum_x y:g_trace.cmds[i].cum_y];
            } else {
                p0 = [self toScreen:g_trace.obs[i-1].rel_x y:g_trace.obs[i-1].rel_y];
                p1 = [self toScreen:g_trace.obs[i].rel_x y:g_trace.obs[i].rel_y];
            }

            if (self.showSpeed) {
                double dt, dx, dy;
                if (isCmd) {
                    dt = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
                    dx = g_trace.cmds[i].cum_x - g_trace.cmds[i-1].cum_x;
                    dy = g_trace.cmds[i].cum_y - g_trace.cmds[i-1].cum_y;
                } else {
                    dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i-1].time_us);
                    dx = g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x;
                    dy = g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y;
                }
                double spd = (dt > 0) ? hypot(dx,dy)/(dt/1000.0) : prevSpd;
                prevSpd = spd;
                CGFloat r,g,b;
                [self speedColor:spd max:globalMaxSpd r:&r g:&g b:&b];
                CGContextSetRGBStrokeColor(ctx, r, g, b, 1);
            } else {
                if (isCmd) CGContextSetRGBStrokeColor(ctx, 78/255.0, 154/255.0, 240/255.0, 1);
                else       CGContextSetRGBStrokeColor(ctx, 240/255.0, 128/255.0, 64/255.0, 1);
            }

            CGContextMoveToPoint(ctx, p0.x, p0.y);
            CGContextAddLineToPoint(ctx, p1.x, p1.y);
            CGContextStrokePath(ctx);
        }

        // Dots (decimated)
        if (self.showDots) {
            uint32_t step = (n > 3000) ? n / 3000 : 1;
            for (uint32_t i = 0; i < n; i += step) {
                NSPoint p;
                if (isCmd) p = [self toScreen:g_trace.cmds[i].cum_x y:g_trace.cmds[i].cum_y];
                else       p = [self toScreen:g_trace.obs[i].rel_x y:g_trace.obs[i].rel_y];

                if (self.showSpeed && i > 0) {
                    double dt, dx, dy;
                    if (isCmd) {
                        dt = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i>0?i-1:0].time_us);
                        dx = g_trace.cmds[i].cum_x - g_trace.cmds[i>0?i-1:0].cum_x;
                        dy = g_trace.cmds[i].cum_y - g_trace.cmds[i>0?i-1:0].cum_y;
                    } else {
                        dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i>0?i-1:0].time_us);
                        dx = g_trace.obs[i].rel_x - g_trace.obs[i>0?i-1:0].rel_x;
                        dy = g_trace.obs[i].rel_y - g_trace.obs[i>0?i-1:0].rel_y;
                    }
                    double spd = (dt > 0) ? hypot(dx,dy)/(dt/1000.0) : 0;
                    CGFloat r,g,b;
                    [self speedColor:spd max:globalMaxSpd r:&r g:&g b:&b];
                    CGContextSetRGBFillColor(ctx, r, g, b, 1);
                } else {
                    if (isCmd) CGContextSetRGBFillColor(ctx, 78/255.0, 154/255.0, 240/255.0, 1);
                    else       CGContextSetRGBFillColor(ctx, 240/255.0, 128/255.0, 64/255.0, 1);
                }
                CGContextAddArc(ctx, p.x, p.y, self.dotSize, 0, 2*M_PI, 0);
                CGContextFillPath(ctx);
            }
        }

        // Start/end markers
        NSPoint sp, ep;
        if (isCmd) {
            sp = [self toScreen:g_trace.cmds[0].cum_x y:g_trace.cmds[0].cum_y];
            ep = [self toScreen:g_trace.cmds[n-1].cum_x y:g_trace.cmds[n-1].cum_y];
        } else {
            sp = [self toScreen:g_trace.obs[0].rel_x y:g_trace.obs[0].rel_y];
            ep = [self toScreen:g_trace.obs[n-1].rel_x y:g_trace.obs[n-1].rel_y];
        }
        CGContextSetRGBFillColor(ctx, 64/255.0, 216/255.0, 128/255.0, 1);
        CGContextAddArc(ctx, sp.x, sp.y, 4, 0, 2*M_PI, 0); CGContextFillPath(ctx);
        CGContextSetRGBFillColor(ctx, 240/255.0, 64/255.0, 96/255.0, 1);
        CGContextAddArc(ctx, ep.x, ep.y, 4, 0, 2*M_PI, 0); CGContextFillPath(ctx);
    };

    // Draw observed underneath, commanded on top
    if (_showObs && nobs >= 2) drawPath(NO);
    if (_showCmd && ncmd >= 2) drawPath(YES);

    // Hover tooltip
    if (_hoverIdx >= 0 && (uint32_t)_hoverIdx < ncmd) {
        uint32_t i = (uint32_t)_hoverIdx;
        NSPoint sp = [self toScreen:g_trace.cmds[i].cum_x y:g_trace.cmds[i].cum_y];

        // Highlight circle
        CGContextSetRGBStrokeColor(ctx, 1, 1, 1, 0.6);
        CGContextSetLineWidth(ctx, 1.5);
        CGContextAddArc(ctx, sp.x, sp.y, 6, 0, 2*M_PI, 0); CGContextStrokePath(ctx);

        // Tooltip background
        NSString* tip = [NSString stringWithFormat:@"Cmd #%d\nt=%.1f ms\npos=(%d, %d)\ndelta=(%d, %d)",
            i, (double)g_trace.cmds[i].time_us/1000.0,
            g_trace.cmds[i].cum_x, g_trace.cmds[i].cum_y,
            g_trace.cmds[i].dx, g_trace.cmds[i].dy];

        NSDictionary* tipAttr = @{
            NSFontAttributeName: [NSFont fontWithName:@"JetBrains Mono" size:11]
                ?: [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular],
            NSForegroundColorAttributeName: col_text0
        };
        NSSize tsz = [tip sizeWithAttributes:tipAttr];
        CGFloat tx = _mousePos.x + 14, ty = _mousePos.y - 10;
        if (tx + tsz.width + 16 > W) tx = _mousePos.x - tsz.width - 24;

        CGContextSetRGBFillColor(ctx, 0, 0, 0, 0.88);
        CGRect tipRect = CGRectMake(tx - 6, ty - 4, tsz.width + 16, tsz.height + 12);
        CGFloat radius = 4;
        CGPathRef path = CGPathCreateWithRoundedRect(tipRect, radius, radius, NULL);
        CGContextAddPath(ctx, path); CGContextFillPath(ctx);
        CGPathRelease(path);
        CGContextSetRGBStrokeColor(ctx, 51/255.0, 51/255.0, 51/255.0, 1);
        CGContextSetLineWidth(ctx, 1);
        path = CGPathCreateWithRoundedRect(tipRect, radius, radius, NULL);
        CGContextAddPath(ctx, path); CGContextStrokePath(ctx);
        CGPathRelease(path);

        [tip drawAtPoint:NSMakePoint(tx + 2, ty + 2) withAttributes:tipAttr];
    }
}

// ---- Mouse interaction ----

- (void)mouseDown:(NSEvent*)event {
    if (event.clickCount == 2) {
        [self fitView]; [self setNeedsDisplay:YES]; return;
    }
    NSPoint loc = [self convertPoint:event.locationInWindow fromView:nil];
    _dragging = YES; _dragStart = loc;
    _dragViewStart = NSMakePoint(_viewX, _viewY);
}
- (void)mouseDragged:(NSEvent*)event {
    if (!_dragging) return;
    NSPoint loc = [self convertPoint:event.locationInWindow fromView:nil];
    _viewX = _dragViewStart.x + (loc.x - _dragStart.x);
    _viewY = _dragViewStart.y + (loc.y - _dragStart.y);
    [self setNeedsDisplay:YES];
}
- (void)mouseUp:(NSEvent*)event { (void)event; _dragging = NO; }

- (void)mouseMoved:(NSEvent*)event {
    NSPoint loc = [self convertPoint:event.locationInWindow fromView:nil];
    _mousePos = loc;
    uint32_t ncmd = g_trace.cmd_count;
    int best = -1; double bestD = 15;
    for (uint32_t i = 0; i < ncmd; i++) {
        NSPoint sp = [self toScreen:g_trace.cmds[i].cum_x y:g_trace.cmds[i].cum_y];
        double d = hypot(sp.x - loc.x, sp.y - loc.y);
        if (d < bestD) { bestD = d; best = (int)i; }
    }
    if (best != _hoverIdx) { _hoverIdx = best; [self setNeedsDisplay:YES]; }
}

- (void)scrollWheel:(NSEvent*)event {
    NSPoint loc = [self convertPoint:event.locationInWindow fromView:nil];
    CGFloat factor = (event.deltaY < 0) ? (1.0/1.15) : 1.15;
    _viewX = loc.x - (loc.x - _viewX) * factor;
    _viewY = loc.y - (loc.y - _viewY) * factor;
    _viewScale *= factor;
    [self setNeedsDisplay:YES];
}

@end

// ============================================================================
// AppDelegate - Window setup, controls, test management
// ============================================================================

@interface AppDelegate : NSObject <NSApplicationDelegate, NSToolbarDelegate>
@property (strong) NSWindow* window;
@property (strong) TraceCanvasView* canvas;
@property (strong) NSScrollView* sidebarScroll;
@property (strong) NSView* sidebarContent;
@property (strong) NSPopUpButton* protoPopup;
@property (strong) NSPopUpButton* testPopup;
@property (strong) NSTextField* portField;
@property (strong) NSButton* connectBtn;
@property (strong) NSButton* runBtn;
@property (strong) NSTextField* statusLabel;
@property (strong) NSTimer* refreshTimer;
@property (nonatomic) BOOL connected;
@property (nonatomic) BOOL testRunning;
@end

@implementation AppDelegate

- (void)applicationDidFinishLaunching:(NSNotification*)notification {
    (void)notification;
    init_colors();
    trace_alloc();
    srand((unsigned)time(NULL));
    proto_init(&g_proto, DEV_KMBOX);

    // ---- Main Window ----
    NSRect screenRect = [[NSScreen mainScreen] visibleFrame];
    CGFloat winW = fmin(1400, screenRect.size.width * 0.85);
    CGFloat winH = fmin(900, screenRect.size.height * 0.85);
    NSRect winRect = NSMakeRect(
        screenRect.origin.x + (screenRect.size.width - winW)/2,
        screenRect.origin.y + (screenRect.size.height - winH)/2,
        winW, winH);

    _window = [[NSWindow alloc] initWithContentRect:winRect
        styleMask:NSWindowStyleMaskTitled|NSWindowStyleMaskClosable|
                  NSWindowStyleMaskResizable|NSWindowStyleMaskMiniaturizable
        backing:NSBackingStoreBuffered defer:NO];
    _window.title = @"KMBox Trace Analyzer";
    _window.minSize = NSMakeSize(800, 500);
    _window.backgroundColor = col_bg0;
    _window.titlebarAppearsTransparent = YES;
    _window.appearance = [NSAppearance appearanceNamed:NSAppearanceNameDarkAqua];

    // ---- Content layout ----
    NSView* content = _window.contentView;
    content.wantsLayer = YES;
    content.layer.backgroundColor = [col_bg0 CGColor];

    // Top control bar
    NSView* topBar = [[NSView alloc] initWithFrame:NSMakeRect(0, winH-52, winW, 52)];
    topBar.wantsLayer = YES;
    topBar.layer.backgroundColor = [col_bg1 CGColor];
    topBar.autoresizingMask = NSViewWidthSizable|NSViewMinYMargin;
    [content addSubview:topBar];

    CGFloat x = 12;
    // Protocol picker
    NSTextField* protoLabel = [NSTextField labelWithString:@"Protocol"];
    protoLabel.font = [NSFont systemFontOfSize:10 weight:NSFontWeightMedium];
    protoLabel.textColor = col_text2;
    protoLabel.frame = NSMakeRect(x, 30, 60, 16);
    [topBar addSubview:protoLabel];

    _protoPopup = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(x, 4, 110, 26) pullsDown:NO];
    [_protoPopup addItemsWithTitles:@[@"KMBox B+", @"Ferrum One", @"MAKCU"]];
    _protoPopup.font = [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
    [_protoPopup setTarget:self]; [_protoPopup setAction:@selector(protoChanged:)];
    [topBar addSubview:_protoPopup];
    x += 120;

    // Port field
    NSTextField* portLabel = [NSTextField labelWithString:@"Serial Port"];
    portLabel.font = [NSFont systemFontOfSize:10 weight:NSFontWeightMedium];
    portLabel.textColor = col_text2;
    portLabel.frame = NSMakeRect(x, 30, 80, 16);
    [topBar addSubview:portLabel];

    _portField = [[NSTextField alloc] initWithFrame:NSMakeRect(x, 4, 200, 26)];
    _portField.stringValue = @"/dev/tty.usbmodem2101";
    _portField.font = [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
    _portField.placeholderString = @"/dev/tty.usbmodem...";
    [topBar addSubview:_portField];
    x += 210;

    // Connect button
    _connectBtn = [[NSButton alloc] initWithFrame:NSMakeRect(x, 4, 80, 26)];
    _connectBtn.title = @"Connect";
    _connectBtn.bezelStyle = NSBezelStyleRounded;
    [_connectBtn setTarget:self]; [_connectBtn setAction:@selector(toggleConnect:)];
    [topBar addSubview:_connectBtn];
    x += 90;

    // Test picker
    NSTextField* testLabel = [NSTextField labelWithString:@"Test"];
    testLabel.font = [NSFont systemFontOfSize:10 weight:NSFontWeightMedium];
    testLabel.textColor = col_text2;
    testLabel.frame = NSMakeRect(x, 30, 40, 16);
    [topBar addSubview:testLabel];

    _testPopup = [[NSPopUpButton alloc] initWithFrame:NSMakeRect(x, 4, 150, 26) pullsDown:NO];
    for (int i = 0; all_tests[i].name; i++) {
        [_testPopup addItemWithTitle:[NSString stringWithUTF8String:all_tests[i].name]];
    }
    _testPopup.font = [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
    [topBar addSubview:_testPopup];
    x += 160;

    // Run button
    _runBtn = [[NSButton alloc] initWithFrame:NSMakeRect(x, 4, 80, 26)];
    _runBtn.title = @"▶ Run";
    _runBtn.bezelStyle = NSBezelStyleRounded;
    [_runBtn setTarget:self]; [_runBtn setAction:@selector(runTest:)];
    [topBar addSubview:_runBtn];
    x += 90;

    // Status
    _statusLabel = [NSTextField labelWithString:@"Disconnected"];
    _statusLabel.font = [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
    _statusLabel.textColor = col_text2;
    _statusLabel.frame = NSMakeRect(x, 10, 300, 20);
    _statusLabel.autoresizingMask = NSViewWidthSizable;
    [topBar addSubview:_statusLabel];

    // Border line
    NSView* borderLine = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, winW, 1)];
    borderLine.wantsLayer = YES;
    borderLine.layer.backgroundColor = [col_border CGColor];
    borderLine.autoresizingMask = NSViewWidthSizable;
    [topBar addSubview:borderLine];

    // ---- Split: Canvas + Sidebar ----
    CGFloat bodyH = winH - 52;
    CGFloat sideW = 320;

    _canvas = [[TraceCanvasView alloc] initWithFrame:NSMakeRect(0, 0, winW - sideW, bodyH)];
    _canvas.autoresizingMask = NSViewWidthSizable | NSViewHeightSizable;
    [content addSubview:_canvas];

    // Sidebar
    NSView* sidebarBg = [[NSView alloc] initWithFrame:NSMakeRect(winW - sideW, 0, sideW, bodyH)];
    sidebarBg.wantsLayer = YES;
    sidebarBg.layer.backgroundColor = [col_bg1 CGColor];
    sidebarBg.autoresizingMask = NSViewHeightSizable | NSViewMinXMargin;
    [content addSubview:sidebarBg];

    // Sidebar border
    NSView* sideBorder = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 1, bodyH)];
    sideBorder.wantsLayer = YES;
    sideBorder.layer.backgroundColor = [col_border CGColor];
    sideBorder.autoresizingMask = NSViewHeightSizable;
    [sidebarBg addSubview:sideBorder];

    _sidebarScroll = [[NSScrollView alloc] initWithFrame:NSMakeRect(1, 0, sideW-1, bodyH)];
    _sidebarScroll.hasVerticalScroller = YES;
    _sidebarScroll.autoresizingMask = NSViewHeightSizable;
    _sidebarScroll.drawsBackground = NO;
    [sidebarBg addSubview:_sidebarScroll];

    _sidebarContent = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, sideW-17, 800)];
    _sidebarScroll.documentView = _sidebarContent;

    [self buildSidebar];

    [_window makeKeyAndOrderFront:nil];

    // Activate after window exists so the app comes to the foreground.
    // On macOS 14+ activateIgnoringOtherApps: is deprecated; use activate.
    if ([NSApp respondsToSelector:@selector(activate)]) {
        [NSApp performSelector:@selector(activate)];
    } else {
        [NSApp activateIgnoringOtherApps:YES];
    }

    // Refresh timer for live updates during tests
    _refreshTimer = [NSTimer scheduledTimerWithTimeInterval:0.05 target:self
        selector:@selector(refreshTick:) userInfo:nil repeats:YES];
}

- (void)buildSidebar {
    // Clear existing
    for (NSView* v in [_sidebarContent.subviews copy]) [v removeFromSuperview];

    CGFloat w = _sidebarContent.frame.size.width - 24;
    __block CGFloat y = 12;

    NSFont* headerFont = [NSFont fontWithName:@"JetBrains Mono" size:10]
        ?: [NSFont monospacedSystemFontOfSize:10 weight:NSFontWeightBold];
    NSFont* labelFont = [NSFont fontWithName:@"JetBrains Mono" size:11]
        ?: [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightRegular];
    NSFont* valueFont = [NSFont fontWithName:@"JetBrains Mono" size:11]
        ?: [NSFont monospacedSystemFontOfSize:11 weight:NSFontWeightMedium];
    NSFont* bigFont = [NSFont fontWithName:@"JetBrains Mono" size:22]
        ?: [NSFont monospacedSystemFontOfSize:22 weight:NSFontWeightBold];

    void (^addHeader)(NSString*) = ^(NSString* text) {
        NSTextField* lbl = [NSTextField labelWithString:[text uppercaseString]];
        lbl.font = headerFont;
        lbl.textColor = col_text3;
        lbl.frame = NSMakeRect(12, y, w, 14);
        [self.sidebarContent addSubview:lbl];
        y += 20;
    };

    void (^addRow)(NSString*, NSString*, NSColor*) = ^(NSString* label, NSString* value, NSColor* color) {
        NSTextField* ll = [NSTextField labelWithString:label];
        ll.font = labelFont; ll.textColor = col_text2;
        ll.frame = NSMakeRect(12, y, w*0.55, 16);
        [self.sidebarContent addSubview:ll];

        NSTextField* vl = [NSTextField labelWithString:value];
        vl.font = valueFont; vl.textColor = color ?: col_text0;
        vl.alignment = NSTextAlignmentRight;
        vl.frame = NSMakeRect(12 + w*0.55, y, w*0.45, 16);
        [self.sidebarContent addSubview:vl];
        y += 18;
    };

    trace_analysis_t* a = &g_trace.analysis;

    if (!g_trace.analysis_valid) {
        addHeader(@"No Data");
        NSTextField* msg = [NSTextField labelWithString:@"Connect to a device and\nrun a test to see results."];
        msg.font = labelFont; msg.textColor = col_text3;
        msg.frame = NSMakeRect(12, y, w, 40);
        [_sidebarContent addSubview:msg];
        y += 50;
    } else {
        // Movement section
        addHeader(@"Movement");
        addRow(@"Commands", [NSString stringWithFormat:@"%u", g_trace.cmd_count], nil);
        addRow(@"Observations", [NSString stringWithFormat:@"%u", g_trace.obs_count], nil);
        addRow(@"Duration", [NSString stringWithFormat:@"%.1f ms", a->total_ms], nil);
        addRow(@"Cmd rate", [NSString stringWithFormat:@"%.0f Hz", a->cmd_rate], nil);
        addRow(@"Obs rate", [NSString stringWithFormat:@"%.0f Hz", a->obs_rate], nil);
        addRow(@"Cmd bbox", [NSString stringWithFormat:@"%d×%d", a->cmd_bbox_w, a->cmd_bbox_h], nil);
        addRow(@"Obs bbox", [NSString stringWithFormat:@"%.0f×%.0f", a->obs_bbox_w, a->obs_bbox_h], nil);
        y += 8;

        // Fidelity section
        addHeader(@"Command Fidelity");
        NSColor* devCol = (a->dev_avg < 2) ? col_green : (a->dev_avg < 5) ? col_yellow : col_red;
        addRow(@"Avg cmd/obs gap", [NSString stringWithFormat:@"%.2f px", a->dev_avg], devCol);
        addRow(@"Max cmd/obs gap", [NSString stringWithFormat:@"%.2f px", a->dev_max], nil);
        addRow(@"Delta repeats", [NSString stringWithFormat:@"%.0f%%", a->cmd_rep_pct], nil);
        addRow(@"Interval CV", [NSString stringWithFormat:@"%.3f", a->int_cv], nil);
        y += 8;

        // Humanization score
        addHeader(@"Humanization Analysis");
        NSColor* hCol = (a->h_score < 35) ? col_red : (a->h_score < 55) ? col_yellow : col_green;
        NSColor* hBg = (a->h_score < 15) ? CLR(58,21,21,1) : (a->h_score < 35) ? CLR(58,42,21,1) :
                       (a->h_score < 55) ? CLR(42,58,21,1) : CLR(21,58,26,1);

        NSView* scoreBox = [[NSView alloc] initWithFrame:NSMakeRect(12, y, w, 44)];
        scoreBox.wantsLayer = YES;
        scoreBox.layer.backgroundColor = [hBg CGColor];
        scoreBox.layer.cornerRadius = 6;
        [_sidebarContent addSubview:scoreBox];

        NSTextField* scoreLbl = [NSTextField labelWithString:
            [NSString stringWithFormat:@"%d / 100", (int)a->h_score]];
        scoreLbl.font = bigFont; scoreLbl.textColor = hCol;
        scoreLbl.frame = NSMakeRect(10, 8, 120, 28);
        [scoreBox addSubview:scoreLbl];

        NSTextField* gradeLbl = [NSTextField labelWithString:
            [NSString stringWithUTF8String:a->h_grade]];
        gradeLbl.font = valueFont; gradeLbl.textColor = hCol;
        gradeLbl.alignment = NSTextAlignmentRight;
        gradeLbl.frame = NSMakeRect(w - 110, 14, 100, 18);
        [scoreBox addSubview:gradeLbl];
        y += 52;

        addRow(@"Perp scatter", [NSString stringWithFormat:@"%.3f px", a->perp_scatter], nil);
        addRow(@"Obs jitter (avg)", [NSString stringWithFormat:@"%.3f px", a->jit_avg], nil);
        addRow(@"Obs jitter (max)", [NSString stringWithFormat:@"%.3f px", a->jit_max], nil);
        addRow(@"Dir reversals", [NSString stringWithFormat:@"%.1f%%", a->dir_flip_rate], nil);
        addRow(@"Speed CV", [NSString stringWithFormat:@"%.3f", a->speed_cv], nil);
        addRow(@"Sub-pixel noise", [NSString stringWithFormat:@"%.1f%%", a->sub_px_pct], nil);
        addRow(@"Accel jerk", [NSString stringWithFormat:@"%.4f", a->accel_jerk], nil);
        addRow(@"Path efficiency", [NSString stringWithFormat:@"%.4f", a->path_eff], nil);
        addRow(@"Total distance", [NSString stringWithFormat:@"%.0f px", a->obs_total_dist], nil);
        y += 8;

        // Interval histogram
        addHeader(@"Command Intervals");
        const char* labels[] = {"<1ms","1-2","2-5","5-10","10-20","20-50","50-100","100+"};
        uint32_t hMax = 1;
        for (int b = 0; b < 8; b++) if (a->cmd_hist[b] > hMax) hMax = a->cmd_hist[b];
        for (int b = 0; b < 8; b++) {
            if (!a->cmd_hist[b]) continue;
            CGFloat barW = (CGFloat)a->cmd_hist[b] / hMax * (w - 100);

            NSTextField* bl = [NSTextField labelWithString:
                [NSString stringWithUTF8String:labels[b]]];
            bl.font = [NSFont monospacedSystemFontOfSize:10 weight:NSFontWeightRegular];
            bl.textColor = col_text3;
            bl.alignment = NSTextAlignmentRight;
            bl.frame = NSMakeRect(12, y, 50, 12);
            [_sidebarContent addSubview:bl];

            NSView* bar = [[NSView alloc] initWithFrame:NSMakeRect(68, y+1, fmax(1, barW), 10)];
            bar.wantsLayer = YES;
            bar.layer.backgroundColor = [CLR(42, 90, 154, 1.0) CGColor];
            bar.layer.cornerRadius = 2;
            [_sidebarContent addSubview:bar];

            NSTextField* cl = [NSTextField labelWithString:
                [NSString stringWithFormat:@"%u", a->cmd_hist[b]]];
            cl.font = [NSFont monospacedSystemFontOfSize:10 weight:NSFontWeightRegular];
            cl.textColor = col_text3;
            cl.frame = NSMakeRect(72 + barW, y, 40, 12);
            [_sidebarContent addSubview:cl];
            y += 14;
        }
        y += 8;
    }

    // Display controls
    addHeader(@"Display");
    NSFont* ctrlFont = [NSFont systemFontOfSize:11];

    void (^addCheckbox)(NSString*, SEL, BOOL) = ^(NSString* title, SEL action, BOOL state) {
        NSButton* cb = [NSButton checkboxWithTitle:title target:self action:action];
        cb.font = ctrlFont;
        cb.state = state ? NSControlStateValueOn : NSControlStateValueOff;
        cb.frame = NSMakeRect(12, y, w, 18);
        [self.sidebarContent addSubview:cb];
        y += 20;
    };

    addCheckbox(@"Commanded path (blue)", @selector(toggleCmd:), _canvas.showCmd);
    addCheckbox(@"Observed path (orange)", @selector(toggleObs:), _canvas.showObs);
    addCheckbox(@"Sample dots", @selector(toggleDots:), _canvas.showDots);
    addCheckbox(@"Deviation lines", @selector(toggleDev:), _canvas.showDev);
    addCheckbox(@"Grid", @selector(toggleGrid:), _canvas.showGrid);
    addCheckbox(@"Color by speed", @selector(toggleSpeed:), _canvas.showSpeed);

    y += 8;

    // Legend
    addHeader(@"Legend");
    NSView* (^colorSwatch)(NSColor*, CGFloat, CGFloat) = ^NSView*(NSColor* color, CGFloat sw, CGFloat sh) {
        NSView* v = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, sw, sh)];
        v.wantsLayer = YES;
        v.layer.backgroundColor = [color CGColor];
        v.layer.cornerRadius = sh > 4 ? sh/2 : 1;
        return v;
    };

    struct { NSColor* c; NSString* label; } legends[] = {
        {col_cmd,   @"Commanded"},
        {col_obs,   @"Observed"},
        {col_dev,   @"Deviation"},
        {col_green, @"Start"},
        {col_red,   @"End"},
    };
    for (int i = 0; i < 5; i++) {
        NSView* sw = colorSwatch(legends[i].c, 18, (i < 3) ? 3 : 6);
        sw.frame = NSMakeRect(12, y + (i < 3 ? 7 : 5), 18, (i < 3) ? 3 : 6);
        [_sidebarContent addSubview:sw];

        NSTextField* ll = [NSTextField labelWithString:legends[i].label];
        ll.font = labelFont; ll.textColor = col_text2;
        ll.frame = NSMakeRect(36, y, 100, 16);
        [_sidebarContent addSubview:ll];
        y += 18;
    }

    y += 12;
    NSTextField* helpLbl = [NSTextField labelWithString:@"Pan: drag · Zoom: scroll · Fit: dbl-click"];
    helpLbl.font = [NSFont systemFontOfSize:10];
    helpLbl.textColor = col_text3;
    helpLbl.frame = NSMakeRect(12, y, w, 14);
    [_sidebarContent addSubview:helpLbl];
    y += 24;

    // Resize content to fit
    NSRect contentFrame = _sidebarContent.frame;
    contentFrame.size.height = y;
    _sidebarContent.frame = contentFrame;
}

// ---- Actions ----

- (void)protoChanged:(id)sender {
    NSInteger idx = [_protoPopup indexOfSelectedItem];
    device_type_t type = (idx == 0) ? DEV_KMBOX : (idx == 1) ? DEV_FERRUM : DEV_MAKCU;
    proto_init(&g_proto, type);
    _statusLabel.stringValue = [NSString stringWithFormat:@"Protocol: %s (baud: %d)",
        g_proto.profile->name, g_proto.profile->default_baud];
}

- (void)toggleConnect:(id)sender {
    if (_connected) {
        // Disconnect
        g_reader_running = false;
        pthread_join(g_reader_thread, NULL);
        close(g_serial_fd); g_serial_fd = -1;
        _connected = NO;
        _connectBtn.title = @"Connect";
        _statusLabel.stringValue = @"Disconnected";
    } else {
        const char* port = [_portField.stringValue UTF8String];
        int baud = g_proto.profile->default_baud;
        g_serial_fd = serial_open(port, baud);
        if (g_serial_fd < 0) {
            _statusLabel.stringValue = [NSString stringWithFormat:@"Failed: %s", strerror(errno)];
            return;
        }
        g_reader_running = true;
        pthread_create(&g_reader_thread, NULL, serial_reader_fn, NULL);
        _connected = YES;
        _connectBtn.title = @"Disconnect";
        _statusLabel.stringValue = [NSString stringWithFormat:@"Connected: %s @ %d",
            g_proto.profile->name, baud];
    }
}

- (void)runTest:(id)sender {
    if (_testRunning) return;
    NSInteger idx = [_testPopup indexOfSelectedItem];
    if (idx < 0 || idx >= NUM_TESTS) return;

    _testRunning = YES;
    _runBtn.title = @"Running…";
    _runBtn.enabled = NO;
    g_stat_ok = 0; g_stat_err = 0; g_stat_sent = 0;

    dispatch_async(dispatch_get_global_queue(DISPATCH_QUEUE_PRIORITY_HIGH, 0), ^{
        all_tests[idx].run();
        trace_analyze();

        dispatch_async(dispatch_get_main_queue(), ^{
            self.testRunning = NO;
            self.runBtn.title = @"▶ Run";
            self.runBtn.enabled = YES;
            self.statusLabel.stringValue = [NSString stringWithFormat:
                @"Done: %s — %lld sent, %lld ok, %lld err",
                all_tests[idx].name, g_stat_sent, g_stat_ok, g_stat_err];
            [self.canvas fitView];
            [self.canvas setNeedsDisplay:YES];
            [self buildSidebar];
        });
    });
}

- (void)refreshTick:(NSTimer*)timer {
    (void)timer;
    if (_testRunning && g_trace.recording) {
        // Update bounds for live rendering
        uint32_t ncmd = g_trace.cmd_count, nobs = g_trace.obs_count;
        double mnx=0,mxx=0,mny=0,mxy=0;
        for (uint32_t i = 0; i < ncmd; i++) {
            if (g_trace.cmds[i].cum_x < mnx) mnx = g_trace.cmds[i].cum_x;
            if (g_trace.cmds[i].cum_x > mxx) mxx = g_trace.cmds[i].cum_x;
            if (g_trace.cmds[i].cum_y < mny) mny = g_trace.cmds[i].cum_y;
            if (g_trace.cmds[i].cum_y > mxy) mxy = g_trace.cmds[i].cum_y;
        }
        for (uint32_t i = 0; i < nobs; i++) {
            if (g_trace.obs[i].rel_x < mnx) mnx = g_trace.obs[i].rel_x;
            if (g_trace.obs[i].rel_x > mxx) mxx = g_trace.obs[i].rel_x;
            if (g_trace.obs[i].rel_y < mny) mny = g_trace.obs[i].rel_y;
            if (g_trace.obs[i].rel_y > mxy) mxy = g_trace.obs[i].rel_y;
        }
        g_trace.all_min_x = mnx; g_trace.all_max_x = mxx;
        g_trace.all_min_y = mny; g_trace.all_max_y = mxy;
        [_canvas fitView];
        [_canvas setNeedsDisplay:YES];

        _statusLabel.stringValue = [NSString stringWithFormat:
            @"Recording: %u cmd, %u obs — %lld sent",
            ncmd, nobs, g_stat_sent];
    }
}

// Display toggle actions
- (void)toggleCmd:(NSButton*)sender   { _canvas.showCmd   = (sender.state == NSControlStateValueOn); [_canvas setNeedsDisplay:YES]; }
- (void)toggleObs:(NSButton*)sender   { _canvas.showObs   = (sender.state == NSControlStateValueOn); [_canvas setNeedsDisplay:YES]; }
- (void)toggleDots:(NSButton*)sender  { _canvas.showDots  = (sender.state == NSControlStateValueOn); [_canvas setNeedsDisplay:YES]; }
- (void)toggleDev:(NSButton*)sender   { _canvas.showDev   = (sender.state == NSControlStateValueOn); [_canvas setNeedsDisplay:YES]; }
- (void)toggleGrid:(NSButton*)sender  { _canvas.showGrid  = (sender.state == NSControlStateValueOn); [_canvas setNeedsDisplay:YES]; }
- (void)toggleSpeed:(NSButton*)sender { _canvas.showSpeed  = (sender.state == NSControlStateValueOn); [_canvas setNeedsDisplay:YES]; }

- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)sender {
    (void)sender; return YES;
}

- (void)applicationWillTerminate:(NSNotification*)notification {
    (void)notification;
    [_refreshTimer invalidate];
    if (_connected) {
        g_reader_running = false;
        pthread_join(g_reader_thread, NULL);
        close(g_serial_fd);
    }
    trace_dealloc();
}

@end

// ============================================================================
// Main Entry Point
// ============================================================================

static void print_usage(const char* prog) {
    printf("KMBox Trace Analyzer\n\n");
    printf("Usage:\n");
    printf("  %s                          Launch GUI\n", prog);
    printf("  %s [options]                CLI test mode\n\n", prog);
    printf("Options:\n");
    printf("  --port PATH     Serial port (required for CLI)\n");
    printf("  --proto NAME    Protocol: kmbox, ferrum, macku (default: kmbox)\n");
    printf("  --test NAME     Test to run (see list below)\n");
    printf("  --baud RATE     Override baud rate\n");
    printf("  --help          Show this help\n\n");
    printf("Tests: ");
    for (int i = 0; all_tests[i].name; i++)
        printf("%s%s", all_tests[i].name, all_tests[i+1].name ? ", " : "\n");
}

int main(int argc, const char* argv[]) {
    // Check for CLI mode
    const char* cli_port = NULL;
    const char* cli_proto = "kmbox";
    const char* cli_test = NULL;
    int cli_baud = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            print_usage(argv[0]); return 0;
        }
        if (!strcmp(argv[i], "--port") && i+1 < argc)  cli_port = argv[++i];
        else if (!strcmp(argv[i], "--proto") && i+1 < argc) cli_proto = argv[++i];
        else if (!strcmp(argv[i], "--test") && i+1 < argc)  cli_test = argv[++i];
        else if (!strcmp(argv[i], "--baud") && i+1 < argc)  cli_baud = atoi(argv[++i]);
    }

    mach_timebase_info(&g_timebase);
    trace_alloc();
    srand((unsigned)time(NULL));

    if (cli_test && cli_port) {
        // CLI mode: run test and exit
        proto_init(&g_proto, device_from_name(cli_proto));
        int baud = cli_baud > 0 ? cli_baud : g_proto.profile->default_baud;

        printf("Opening %s (%s @ %d)...\n", cli_port, g_proto.profile->name, baud);
        g_serial_fd = serial_open(cli_port, baud);
        if (g_serial_fd < 0) { perror("open"); trace_dealloc(); return 1; }

        g_reader_running = true;
        pthread_create(&g_reader_thread, NULL, serial_reader_fn, NULL);

        // Find and run test
        bool found = false;
        for (int i = 0; all_tests[i].name; i++) {
            if (!strcmp(all_tests[i].name, cli_test)) {
                printf("Running test: %s\n", cli_test);
                all_tests[i].run();
                trace_analyze();
                trace_analysis_t* a = &g_trace.analysis;
                printf("\n── Results ──\n");
                printf("Commands:     %u\n", g_trace.cmd_count);
                printf("Observations: %u\n", g_trace.obs_count);
                printf("Duration:     %.1f ms\n", a->total_ms);
                printf("Avg gap:      %.2f px\n", a->dev_avg);
                printf("Humanization: %d/100 (%s)\n", (int)a->h_score, a->h_grade);
                printf("Perp scatter: %.3f px\n", a->perp_scatter);
                printf("Jitter avg:   %.3f px\n", a->jit_avg);
                printf("Speed CV:     %.3f\n", a->speed_cv);
                printf("Sent/OK/ERR:  %lld/%lld/%lld\n", g_stat_sent, g_stat_ok, g_stat_err);
                found = true; break;
            }
        }
        if (!found) { fprintf(stderr, "Unknown test: %s\n", cli_test); }

        g_reader_running = false;
        pthread_join(g_reader_thread, NULL);
        close(g_serial_fd);
        trace_dealloc();
        return found ? 0 : 1;
    }

    // GUI mode
    @autoreleasepool {
        NSApplication* app = [NSApplication sharedApplication];
        [app setActivationPolicy:NSApplicationActivationPolicyRegular];

        // Create menu bar
        NSMenu* menuBar = [[NSMenu alloc] init];
        NSMenuItem* appMenuItem = [[NSMenuItem alloc] init];
        [menuBar addItem:appMenuItem];
        NSMenu* appMenu = [[NSMenu alloc] init];
        [appMenu addItemWithTitle:@"Quit KMBox Tester"
                 action:@selector(terminate:) keyEquivalent:@"q"];
        appMenuItem.submenu = appMenu;
        [app setMainMenu:menuBar];

        AppDelegate* delegate = [[AppDelegate alloc] init];
        [app setDelegate:delegate];
        [app run];
    }

    trace_dealloc();
    return 0;
}
