/**
 * app_common.c - Shared Application Logic (Platform-Independent)
 *
 * Contains all test patterns, trace recording/analysis, and serial
 * management used by all three platform builds (macOS, Windows, Linux).
 *
 * This file does NOT contain any GUI code or platform-specific headers
 * beyond what platform.h abstracts.
 */

#include "app_common.h"
#include <errno.h>

// ============================================================================
// Globals
// ============================================================================

trace_data_t     g_trace;
plat_serial_t    g_serial_fd = PLAT_SERIAL_INVALID;
proto_state_t    g_proto;
volatile int64_t g_stat_ok  = 0;
volatile int64_t g_stat_err = 0;
volatile int64_t g_stat_sent = 0;

plat_thread_t    g_reader_thread;
volatile bool    g_reader_running = false;

// ============================================================================
// Trace Management
// ============================================================================

void trace_alloc(void) {
    plat_time_init();
    g_trace.cmds = (trace_cmd_t*)calloc(TRACE_CMD_MAX, sizeof(trace_cmd_t));
    g_trace.obs  = (trace_obs_t*)calloc(TRACE_OBS_MAX, sizeof(trace_obs_t));
    plat_mutex_init(&g_trace.cmd_mutex);
}

void trace_dealloc(void) {
    free(g_trace.cmds);
    free(g_trace.obs);
    plat_mutex_destroy(&g_trace.cmd_mutex);
}

static PLAT_THREAD_RETURN trace_poller_fn(void* arg) {
    (void)arg;
    while (g_trace.poller_running) {
        if (g_trace.recording && g_trace.obs_count < TRACE_OBS_MAX) {
            plat_cursor_pos_t pos;
            if (plat_get_cursor_pos(&pos)) {
                uint32_t idx = g_trace.obs_count;
                g_trace.obs[idx].time_us = plat_time_us() - g_trace.start_us;
                g_trace.obs[idx].abs_x = pos.x;
                g_trace.obs[idx].abs_y = pos.y;
                g_trace.obs[idx].rel_x = pos.x - g_trace.start_abs_x;
                g_trace.obs[idx].rel_y = pos.y - g_trace.start_abs_y;
                plat_memory_barrier();
                g_trace.obs_count = idx + 1;
            }
        }
        plat_usleep(500);
    }
#ifdef PLATFORM_WINDOWS
    return 0;
#else
    return NULL;
#endif
}

void trace_start(const char* name) {
    plat_cursor_pos_t pos = {0, 0};
    plat_get_cursor_pos(&pos);

    g_trace.cmd_count = 0;
    g_trace.obs_count = 0;
    g_trace.start_us = plat_time_us();
    g_trace.start_abs_x = pos.x;
    g_trace.start_abs_y = pos.y;
    g_trace.analysis_valid = false;
#ifdef PLATFORM_WINDOWS
    strncpy(g_trace.test_name, name, sizeof(g_trace.test_name) - 1);
    g_trace.test_name[sizeof(g_trace.test_name) - 1] = '\0';
#else
    strlcpy(g_trace.test_name, name, sizeof(g_trace.test_name));
#endif

    g_trace.poller_running = true;
    g_trace.recording = true;
    plat_thread_create(&g_trace.poller_thread, trace_poller_fn, NULL);

    int w = 0;
    while (g_trace.obs_count == 0 && w < 200) { plat_usleep(500); w++; }
}

void trace_stop(void) {
    g_trace.recording = false;
    g_trace.poller_running = false;
    plat_thread_join(g_trace.poller_thread);
}

void trace_record_cmd(int16_t dx, int16_t dy) {
    if (!g_trace.recording) return;
    plat_mutex_lock(&g_trace.cmd_mutex);
    uint32_t idx = g_trace.cmd_count;
    if (idx >= TRACE_CMD_MAX) { plat_mutex_unlock(&g_trace.cmd_mutex); return; }
    g_trace.cmds[idx].time_us = plat_time_us() - g_trace.start_us;
    g_trace.cmds[idx].dx = dx;
    g_trace.cmds[idx].dy = dy;
    g_trace.cmds[idx].cum_x = (idx > 0) ? g_trace.cmds[idx-1].cum_x + dx : dx;
    g_trace.cmds[idx].cum_y = (idx > 0) ? g_trace.cmds[idx-1].cum_y + dy : dy;
    g_trace.cmd_count = idx + 1;
    plat_mutex_unlock(&g_trace.cmd_mutex);
}

// ============================================================================
// Trace Analysis
// ============================================================================

void trace_analyze(void) {
    trace_analysis_t* a = &g_trace.analysis;
    memset(a, 0, sizeof(*a));

    uint32_t ncmd = g_trace.cmd_count;
    uint32_t nobs = g_trace.obs_count;
    if (ncmd < 2 && nobs < 2) return;

    /* Bounds */
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

    /* Duration and rates */
    uint64_t totalUs = 0;
    if (ncmd > 0) totalUs = g_trace.cmds[ncmd-1].time_us;
    if (nobs > 0 && g_trace.obs[nobs-1].time_us > totalUs)
        totalUs = g_trace.obs[nobs-1].time_us;
    a->total_ms = (double)totalUs / 1000.0;
    a->cmd_rate = (totalUs > 0 && ncmd > 1) ? (double)(ncmd-1) / ((double)totalUs / 1e6) : 0;
    a->obs_rate = (totalUs > 0 && nobs > 1) ? (double)(nobs-1) / ((double)totalUs / 1e6) : 0;

    /* Deviation analysis */
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

    /* Jitter */
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

    /* Observed path stats */
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

    /* Command timing */
    double intSum = 0, intSqSum = 0;
    for (uint32_t i = 1; i < ncmd; i++) {
        double iv = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
        intSum += iv; intSqSum += iv*iv;
    }
    double intMean = (ncmd > 2) ? intSum / (ncmd-1) : 0;
    double intVar = (ncmd > 2) ? (intSqSum / (ncmd-1) - intMean*intMean) : 0;
    if (intVar < 0) intVar = 0;
    a->int_cv = (intMean > 0) ? sqrt(intVar) / intMean : 0;

    /* Command delta repeats */
    uint32_t reps = 0;
    for (uint32_t i = 1; i < ncmd; i++)
        if (g_trace.cmds[i].dx == g_trace.cmds[i-1].dx &&
            g_trace.cmds[i].dy == g_trace.cmds[i-1].dy) reps++;
    a->cmd_rep_pct = (ncmd > 1) ? (double)reps / (ncmd-1) * 100.0 : 0;

    /* Perp scatter */
    double psSum = 0; uint32_t psN = 0;
    for (uint32_t i = 10; i < nobs; i += 5) {
        double wx = g_trace.obs[i].rel_x - g_trace.obs[i-10].rel_x;
        double wy = g_trace.obs[i].rel_y - g_trace.obs[i-10].rel_y;
        double wl = hypot(wx, wy);
        if (wl < 2.0) continue;
        double ppx = -wy/wl, ppy = wx/wl;
        for (uint32_t j = i-9; j < i; j++) {
            double ddx = g_trace.obs[j].rel_x - g_trace.obs[i-10].rel_x;
            double ddy = g_trace.obs[j].rel_y - g_trace.obs[i-10].rel_y;
            psSum += fabs(ddx*ppx + ddy*ppy); psN++;
        }
    }
    a->perp_scatter = (psN > 0) ? psSum / psN : 0;

    /* Accel jerk */
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

    /* Interval histogram */
    for (uint32_t i = 1; i < ncmd; i++) {
        double ms = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us) / 1000.0;
        int b = (ms<1)?0:(ms<2)?1:(ms<5)?2:(ms<10)?3:(ms<20)?4:(ms<50)?5:(ms<100)?6:7;
        a->cmd_hist[b]++;
    }

    /* Humanization score */
    double h = 0;
    if (a->perp_scatter > 0.05)  h += fmin(a->perp_scatter / 0.5, 1.0) * 25;
    if (a->jit_avg >= 0.1)       h += fmin(a->jit_avg / 0.6, 1.0) * 20;
    if (a->dir_flip_rate > 2)    h += fmin(a->dir_flip_rate / 15.0, 1.0) * 15;
    if (a->speed_cv > 0.05)      h += fmin(a->speed_cv / 0.25, 1.0) * 15;
    if (a->sub_px_pct > 1)       h += fmin(a->sub_px_pct / 8.0, 1.0) * 10;
    if (a->accel_jerk > 0.001)   h += fmin(a->accel_jerk / 0.05, 1.0) * 10;
    if (a->path_eff < 0.995)     h += fmin((1.0 - a->path_eff) / 0.05, 1.0) * 5;
    if (h > 100) h = 100;
    a->h_score = h;
    a->h_grade = (h < 10) ? "Robotic" : (h < 25) ? "Minimal" :
                 (h < 50) ? "Moderate" : (h < 75) ? "Good" : "Excellent";

    g_trace.analysis_valid = true;
}

// ============================================================================
// Serial / Protocol Helpers
// ============================================================================

void send_move_traced(int dx, int dy) {
    trace_record_cmd((int16_t)dx, (int16_t)dy);

#ifdef PLATFORM_WINDOWS
    if (g_serial_fd == PLAT_SERIAL_INVALID) return;
#else
    if (g_serial_fd == PLAT_SERIAL_INVALID) return;
#endif

    uint8_t buf[64];
    int len = proto_fmt_move(&g_proto, buf, sizeof(buf), (int16_t)dx, (int16_t)dy);
    if (len <= 0) return;

    int w = plat_serial_write(g_serial_fd, buf, len);
    if (w <= 0) {
        plat_usleep(200);
        w = plat_serial_write(g_serial_fd, buf, len);
    }
    if (w > 0) plat_atomic_inc(&g_stat_sent);
    plat_usleep(100);
}

static void response_cb(proto_result_t result, const uint8_t* data,
                        uint16_t len, void* ctx) {
    (void)data; (void)len; (void)ctx;
    if (result == PROTO_OK) plat_atomic_inc(&g_stat_ok);
    else plat_atomic_inc(&g_stat_err);
}

static PLAT_THREAD_RETURN serial_reader_fn(void* arg) {
    (void)arg;
    uint8_t buf[256];
    while (g_reader_running) {
        int n = plat_serial_read(g_serial_fd, buf, sizeof(buf));
        if (n > 0) {
            proto_parse(&g_proto, buf, (size_t)n, response_cb, NULL);
        } else if (n < 0) {
            break;  /* Fatal error */
        } else {
            plat_usleep(1000);
        }
    }
#ifdef PLATFORM_WINDOWS
    return 0;
#else
    return NULL;
#endif
}

void serial_reader_start(void) {
    g_reader_running = true;
    plat_thread_create(&g_reader_thread, serial_reader_fn, NULL);
}

void serial_reader_stop(void) {
    g_reader_running = false;
    plat_thread_join(g_reader_thread);
}

// ============================================================================
// Aimbot Helper
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
        plat_usleep(frame_us);
    }
}

// ============================================================================
// Test Implementations
// ============================================================================

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
    trace_start("precise"); plat_usleep(200000);
    for (int r = 0; r < 40; r++)
        for (int i = 0; i < 16; i++) { send_move_traced(mv[i][0],mv[i][1]); plat_usleep(2000); }
    trace_stop();
}

static void test_flicks(void) {
    int fl[][2] = {{127,0},{-127,0},{0,127},{0,-127},{90,90},{-90,-90},{90,-90},{-90,90}};
    trace_start("flicks"); plat_usleep(200000);
    for (int r = 0; r < 20; r++)
        for (int i = 0; i < 8; i++) { send_move_traced(fl[i][0],fl[i][1]); plat_usleep(8000); }
    trace_stop();
}

static void test_sweep(void) {
    trace_start("sweep");
    for (int i = 0; i < 500; i++) { send_move_traced(5, 0); plat_usleep(1000); }
    plat_usleep(100000);
    for (int i = 0; i < 500; i++) { send_move_traced(-5, 0); plat_usleep(1000); }
    trace_stop();
}

static void test_mixed(void) {
    trace_start("mixed");
    for (int c = 0; c < 8; c++) {
        for (int i=0;i<100;i++) { send_move_traced(2,1); plat_usleep(2000); }
        send_move_traced(-100,-50); plat_usleep(20000);
        for (int i=0;i<60;i++) { send_move_traced(-1,2); plat_usleep(2000); }
        send_move_traced(80,40); plat_usleep(30000);
        for (int i=0;i<40;i++) { send_move_traced(0,-3); plat_usleep(1000); }
    }
    trace_stop();
}

static void test_aim_approach(void) {
    trace_start("aim_approach"); plat_usleep(200000);
    aimbot_converge(50,30,0.8f,16667,60); plat_usleep(500000);
    aimbot_converge(-50,-30,0.8f,16667,60); plat_usleep(500000);
    aimbot_converge(200,-100,0.8f,16667,120); plat_usleep(500000);
    aimbot_converge(-200,100,0.8f,16667,120); plat_usleep(500000);
    aimbot_converge(500,-300,0.8f,16667,200); plat_usleep(500000);
    aimbot_converge(-500,300,0.8f,16667,200); plat_usleep(300000);
    aimbot_converge(200,100,1.5f,16667,200); plat_usleep(500000);
    aimbot_converge(-200,-100,1.5f,16667,200);
    trace_stop();
}

static void test_aim_flick(void) {
    struct { float x,y; } tgts[] = {{300,-50},{-150,200},{400,100},{-250,-150},{100,50}};
    trace_start("aim_flick"); plat_usleep(200000);
    for (int rep = 0; rep < 3; rep++) {
        float cx=0,cy=0;
        for (int t=0;t<5;t++) {
            aimbot_converge(tgts[t].x-cx,tgts[t].y-cy,0.8f,16667,80);
            cx=tgts[t].x; cy=tgts[t].y;
            plat_usleep(100000 + (rand()%200000));
        }
        aimbot_converge(-cx,-cy,0.8f,16667,120); plat_usleep(500000);
    }
    trace_stop();
}

static void test_aim_recoil(void) {
    int ry[]={-8,-9,-10,-11,-12,-11,-10,-9,-8,-7,-6,-5,-5,-4,-4,-3,-3,-3,-2,-2,-2,-1,-1,-1,-1,0,0,0,0,0};
    int rx[]={ 0, 0, 1, 0,-1, 0, 1, 2, 1, 0,-1,-2,-3,-2,-1, 0, 1, 2, 3, 2, 1, 0,-1,-2,-1,0,1,0,-1,0};
    trace_start("aim_recoil"); plat_usleep(200000);
    for (int burst=0;burst<6;burst++) {
        for (int b=0;b<30;b++) {
            for (int f=0;f<6;f++) {
                int dx = (f==0)?rx[b]:0;
                int dy = ry[b]/6;
                if (f < abs(ry[b])%6) dy += (ry[b]<0)?-1:1;
                send_move_traced(dx,dy); plat_usleep(2800);
            }
        }
        plat_usleep(800000);
    }
    trace_stop();
}

static void test_aim_track(void) {
    trace_start("aim_track"); plat_usleep(200000);
    float cx=0,cy=0;
    for (int f=0;f<600;f++) {
        float t=(float)f/60.0f;
        float tx=150*sinf(2*(float)M_PI*0.5f*t), ty=30*sinf(2*(float)M_PI*1.0f*t);
        float mx=(tx-cx)/0.8f, my=(ty-cy)/0.8f;
        int dx=(int)mx, dy=(int)my;
        if(dx>127)dx=127;if(dx<-127)dx=-127;if(dy>127)dy=127;if(dy<-127)dy=-127;
        if(dx||dy){send_move_traced(dx,dy);cx+=dx;cy+=dy;}
        plat_usleep(16667);
    }
    cx=0;cy=0;
    for (int f=0;f<300;f++) {
        float t=(float)f/60.0f;
        float tx=200*sinf(2*(float)M_PI*2.0f*t), ty=50*cosf(2*(float)M_PI*1.5f*t);
        float mx=(tx-cx)/0.8f, my=(ty-cy)/0.8f;
        int dx=(int)mx, dy=(int)my;
        if(dx>127)dx=127;if(dx<-127)dx=-127;if(dy>127)dy=127;if(dy<-127)dy=-127;
        if(dx||dy){send_move_traced(dx,dy);cx+=dx;cy+=dy;}
        plat_usleep(16667);
    }
    trace_stop();
}

static void test_aim_full(void) {
    trace_start("aim_full"); plat_usleep(200000);
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
                send_move_traced(dx,dy); plat_usleep(2800);
            }
            cx+=(int)adx; cy+=(int)ady;
        }
        plat_usleep(300000+(rand()%500000));
    }
    aimbot_converge(-cx,-cy,1.5f,16667,200);
    trace_stop();
}

static void test_diag_tremor(void) {
    trace_start("diag_tremor"); plat_usleep(300000);
    for (int i=0;i<3000;i++) { send_move_traced(0,0); plat_usleep(1000); }
    plat_usleep(200000); trace_stop();
}

static void test_diag_line(void) {
    trace_start("diag_line"); plat_usleep(300000);
    for (int i=0;i<2000;i++) { send_move_traced(5,0); plat_usleep(1000); }
    plat_usleep(200000); trace_stop();
}

static void test_diag_repeat(void) {
    trace_start("diag_repeat"); plat_usleep(300000);
    for (int i=0;i<1000;i++) { send_move_traced(3,3); plat_usleep(2000); }
    plat_usleep(200000); trace_stop();
}

static void test_diag_overshoot(void) {
    trace_start("diag_overshoot"); plat_usleep(300000);
    for (int i=0;i<20;i++) {
        send_move_traced(50*((i%2==0)?1:-1),0); plat_usleep(500000);
    }
    plat_usleep(200000); trace_stop();
}

static void test_diag_ease(void) {
    trace_start("diag_ease"); plat_usleep(500000);
    send_move_traced(100,0);plat_usleep(1000000);
    send_move_traced(0,100);plat_usleep(1000000);
    send_move_traced(70,70);plat_usleep(1000000);
    send_move_traced(-100,0);plat_usleep(1000000);
    send_move_traced(0,-100);plat_usleep(1000000);
    send_move_traced(-70,-70);plat_usleep(1000000);
    trace_stop();
}

// ============================================================================
// Test Table
// ============================================================================

const test_def_t all_tests[] = {
    {"rapid",          "Basic",  "10k iterations +/-10px, max throughput",        test_rapid},
    {"precise",        "Basic",  "1-5px movements @ 500Hz",                      test_precise},
    {"flicks",         "Basic",  "+/-127px flicks @ 125Hz",                      test_flicks},
    {"sweep",          "Basic",  "Horizontal sweep @ 1kHz",                      test_sweep},
    {"mixed",          "Basic",  "Micro-adjustments + flicks @ 500Hz",           test_mixed},
    {"aim_approach",   "Aimbot", "Proportional controller approach @ 60Hz",      test_aim_approach},
    {"aim_flick",      "Aimbot", "Flick sequences with target switching",        test_aim_flick},
    {"aim_recoil",     "Aimbot", "AK-47 recoil compensation pattern",            test_aim_recoil},
    {"aim_track",      "Aimbot", "Moving target tracking with phase lag",        test_aim_track},
    {"aim_full",       "Aimbot", "Full engagement: flick > track > spray",       test_aim_full},
    {"diag_tremor",    "Diag",   "Send (0,0) for 3s - isolate tremor",          test_diag_tremor},
    {"diag_line",      "Diag",   "Identical (5,0) @ 1kHz - check perp jitter",  test_diag_line},
    {"diag_repeat",    "Diag",   "Constant (3,3) @ 500Hz - delta breaking",     test_diag_repeat},
    {"diag_overshoot", "Diag",   "50px moves w/ gaps - check overshoot",        test_diag_overshoot},
    {"diag_ease",      "Diag",   "Single 100px move - velocity S-curve",        test_diag_ease},
    {NULL, NULL, NULL, NULL}
};

// ============================================================================
// CLI Helpers
// ============================================================================

void cli_print_results(const char* test_name) {
    trace_analysis_t* a = &g_trace.analysis;
    printf("\n-- Results: %s --\n", test_name);
    printf("Commands:     %u\n", g_trace.cmd_count);
    printf("Observations: %u\n", g_trace.obs_count);
    printf("Duration:     %.1f ms\n", a->total_ms);
    printf("Avg gap:      %.2f px\n", a->dev_avg);
    printf("Humanization: %d/100 (%s)\n", (int)a->h_score, a->h_grade);
    printf("Perp scatter: %.3f px\n", a->perp_scatter);
    printf("Jitter avg:   %.3f px\n", a->jit_avg);
    printf("Speed CV:     %.3f\n", a->speed_cv);
    printf("Sent/OK/ERR:  %lld/%lld/%lld\n",
           (long long)g_stat_sent, (long long)g_stat_ok, (long long)g_stat_err);
}

void print_usage(const char* prog) {
    printf("KMBox Trace Analyzer\n\n");
    printf("Usage:\n");
    printf("  %s                          Launch GUI\n", prog);
    printf("  %s [options]                CLI test mode\n\n", prog);
    printf("Options:\n");
    printf("  --port PATH     Serial port (required for CLI)\n");
    printf("  --proto NAME    Protocol: kmbox, ferrum, makcu (default: kmbox)\n");
    printf("  --test NAME     Test to run (see list below)\n");
    printf("  --baud RATE     Override baud rate\n");
    printf("  --help          Show this help\n\n");
    printf("Tests: ");
    for (int i = 0; all_tests[i].name; i++)
        printf("%s%s", all_tests[i].name, all_tests[i+1].name ? ", " : "\n");
}
