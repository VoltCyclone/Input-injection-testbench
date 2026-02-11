/**
 * app_linux.c - Linux Native Application (GTK3 + Cairo)
 *
 * Full-featured Linux port of the KMBox Trace Analyzer with:
 *   - GTK3 window with dark instrumentation aesthetic
 *   - Cairo trace rendering with speed coloring
 *   - POSIX serial port (/dev/ttyUSB*, /dev/ttyACM*) support
 *   - clock_gettime high-res timing
 *   - Pan/zoom canvas, sidebar with stats
 *
 * Build:
 *   gcc -O2 -o kmbox_tester app_linux.c app_common.c protocols.c \
 *       $(pkg-config --cflags --libs gtk+-3.0) -lm -lpthread
 */

#define _GNU_SOURCE
#include <gtk/gtk.h>
#include <cairo.h>
#include <time.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>
#include <math.h>
#include <signal.h>

#include "app_common.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// ============================================================================
// Platform Implementation: Timing
// ============================================================================

static struct timespec g_time_start;

void plat_time_init(void) {
    clock_gettime(CLOCK_MONOTONIC, &g_time_start);
}

uint64_t plat_time_us(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t sec  = (uint64_t)(now.tv_sec - g_time_start.tv_sec);
    int64_t nsec = now.tv_nsec - g_time_start.tv_nsec;
    return sec * 1000000ULL + (uint64_t)(nsec / 1000);
}

// ============================================================================
// Platform Implementation: Serial Port
// ============================================================================

plat_serial_t plat_serial_open(const char* port, int baud) {
    int fd = open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) return PLAT_SERIAL_INVALID;

    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) { close(fd); return PLAT_SERIAL_INVALID; }

    speed_t speed;
    switch (baud) {
        case 9600:    speed = B9600;    break;
        case 19200:   speed = B19200;   break;
        case 38400:   speed = B38400;   break;
        case 57600:   speed = B57600;   break;
        case 115200:  speed = B115200;  break;
        case 230400:  speed = B230400;  break;
        case 460800:  speed = B460800;  break;
        case 500000:  speed = B500000;  break;
        case 576000:  speed = B576000;  break;
        case 921600:  speed = B921600;  break;
        case 1000000: speed = B1000000; break;
        case 1500000: speed = B1500000; break;
        case 2000000: speed = B2000000; break;
        case 3000000: speed = B3000000; break;
        case 4000000: speed = B4000000; break;
        default:      speed = B115200;  break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);
    tty.c_cflag = (tty.c_cflag & ~(PARENB|CSTOPB|CSIZE|CRTSCTS)) | CS8 | CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON|ECHO|ECHOE|ECHONL|ISIG);
    tty.c_iflag &= ~(IXON|IXOFF|IXANY|IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL);
    tty.c_oflag &= ~(OPOST|ONLCR);
    tty.c_cc[VTIME] = 1;
    tty.c_cc[VMIN]  = 0;
    if (tcsetattr(fd, TCSANOW, &tty) != 0) { close(fd); return PLAT_SERIAL_INVALID; }
    tcflush(fd, TCIOFLUSH);
    return fd;
}

void plat_serial_close(plat_serial_t s) {
    if (s != PLAT_SERIAL_INVALID) close(s);
}

int plat_serial_write(plat_serial_t s, const uint8_t* data, int len) {
    ssize_t w = write(s, data, (size_t)len);
    if (w < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
        usleep(200);
        w = write(s, data, (size_t)len);
    }
    return (int)w;
}

int plat_serial_read(plat_serial_t s, uint8_t* buf, int buflen) {
    ssize_t n = read(s, buf, (size_t)buflen);
    if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) return 0;
        return -1;
    }
    return (int)n;
}

// ============================================================================
// Platform Implementation: Cursor Position
// ============================================================================

bool plat_get_cursor_pos(plat_cursor_pos_t* pos) {
    /* Use GDK to get cursor position — works on X11 and Wayland */
    GdkDisplay* display = gdk_display_get_default();
    if (!display) return false;

    GdkSeat* seat = gdk_display_get_default_seat(display);
    if (!seat) return false;

    GdkDevice* pointer = gdk_seat_get_pointer(seat);
    if (!pointer) return false;

    int x, y;
    gdk_device_get_position(pointer, NULL, &x, &y);
    pos->x = (double)x;
    pos->y = (double)y;
    return true;
}

// ============================================================================
// Application State
// ============================================================================

static GtkWidget* g_window;
static GtkWidget* g_canvas;
static GtkWidget* g_sidebar;
static GtkWidget* g_combo_proto;
static GtkWidget* g_combo_test;
static GtkWidget* g_entry_port;
static GtkWidget* g_btn_connect;
static GtkWidget* g_btn_run;
static GtkWidget* g_label_status;

static gboolean g_app_connected = FALSE;
static gboolean g_app_test_running = FALSE;

/* Canvas view state */
static double g_view_x = 0, g_view_y = 0, g_view_scale = 1.0;
static gboolean g_show_cmd = TRUE, g_show_obs = TRUE, g_show_dots = TRUE;
static gboolean g_show_dev = FALSE, g_show_grid = TRUE, g_show_speed = TRUE;
static gboolean g_dragging = FALSE;
static double g_drag_sx, g_drag_sy, g_drag_vx, g_drag_vy;

// ============================================================================
// Drawing Helpers
// ============================================================================

static void speed_color(double s, double mx, double* r, double* g, double* b) {
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

static void screen_pt(double px, double py, double* sx, double* sy) {
    *sx = px * g_view_scale + g_view_x;
    *sy = py * g_view_scale + g_view_y;
}

static void fit_view(int W, int H) {
    double rx = g_trace.all_max_x - g_trace.all_min_x;
    double ry = g_trace.all_max_y - g_trace.all_min_y;
    if (rx < 10) rx = 10; if (ry < 10) ry = 10;
    double pad = 60;
    g_view_scale = fmin((W - pad*2) / rx, (H - pad*2) / ry) * 0.9;
    double cx = g_trace.all_min_x + rx/2, cy = g_trace.all_min_y + ry/2;
    g_view_x = W/2.0 - cx * g_view_scale;
    g_view_y = H/2.0 - cy * g_view_scale;
}

// ============================================================================
// Canvas Drawing
// ============================================================================

static gboolean on_canvas_draw(GtkWidget* widget, cairo_t* cr, gpointer data) {
    (void)data;
    int W = gtk_widget_get_allocated_width(widget);
    int H = gtk_widget_get_allocated_height(widget);

    /* Background */
    cairo_set_source_rgb(cr, 6/255.0, 6/255.0, 10/255.0);
    cairo_rectangle(cr, 0, 0, W, H);
    cairo_fill(cr);

    uint32_t ncmd = g_trace.cmd_count;
    uint32_t nobs = g_trace.obs_count;

    /* Grid */
    if (g_show_grid && g_view_scale > 0) {
        double gs = pow(10, floor(log10(100.0 / g_view_scale)));
        double d0x = (0 - g_view_x) / g_view_scale;
        double d1x = (W - g_view_x) / g_view_scale;
        double d0y = (0 - g_view_y) / g_view_scale;
        double d1y = (H - g_view_y) / g_view_scale;

        cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, 9);

        for (double gx = floor(d0x/gs)*gs; gx <= d1x; gx += gs) {
            double sx = gx * g_view_scale + g_view_x;
            if (fabs(gx) < 0.01) {
                cairo_set_source_rgb(cr, 37/255.0, 37/255.0, 53/255.0);
                cairo_set_line_width(cr, 1);
            } else {
                cairo_set_source_rgb(cr, 22/255.0, 22/255.0, 32/255.0);
                cairo_set_line_width(cr, 0.5);
            }
            cairo_move_to(cr, sx, 0); cairo_line_to(cr, sx, H); cairo_stroke(cr);

            cairo_set_source_rgb(cr, 42/255.0, 42/255.0, 54/255.0);
            char lbl[32]; snprintf(lbl, sizeof(lbl), "%.0f", gx);
            cairo_move_to(cr, sx+2, 12);
            cairo_show_text(cr, lbl);
        }
        for (double gy = floor(d0y/gs)*gs; gy <= d1y; gy += gs) {
            double sy = gy * g_view_scale + g_view_y;
            if (fabs(gy) < 0.01) {
                cairo_set_source_rgb(cr, 37/255.0, 37/255.0, 53/255.0);
                cairo_set_line_width(cr, 1);
            } else {
                cairo_set_source_rgb(cr, 22/255.0, 22/255.0, 32/255.0);
                cairo_set_line_width(cr, 0.5);
            }
            cairo_move_to(cr, 0, sy); cairo_line_to(cr, W, sy); cairo_stroke(cr);

            cairo_set_source_rgb(cr, 42/255.0, 42/255.0, 54/255.0);
            char lbl[32]; snprintf(lbl, sizeof(lbl), "%.0f", gy);
            cairo_move_to(cr, 2, sy + 12);
            cairo_show_text(cr, lbl);
        }
    }

    /* Origin crosshair */
    {
        double ox, oy;
        screen_pt(0, 0, &ox, &oy);
        cairo_set_source_rgb(cr, 51/255.0, 51/255.0, 64/255.0);
        cairo_set_line_width(cr, 1);
        cairo_arc(cr, ox, oy, 6, 0, 2*M_PI); cairo_stroke(cr);
        cairo_move_to(cr, ox-8, oy); cairo_line_to(cr, ox+8, oy); cairo_stroke(cr);
        cairo_move_to(cr, ox, oy-8); cairo_line_to(cr, ox, oy+8); cairo_stroke(cr);
    }

    if (ncmd < 2 && nobs < 2) {
        cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
        cairo_set_font_size(cr, 14);
        cairo_set_source_rgb(cr, 72/255.0, 72/255.0, 96/255.0);
        const char* msg = "Run a test to see trace data";
        cairo_text_extents_t ext;
        cairo_text_extents(cr, msg, &ext);
        cairo_move_to(cr, W/2.0 - ext.width/2, H/2.0);
        cairo_show_text(cr, msg);
        return TRUE;
    }

    /* Compute max speeds */
    double cmdMaxSpd = 0, obsMaxSpd = 0;
    if (g_show_speed) {
        for (uint32_t i = 1; i < ncmd; i++) {
            double dt = (double)(g_trace.cmds[i].time_us - g_trace.cmds[i-1].time_us);
            if (dt <= 0) continue;
            double s = hypot(g_trace.cmds[i].cum_x - g_trace.cmds[i-1].cum_x,
                             g_trace.cmds[i].cum_y - g_trace.cmds[i-1].cum_y) / (dt/1000.0);
            if (s > cmdMaxSpd) cmdMaxSpd = s;
        }
        for (uint32_t i = 1; i < nobs; i++) {
            double dt = (double)(g_trace.obs[i].time_us - g_trace.obs[i-1].time_us);
            if (dt <= 0) continue;
            double s = hypot(g_trace.obs[i].rel_x - g_trace.obs[i-1].rel_x,
                             g_trace.obs[i].rel_y - g_trace.obs[i-1].rel_y) / (dt/1000.0);
            if (s > obsMaxSpd) obsMaxSpd = s;
        }
    }
    double globalMaxSpd = fmax(cmdMaxSpd, obsMaxSpd);

    /* Draw paths */
    for (int pass = 0; pass < 2; pass++) {
        gboolean isCmd = (pass == 1);
        if (isCmd && !g_show_cmd) continue;
        if (!isCmd && !g_show_obs) continue;
        uint32_t n = isCmd ? ncmd : nobs;
        if (n < 2) continue;

        cairo_set_line_width(cr, 1.5);
        cairo_set_line_cap(cr, CAIRO_LINE_CAP_ROUND);
        cairo_set_line_join(cr, CAIRO_LINE_JOIN_ROUND);

        double prevSpd = 0;
        for (uint32_t i = 1; i < n; i++) {
            double x0, y0, x1, y1;
            if (isCmd) {
                screen_pt(g_trace.cmds[i-1].cum_x, g_trace.cmds[i-1].cum_y, &x0, &y0);
                screen_pt(g_trace.cmds[i].cum_x, g_trace.cmds[i].cum_y, &x1, &y1);
            } else {
                screen_pt(g_trace.obs[i-1].rel_x, g_trace.obs[i-1].rel_y, &x0, &y0);
                screen_pt(g_trace.obs[i].rel_x, g_trace.obs[i].rel_y, &x1, &y1);
            }

            if (g_show_speed) {
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
                double r, g, b; speed_color(spd, globalMaxSpd, &r, &g, &b);
                cairo_set_source_rgb(cr, r, g, b);
            } else {
                if (isCmd) cairo_set_source_rgb(cr, 78/255.0, 154/255.0, 240/255.0);
                else       cairo_set_source_rgb(cr, 240/255.0, 128/255.0, 64/255.0);
            }

            cairo_move_to(cr, x0, y0);
            cairo_line_to(cr, x1, y1);
            cairo_stroke(cr);
        }

        /* Start/end markers */
        double sx, sy, ex, ey;
        if (isCmd) {
            screen_pt(g_trace.cmds[0].cum_x, g_trace.cmds[0].cum_y, &sx, &sy);
            screen_pt(g_trace.cmds[n-1].cum_x, g_trace.cmds[n-1].cum_y, &ex, &ey);
        } else {
            screen_pt(g_trace.obs[0].rel_x, g_trace.obs[0].rel_y, &sx, &sy);
            screen_pt(g_trace.obs[n-1].rel_x, g_trace.obs[n-1].rel_y, &ex, &ey);
        }
        cairo_set_source_rgb(cr, 64/255.0, 216/255.0, 128/255.0);
        cairo_arc(cr, sx, sy, 4, 0, 2*M_PI); cairo_fill(cr);
        cairo_set_source_rgb(cr, 240/255.0, 64/255.0, 96/255.0);
        cairo_arc(cr, ex, ey, 4, 0, 2*M_PI); cairo_fill(cr);
    }

    return TRUE;
}

// ============================================================================
// Sidebar Drawing
// ============================================================================

static gboolean on_sidebar_draw(GtkWidget* widget, cairo_t* cr, gpointer data) {
    (void)data;
    int W = gtk_widget_get_allocated_width(widget);
    int H = gtk_widget_get_allocated_height(widget);

    cairo_set_source_rgb(cr, 11/255.0, 11/255.0, 18/255.0);
    cairo_rectangle(cr, 0, 0, W, H);
    cairo_fill(cr);

    /* Border */
    cairo_set_source_rgb(cr, 30/255.0, 30/255.0, 48/255.0);
    cairo_set_line_width(cr, 1);
    cairo_move_to(cr, 0, 0); cairo_line_to(cr, 0, H); cairo_stroke(cr);

    cairo_select_font_face(cr, "monospace", CAIRO_FONT_SLANT_NORMAL, CAIRO_FONT_WEIGHT_NORMAL);
    double y = 16;
    double w = W - 24;
    trace_analysis_t* a = &g_trace.analysis;

    #define SB_HEADER(text) do { \
        cairo_set_font_size(cr, 10); \
        cairo_set_source_rgb(cr, 72/255.0, 72/255.0, 96/255.0); \
        cairo_move_to(cr, 12, y); cairo_show_text(cr, text); \
        y += 18; \
    } while(0)

    #define SB_ROW(label, value, r, g, b) do { \
        cairo_set_font_size(cr, 11); \
        cairo_set_source_rgb(cr, 112/255.0, 112/255.0, 136/255.0); \
        cairo_move_to(cr, 12, y); cairo_show_text(cr, label); \
        cairo_set_source_rgb(cr, r, g, b); \
        cairo_text_extents_t _ext; cairo_text_extents(cr, value, &_ext); \
        cairo_move_to(cr, 12 + w - _ext.width, y); cairo_show_text(cr, value); \
        y += 17; \
    } while(0)

    if (!g_trace.analysis_valid) {
        SB_HEADER("NO DATA");
        cairo_set_font_size(cr, 11);
        cairo_set_source_rgb(cr, 72/255.0, 72/255.0, 96/255.0);
        cairo_move_to(cr, 12, y);
        cairo_show_text(cr, "Connect and run a test");
    } else {
        char buf[128];

        SB_HEADER("MOVEMENT");
        snprintf(buf, sizeof(buf), "%u", g_trace.cmd_count);
        SB_ROW("Commands", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%u", g_trace.obs_count);
        SB_ROW("Observations", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.1f ms", a->total_ms);
        SB_ROW("Duration", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.0f Hz", a->cmd_rate);
        SB_ROW("Cmd rate", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.0f Hz", a->obs_rate);
        SB_ROW("Obs rate", buf, 232/255.0, 232/255.0, 240/255.0);
        y += 6;

        SB_HEADER("COMMAND FIDELITY");
        double dr, dg, db;
        if (a->dev_avg < 2) { dr=64/255.0; dg=216/255.0; db=128/255.0; }
        else if (a->dev_avg < 5) { dr=232/255.0; dg=200/255.0; db=64/255.0; }
        else { dr=240/255.0; dg=64/255.0; db=96/255.0; }
        snprintf(buf, sizeof(buf), "%.2f px", a->dev_avg);
        SB_ROW("Avg gap", buf, dr, dg, db);
        snprintf(buf, sizeof(buf), "%.2f px", a->dev_max);
        SB_ROW("Max gap", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.0f%%", a->cmd_rep_pct);
        SB_ROW("Delta reps", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.3f", a->int_cv);
        SB_ROW("Interval CV", buf, 232/255.0, 232/255.0, 240/255.0);
        y += 6;

        SB_HEADER("HUMANIZATION");
        double hr, hg, hb;
        if (a->h_score < 35) { hr=240/255.0; hg=64/255.0; hb=96/255.0; }
        else if (a->h_score < 55) { hr=232/255.0; hg=200/255.0; hb=64/255.0; }
        else { hr=64/255.0; hg=216/255.0; hb=128/255.0; }
        cairo_set_source_rgb(cr, hr, hg, hb);
        cairo_set_font_size(cr, 22);
        snprintf(buf, sizeof(buf), "%d / 100", (int)a->h_score);
        cairo_move_to(cr, 12, y + 4); cairo_show_text(cr, buf);
        cairo_set_font_size(cr, 11);
        cairo_text_extents_t ext; cairo_text_extents(cr, a->h_grade, &ext);
        cairo_move_to(cr, 12 + w - ext.width, y); cairo_show_text(cr, a->h_grade);
        y += 28;

        snprintf(buf, sizeof(buf), "%.3f px", a->perp_scatter);
        SB_ROW("Perp scatter", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.3f px", a->jit_avg);
        SB_ROW("Jitter avg", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.1f%%", a->dir_flip_rate);
        SB_ROW("Dir reversals", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.3f", a->speed_cv);
        SB_ROW("Speed CV", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.1f%%", a->sub_px_pct);
        SB_ROW("Sub-pixel", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.4f", a->accel_jerk);
        SB_ROW("Accel jerk", buf, 232/255.0, 232/255.0, 240/255.0);
        snprintf(buf, sizeof(buf), "%.4f", a->path_eff);
        SB_ROW("Path eff", buf, 232/255.0, 232/255.0, 240/255.0);
    }

    #undef SB_HEADER
    #undef SB_ROW

    return TRUE;
}

// ============================================================================
// Canvas Mouse Events
// ============================================================================

static gboolean on_canvas_button_press(GtkWidget* w, GdkEventButton* ev, gpointer data) {
    (void)w; (void)data;
    if (ev->button == 1) {
        if (ev->type == GDK_2BUTTON_PRESS) {
            int W = gtk_widget_get_allocated_width(g_canvas);
            int H = gtk_widget_get_allocated_height(g_canvas);
            fit_view(W, H);
            gtk_widget_queue_draw(g_canvas);
        } else {
            g_dragging = TRUE;
            g_drag_sx = ev->x; g_drag_sy = ev->y;
            g_drag_vx = g_view_x; g_drag_vy = g_view_y;
        }
    }
    return TRUE;
}

static gboolean on_canvas_button_release(GtkWidget* w, GdkEventButton* ev, gpointer data) {
    (void)w; (void)ev; (void)data;
    g_dragging = FALSE;
    return TRUE;
}

static gboolean on_canvas_motion(GtkWidget* w, GdkEventMotion* ev, gpointer data) {
    (void)w; (void)data;
    if (g_dragging) {
        g_view_x = g_drag_vx + (ev->x - g_drag_sx);
        g_view_y = g_drag_vy + (ev->y - g_drag_sy);
        gtk_widget_queue_draw(g_canvas);
    }
    return TRUE;
}

static gboolean on_canvas_scroll(GtkWidget* w, GdkEventScroll* ev, gpointer data) {
    (void)w; (void)data;
    double factor;
    if (ev->direction == GDK_SCROLL_UP) factor = 1.15;
    else if (ev->direction == GDK_SCROLL_DOWN) factor = 1.0/1.15;
    else {
        /* Smooth scrolling */
        double dy = 0;
        gdk_event_get_scroll_deltas((GdkEvent*)ev, NULL, &dy);
        factor = (dy < 0) ? 1.15 : (1.0/1.15);
    }
    g_view_x = ev->x - (ev->x - g_view_x) * factor;
    g_view_y = ev->y - (ev->y - g_view_y) * factor;
    g_view_scale *= factor;
    gtk_widget_queue_draw(g_canvas);
    return TRUE;
}

// ============================================================================
// Callbacks
// ============================================================================

static void on_proto_changed(GtkComboBox* combo, gpointer data) {
    (void)data;
    int idx = gtk_combo_box_get_active(combo);
    device_type_t type = (idx == 0) ? DEV_KMBOX : (idx == 1) ? DEV_FERRUM : DEV_MAKCU;
    proto_init(&g_proto, type);
    char msg[128];
    snprintf(msg, sizeof(msg), "Protocol: %s (baud: %d)", g_proto.profile->name, g_proto.profile->default_baud);
    gtk_label_set_text(GTK_LABEL(g_label_status), msg);
}

static void on_connect_clicked(GtkButton* btn, gpointer data) {
    (void)data;
    if (g_app_connected) {
        serial_reader_stop();
        plat_serial_close(g_serial_fd);
        g_serial_fd = PLAT_SERIAL_INVALID;
        g_app_connected = FALSE;
        gtk_button_set_label(btn, "Connect");
        gtk_label_set_text(GTK_LABEL(g_label_status), "Disconnected");
        return;
    }

    const char* port = gtk_entry_get_text(GTK_ENTRY(g_entry_port));
    int baud = g_proto.profile->default_baud;
    g_serial_fd = plat_serial_open(port, baud);
    if (g_serial_fd == PLAT_SERIAL_INVALID) {
        char msg[256];
        snprintf(msg, sizeof(msg), "Failed: %s", strerror(errno));
        gtk_label_set_text(GTK_LABEL(g_label_status), msg);
        return;
    }
    serial_reader_start();
    g_app_connected = TRUE;
    gtk_button_set_label(btn, "Disconnect");

    char msg[256];
    snprintf(msg, sizeof(msg), "Connected: %s @ %d", g_proto.profile->name, baud);
    gtk_label_set_text(GTK_LABEL(g_label_status), msg);
}

static gboolean on_test_done(gpointer data) {
    (void)data;
    g_app_test_running = FALSE;
    gtk_button_set_label(GTK_BUTTON(g_btn_run), "\u25B6 Run");
    gtk_widget_set_sensitive(g_btn_run, TRUE);

    int W = gtk_widget_get_allocated_width(g_canvas);
    int H = gtk_widget_get_allocated_height(g_canvas);
    fit_view(W, H);
    gtk_widget_queue_draw(g_canvas);
    gtk_widget_queue_draw(g_sidebar);

    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(g_combo_test));
    char msg[256];
    snprintf(msg, sizeof(msg), "Done: %s - %lld sent, %lld ok, %lld err",
             all_tests[idx].name, (long long)g_stat_sent, (long long)g_stat_ok, (long long)g_stat_err);
    gtk_label_set_text(GTK_LABEL(g_label_status), msg);
    return G_SOURCE_REMOVE;
}

static int g_run_test_idx = -1;

static void* test_thread_fn(void* arg) {
    (void)arg;
    int idx = g_run_test_idx;
    if (idx >= 0 && idx < NUM_TESTS) {
        all_tests[idx].run();
        trace_analyze();
    }
    g_idle_add(on_test_done, NULL);
    return NULL;
}

static void on_run_clicked(GtkButton* btn, gpointer data) {
    (void)data;
    if (g_app_test_running) return;
    int idx = gtk_combo_box_get_active(GTK_COMBO_BOX(g_combo_test));
    if (idx < 0 || idx >= NUM_TESTS) return;

    g_app_test_running = TRUE;
    g_run_test_idx = idx;
    g_stat_ok = 0; g_stat_err = 0; g_stat_sent = 0;
    gtk_widget_set_sensitive(GTK_WIDGET(btn), FALSE);
    gtk_button_set_label(btn, "Running...");

    pthread_t t;
    pthread_create(&t, NULL, test_thread_fn, NULL);
    pthread_detach(t);
}

static gboolean on_refresh_tick(gpointer data) {
    (void)data;
    if (g_app_test_running && g_trace.recording) {
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

        int W = gtk_widget_get_allocated_width(g_canvas);
        int H = gtk_widget_get_allocated_height(g_canvas);
        fit_view(W, H);
        gtk_widget_queue_draw(g_canvas);

        char msg[256];
        snprintf(msg, sizeof(msg), "Recording: %u cmd, %u obs - %lld sent",
                 ncmd, nobs, (long long)g_stat_sent);
        gtk_label_set_text(GTK_LABEL(g_label_status), msg);
    }
    return G_SOURCE_CONTINUE;
}

// ============================================================================
// Display Toggle Callbacks
// ============================================================================

static void on_chk_cmd(GtkToggleButton* t, gpointer d) { (void)d; g_show_cmd = gtk_toggle_button_get_active(t); gtk_widget_queue_draw(g_canvas); }
static void on_chk_obs(GtkToggleButton* t, gpointer d) { (void)d; g_show_obs = gtk_toggle_button_get_active(t); gtk_widget_queue_draw(g_canvas); }
static void on_chk_dots(GtkToggleButton* t, gpointer d) { (void)d; g_show_dots = gtk_toggle_button_get_active(t); gtk_widget_queue_draw(g_canvas); }
static void on_chk_dev(GtkToggleButton* t, gpointer d) { (void)d; g_show_dev = gtk_toggle_button_get_active(t); gtk_widget_queue_draw(g_canvas); }
static void on_chk_grid(GtkToggleButton* t, gpointer d) { (void)d; g_show_grid = gtk_toggle_button_get_active(t); gtk_widget_queue_draw(g_canvas); }
static void on_chk_speed(GtkToggleButton* t, gpointer d) { (void)d; g_show_speed = gtk_toggle_button_get_active(t); gtk_widget_queue_draw(g_canvas); }

// ============================================================================
// CSS for Dark Theme
// ============================================================================

static const char* css_style =
    "window { background-color: #06060a; }"
    "label { color: #b0b0c0; font-family: monospace; font-size: 11px; }"
    "entry { background-color: #16161a; color: #e8e8f0; border: 1px solid #1e1e30;"
    "        font-family: monospace; font-size: 11px; padding: 2px 4px; }"
    "button { background-color: #16161a; color: #e8e8f0; border: 1px solid #1e1e30;"
    "         font-size: 11px; padding: 2px 8px; }"
    "button:hover { background-color: #22222a; }"
    "combobox button { background-color: #16161a; color: #e8e8f0; border: 1px solid #1e1e30; }"
    "checkbutton label { color: #7070a0; font-size: 11px; }"
    ".topbar { background-color: #0b0b12; border-bottom: 1px solid #1e1e30; }";

// ============================================================================
// Build GUI
// ============================================================================

static void activate_app(GtkApplication* app, gpointer user_data) {
    (void)user_data;

    /* CSS */
    GtkCssProvider* css = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css, css_style, -1, NULL);
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
        GTK_STYLE_PROVIDER(css), GTK_STYLE_PROVIDER_PRIORITY_APPLICATION);

    /* Window */
    g_window = gtk_application_window_new(app);
    gtk_window_set_title(GTK_WINDOW(g_window), "KMBox Trace Analyzer");
    gtk_window_set_default_size(GTK_WINDOW(g_window), 1400, 900);

    /* Main vertical layout */
    GtkWidget* vbox = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_container_add(GTK_CONTAINER(g_window), vbox);

    /* Top bar */
    GtkWidget* topbar = gtk_box_new(GTK_ORIENTATION_HORIZONTAL, 8);
    GtkStyleContext* tctx = gtk_widget_get_style_context(topbar);
    gtk_style_context_add_class(tctx, "topbar");
    gtk_widget_set_margin_start(topbar, 8);
    gtk_widget_set_margin_end(topbar, 8);
    gtk_widget_set_margin_top(topbar, 6);
    gtk_widget_set_margin_bottom(topbar, 6);
    gtk_box_pack_start(GTK_BOX(vbox), topbar, FALSE, FALSE, 0);

    /* Protocol combo */
    gtk_box_pack_start(GTK_BOX(topbar), gtk_label_new("Protocol:"), FALSE, FALSE, 0);
    g_combo_proto = gtk_combo_box_text_new();
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(g_combo_proto), "KMBox B+");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(g_combo_proto), "Ferrum One");
    gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(g_combo_proto), "MAKCU");
    gtk_combo_box_set_active(GTK_COMBO_BOX(g_combo_proto), 0);
    g_signal_connect(g_combo_proto, "changed", G_CALLBACK(on_proto_changed), NULL);
    gtk_box_pack_start(GTK_BOX(topbar), g_combo_proto, FALSE, FALSE, 0);

    /* Port entry */
    gtk_box_pack_start(GTK_BOX(topbar), gtk_label_new("Port:"), FALSE, FALSE, 0);
    g_entry_port = gtk_entry_new();
    gtk_entry_set_text(GTK_ENTRY(g_entry_port), "/dev/ttyUSB0");
    gtk_entry_set_width_chars(GTK_ENTRY(g_entry_port), 20);
    gtk_box_pack_start(GTK_BOX(topbar), g_entry_port, FALSE, FALSE, 0);

    /* Connect button */
    g_btn_connect = gtk_button_new_with_label("Connect");
    g_signal_connect(g_btn_connect, "clicked", G_CALLBACK(on_connect_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(topbar), g_btn_connect, FALSE, FALSE, 0);

    /* Test combo */
    gtk_box_pack_start(GTK_BOX(topbar), gtk_label_new("Test:"), FALSE, FALSE, 0);
    g_combo_test = gtk_combo_box_text_new();
    for (int i = 0; all_tests[i].name; i++)
        gtk_combo_box_text_append_text(GTK_COMBO_BOX_TEXT(g_combo_test), all_tests[i].name);
    gtk_combo_box_set_active(GTK_COMBO_BOX(g_combo_test), 0);
    gtk_box_pack_start(GTK_BOX(topbar), g_combo_test, FALSE, FALSE, 0);

    /* Run button */
    g_btn_run = gtk_button_new_with_label("\u25B6 Run");
    g_signal_connect(g_btn_run, "clicked", G_CALLBACK(on_run_clicked), NULL);
    gtk_box_pack_start(GTK_BOX(topbar), g_btn_run, FALSE, FALSE, 0);

    /* Status label */
    g_label_status = gtk_label_new("Disconnected");
    gtk_label_set_xalign(GTK_LABEL(g_label_status), 0);
    gtk_box_pack_start(GTK_BOX(topbar), g_label_status, TRUE, TRUE, 0);

    /* Main horizontal split: Canvas + Sidebar */
    GtkWidget* hpaned = gtk_paned_new(GTK_ORIENTATION_HORIZONTAL);
    gtk_paned_set_position(GTK_PANED(hpaned), 1080);
    gtk_box_pack_start(GTK_BOX(vbox), hpaned, TRUE, TRUE, 0);

    /* Canvas */
    g_canvas = gtk_drawing_area_new();
    gtk_widget_set_size_request(g_canvas, 400, 300);
    gtk_widget_add_events(g_canvas, GDK_BUTTON_PRESS_MASK | GDK_BUTTON_RELEASE_MASK |
                          GDK_POINTER_MOTION_MASK | GDK_SCROLL_MASK | GDK_SMOOTH_SCROLL_MASK);
    g_signal_connect(g_canvas, "draw", G_CALLBACK(on_canvas_draw), NULL);
    g_signal_connect(g_canvas, "button-press-event", G_CALLBACK(on_canvas_button_press), NULL);
    g_signal_connect(g_canvas, "button-release-event", G_CALLBACK(on_canvas_button_release), NULL);
    g_signal_connect(g_canvas, "motion-notify-event", G_CALLBACK(on_canvas_motion), NULL);
    g_signal_connect(g_canvas, "scroll-event", G_CALLBACK(on_canvas_scroll), NULL);
    gtk_paned_pack1(GTK_PANED(hpaned), g_canvas, TRUE, TRUE);

    /* Sidebar container */
    GtkWidget* sidebar_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 0);
    gtk_widget_set_size_request(sidebar_box, 320, -1);

    /* Sidebar drawing area */
    g_sidebar = gtk_drawing_area_new();
    gtk_widget_set_size_request(g_sidebar, 320, 500);
    g_signal_connect(g_sidebar, "draw", G_CALLBACK(on_sidebar_draw), NULL);
    gtk_box_pack_start(GTK_BOX(sidebar_box), g_sidebar, TRUE, TRUE, 0);

    /* Display checkboxes */
    GtkWidget* chk_box = gtk_box_new(GTK_ORIENTATION_VERTICAL, 2);
    gtk_widget_set_margin_start(chk_box, 12);
    gtk_widget_set_margin_top(chk_box, 8);
    gtk_widget_set_margin_bottom(chk_box, 8);

    struct { const char* label; GCallback cb; gboolean active; } checks[] = {
        {"Commanded path (blue)", G_CALLBACK(on_chk_cmd), TRUE},
        {"Observed path (orange)", G_CALLBACK(on_chk_obs), TRUE},
        {"Sample dots", G_CALLBACK(on_chk_dots), TRUE},
        {"Deviation lines", G_CALLBACK(on_chk_dev), FALSE},
        {"Grid", G_CALLBACK(on_chk_grid), TRUE},
        {"Color by speed", G_CALLBACK(on_chk_speed), TRUE},
    };
    for (int i = 0; i < 6; i++) {
        GtkWidget* chk = gtk_check_button_new_with_label(checks[i].label);
        gtk_toggle_button_set_active(GTK_TOGGLE_BUTTON(chk), checks[i].active);
        g_signal_connect(chk, "toggled", checks[i].cb, NULL);
        gtk_box_pack_start(GTK_BOX(chk_box), chk, FALSE, FALSE, 0);
    }
    gtk_box_pack_start(GTK_BOX(sidebar_box), chk_box, FALSE, FALSE, 0);

    gtk_paned_pack2(GTK_PANED(hpaned), sidebar_box, FALSE, FALSE);

    /* Refresh timer */
    g_timeout_add(50, on_refresh_tick, NULL);

    gtk_widget_show_all(g_window);
}

// ============================================================================
// Main Entry Point
// ============================================================================

int main(int argc, char* argv[]) {
    trace_alloc();
    srand((unsigned)time(NULL));
    proto_init(&g_proto, DEV_KMBOX);

    /* Check for CLI mode */
    const char* cli_port = NULL;
    const char* cli_proto_name = "kmbox";
    const char* cli_test = NULL;
    int cli_baud = 0;

    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--help") || !strcmp(argv[i], "-h")) {
            print_usage(argv[0]); trace_dealloc(); return 0;
        }
        if (!strcmp(argv[i], "--port") && i+1 < argc)  cli_port = argv[++i];
        else if (!strcmp(argv[i], "--proto") && i+1 < argc) cli_proto_name = argv[++i];
        else if (!strcmp(argv[i], "--test") && i+1 < argc)  cli_test = argv[++i];
        else if (!strcmp(argv[i], "--baud") && i+1 < argc)  cli_baud = atoi(argv[++i]);
    }

    if (cli_test && cli_port) {
        /* CLI mode */
        proto_init(&g_proto, device_from_name(cli_proto_name));
        int baud = cli_baud > 0 ? cli_baud : g_proto.profile->default_baud;

        printf("Opening %s (%s @ %d)...\n", cli_port, g_proto.profile->name, baud);
        g_serial_fd = plat_serial_open(cli_port, baud);
        if (g_serial_fd == PLAT_SERIAL_INVALID) { perror("open"); trace_dealloc(); return 1; }

        serial_reader_start();

        bool found = false;
        for (int i = 0; all_tests[i].name; i++) {
            if (!strcmp(all_tests[i].name, cli_test)) {
                printf("Running test: %s\n", cli_test);
                all_tests[i].run();
                trace_analyze();
                cli_print_results(cli_test);
                found = true; break;
            }
        }
        if (!found) fprintf(stderr, "Unknown test: %s\n", cli_test);

        serial_reader_stop();
        plat_serial_close(g_serial_fd);
        trace_dealloc();
        return found ? 0 : 1;
    }

    /* GUI mode */
    GtkApplication* app = gtk_application_new("dev.kmbox.tester", G_APPLICATION_DEFAULT_FLAGS);
    g_signal_connect(app, "activate", G_CALLBACK(activate_app), NULL);
    int status = g_application_run(G_APPLICATION(app), 0, NULL);
    g_object_unref(app);

    if (g_app_connected) {
        serial_reader_stop();
        plat_serial_close(g_serial_fd);
    }
    trace_dealloc();
    return status;
}
